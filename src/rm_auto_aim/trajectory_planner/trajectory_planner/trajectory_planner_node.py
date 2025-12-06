# Copyright (C) FYT Vision Group. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Trajectory Planner Node

Main ROS2 node that integrates all trajectory planning modules.
Provides Action server for trajectory_plan and Service server for set_target_robot.
"""

import math
import numpy as np
from typing import Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

from rm_interfaces.msg import (
    TrackedRobots,
    TrackPredictionWindows,
    GimbalCmd,
    GimbalTrajectory,
    GimbalState
)
from rm_interfaces.srv import SetTargetRobot
from rm_interfaces.action import TrajectoryPlan

from trajectory_planner.gimbal_model import GimbalModel, GimbalConfig
from trajectory_planner.mpc_controller import MPCController, MPCConfig
from trajectory_planner.target_predictor import TargetPredictor, TargetPredictorConfig
from trajectory_planner.target_manager import TargetManager, TargetManagerConfig
from trajectory_planner.ballistic_client import BallisticClient, BallisticClientConfig


@dataclass
class TopicConfig:
    """话题配置"""
    
    # 订阅话题
    prediction_windows_sub: str = "/armor_tracker/prediction_windows"
    robots_sub: str = "/robot_pose_estimator/robots"
    joint_states_sub: str = "/joint_states"
    
    # 发布话题
    gimbal_cmd_pub: str = "/trajectory_planner/gimbal_cmd"
    trajectory_pub: str = "/trajectory_planner/trajectory"
    markers_pub: str = "/trajectory_planner/markers"
    
    # 服务
    set_target_service: str = "~/set_target_robot"
    ballistic_service: str = "/ballistic_solver/solve"
    
    # Action
    trajectory_action: str = "~/trajectory_plan"


@dataclass
class PlannerConfig:
    """规划器配置"""
    
    # 控制频率 (Hz)
    control_rate: float = 100.0
    
    # 开火判断阈值
    fire_yaw_threshold: float = 0.05      # yaw误差阈值 (rad), ~3度
    fire_pitch_threshold: float = 0.05    # pitch误差阈值 (rad)
    fire_distance_min: float = 1.0        # 最小开火距离 (m)
    fire_distance_max: float = 8.0        # 最大开火距离 (m)
    fire_confidence_threshold: float = 0.5  # 开火置信度阈值
    
    # 目标丢失超时
    target_lost_timeout: float = 0.5      # 目标丢失超时 (秒)
    
    # 调试模式
    debug: bool = False
    
    # 子弹速度
    bullet_speed: float = 28.0


class TrajectoryPlannerNode(Node):
    """
    轨迹规划器节点
    
    功能:
    1. 订阅 armor_tracker 的预测窗口，获取装甲板预测轨迹
    2. 订阅 robot_pose_estimator 的机器人信息
    3. 通过 SetTargetRobot service 接收目标设置请求
    4. 提供 TrajectoryPlan action 用于轨迹规划控制
    5. 使用 MPC 优化 yaw 控制
    6. 调用 ballistic_solver 计算 pitch
    7. 发布 GimbalCmd 消息
    """
    
    # 规划状态常量
    STATUS_IDLE = 0
    STATUS_PLANNING = 1
    STATUS_TRACKING = 2
    STATUS_LOST_TARGET = 3
    
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # 设置日志级别为 INFO（生产环境）或 DEBUG（调试时）
        # DEBUG 模式会显著降低性能
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        self.get_logger().info("Initializing Trajectory Planner Node...")
        
        # 声明参数
        self._declare_parameters()
        
        # 加载配置
        self.topic_config = self._build_topic_config()
        self.planner_config = self._build_planner_config()
        gimbal_config = self._build_gimbal_config()
        mpc_config = self._build_mpc_config()
        predictor_config = self._build_predictor_config()
        manager_config = self._build_manager_config()
        ballistic_config = self._build_ballistic_config()
        
        # 创建核心模块
        self.gimbal_model = GimbalModel(gimbal_config)
        self.mpc_controller = MPCController(self.gimbal_model, mpc_config)
        self.target_predictor = TargetPredictor(predictor_config)
        self.target_manager = TargetManager(manager_config)
        
        # 回调组 - 全部使用 ReentrantCallbackGroup 避免阻塞
        # MutuallyExclusiveCallbackGroup 会导致回调互相阻塞，降低帧率
        self.service_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()  # 定时器也用可重入组
        self.subscription_callback_group = ReentrantCallbackGroup()  # 订阅使用可重入组
        
        # 创建弹道解算客户端 (需要在订阅之前)
        self.ballistic_client = BallisticClient(self, ballistic_config)
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅者 - 使用可重入回调组，确保与定时器不互斥
        self.prediction_windows_sub = self.create_subscription(
            TrackPredictionWindows,
            self.topic_config.prediction_windows_sub,
            self._prediction_windows_callback,
            sensor_qos,
            callback_group=self.subscription_callback_group
        )
        
        self.robots_sub = self.create_subscription(
            TrackedRobots,
            self.topic_config.robots_sub,
            self._robots_callback,
            sensor_qos,
            callback_group=self.subscription_callback_group
        )
        
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.topic_config.joint_states_sub,
            self._joint_states_callback,
            sensor_qos,
            callback_group=self.subscription_callback_group
        )
        
        # 发布者
        self.gimbal_cmd_pub = self.create_publisher(
            GimbalCmd,
            self.topic_config.gimbal_cmd_pub,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            GimbalTrajectory,
            self.topic_config.trajectory_pub,
            10
        )
        
        if self.planner_config.debug:
            self.markers_pub = self.create_publisher(
                MarkerArray,
                self.topic_config.markers_pub,
                10
            )
        else:
            self.markers_pub = None
        
        # 服务 - SetTargetRobot
        self.set_target_service = self.create_service(
            SetTargetRobot,
            self.topic_config.set_target_service,
            self._handle_set_target_robot,
            callback_group=self.service_callback_group
        )
        
        # Action - TrajectoryPlan
        self.trajectory_action_server = ActionServer(
            self,
            TrajectoryPlan,
            self.topic_config.trajectory_action,
            execute_callback=self._execute_trajectory_plan,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_callback_group
        )
        
        # 状态变量
        self._current_gimbal_state = np.array([0.0, 0.0, 0.0])  # [yaw, yaw_vel, yaw_acc]
        self._current_pitch = 0.0
        self._planning_status = self.STATUS_IDLE
        self._last_target_time: Optional[float] = None
        self._is_tracking_enabled = False
        self._total_tracking_time = 0.0
        
        # 控制定时器
        control_period = 1.0 / self.planner_config.control_rate
        self.control_timer = self.create_timer(
            control_period,
            self._control_loop,
            callback_group=self.timer_callback_group
        )
        
        # 等待弹道解算服务
        self.get_logger().info("Waiting for ballistic solver service...")
        if self.ballistic_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Ballistic solver service available")
        else:
            self.get_logger().warn("Ballistic solver service not available, using simple estimation")
        
        self.get_logger().info("Trajectory Planner Node initialized")
        self.get_logger().info(f"  Subscribing predictions from: {self.topic_config.prediction_windows_sub}")
        self.get_logger().info(f"  Subscribing robots from: {self.topic_config.robots_sub}")
        self.get_logger().info(f"  Publishing gimbal cmd to: {self.topic_config.gimbal_cmd_pub}")
        self.get_logger().info(f"  Service: {self.topic_config.set_target_service}")
        self.get_logger().info(f"  Action: {self.topic_config.trajectory_action}")
    
    # ==================== 参数声明和配置构建 ====================
    
    def _declare_parameters(self):
        """声明ROS参数"""
        # 话题配置
        self.declare_parameter('topics.prediction_windows_sub', '/armor_tracker/prediction_windows')
        self.declare_parameter('topics.robots_sub', '/robot_pose_estimator/robots')
        self.declare_parameter('topics.joint_states_sub', '/joint_states')
        self.declare_parameter('topics.gimbal_cmd_pub', '/trajectory_planner/gimbal_cmd')
        self.declare_parameter('topics.trajectory_pub', '/trajectory_planner/trajectory')
        self.declare_parameter('topics.markers_pub', '/trajectory_planner/markers')
        self.declare_parameter('topics.ballistic_service', '/ballistic_solver/solve')
        
        # 规划器配置
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('fire_yaw_threshold', 0.05)
        self.declare_parameter('fire_pitch_threshold', 0.05)
        self.declare_parameter('fire_distance_min', 1.0)
        self.declare_parameter('fire_distance_max', 8.0)
        self.declare_parameter('fire_confidence_threshold', 0.5)
        self.declare_parameter('target_lost_timeout', 0.5)
        self.declare_parameter('bullet_speed', 28.0)
        self.declare_parameter('debug', False)
        
        # 云台配置
        self.declare_parameter('gimbal.dt', 0.01)
        self.declare_parameter('gimbal.omega_max_deg', 360.0)
        self.declare_parameter('gimbal.theta_min_deg', -90.0)
        self.declare_parameter('gimbal.theta_max_deg', 90.0)
        
        # MPC配置
        self.declare_parameter('mpc.prediction_horizon', 18)
        self.declare_parameter('mpc.q_theta', 100.0)
        self.declare_parameter('mpc.q_omega', 10.0)
        self.declare_parameter('mpc.q_alpha', 1.0)
        self.declare_parameter('mpc.r_control', 0.01)
        self.declare_parameter('mpc.s_smooth', 5.0)
        
        # 目标预测器配置
        self.declare_parameter('predictor.min_confidence', 0.3)
        self.declare_parameter('predictor.max_distance', 10.0)  # 最大距离10米
        self.declare_parameter('predictor.selection_strategy', 'nearest_yaw')
        
        # 目标管理器配置
        self.declare_parameter('manager.target_timeout', 2.0)
        self.declare_parameter('manager.robot_info_timeout', 1.0)
    
    def _build_topic_config(self) -> TopicConfig:
        """构建话题配置"""
        return TopicConfig(
            prediction_windows_sub=self.get_parameter('topics.prediction_windows_sub').value,
            robots_sub=self.get_parameter('topics.robots_sub').value,
            joint_states_sub=self.get_parameter('topics.joint_states_sub').value,
            gimbal_cmd_pub=self.get_parameter('topics.gimbal_cmd_pub').value,
            trajectory_pub=self.get_parameter('topics.trajectory_pub').value,
            markers_pub=self.get_parameter('topics.markers_pub').value,
            ballistic_service=self.get_parameter('topics.ballistic_service').value,
        )
    
    def _build_planner_config(self) -> PlannerConfig:
        """构建规划器配置"""
        return PlannerConfig(
            control_rate=self.get_parameter('control_rate').value,
            fire_yaw_threshold=self.get_parameter('fire_yaw_threshold').value,
            fire_pitch_threshold=self.get_parameter('fire_pitch_threshold').value,
            fire_distance_min=self.get_parameter('fire_distance_min').value,
            fire_distance_max=self.get_parameter('fire_distance_max').value,
            fire_confidence_threshold=self.get_parameter('fire_confidence_threshold').value,
            target_lost_timeout=self.get_parameter('target_lost_timeout').value,
            bullet_speed=self.get_parameter('bullet_speed').value,
            debug=self.get_parameter('debug').value,
        )
    
    def _build_gimbal_config(self) -> GimbalConfig:
        """构建云台配置"""
        config = GimbalConfig(
            dt=self.get_parameter('gimbal.dt').value,
            theta_min=np.deg2rad(self.get_parameter('gimbal.theta_min_deg').value),
            theta_max=np.deg2rad(self.get_parameter('gimbal.theta_max_deg').value),
        )
        omega_max_deg = self.get_parameter('gimbal.omega_max_deg').value
        config.compute_limits_from_omega(omega_max_deg)
        return config
    
    def _build_mpc_config(self) -> MPCConfig:
        """构建MPC配置"""
        return MPCConfig(
            prediction_horizon=self.get_parameter('mpc.prediction_horizon').value,
            dt=self.get_parameter('gimbal.dt').value,
            q_theta=self.get_parameter('mpc.q_theta').value,
            q_omega=self.get_parameter('mpc.q_omega').value,
            q_alpha=self.get_parameter('mpc.q_alpha').value,
            r_control=self.get_parameter('mpc.r_control').value,
            s_smooth=self.get_parameter('mpc.s_smooth').value,
        )
    
    def _build_predictor_config(self) -> TargetPredictorConfig:
        """构建目标预测器配置"""
        return TargetPredictorConfig(
            min_confidence=self.get_parameter('predictor.min_confidence').value,
            prediction_steps=self.get_parameter('mpc.prediction_horizon').value,
            dt=self.get_parameter('gimbal.dt').value,
            max_distance=self.get_parameter('predictor.max_distance').value,
            selection_strategy=self.get_parameter('predictor.selection_strategy').value,
        )
    
    def _build_manager_config(self) -> TargetManagerConfig:
        """构建目标管理器配置"""
        return TargetManagerConfig(
            target_timeout=self.get_parameter('manager.target_timeout').value,
            robot_info_timeout=self.get_parameter('manager.robot_info_timeout').value,
            min_confidence=self.get_parameter('predictor.min_confidence').value,
        )
    
    def _build_ballistic_config(self) -> BallisticClientConfig:
        """构建弹道解算客户端配置"""
        return BallisticClientConfig(
            service_name=self.get_parameter('topics.ballistic_service').value,
            default_bullet_speed=self.get_parameter('bullet_speed').value,
        )
    
    # ==================== 订阅回调 ====================
    
    def _prediction_windows_callback(self, msg: TrackPredictionWindows):
        """预测窗口回调"""
        self.get_logger().debug(f"Received prediction windows: {len(msg.windows)} windows")
        for i, window in enumerate(msg.windows):
            self.get_logger().debug(f"  Window {i}: track_id={window.track_id}, armor_id={window.armor_id}, "
                                   f"armor_type={window.armor_type}, predictions={len(window.predictions)}")
        self.target_predictor.update_predictions(msg)
    
    def _robots_callback(self, msg: TrackedRobots):
        """机器人信息回调"""
        self.get_logger().debug(f"Received robots: {len(msg.robots)} robots")
        for i, robot in enumerate(msg.robots):
            self.get_logger().debug(f"  Robot {i}: id={robot.robot_id}, confidence={robot.confidence:.3f}, "
                                   f"position=({robot.center_position.x:.2f}, {robot.center_position.y:.2f}, {robot.center_position.z:.2f}), "
                                   f"bound_armor_ids={list(robot.bound_armor_ids)}")
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.target_manager.update_robots(msg, current_time)
    
    def _joint_states_callback(self, msg: JointState):
        """关节状态回调 - 获取当前云台状态"""
        self.get_logger().debug(f"Received joint states: {len(msg.name)} joints")
        
        # 查找yaw关节
        try:
            yaw_idx = msg.name.index('yaw_joint')
            old_yaw = self._current_gimbal_state[0]
            self._current_gimbal_state[0] = msg.position[yaw_idx]
            if len(msg.velocity) > yaw_idx:
                self._current_gimbal_state[1] = msg.velocity[yaw_idx]
            self.get_logger().debug(f"  Yaw joint: pos={self._current_gimbal_state[0]:.3f} "
                                   f"(changed: {abs(self._current_gimbal_state[0] - old_yaw):.3f}), "
                                   f"vel={self._current_gimbal_state[1]:.3f}")
        except ValueError:
            self.get_logger().debug("  Yaw joint not found in joint states")
        
        # 查找pitch关节
        try:
            pitch_idx = msg.name.index('pitch_joint')
            old_pitch = self._current_pitch
            self._current_pitch = msg.position[pitch_idx]
            self.get_logger().debug(f"  Pitch joint: pos={self._current_pitch:.3f} "
                                   f"(changed: {abs(self._current_pitch - old_pitch):.3f})")
        except ValueError:
            self.get_logger().debug("  Pitch joint not found in joint states")
    
    # ==================== 服务处理 ====================
    
    def _handle_set_target_robot(self, 
                                  request: SetTargetRobot.Request,
                                  response: SetTargetRobot.Response) -> SetTargetRobot.Response:
        """处理设置目标机器人服务请求"""
        self.get_logger().info(f"SetTargetRobot service called: robot_id='{request.robot_id}'")
        
        success, message, previous_id = self.target_manager.set_target_robot(request.robot_id)
        
        self.get_logger().debug(f"SetTargetRobot result: success={success}, message='{message}', "
                               f"previous_id='{previous_id}'")
        
        response.success = success
        response.message = message
        response.previous_robot_id = previous_id
        
        if success and request.robot_id:
            self.get_logger().info(f"Target robot set to: {request.robot_id}")
            self._planning_status = self.STATUS_PLANNING
        elif success and not request.robot_id:
            self.get_logger().info("Target robot cleared")
            self._planning_status = self.STATUS_IDLE
        
        return response
    
    # ==================== Action处理 ====================
    
    def _goal_callback(self, goal_request) -> GoalResponse:
        """Action目标回调"""
        self.get_logger().info(f"Received trajectory plan goal for robot: {goal_request.robot_id}")
        self.get_logger().debug(f"Goal callback: current planning status: {self._planning_status}")
        
        # 允许在 IDLE 或 PLANNING 状态下接受 goal
        # PLANNING 状态可能由 SetTargetRobot service 设置，此时应该接受 action goal
        if self._planning_status not in (self.STATUS_IDLE, self.STATUS_PLANNING):
            self.get_logger().warn(f"Goal callback: rejecting goal, current status: {self._planning_status}")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Action取消回调"""
        self.get_logger().info("Received cancel request for trajectory plan")
        return CancelResponse.ACCEPT
    
    async def _execute_trajectory_plan(self, goal_handle: ServerGoalHandle):
        """执行轨迹规划Action"""
        request = goal_handle.request
        
        self.get_logger().info(f"Executing trajectory plan for robot: {request.robot_id}")
        self.get_logger().debug(f"Execute trajectory plan: enable_tracking={request.enable_tracking}, "
                               f"robot_id={request.robot_id}")
        
        # 设置目标
        self.target_manager.set_target_robot(request.robot_id)
        self._is_tracking_enabled = request.enable_tracking
        self._total_tracking_time = 0.0
        self._planning_status = self.STATUS_PLANNING
        
        # 创建反馈和结果
        feedback = TrajectoryPlan.Feedback()
        result = TrajectoryPlan.Result()
        
        # 控制循环
        rate = self.create_rate(self.planner_config.control_rate)
        start_time = self.get_clock().now()
        
        self.get_logger().debug(f"Execute trajectory plan: starting control loop with rate {self.planner_config.control_rate} Hz")
        
        while rclpy.ok() and self._is_tracking_enabled:
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                self.get_logger().debug("Execute trajectory plan: action canceled")
                goal_handle.canceled()
                result.success = False
                result.message = "Action canceled"
                result.total_tracking_time = self._total_tracking_time
                self._planning_status = self.STATUS_IDLE
                return result
            
            # 获取目标信息
            target_info = self.target_predictor.get_target_info()
            
            self.get_logger().debug(f"Execute trajectory plan: target_info available: {target_info is not None}")
            
            if target_info:
                self._planning_status = self.STATUS_TRACKING
                self._last_target_time = self.get_clock().now().nanoseconds * 1e-9
                
                self.get_logger().debug(f"Execute trajectory plan: target info - distance={target_info['distance']:.2f}, "
                                       f"yaw_from_origin={target_info['yaw_from_origin']:.3f}, "
                                       f"confidence={target_info.get('confidence', 0.0):.3f}")
                
                # 计算误差
                yaw_error = GimbalModel.angle_difference(
                    target_info['yaw_from_origin'],
                    self._current_gimbal_state[0]
                )
                
                self.get_logger().debug(f"Execute trajectory plan: yaw error: {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f} deg)")
                
                # 简单的pitch误差估算
                pitch_error = 0.0  # TODO: 更准确的pitch误差计算
                
                # 更新反馈
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.current_robot_id = request.robot_id
                feedback.yaw_error = yaw_error
                feedback.pitch_error = pitch_error
                feedback.distance = target_info['distance']
                feedback.target_locked = abs(yaw_error) < self.planner_config.fire_yaw_threshold
                feedback.planning_status = self._planning_status
                
                self.get_logger().debug(f"Execute trajectory plan: feedback - target_locked={feedback.target_locked}, "
                                       f"planning_status={feedback.planning_status}")
                
                goal_handle.publish_feedback(feedback)
            else:
                # 检查目标丢失超时
                current_time = self.get_clock().now().nanoseconds * 1e-9
                if self._last_target_time is not None:
                    time_since_lost = current_time - self._last_target_time
                    self.get_logger().debug(f"Execute trajectory plan: target lost, time_since_lost={time_since_lost:.3f}s, "
                                           f"timeout={self.planner_config.target_lost_timeout:.3f}s")
                    if time_since_lost > self.planner_config.target_lost_timeout:
                        self.get_logger().debug("Execute trajectory plan: target lost timeout exceeded")
                        self._planning_status = self.STATUS_LOST_TARGET
                        
                        feedback.header.stamp = self.get_clock().now().to_msg()
                        feedback.planning_status = self.STATUS_LOST_TARGET
                        feedback.target_locked = False
                        goal_handle.publish_feedback(feedback)
            
            # 更新跟踪时间
            self._total_tracking_time = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            
            rate.sleep()
        
        self.get_logger().debug(f"Execute trajectory plan: loop exited, total_tracking_time={self._total_tracking_time:.3f}s")
        
        # 完成
        goal_handle.succeed()
        result.success = True
        result.message = "Trajectory planning completed"
        result.total_tracking_time = self._total_tracking_time
        self._planning_status = self.STATUS_IDLE
        
        return result
    
    # ==================== 控制循环 ====================
    
    def _control_loop(self):
        """主控制循环"""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        self.get_logger().debug(f"Control loop: status={self._planning_status}, time={current_time:.3f}")
        
        # 仅在 IDLE 状态且没有目标时才发布空闲命令
        # PLANNING/TRACKING 状态由 Service 或 Action 触发后进入工作模式
        if self._planning_status == self.STATUS_IDLE:
            # 检查是否有通过 service 设置的目标（即使 action 未启动）
            if not self.target_manager.target_robot_id:
                self.get_logger().debug("Control loop: IDLE status and no target, publishing idle command")
                self._publish_idle_cmd()
                return
            else:
                # 有目标但状态是 IDLE，可能是 service 设置后未启动 action，自动切换到 PLANNING
                self.get_logger().debug(f"Control loop: IDLE but has target {self.target_manager.target_robot_id}, switching to PLANNING")
                self._planning_status = self.STATUS_PLANNING
        
        # 检查目标是否有效
        if not self.target_manager.is_target_valid(current_time):
            # 原有提示
            self.get_logger().debug("Control loop: target not valid, publishing idle command")

            # 详细调试信息，帮助排查为什么无效
            try:
                target_id = self.target_manager.target_robot_id
                available_ids = self.target_manager.available_robot_ids
                min_conf = getattr(self.target_manager.config, 'min_confidence', None)
                robot_info = self.target_manager.get_target_robot_info()
                last_update = getattr(self.target_manager, '_last_robots_update_time', None)
                time_since_update = None
                if last_update is not None:
                    time_since_update = current_time - last_update

                self.get_logger().debug(
                    f"Control loop debug: target_id={target_id}, available_ids={available_ids}, "
                    f"min_conf={min_conf}, last_update={last_update}, time_since_update={time_since_update}"
                )

                if robot_info is None:
                    self.get_logger().debug("Control loop debug: target robot info is None (not tracked or cleared)")
                else:
                    self.get_logger().debug(
                        f"Control loop debug: robot_info: id={robot_info.robot_id}, "
                        f"conf={robot_info.confidence:.3f}, num_armors={robot_info.num_armors}, "
                        f"bound_armor_ids={robot_info.bound_armor_ids}"
                    )
            except Exception as e:
                self.get_logger().warning(f"Control loop debug: failed to gather target_manager debug info: {e}")

            # 保持原有行为：发布空闲命令
            self._publish_idle_cmd()
            return
        
        # 获取目标装甲板ID列表（现在是track_id字符串）
        target_armor_ids = self.target_manager.get_target_armor_ids()
        self.get_logger().debug(f"Control loop: target armor IDs (track_ids): {target_armor_ids}")
        
        # 将track_id字符串转换为整数列表
        target_track_ids = []
        for track_id_str in target_armor_ids:
            try:
                track_id = int(track_id_str)
                target_track_ids.append(track_id)
            except ValueError:
                # 如果不是数字，可能是旧格式的armor_id，尝试通过armor_id筛选
                track_ids = self.target_predictor.filter_by_robot_id(track_id_str)
                target_track_ids.extend(track_ids)
        
        # 如果 bound_armor_ids 为空但有目标机器人，通过 robot_id（=armor_id）来过滤
        if not target_track_ids:
            target_robot_id = self.target_manager.target_robot_id
            if target_robot_id:
                self.get_logger().debug(f"Control loop: bound_armor_ids empty, filtering by robot_id={target_robot_id}")
                target_track_ids = self.target_predictor.filter_by_robot_id(target_robot_id)
        
        self.get_logger().debug(f"Control loop: target track IDs: {target_track_ids}")
        
        # 选择最佳装甲板
        current_yaw = self._current_gimbal_state[0]
        selected_track_id = self.target_predictor.select_best_armor(
            reference_yaw=current_yaw,
            target_track_ids=target_track_ids if target_track_ids else None
        )
        
        self.get_logger().debug(f"Control loop: selected track ID: {selected_track_id}")
        
        if selected_track_id is None:
            self.get_logger().debug("Control loop: no track selected, publishing idle command")
            self._publish_idle_cmd()
            self._planning_status = self.STATUS_LOST_TARGET
            return
        
        self._planning_status = self.STATUS_TRACKING
        self._last_target_time = current_time
        
        # 获取目标yaw轨迹
        target_yaw_trajectory = self.target_predictor.get_target_yaw_trajectory(selected_track_id)
        
        if target_yaw_trajectory is None:
            self.get_logger().debug("Control loop: no yaw trajectory available, publishing idle command")
            self._publish_idle_cmd()
            return
        
        self.get_logger().debug(f"Control loop: target yaw trajectory length: {len(target_yaw_trajectory)}")
        
        # MPC优化求解
        u_optimal, U_sequence, solve_time = self.mpc_controller.solve(
            self._current_gimbal_state,
            target_yaw_trajectory
        )
        
        self.get_logger().debug(f"Control loop: MPC solve time: {solve_time:.3f}ms, u_optimal: {u_optimal}")
        
        # 更新云台状态 (仿真/预测)
        next_state = self.gimbal_model.predict(self._current_gimbal_state, u_optimal)
        self.get_logger().debug(f"Control loop: predicted next state: yaw={next_state[0]:.3f}, "
                               f"yaw_vel={next_state[1]:.3f}, yaw_acc={next_state[2]:.3f}")
        
        # 计算pitch
        target_info = self.target_predictor.get_target_info(selected_track_id)
        pitch = 0.0
        distance = 0.0
        
        if target_info:
            distance = target_info['distance']
            target_position = target_info['position']
            target_velocity = target_info['velocity']
            
            self.get_logger().debug(f"Control loop: target info - distance={distance:.2f}, "
                                   f"position=({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}), "
                                   f"velocity=({target_velocity[0]:.2f}, {target_velocity[1]:.2f}, {target_velocity[2]:.2f})")
            
            # 使用弹道解算服务计算pitch
            if self.ballistic_client.is_service_available():
                self.get_logger().debug("Control loop: using ballistic solver service")
                # 在优化后的yaw方向上寻找可见装甲板并计算pitch
                ballistic_result = self.ballistic_client.compute_pitch_from_yaw(
                    yaw=next_state[0],
                    target_position=tuple(target_position),
                    target_velocity=tuple(target_velocity),
                    bullet_speed=self.planner_config.bullet_speed
                )
                if ballistic_result.success:
                    pitch = ballistic_result.pitch
                    self.get_logger().debug(f"Control loop: ballistic solver success, pitch={pitch:.3f}")
                else:
                    self.get_logger().debug("Control loop: ballistic solver failed, using simple estimate")
                    # 使用简单估算
                    pitch = self.ballistic_client.simple_pitch_estimate(
                        distance=distance,
                        height=target_position[2],
                        bullet_speed=self.planner_config.bullet_speed
                    )
            else:
                self.get_logger().debug("Control loop: ballistic service not available, using simple estimate")
                # 使用简单估算
                pitch = self.ballistic_client.simple_pitch_estimate(
                    distance=distance,
                    height=target_position[2],
                    bullet_speed=self.planner_config.bullet_speed
                )
        else:
            self.get_logger().debug("Control loop: no target info available")
        
        # 计算yaw误差
        yaw_error = GimbalModel.angle_difference(
            target_yaw_trajectory[0],
            self._current_gimbal_state[0]
        )
        
        self.get_logger().debug(f"Control loop: yaw error: {yaw_error:.3f} rad ({math.degrees(yaw_error):.1f} deg)")
        
        # 判断是否可以开火
        fire_advice = self._compute_fire_advice(
            yaw_error=yaw_error,
            pitch_error=pitch - self._current_pitch,
            distance=distance,
            confidence=target_info['confidence'] if target_info else 0.0
        )
        
        self.get_logger().debug(f"Control loop: fire advice: {fire_advice}")
        
        # 发布控制指令
        self._publish_gimbal_cmd(
            pitch=pitch,
            yaw=next_state[0],
            yaw_diff=yaw_error,
            pitch_diff=pitch - self._current_pitch,
            distance=distance,
            fire_advice=fire_advice
        )
        
        # 发布轨迹
        self._publish_trajectory(U_sequence)
        
        # 发布可视化标记
        if self.planner_config.debug and self.markers_pub:
            self._publish_markers(target_info, target_yaw_trajectory)
    
    def _compute_fire_advice(self, 
                             yaw_error: float,
                             pitch_error: float,
                             distance: float,
                             confidence: float) -> bool:
        """计算开火建议"""
        self.get_logger().debug(f"Compute fire advice: yaw_error={yaw_error:.3f}, pitch_error={pitch_error:.3f}, "
                               f"distance={distance:.2f}, confidence={confidence:.3f}")
        
        # 检查yaw误差
        if abs(yaw_error) > self.planner_config.fire_yaw_threshold:
            self.get_logger().debug(f"Fire advice: yaw error {abs(yaw_error):.3f} > threshold {self.planner_config.fire_yaw_threshold:.3f}")
            return False
        
        # 检查pitch误差
        if abs(pitch_error) > self.planner_config.fire_pitch_threshold:
            self.get_logger().debug(f"Fire advice: pitch error {abs(pitch_error):.3f} > threshold {self.planner_config.fire_pitch_threshold:.3f}")
            return False
        
        # 检查距离
        if distance < self.planner_config.fire_distance_min:
            self.get_logger().debug(f"Fire advice: distance {distance:.2f} < min {self.planner_config.fire_distance_min:.2f}")
            return False
        if distance > self.planner_config.fire_distance_max:
            self.get_logger().debug(f"Fire advice: distance {distance:.2f} > max {self.planner_config.fire_distance_max:.2f}")
            return False
        
        # 检查置信度
        if confidence < self.planner_config.fire_confidence_threshold:
            self.get_logger().debug(f"Fire advice: confidence {confidence:.3f} < threshold {self.planner_config.fire_confidence_threshold:.3f}")
            return False
        
        self.get_logger().debug("Fire advice: all conditions met, can fire")
        return True
    
    def _publish_gimbal_cmd(self, 
                            pitch: float,
                            yaw: float,
                            yaw_diff: float,
                            pitch_diff: float,
                            distance: float,
                            fire_advice: bool):
        """发布云台控制指令"""
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"  # 设置坐标系
        
        # 检查NaN值并替换为安全值
        msg.pitch = pitch if not math.isnan(pitch) else self._current_pitch
        msg.yaw = yaw if not math.isnan(yaw) else self._current_gimbal_state[0]
        msg.yaw_diff = yaw_diff if not math.isnan(yaw_diff) else 0.0
        msg.pitch_diff = pitch_diff if not math.isnan(pitch_diff) else 0.0
        msg.distance = distance if not math.isnan(distance) else -1.0
        msg.fire_advice = fire_advice
        
        self.get_logger().debug(f"Publishing gimbal cmd: pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}, "
                               f"yaw_diff={msg.yaw_diff:.3f}, pitch_diff={msg.pitch_diff:.3f}, "
                               f"distance={msg.distance:.2f}, fire_advice={fire_advice}")
        
        self.gimbal_cmd_pub.publish(msg)
    
    def _publish_idle_cmd(self):
        """发布空闲状态控制指令"""
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"  # 设置坐标系
        msg.pitch = self._current_pitch
        msg.yaw = self._current_gimbal_state[0]
        msg.yaw_diff = 0.0
        msg.pitch_diff = 0.0
        msg.distance = -1.0  # 无效距离
        msg.fire_advice = False
        
        self.get_logger().debug(f"Publishing idle cmd: pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}")
        self.gimbal_cmd_pub.publish(msg)
    
    def _publish_trajectory(self, control_sequence: np.ndarray):
        """发布轨迹"""
        msg = GimbalTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.dt = self.mpc_controller.config.dt
        
        # 预测轨迹
        trajectory = self.mpc_controller.predict_state(
            self._current_gimbal_state,
            control_sequence
        )
        
        for state in trajectory:
            gimbal_state = GimbalState()
            gimbal_state.yaw = state[0]
            gimbal_state.yaw_velocity = state[1]
            gimbal_state.yaw_acceleration = state[2]
            msg.trajectory.append(gimbal_state)
        
        msg.total_time = len(control_sequence) * msg.dt
        
        self.trajectory_pub.publish(msg)
    
    def _publish_markers(self, target_info: Optional[dict], target_trajectory: np.ndarray):
        """
        发布可视化标记
        
        包括:
        1. 目标位置球体标记
        2. 目标速度箭头标记
        3. 预测yaw轨迹点标记
        4. 弹道轨迹线标记（从云台到目标）
        5. 命中预测点标记
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0
        
        # 目标位置标记
        if target_info:
            # 1. 目标位置球体（红色）
            target_marker = Marker()
            target_marker.header.frame_id = "odom"
            target_marker.header.stamp = now
            target_marker.ns = "trajectory_planner/target"
            target_marker.id = marker_id
            marker_id += 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            pos = target_info['position']
            target_marker.pose.position.x = pos[0]
            target_marker.pose.position.y = pos[1]
            target_marker.pose.position.z = pos[2]
            target_marker.pose.orientation.w = 1.0
            
            target_marker.scale.x = 0.1
            target_marker.scale.y = 0.1
            target_marker.scale.z = 0.1
            
            target_marker.color.r = 1.0
            target_marker.color.g = 0.0
            target_marker.color.b = 0.0
            target_marker.color.a = 0.8
            
            marker_array.markers.append(target_marker)
            
            # 2. 目标速度箭头（黄色）
            vel = target_info.get('velocity', [0, 0, 0])
            if np.linalg.norm(vel) > 0.01:
                velocity_marker = Marker()
                velocity_marker.header.frame_id = "odom"
                velocity_marker.header.stamp = now
                velocity_marker.ns = "trajectory_planner/velocity"
                velocity_marker.id = marker_id
                marker_id += 1
                velocity_marker.type = Marker.ARROW
                velocity_marker.action = Marker.ADD
                
                # 箭头起点和终点
                start_point = Point()
                start_point.x = pos[0]
                start_point.y = pos[1]
                start_point.z = pos[2]
                
                end_point = Point()
                end_point.x = pos[0] + vel[0]
                end_point.y = pos[1] + vel[1]
                end_point.z = pos[2] + vel[2]
                
                velocity_marker.points = [start_point, end_point]
                
                velocity_marker.scale.x = 0.03  # 箭杆直径
                velocity_marker.scale.y = 0.05  # 箭头直径
                velocity_marker.scale.z = 0.0
                
                velocity_marker.color.r = 1.0
                velocity_marker.color.g = 1.0
                velocity_marker.color.b = 0.0
                velocity_marker.color.a = 0.8
                
                marker_array.markers.append(velocity_marker)
            
            # 3. 弹道轨迹线（粉色/绿色，根据开火状态）
            yaw = self._current_gimbal_state[0]
            pitch = self._current_pitch
            distance = target_info.get('distance', 5.0)
            
            # 计算弹道轨迹
            trajectory_3d = self.ballistic_client.calculate_trajectory_3d(
                target_position=tuple(pos),
                pitch=pitch,
                yaw=yaw,
                bullet_speed=self.planner_config.bullet_speed,
                sample_interval=0.03
            )
            
            if trajectory_3d:
                trajectory_marker = Marker()
                # Use gimbal_link to match armor_solver trajectory visualization
                trajectory_marker.header.frame_id = "gimbal_link"
                trajectory_marker.header.stamp = now
                trajectory_marker.ns = "trajectory_planner/ballistic"
                trajectory_marker.id = marker_id
                marker_id += 1
                trajectory_marker.type = Marker.LINE_STRIP
                trajectory_marker.action = Marker.ADD
                
                for point in trajectory_3d:
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = point[2]
                    trajectory_marker.points.append(p)
                
                trajectory_marker.scale.x = 0.01  # 线宽
                
                # 根据开火建议设置颜色
                fire_advice = self._compute_fire_advice(
                    yaw_error=abs(GimbalModel.angle_difference(
                        target_info.get('yaw_from_origin', yaw), yaw)),
                    pitch_error=0.0,
                    distance=distance,
                    confidence=target_info.get('confidence', 0.0)
                )
                
                if fire_advice:
                    # 可开火：绿色
                    trajectory_marker.color.r = 0.0
                    trajectory_marker.color.g = 1.0
                    trajectory_marker.color.b = 0.0
                else:
                    # 未锁定：粉色
                    trajectory_marker.color.r = 1.0
                    trajectory_marker.color.g = 0.75
                    trajectory_marker.color.b = 0.79
                trajectory_marker.color.a = 1.0
                
                marker_array.markers.append(trajectory_marker)
            
            # 4. 预测点命中位置（青色球体）
            if trajectory_3d:
                hit_marker = Marker()
                # Hit point is in gimbal frame (match ballistic trajectory)
                hit_marker.header.frame_id = "gimbal_link"
                hit_marker.header.stamp = now
                hit_marker.ns = "trajectory_planner/hit_point"
                hit_marker.id = marker_id
                marker_id += 1
                hit_marker.type = Marker.SPHERE
                hit_marker.action = Marker.ADD
                
                # 使用弹道终点作为命中预测点
                last_point = trajectory_3d[-1]
                hit_marker.pose.position.x = last_point[0]
                hit_marker.pose.position.y = last_point[1]
                hit_marker.pose.position.z = last_point[2]
                hit_marker.pose.orientation.w = 1.0
                
                hit_marker.scale.x = 0.08
                hit_marker.scale.y = 0.08
                hit_marker.scale.z = 0.08
                
                hit_marker.color.r = 0.0
                hit_marker.color.g = 1.0
                hit_marker.color.b = 1.0
                hit_marker.color.a = 0.8
                
                marker_array.markers.append(hit_marker)
        
        # 5. 预测yaw轨迹点（渐变颜色点序列）
        if len(target_trajectory) > 0:
            yaw_trajectory_marker = Marker()
            # Yaw trajectory points are easier to interpret in gimbal frame
            yaw_trajectory_marker.header.frame_id = "gimbal_link"
            yaw_trajectory_marker.header.stamp = now
            yaw_trajectory_marker.ns = "trajectory_planner/yaw_trajectory"
            yaw_trajectory_marker.id = marker_id
            marker_id += 1
            yaw_trajectory_marker.type = Marker.POINTS
            yaw_trajectory_marker.action = Marker.ADD
            
            yaw_trajectory_marker.scale.x = 0.03
            yaw_trajectory_marker.scale.y = 0.03
            
            # 使用预测的yaw轨迹创建可视化点
            # 假设目标距离固定，仅展示yaw方向变化
            target_distance = target_info.get('distance', 3.0) if target_info else 3.0
            target_height = target_info['position'][2] if target_info else 0.3
            
            for i, yaw_angle in enumerate(target_trajectory):
                p = Point()
                p.x = target_distance * math.cos(yaw_angle)
                p.y = target_distance * math.sin(yaw_angle)
                p.z = target_height
                yaw_trajectory_marker.points.append(p)
                
                # 渐变颜色：蓝色 -> 青色
                color = ColorRGBA()
                ratio = i / max(len(target_trajectory) - 1, 1)
                color.r = 0.0
                color.g = ratio
                color.b = 1.0
                color.a = 1.0 - ratio * 0.5  # 逐渐透明
                yaw_trajectory_marker.colors.append(color)
            
            marker_array.markers.append(yaw_trajectory_marker)
        
        # 6. 清除旧的标记（通过发布 DELETE_ALL 然后再发布新标记）
        # 为了避免残留标记，添加一个清除标记
        delete_marker = Marker()
        # Clear markers in gimbal frame to avoid leftover artifacts
        delete_marker.header.frame_id = "gimbal_link"
        delete_marker.header.stamp = now
        delete_marker.ns = "trajectory_planner/cleanup"
        delete_marker.action = Marker.DELETEALL
        
        # 先发布清除标记，再发布新标记
        cleanup_array = MarkerArray()
        cleanup_array.markers.append(delete_marker)
        self.markers_pub.publish(cleanup_array)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryPlannerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
