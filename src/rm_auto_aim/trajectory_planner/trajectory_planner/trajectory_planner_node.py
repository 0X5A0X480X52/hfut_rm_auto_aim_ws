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

from std_msgs.msg import Header
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
        
        # 回调组
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # 创建弹道解算客户端 (需要在订阅之前)
        self.ballistic_client = BallisticClient(self, ballistic_config)
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅者
        self.prediction_windows_sub = self.create_subscription(
            TrackPredictionWindows,
            self.topic_config.prediction_windows_sub,
            self._prediction_windows_callback,
            sensor_qos
        )
        
        self.robots_sub = self.create_subscription(
            TrackedRobots,
            self.topic_config.robots_sub,
            self._robots_callback,
            sensor_qos
        )
        
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.topic_config.joint_states_sub,
            self._joint_states_callback,
            sensor_qos
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
        self.declare_parameter('predictor.max_distance', 10.0)
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
        self.target_predictor.update_predictions(msg)
    
    def _robots_callback(self, msg: TrackedRobots):
        """机器人信息回调"""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.target_manager.update_robots(msg, current_time)
    
    def _joint_states_callback(self, msg: JointState):
        """关节状态回调 - 获取当前云台状态"""
        # 查找yaw关节
        try:
            yaw_idx = msg.name.index('yaw_joint')
            self._current_gimbal_state[0] = msg.position[yaw_idx]
            if len(msg.velocity) > yaw_idx:
                self._current_gimbal_state[1] = msg.velocity[yaw_idx]
        except ValueError:
            pass
        
        # 查找pitch关节
        try:
            pitch_idx = msg.name.index('pitch_joint')
            self._current_pitch = msg.position[pitch_idx]
        except ValueError:
            pass
    
    # ==================== 服务处理 ====================
    
    def _handle_set_target_robot(self, 
                                  request: SetTargetRobot.Request,
                                  response: SetTargetRobot.Response) -> SetTargetRobot.Response:
        """处理设置目标机器人服务请求"""
        success, message, previous_id = self.target_manager.set_target_robot(request.robot_id)
        
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
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Action取消回调"""
        self.get_logger().info("Received cancel request for trajectory plan")
        return CancelResponse.ACCEPT
    
    async def _execute_trajectory_plan(self, goal_handle: ServerGoalHandle):
        """执行轨迹规划Action"""
        request = goal_handle.request
        
        self.get_logger().info(f"Executing trajectory plan for robot: {request.robot_id}")
        
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
        
        while rclpy.ok() and self._is_tracking_enabled:
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Action canceled"
                result.total_tracking_time = self._total_tracking_time
                self._planning_status = self.STATUS_IDLE
                return result
            
            # 获取目标信息
            target_info = self.target_predictor.get_target_info()
            
            if target_info:
                self._planning_status = self.STATUS_TRACKING
                self._last_target_time = self.get_clock().now().nanoseconds * 1e-9
                
                # 计算误差
                yaw_error = GimbalModel.angle_difference(
                    target_info['yaw_from_origin'],
                    self._current_gimbal_state[0]
                )
                
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
                
                goal_handle.publish_feedback(feedback)
            else:
                # 检查目标丢失超时
                current_time = self.get_clock().now().nanoseconds * 1e-9
                if self._last_target_time is not None:
                    if current_time - self._last_target_time > self.planner_config.target_lost_timeout:
                        self._planning_status = self.STATUS_LOST_TARGET
                        
                        feedback.header.stamp = self.get_clock().now().to_msg()
                        feedback.planning_status = self.STATUS_LOST_TARGET
                        feedback.target_locked = False
                        goal_handle.publish_feedback(feedback)
            
            # 更新跟踪时间
            self._total_tracking_time = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            
            rate.sleep()
        
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
        if self._planning_status == self.STATUS_IDLE:
            return
        
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # 检查目标是否有效
        if not self.target_manager.is_target_valid(current_time):
            self._publish_idle_cmd()
            return
        
        # 获取目标装甲板ID列表
        target_armor_ids = self.target_manager.get_target_armor_ids()
        
        # 设置目标预测器的过滤条件
        self.target_predictor.set_target_armor_ids(target_armor_ids)
        
        # 获取目标装甲板track_ids
        target_track_ids = []
        for armor_id in target_armor_ids:
            track_ids = self.target_predictor.filter_by_robot_id(armor_id)
            target_track_ids.extend(track_ids)
        
        # 选择最佳装甲板
        current_yaw = self._current_gimbal_state[0]
        selected_track_id = self.target_predictor.select_best_armor(
            reference_yaw=current_yaw,
            target_track_ids=target_track_ids if target_track_ids else None
        )
        
        if selected_track_id is None:
            self._publish_idle_cmd()
            self._planning_status = self.STATUS_LOST_TARGET
            return
        
        self._planning_status = self.STATUS_TRACKING
        self._last_target_time = current_time
        
        # 获取目标yaw轨迹
        target_yaw_trajectory = self.target_predictor.get_target_yaw_trajectory(selected_track_id)
        
        if target_yaw_trajectory is None:
            self._publish_idle_cmd()
            return
        
        # MPC优化求解
        u_optimal, U_sequence, solve_time = self.mpc_controller.solve(
            self._current_gimbal_state,
            target_yaw_trajectory
        )
        
        # 更新云台状态 (仿真/预测)
        next_state = self.gimbal_model.predict(self._current_gimbal_state, u_optimal)
        
        # 计算pitch
        target_info = self.target_predictor.get_target_info(selected_track_id)
        pitch = 0.0
        distance = 0.0
        
        if target_info:
            distance = target_info['distance']
            target_position = target_info['position']
            target_velocity = target_info['velocity']
            
            # 使用弹道解算服务计算pitch
            if self.ballistic_client.is_service_available():
                # 在优化后的yaw方向上寻找可见装甲板并计算pitch
                ballistic_result = self.ballistic_client.compute_pitch_from_yaw(
                    yaw=next_state[0],
                    target_position=tuple(target_position),
                    target_velocity=tuple(target_velocity),
                    bullet_speed=self.planner_config.bullet_speed
                )
                if ballistic_result.success:
                    pitch = ballistic_result.pitch
                else:
                    # 使用简单估算
                    pitch = self.ballistic_client.simple_pitch_estimate(
                        distance=distance,
                        height=target_position[2],
                        bullet_speed=self.planner_config.bullet_speed
                    )
            else:
                # 使用简单估算
                pitch = self.ballistic_client.simple_pitch_estimate(
                    distance=distance,
                    height=target_position[2],
                    bullet_speed=self.planner_config.bullet_speed
                )
        
        # 计算yaw误差
        yaw_error = GimbalModel.angle_difference(
            target_yaw_trajectory[0],
            self._current_gimbal_state[0]
        )
        
        # 判断是否可以开火
        fire_advice = self._compute_fire_advice(
            yaw_error=yaw_error,
            pitch_error=pitch - self._current_pitch,
            distance=distance,
            confidence=target_info['confidence'] if target_info else 0.0
        )
        
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
        # 检查yaw误差
        if abs(yaw_error) > self.planner_config.fire_yaw_threshold:
            return False
        
        # 检查pitch误差
        if abs(pitch_error) > self.planner_config.fire_pitch_threshold:
            return False
        
        # 检查距离
        if distance < self.planner_config.fire_distance_min:
            return False
        if distance > self.planner_config.fire_distance_max:
            return False
        
        # 检查置信度
        if confidence < self.planner_config.fire_confidence_threshold:
            return False
        
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
        msg.pitch = pitch
        msg.yaw = yaw
        msg.yaw_diff = yaw_diff
        msg.pitch_diff = pitch_diff
        msg.distance = distance
        msg.fire_advice = fire_advice
        
        self.gimbal_cmd_pub.publish(msg)
    
    def _publish_idle_cmd(self):
        """发布空闲状态控制指令"""
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pitch = self._current_pitch
        msg.yaw = self._current_gimbal_state[0]
        msg.yaw_diff = 0.0
        msg.pitch_diff = 0.0
        msg.distance = 0.0
        msg.fire_advice = False
        
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
        """发布可视化标记"""
        marker_array = MarkerArray()
        
        # 目标位置标记
        if target_info:
            target_marker = Marker()
            target_marker.header.frame_id = "odom"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = "trajectory_planner"
            target_marker.id = 0
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
