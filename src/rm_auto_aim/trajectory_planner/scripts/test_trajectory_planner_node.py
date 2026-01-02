#!/usr/bin/env python3
"""
Trajectory Planner ROS2 测试节点

功能:
- 模拟装甲板运动（4个装甲板，整体匀加速 + 相对旋转）
- 发布 TrackPredictionWindows 话题（模拟armor_tracker输出）
- 发布 JointState 话题（模拟云台状态反馈）
- 发布 TrackedRobots 话题（可选，模拟目标机器人信息）
- 订阅 GimbalCmd 话题（验证trajectory_planner输出）
- 提供实时可视化和性能统计

参考: trajectory_planner/demo/test05_qp_mpc.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time
from collections import deque

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time as RosTime

from rm_interfaces.msg import (
    TrackPredictionWindows,
    TrackPredictionWindow,
    TrackWindowState,
    TrackedRobots,
    TrackedRobot,
    GimbalCmd
)
from rm_interfaces.srv import SetTargetRobot


def wrap_angle(angle):
    """归一化角度到 [-π, π]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class TrajectoryPlannerTester(Node):
    """Trajectory Planner 测试节点"""
    
    def __init__(self):
        super().__init__('trajectory_planner_tester')
        
        # 声明参数
        self.declare_parameter('dt', 0.02)  # 发布频率 50Hz
        self.declare_parameter('num_plates', 4)  # 装甲板数量
        self.declare_parameter('base_omega', 2.0)  # 基础旋转角速度 rad/s
        self.declare_parameter('center_alpha', 0.1)  # 中心角加速度 rad/s²
        self.declare_parameter('prediction_steps', 18)  # 预测步数
        self.declare_parameter('measurement_noise_std', 0.035)  # 观测噪声 (2度)
        self.declare_parameter('enable_robots_pub', True)  # 是否发布robots话题
        self.declare_parameter('prediction_topic', '/armor_tracker/prediction_windows')  # 发布预测窗口的话题（可重写）
        self.declare_parameter('target_robot_id', '1')  # 自动设置的目标机器人ID
        self.declare_parameter('set_target_delay', 2.0)  # 启动后多少秒自动设置目标（秒）
        
        # 获取参数
        self.dt = self.get_parameter('dt').value
        self.num_plates = self.get_parameter('num_plates').value
        self.base_omega = self.get_parameter('base_omega').value
        self.center_alpha = self.get_parameter('center_alpha').value
        self.prediction_steps = self.get_parameter('prediction_steps').value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').value
        self.enable_robots_pub = self.get_parameter('enable_robots_pub').value
        self.prediction_topic = self.get_parameter('prediction_topic').value
        self.target_robot_id = self.get_parameter('target_robot_id').value
        self.set_target_delay = self.get_parameter('set_target_delay').value
        
        # 装甲板状态初始化
        self.plate_base_angles = np.array([0.0, np.pi/2, np.pi, 3*np.pi/2])[:self.num_plates]
        self.plate_omegas = np.ones(self.num_plates) * self.base_omega
        self.plate_angles = self.plate_base_angles.copy()
        
        # 中心运动状态
        self.center_angle = 0.0
        self.center_omega = 0.5  # rad/s
        self.center_alpha_val = self.center_alpha
        
        # 云台模拟状态（用于joint_states发布）
        self.gimbal_yaw = 0.0
        self.gimbal_pitch = 0.0
        
        # 性能统计
        self.cmd_count = 0
        self.last_cmd_time = None
        self.cmd_intervals = deque(maxlen=100)
        self.tracking_errors = deque(maxlen=100)
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布者
        self.prediction_windows_pub = self.create_publisher(
            TrackPredictionWindows,
            self.prediction_topic,
            10
        )
        
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        if self.enable_robots_pub:
            self.robots_pub = self.create_publisher(
                TrackedRobots,
                '/robot_pose_estimator/robots',  # 匹配 trajectory_planner 订阅的话题
                10
            )
        
        # 订阅者 - 监听trajectory_planner输出
        self.gimbal_cmd_sub = self.create_subscription(
            GimbalCmd,
            '/trajectory_planner/gimbal_cmd',
            self._gimbal_cmd_callback,
            10
        )
        
        # Service客户端 - 设置目标机器人
        self.set_target_client = self.create_client(
            SetTargetRobot,
            '/trajectory_planner/set_target_robot'
        )
        
        # 定时器 - 发布模拟数据
        self.timer = self.create_timer(self.dt, self._timer_callback)
        
        # 统计定时器 - 每5秒打印一次
        self.stats_timer = self.create_timer(5.0, self._print_stats)
        
        # 延迟设置目标的单次定时器
        if self.set_target_delay > 0.0 and self.target_robot_id:
            self.set_target_timer = self.create_timer(
                self.set_target_delay, 
                self._call_set_target_service
            )
        
        self.iteration = 0
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Trajectory Planner Tester 已启动")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"配置参数:")
        self.get_logger().info(f"  发布频率: {1.0/self.dt:.0f} Hz (dt={self.dt:.3f}s)")
        self.get_logger().info(f"  装甲板数量: {self.num_plates}")
        self.get_logger().info(f"  旋转角速度: {self.base_omega:.2f} rad/s ({np.rad2deg(self.base_omega):.1f}°/s)")
        self.get_logger().info(f"  中心角加速度: {self.center_alpha:.3f} rad/s² ({np.rad2deg(self.center_alpha):.2f}°/s²)")
        self.get_logger().info(f"  预测步数: {self.prediction_steps}")
        self.get_logger().info(f"  观测噪声: {np.rad2deg(self.measurement_noise_std):.2f}°")
        self.get_logger().info(f"  发布机器人信息: {'是' if self.enable_robots_pub else '否'}")
        self.get_logger().info("=" * 70)
        self.get_logger().info("等待 trajectory_planner 节点连接...")
    
    def _timer_callback(self):
        """定时器回调 - 发布模拟数据"""
        current_time = self.get_clock().now()
        
        # 更新装甲板运动状态
        self._update_plate_states()
        
        # 发布预测窗口
        self._publish_prediction_windows(current_time)
        
        # 发布云台状态
        self._publish_joint_states(current_time)
        
        # 发布机器人信息（可选）
        if self.enable_robots_pub:
            self._publish_robots(current_time)
        
        self.iteration += 1
    
    def _update_plate_states(self):
        """更新装甲板运动状态"""
        # 更新中心运动（匀加速）
        self.center_omega += self.center_alpha_val * self.dt
        self.center_angle = wrap_angle(self.center_angle + self.center_omega * self.dt)
        
        # 更新相对旋转（加入小扰动模拟真实情况）
        noise = 0.01 * np.random.randn(self.num_plates) * 0.3
        self.plate_omegas += noise
        self.plate_angles = wrap_angle(self.plate_angles + self.plate_omegas * self.dt)
    
    def _publish_prediction_windows(self, current_time):
        """发布预测窗口消息"""
        msg = TrackPredictionWindows()
        msg.header = Header()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        
        # 为每个装甲板创建预测窗口
        for i in range(self.num_plates):
            window = self._create_prediction_window(i, current_time)
            msg.windows.append(window)
        
        self.prediction_windows_pub.publish(msg)
    
    def _create_prediction_window(self, plate_idx, current_time):
        """为指定装甲板创建预测窗口"""
        window = TrackPredictionWindow()
        window.header = Header()
        window.header.stamp = current_time.to_msg()
        window.header.frame_id = 'camera_optical_frame'
        
        window.track_id = plate_idx + 1  # track_id从1开始
        window.armor_id = str((plate_idx % 4) + 1)  # "1", "2", "3", "4"
        window.armor_type = 'small' if plate_idx < 2 else 'large'
        
        window.prediction_steps = self.prediction_steps
        window.predict_interval = 1
        window.current_iteration = self.iteration
        
        # 生成预测状态序列（包含当前时刻）
        for k in range(self.prediction_steps + 1):
            state = self._predict_plate_state(plate_idx, k)
            window.predictions.append(state)
        
        return window
    
    def _predict_plate_state(self, plate_idx, k_step):
        """预测装甲板在k步后的状态"""
        state = TrackWindowState()
        
        # 时间戳
        future_time_ns = (self.iteration + k_step) * int(self.dt * 1e9)
        state.timestamp = RosTime(sec=int(future_time_ns // 1e9), 
                                  nanosec=int(future_time_ns % 1e9))
        state.iteration = self.iteration + k_step
        
        # 预测相对角度
        relative_angle_pred = self.plate_angles[plate_idx] + self.plate_omegas[plate_idx] * k_step * self.dt
        
        # 预测中心角度（匀加速）
        center_omega_pred = self.center_omega + self.center_alpha_val * k_step * self.dt
        center_angle_pred = self.center_angle + self.center_omega * k_step * self.dt + \
                           0.5 * self.center_alpha_val * (k_step * self.dt) ** 2
        
        # 绝对角度（加入观测噪声）
        absolute_angle = wrap_angle(relative_angle_pred + center_angle_pred)
        if k_step == 0:  # 当前时刻加噪声
            absolute_angle += np.random.randn() * self.measurement_noise_std
        
        # 计算3D位置（假设距离2.5m，装甲板在水平面上）
        distance = 2.5
        state.position = Point(
            x=distance * math.cos(absolute_angle),
            y=distance * math.sin(absolute_angle),
            z=-0.1
        )
        
        # 计算速度（数值微分）
        omega_absolute = self.plate_omegas[plate_idx] + center_omega_pred
        state.velocity = Vector3(
            x=-distance * omega_absolute * math.sin(absolute_angle),
            y=distance * omega_absolute * math.cos(absolute_angle),
            z=0.0
        )
        
        # yaw角（装甲板自身姿态，这里简化为绝对角度+π/2）
        state.yaw = wrap_angle(absolute_angle + np.pi/2)
        state.yaw_velocity = omega_absolute
        
        # 置信度（根据距离衰减）
        state.confidence = 0.95 * math.exp(-distance / 10.0)
        
        # 来源类型
        if k_step == 0:
            state.source_type = 0  # DETECT
        elif k_step <= 3:
            state.source_type = 1  # ESTIMATE
        else:
            state.source_type = 2  # PREDICT
        
        state.tracking_state = 2  # TRACKING
        
        return state
    
    def _publish_joint_states(self, current_time):
        """发布云台状态"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'gimbal_base'
        
        # 云台yaw和pitch（这里模拟云台尝试跟踪装甲板中心）
        # 实际使用中，应该从trajectory_planner的输出反馈回来
        target_yaw = self.center_angle
        self.gimbal_yaw = 0.9 * self.gimbal_yaw + 0.1 * target_yaw  # 简单的低通滤波
        
        msg.name = ['yaw_joint', 'pitch_joint']
        msg.position = [float(self.gimbal_yaw), float(self.gimbal_pitch)]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        
        self.joint_states_pub.publish(msg)
    
    def _publish_robots(self, current_time):
        """发布机器人信息"""
        msg = TrackedRobots()
        msg.header = Header()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        
        # 创建一个4装甲板机器人 (ID与 target_robot_id 参数一致)
        robot = TrackedRobot()
        robot.header = msg.header
        robot.robot_id = self.target_robot_id  # 使用与 SetTargetRobot 相同的 ID
        robot.robot_type = 1  # STANDARD_4
        robot.confidence = 0.95  # 高置信度，确保通过过滤
        robot.num_armors = self.num_plates
        
        # 绑定装甲板 track_id (字符串列表)
        robot.bound_armor_ids = [str(i+1) for i in range(self.num_plates)]
        
        # 机器人中心位置（基于中心角度）
        distance = 2.5
        robot.center_position = Point(
            x=distance * math.cos(self.center_angle),
            y=distance * math.sin(self.center_angle),
            z=-0.1
        )
        
        robot.center_velocity = Vector3(
            x=-distance * self.center_omega * math.sin(self.center_angle),
            y=distance * self.center_omega * math.cos(self.center_angle),
            z=0.0
        )
        
        robot.yaw = self.center_angle
        robot.yaw_velocity = self.center_omega
        robot.radius = 0.2  # 旋转半径
        
        msg.robots.append(robot)
        self.robots_pub.publish(msg)
    
    def _gimbal_cmd_callback(self, msg: GimbalCmd):
        """接收trajectory_planner的控制指令"""
        current_time = self.get_clock().now()
        
        # 统计
        self.cmd_count += 1
        if self.last_cmd_time is not None:
            interval = (current_time - self.last_cmd_time).nanoseconds / 1e9
            self.cmd_intervals.append(interval)
        self.last_cmd_time = current_time
        
        # 计算跟踪误差（云台指令yaw vs 装甲板中心）
        # 注意：这里假设yaw_diff字段包含了误差信息
        if hasattr(msg, 'yaw_diff'):
            self.tracking_errors.append(abs(msg.yaw_diff))
        
        # 每100条指令打印一次
        if self.cmd_count % 100 == 0:
            self.get_logger().info(
                f"已收到 {self.cmd_count} 条控制指令 | "
                f"最新: yaw={np.rad2deg(msg.yaw):.2f}° pitch={np.rad2deg(msg.pitch):.2f}°"
            )
    
    def _print_stats(self):
        """打印统计信息"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"运行时间: {elapsed:.1f}s | 迭代: {self.iteration}")
        self.get_logger().info(f"装甲板状态:")
        self.get_logger().info(f"  中心角度: {np.rad2deg(self.center_angle):.2f}° | "
                              f"中心角速度: {np.rad2deg(self.center_omega):.2f}°/s")
        
        if self.cmd_count > 0:
            avg_interval = np.mean(self.cmd_intervals) if self.cmd_intervals else 0.0
            avg_freq = 1.0 / avg_interval if avg_interval > 0 else 0.0
            avg_error = np.rad2deg(np.mean(self.tracking_errors)) if self.tracking_errors else 0.0
            
            self.get_logger().info(f"Trajectory Planner 性能:")
            self.get_logger().info(f"  收到指令数: {self.cmd_count}")
            self.get_logger().info(f"  平均频率: {avg_freq:.1f} Hz")
            if self.tracking_errors:
                self.get_logger().info(f"  平均跟踪误差: {avg_error:.2f}°")
        else:
            self.get_logger().warn("尚未收到 trajectory_planner 的控制指令！")
            self.get_logger().warn("请检查:")
            self.get_logger().warn("  1. trajectory_planner 节点是否运行")
            self.get_logger().warn("  2. 话题名称是否正确")
            self.get_logger().warn("  3. QoS配置是否匹配")
        
        self.get_logger().info("=" * 70)
    
    def _call_set_target_service(self):
        """调用 SetTargetRobot service 设置目标机器人"""
        # 取消定时器（只调用一次）
        if hasattr(self, 'set_target_timer'):
            self.set_target_timer.cancel()
        
        # 等待 service 可用
        if not self.set_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetTargetRobot service 不可用！无法设置目标。")
            return
        
        # 创建请求
        request = SetTargetRobot.Request()
        request.robot_id = self.target_robot_id
        
        self.get_logger().info(f"正在调用 SetTargetRobot service: robot_id='{self.target_robot_id}'...")
        
        # 异步调用
        future = self.set_target_client.call_async(request)
        future.add_done_callback(self._set_target_response_callback)
    
    def _set_target_response_callback(self, future):
        """SetTargetRobot service 响应回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"✓ 目标设置成功: robot_id='{self.target_robot_id}' | "
                    f"消息: {response.message} | 前一个目标: '{response.previous_robot_id}'"
                )
                self.get_logger().info("MPC 审计日志将开始记录到 /tmp/mpc_audit.csv")
            else:
                self.get_logger().error(
                    f"✗ 目标设置失败: {response.message}"
                )
        except Exception as e:
            self.get_logger().error(f"SetTargetRobot service 调用异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrajectoryPlannerTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("\n正在关闭测试节点...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
