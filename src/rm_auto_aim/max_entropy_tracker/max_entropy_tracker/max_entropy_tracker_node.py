#!/usr/bin/env python3
"""
Max Entropy Tracker ROS2 节点

订阅装甲板检测结果，使用 Max Entropy UKF 进行机器人位姿估计，
发布跟踪结果和可视化 Marker
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time
from collections import defaultdict
from typing import Dict, List, Optional

# ROS2 消息类型
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import MarkerArray
from rm_interfaces.msg import Armors, Armor, Target

# 本地模块
from .core.config import UnifiedConfig, TranslationModel
from .core.observation import ObservationData
from .msg_converter import armor_to_observation, quaternion_to_yaw, pose_to_observation
from .tf_handler import TFHandler
from .tracker_manager import TrackerManager
from .visualization import build_tracker_markers
from .csv_logger import CSVLogger


class MaxEntropyTrackerNode(Node):
    """
    Max Entropy Tracker ROS2 节点
    
    功能:
    1. 订阅 /armor_detector/armors 获取装甲板检测结果
    2. 对每个机器人维护独立的 AdaptiveArmorTracker
    3. 定时执行 predict，收到观测时执行 update
    4. 发布 Target 消息和可视化 Marker
    5. 可选：记录观测和估计结果到 CSV 文件
    """
    
    def __init__(self):
        super().__init__('max_entropy_tracker_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 获取参数
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.predict_rate = self.get_parameter('predict_rate').value
        self.default_r1 = self.get_parameter('default_r1').value
        self.default_r2 = self.get_parameter('default_r2').value
        self.default_dza = self.get_parameter('default_dza').value
        self.tracker_timeout = self.get_parameter('tracker_timeout').value
        self.enable_csv_logging = self.get_parameter('enable_csv_logging').value
        self.csv_log_path = self.get_parameter('csv_log_path').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.visualization_frame = self.get_parameter('visualization_frame').value
        
        # 创建配置并应用参数
        self.config = UnifiedConfig.create_default()
        self._apply_parameters_to_config()

        # 创建 TF 处理器（直接使用 odom 世界坐标系）
        self.tf_handler = TFHandler(self, self.target_frame)
        
        # 创建跟踪管理器
        dt = 1.0 / self.predict_rate if self.predict_rate > 0 else 0.01
        self.tracker_manager = TrackerManager(
            config=self.config,
            dt=dt,
            default_r1=self.default_r1,
            default_r2=self.default_r2,
            default_dza=self.default_dza,
            timeout_seconds=self.tracker_timeout,
            enable_oscillation_detection=self.get_parameter('enable_oscillation_detection').value
        )
        
        # 创建 CSV 日志记录器
        self.csv_logger: Optional[CSVLogger] = None
        if self.enable_csv_logging:
            self.csv_logger = CSVLogger(self.csv_log_path)
            self.get_logger().info(f"CSV logging enabled: {self.csv_log_path}")
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅装甲板检测结果
        self.armors_sub = self.create_subscription(
            Armors,
            '/armor_detector/armors',
            self.armors_callback,
            sensor_qos
        )
        
        # 发布 Target 消息
        self.target_pub = self.create_publisher(
            Target,
            '/max_entropy_tracker/target',
            10
        )
        
        # 发布可视化 Marker
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/max_entropy_tracker/markers',
            10
        )
        
        # Predict is performed only when observations arrive (triggered updates).
        # Do not create periodic predict timer to avoid publishing stale/erroneous targets.
        self.predict_timer = None
        
        # 打印所有参数以便排查配置文件加载情况
        self._log_all_parameters()

        self.get_logger().info(
            f"MaxEntropyTrackerNode initialized: "
            f"target_frame={self.target_frame}, "
            f"update_mode=triggered_on_observation"
        )

    def _log_all_parameters(self):
        """打印所有已声明参数的名称和值"""
        try:
            self.get_logger().info("Node parameters (declared):")
            for name in getattr(self, '_declared_parameters', []):
                try:
                    val = self.get_parameter(name).value
                except Exception:
                    val = '<not set>'
                self.get_logger().info(f"  {name}: {val}")
        except Exception as e:
            self.get_logger().error(f"Failed to log parameters: {e}")
    
    def _declare_parameters(self):
        """声明节点参数并记录已声明参数名以便后续打印"""
        self._declared_parameters = []

        def d(name, default):
            self.declare_parameter(name, default)
            self._declared_parameters.append(name)

        # 基本参数
        d('target_frame', 'odom')
        d('source_frame', 'camera_optical_frame')
        d('predict_rate', 100.0)  # Hz
        d('default_r1', 0.15)
        d('default_r2', 0.20)
        d('default_dza', 0.0)
        d('tracker_timeout', 3.0)  # 秒
        d('enable_csv_logging', False)
        d('csv_log_path', '/tmp/max_entropy_tracker_log.csv')
        d('debug_mode', False)
        d('enable_oscillation_detection', False)
        
        # 可视化参数
        d('visualization_frame', 'odom')  # 可视化发布的坐标系

        # UKF 参数
        d('ukf.alpha', 0.001)
        d('ukf.beta', 2.0)
        d('ukf.kappa', 0.0)
        d('ukf.obs_noise_pos', 0.05)
        d('ukf.obs_noise_yaw', 0.05)
        d('ukf.dual_obs_noise_pos', 0.01)
        d('ukf.dual_obs_noise_yaw', 0.03)
        d('ukf.dual_obs_geometry_noise_scale', 0.2)
        d('ukf.single_obs_update_weight_pos', 0.05)
        d('ukf.enable_innovation_gating', False)
        d('ukf.innovation_gate_chi2_threshold', 9.49)

        # Motion model 参数
        d('motion.translation_model', 'CA')
        d('motion.cv_process_noise_vel', 0.5)
        d('motion.ca_process_noise_acc', 1.0)
        d('motion.singer_alpha', 0.5)
        d('motion.singer_sigma', 2.0)
        d('motion.process_noise_r', 0.02)
        d('motion.process_noise_dz', 0.005)

        # Spin model 参数
        d('spin.spin_process_noise_yaw_rate', 0.3)
        d('spin.spin_process_noise_yaw_acc', 1.0)
        d('spin.spin_process_noise_delta_rate', 0.3)
        d('spin.spin_process_noise_delta_acc', 3.0)

        # Entropy 参数
        d('entropy.temperature', 2.0)
        d('entropy.use_adaptive', True)
        d('entropy.k_prior_weight', 0.7)

        # Tracker 参数
        d('tracker.tracking_thres', 2)
        d('tracker.lost_thres', 8)
        d('tracker.temp_lost_thres', 3)
        d('tracker.max_match_distance', 2.0)
        d('tracker.max_match_yaw_diff', 1.0)
        d('tracker.n_panels', 4)
        d('tracker.panel_angle_step', 1.5707963267948966)

        # Constraint 参数
        d('constraints.min_radius', 0.12)
        d('constraints.max_radius', 0.5)
        d('constraints.min_dz', -1.0)
        d('constraints.max_dz', 1.0)

    def _apply_parameters_to_config(self):
        """将节点参数应用到 UnifiedConfig 中"""
        try:
            # UKF
            self.config.ukf.alpha = float(self.get_parameter('ukf.alpha').value)
            self.config.ukf.beta = float(self.get_parameter('ukf.beta').value)
            self.config.ukf.kappa = float(self.get_parameter('ukf.kappa').value)
            self.config.ukf.obs_noise_pos = float(self.get_parameter('ukf.obs_noise_pos').value)
            self.config.ukf.obs_noise_yaw = float(self.get_parameter('ukf.obs_noise_yaw').value)
            self.config.ukf.dual_obs_noise_pos = float(self.get_parameter('ukf.dual_obs_noise_pos').value)
            self.config.ukf.dual_obs_noise_yaw = float(self.get_parameter('ukf.dual_obs_noise_yaw').value)
            self.config.ukf.dual_obs_geometry_noise_scale = float(self.get_parameter('ukf.dual_obs_geometry_noise_scale').value)
            self.config.ukf.single_obs_update_weight_pos = float(self.get_parameter('ukf.single_obs_update_weight_pos').value)
            self.config.ukf.enable_innovation_gating = bool(self.get_parameter('ukf.enable_innovation_gating').value)
            self.config.ukf.innovation_gate_chi2_threshold = float(self.get_parameter('ukf.innovation_gate_chi2_threshold').value)

            # Motion
            tm = str(self.get_parameter('motion.translation_model').value)
            try:
                self.config.motion.translation_model = TranslationModel[tm]
            except Exception:
                self.get_logger().warn(f"Unknown translation_model '{tm}', using default")
            self.config.motion.cv_process_noise_vel = float(self.get_parameter('motion.cv_process_noise_vel').value)
            self.config.motion.ca_process_noise_acc = float(self.get_parameter('motion.ca_process_noise_acc').value)
            self.config.motion.singer_alpha = float(self.get_parameter('motion.singer_alpha').value)
            self.config.motion.singer_sigma = float(self.get_parameter('motion.singer_sigma').value)
            self.config.motion.process_noise_r = float(self.get_parameter('motion.process_noise_r').value)
            self.config.motion.process_noise_dz = float(self.get_parameter('motion.process_noise_dz').value)

            # Spin
            self.config.spin.spin_process_noise_yaw_rate = float(self.get_parameter('spin.spin_process_noise_yaw_rate').value)
            self.config.spin.spin_process_noise_yaw_acc = float(self.get_parameter('spin.spin_process_noise_yaw_acc').value)
            self.config.spin.spin_process_noise_delta_rate = float(self.get_parameter('spin.spin_process_noise_delta_rate').value)
            self.config.spin.spin_process_noise_delta_acc = float(self.get_parameter('spin.spin_process_noise_delta_acc').value)

            # Entropy
            self.config.entropy.temperature = float(self.get_parameter('entropy.temperature').value)
            self.config.entropy.use_adaptive = bool(self.get_parameter('entropy.use_adaptive').value)
            self.config.entropy.k_prior_weight = float(self.get_parameter('entropy.k_prior_weight').value)

            # Tracker
            self.config.tracker.tracking_thres = int(self.get_parameter('tracker.tracking_thres').value)
            self.config.tracker.lost_thres = int(self.get_parameter('tracker.lost_thres').value)
            self.config.tracker.temp_lost_thres = int(self.get_parameter('tracker.temp_lost_thres').value)
            self.config.tracker.max_match_distance = float(self.get_parameter('tracker.max_match_distance').value)
            self.config.tracker.max_match_yaw_diff = float(self.get_parameter('tracker.max_match_yaw_diff').value)
            self.config.tracker.n_panels = int(self.get_parameter('tracker.n_panels').value)
            self.config.tracker.panel_angle_step = float(self.get_parameter('tracker.panel_angle_step').value)

            # Constraints
            self.config.constraints.min_radius = float(self.get_parameter('constraints.min_radius').value)
            self.config.constraints.max_radius = float(self.get_parameter('constraints.max_radius').value)
            self.config.constraints.min_dz = float(self.get_parameter('constraints.min_dz').value)
            self.config.constraints.max_dz = float(self.get_parameter('constraints.max_dz').value)

            self.get_logger().info("Parameters applied to UnifiedConfig")
        except Exception as e:
            self.get_logger().error(f"Failed to apply parameters to config: {e}")
    
    def armors_callback(self, msg: Armors):
        """
        装甲板消息回调
        
        Args:
            msg: Armors 消息
        """
        if len(msg.armors) == 0:
            return
        
        # 使用消息时间戳而非系统时间
        msg_time = Time.from_msg(msg.header.stamp)
        current_time = msg_time.nanoseconds / 1e9  # 转换为秒

        # Trigger-based prediction: predict all trackers up to current time before processing new observations
        self.tracker_manager.predict_all(current_time)
        removed = self.tracker_manager.remove_stale_trackers(current_time)
        if removed and self.debug_mode:
            self.get_logger().info(f"Removed stale trackers: {removed}")
        
        # 按机器人ID分组装甲板
        observations_by_robot: Dict[str, List[ObservationData]] = defaultdict(list)
        raw_observations_by_robot: Dict[str, List[ObservationData]] = defaultdict(list)
        
        source_frame = msg.header.frame_id if msg.header.frame_id else self.source_frame
        
        for armor in msg.armors:
            # 记录原始观测数据（相机坐标系）
            raw_obs = armor_to_observation(
                armor,
                timestamp=msg_time.nanoseconds / 1e9
            )
            
            # 变换到 odom 世界坐标系
            obs = self.tf_handler.transform_armor_to_observation(
                armor,
                source_frame,
                msg_time
            )
            
            if obs is None:
                if self.debug_mode:
                    self.get_logger().warn(
                        f"TF transform failed for armor {armor.number}"
                    )
                continue
            
            # 按机器人ID分组
            robot_id = armor.number
            observations_by_robot[robot_id].append(obs)
            raw_observations_by_robot[robot_id].append(raw_obs)
        
        # 更新各机器人的跟踪器
        for robot_id, observations in observations_by_robot.items():
            success = self.tracker_manager.update(robot_id, observations, current_time)
            
            # CSV 日志记录
            if success and self.csv_logger is not None:
                tracker = self.tracker_manager.get_tracker(robot_id)
                if tracker is not None and tracker.is_initialized:
                    raw_observations = raw_observations_by_robot[robot_id]
                    for raw_obs, obs in zip(raw_observations, observations):
                        self.csv_logger.log(
                            timestamp=current_time,
                            robot_id=robot_id,
                            raw_observation=raw_obs,
                            observation=obs,
                            tracker=tracker
                        )
        
        # 发布结果
        self._publish_results(msg.header)
    
    def predict_callback(self):
        """定时预测回调（如果启用）"""
        # 注意：定时器回调没有消息时间戳，使用系统时间
        current_time = time.time()
        
        # 对所有跟踪器执行预测
        self.tracker_manager.predict_all(current_time)
        
        # 移除超时的跟踪器
        removed = self.tracker_manager.remove_stale_trackers(current_time)
        if removed and self.debug_mode:
            self.get_logger().info(f"Removed stale trackers: {removed}")
    
    def _publish_results(self, header: Header):
        """
        发布跟踪结果
        
        Args:
            header: 消息头
        """
        # 获取所有正在跟踪的机器人
        tracking_robots = self.tracker_manager.get_tracking_robots()
        
        if len(tracking_robots) == 0:
            return
        
        # 发布每个机器人的 Target 消息
        for robot_id in tracking_robots:
            tracker = self.tracker_manager.get_tracker(robot_id)
            if tracker is None or not tracker.is_tracking:
                continue
            
            target_msg = self._build_target_message(header, robot_id, tracker)
            self.target_pub.publish(target_msg)
        
        # 发布可视化 Marker（直接在odom坐标系中）
        marker_array = build_tracker_markers(
            self.visualization_frame,
            self.tracker_manager.get_all_trackers(),
            header.stamp
        )
        self.marker_pub.publish(marker_array)
    
    def _build_target_message(self, header: Header, robot_id: str, tracker) -> Target:
        """
        构建 Target 消息
        
        注意：tracker 内部使用虚拟坐标系，发布时需要变换到 odom 坐标系
        
        Args:
            header: 消息头
            robot_id: 机器人ID
            tracker: AdaptiveArmorTracker 实例
            
        Returns:
            Target 消息
        """
        target = Target()
        target.header = header
        target.header.frame_id = self.target_frame
        
        target.tracking = True
        target.id = robot_id
        target.armors_num = 4  # 4面装甲板
        
        # 获取位置和 yaw（已经在 odom 坐标系中）
        pos_odom = tracker.get_center_position()
        yaw_odom = tracker.get_yaw()
        target.position = Point(x=float(pos_odom[0]), y=float(pos_odom[1]), z=float(pos_odom[2]))
        
        # 获取速度（已经在 odom 坐标系中）
        state = tracker.get_state()
        vel_odom = np.array([
            state.get('vx', 0.0),
            state.get('vy', 0.0),
            state.get('vz', 0.0)
        ])
        target.velocity = Vector3(
            x=float(vel_odom[0]),
            y=float(vel_odom[1]),
            z=float(vel_odom[2])
        )
        
        # yaw 和 yaw 速度（已经在 odom 坐标系中）
        target.yaw = float(yaw_odom)
        target.v_yaw = float(state.get('delta_rate', 0.0))
        
        # 半径（结构参数，坐标系无关）
        r1, r2 = tracker.get_radii()
        target.radius_1 = float(r1)
        target.radius_2 = float(r2)
        
        # 高度差（结构参数，坐标系无关）
        target.d_za = float(tracker.get_dza())
        target.d_zc = 0.0  # 暂不支持
        
        # 误差信息（暂不计算）
        target.yaw_diff = 0.0
        target.position_diff = 0.0
        
        return target


def main(args=None):
    rclpy.init(args=args)
    
    node = MaxEntropyTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭 CSV 日志
        if node.csv_logger is not None:
            node.csv_logger.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
