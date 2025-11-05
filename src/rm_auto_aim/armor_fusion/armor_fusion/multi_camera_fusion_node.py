#!/usr/bin/env python3
"""
Modular Multi-Camera Armor Fusion Node

This file is the thin entrypoint that composes the modular pieces from:
 - types.py
 - transforms.py
 - clustering.py
 - ba.py
 - visualization.py

Behavior is unchanged; implementation is split for maintainability.
"""

from typing import List, Dict
from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import MarkerArray

from rm_interfaces.msg import Armor, Armors

from .types import ArmorMeasurement
from .transforms import transform_to_base_link
from .clustering import cluster_measurements, merge_close_clusters
from .ba import BundleAdjustment
from .visualization import build_visualization_markers


class MultiCameraFusionNode(Node):
    """多摄像头装甲板融合节点（模块化）"""

    def __init__(self):
        super().__init__('multi_camera_fusion_node')

        # 参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topics', ['camera1/armors', 'camera2/armors']),
                ('target_frame', 'base_link'),
                ('dbscan_eps', 0.3),
                ('dbscan_min_samples', 1),
                ('max_cluster_noise', 0.5),
                ('publish_rate', 100.0),
                ('enable_visualization', True),
                ('measurement_buffer_size', 10),
                ('sync_timeout', 0.05),
            ]
        )

        self.camera_topics = self.get_parameter('camera_topics').value
        self.target_frame = self.get_parameter('target_frame').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.max_cluster_noise = self.get_parameter('max_cluster_noise').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.sync_timeout = self.get_parameter('sync_timeout').value

        self.get_logger().info(f'Subscribing to {len(self.camera_topics)} camera topics')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers and buffers
        self.subscribers = []
        self.measurement_buffers: Dict[str, deque] = {}
        self.lock = threading.Lock()

        for topic in self.camera_topics:
            sub = self.create_subscription(Armors, topic, lambda msg, t=topic: self.armors_callback(msg, t), qos_profile)
            self.subscribers.append(sub)
            self.measurement_buffers[topic] = deque(maxlen=self.get_parameter('measurement_buffer_size').value)
            self.get_logger().info(f'Subscribed to {topic}')

        # Publishers
        self.fused_armors_pub = self.create_publisher(Armors, 'armor_fusion/armors', qos_profile)
        if self.enable_viz:
            self.marker_pub = self.create_publisher(MarkerArray, 'armor_fusion/markers', 10)

        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_and_publish)

        # Optimizer
        self.ba_optimizer = BundleAdjustment()

        # Stats
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info('Multi-camera fusion node initialized (modular)')

    def armors_callback(self, msg: Armors, topic: str):
        with self.lock:
            self.measurement_buffers[topic].append(msg)

    def process_and_publish(self):
        with self.lock:
            all_measurements: List[ArmorMeasurement] = []
            current_time = self.get_clock().now()

            for topic, buffer in self.measurement_buffers.items():
                if len(buffer) == 0:
                    continue
                msg = buffer[-1]
                msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
                time_diff = (current_time - msg_time).nanoseconds / 1e9
                if time_diff > self.sync_timeout:
                    continue
                for armor in msg.armors:
                    m = transform_to_base_link(self.tf_buffer, armor, msg.header.frame_id, self.target_frame, msg.header.stamp)
                    if m is not None:
                        all_measurements.append(m)

        if len(all_measurements) == 0:
            empty_msg = Armors()
            empty_msg.header.stamp = self.get_clock().now().to_msg()
            empty_msg.header.frame_id = self.target_frame
            empty_msg.armors = []
            self.fused_armors_pub.publish(empty_msg)
            return

        clusters = cluster_measurements(all_measurements, self.dbscan_eps, self.dbscan_min_samples)
        clusters = merge_close_clusters(clusters, self.max_cluster_noise)

        fused_armors = []
        for cluster_id, cluster_measurements in clusters.items():
            optimized_position, residual = self.ba_optimizer.optimize_position(cluster_measurements)
            fused_orientation = self.ba_optimizer.fuse_orientation(cluster_measurements)
            numbers = [m.number for m in cluster_measurements]
            types = [m.armor_type for m in cluster_measurements]
            most_common_number = max(set(numbers), key=numbers.count)
            most_common_type = max(set(types), key=types.count)

            fused_armor = Armor()
            fused_armor.number = most_common_number
            fused_armor.type = most_common_type
            fused_armor.pose.position = Point(x=float(optimized_position[0]), y=float(optimized_position[1]), z=float(optimized_position[2]))
            fused_armor.pose.orientation = Quaternion(x=float(fused_orientation[0]), y=float(fused_orientation[1]), z=float(fused_orientation[2]), w=float(fused_orientation[3]))
            fused_armor.distance_to_image_center = float((optimized_position[:2] ** 2).sum() ** 0.5)
            fused_armors.append(fused_armor)

        fused_msg = Armors()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = self.target_frame
        fused_msg.armors = fused_armors
        self.fused_armors_pub.publish(fused_msg)

        if self.enable_viz:
            marker_array = build_visualization_markers(self.target_frame, clusters, fused_armors)
            self.marker_pub.publish(marker_array)

        self.frame_count += 1
        if (self.get_clock().now() - self.last_log_time).nanoseconds > 5e9:
            self.get_logger().info(f'Processed {self.frame_count} frames, current: {len(all_measurements)} measurements -> {len(fused_armors)} targets')
            self.frame_count = 0
            self.last_log_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
