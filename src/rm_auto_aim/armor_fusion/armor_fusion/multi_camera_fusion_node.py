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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import MarkerArray

from rm_interfaces.msg import Armor, Armors

from armor_fusion.types import ArmorMeasurement
from armor_fusion.transforms import transform_to_base_link
from armor_fusion.clustering import cluster_measurements, merge_close_clusters
from armor_fusion.ba import BundleAdjustment
from armor_fusion.visualization import build_visualization_markers


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
                ('tf_wait_timeout', 2.0),
                ('enable_visualization', True),
                ('console_debug', False),
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
        self.console_debug = self.get_parameter('console_debug').value
        self.sync_timeout = self.get_parameter('sync_timeout').value
        self.tf_wait_timeout = self.get_parameter('tf_wait_timeout').value

        self.get_logger().info(f'Subscribing to {len(self.camera_topics)} camera topics')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for TF tree to contain the target_frame (best-effort, bounded)
        try:
            start_ns = self.get_clock().now().nanoseconds
            deadline_ns = start_ns + int(self.tf_wait_timeout * 1e9)
            waited = False
            while self.get_clock().now().nanoseconds < deadline_ns:
                try:
                    # Try a trivial lookup; using same frame for source/target will raise
                    # if the frame does not yet exist in the buffer.
                    self.tf_buffer.lookup_transform(self.target_frame, self.target_frame, self.get_clock().now().to_msg(), timeout=rclpy.duration.Duration(seconds=0.05))
                    if waited:
                        self.get_logger().info(f"TF frame '{self.target_frame}' available after waiting; continuing startup")
                    break
                except Exception:
                    # not available yet, wait a short while
                    waited = True
                    time.sleep(0.1)
            else:
                if waited:
                    self.get_logger().warn(f"Timed out waiting {self.tf_wait_timeout}s for TF frame '{self.target_frame}'; continuing startup (TF lookups may fail until transforms arrive)")
        except Exception as e:
            # Don't let TF readiness waiting break startup; log and continue
            self.get_logger().warn(f"Exception while waiting for TF readiness: {e}")

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
        # Throttle console debug messages to at most once per second
        self.last_console_debug_time = self.get_clock().now()

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
                    m = transform_to_base_link(self.tf_buffer, armor, msg.header.frame_id, self.target_frame, msg.header.stamp, self.get_logger())
                    if m is not None:
                        all_measurements.append(m)

        if len(all_measurements) == 0:
            empty_msg = Armors()
            empty_msg.header.stamp = self.get_clock().now().to_msg()
            empty_msg.header.frame_id = self.target_frame
            empty_msg.armors = []
            # Detailed debug output when no measurements were collected
            if self.console_debug:
                dbg_lines = [f"[armor_fusion debug] No measurements collected -> publishing empty Armors (target_frame={self.target_frame})"]
                for topic, buffer in self.measurement_buffers.items():
                    if len(buffer) == 0:
                        dbg_lines.append(f"  {topic}: buffer empty")
                        continue
                    last = buffer[-1]
                    try:
                        msg_time = rclpy.time.Time.from_msg(last.header.stamp)
                        time_diff = (self.get_clock().now() - msg_time).nanoseconds / 1e9
                    except Exception:
                        time_diff = None
                    dbg_lines.append(f"  {topic}: last_msg_count={len(last.armors)}, frame_id={last.header.frame_id}, time_diff={time_diff}")
                    # Check TF availability for the message frame
                    try:
                        self.tf_buffer.lookup_transform(self.target_frame, last.header.frame_id, last.header.stamp, timeout=rclpy.duration.Duration(seconds=0.05))
                        dbg_lines.append(f"    TF lookup: OK for frame {last.header.frame_id}")
                    except Exception as e:
                        dbg_lines.append(f"    TF lookup: FAILED for frame {last.header.frame_id}: {e}")
                    if len(last.armors) > 0:
                        armor_summaries = []
                        for a in last.armors:
                            armor_summaries.append(f"num={a.number},type={a.type},pos=({a.pose.position.x:.2f},{a.pose.position.y:.2f},{a.pose.position.z:.2f})")
                        dbg_lines.append(f"    last_armors: " + ", ".join(armor_summaries))
                now_ns = self.get_clock().now().nanoseconds
                if (now_ns - self.last_console_debug_time.nanoseconds) > 1e9:
                    self.get_logger().info("\n".join(dbg_lines))
                    self.last_console_debug_time = self.get_clock().now()

            self.fused_armors_pub.publish(empty_msg)
            return

        clusters = cluster_measurements(all_measurements, self.dbscan_eps, self.dbscan_min_samples)
        clusters = merge_close_clusters(clusters, self.max_cluster_noise)

        fused_armors = []
        for cluster_id, meas_list in clusters.items():
            optimized_position, residual = self.ba_optimizer.optimize_position(meas_list)
            fused_orientation = self.ba_optimizer.fuse_orientation(meas_list)
            numbers = [m.number for m in meas_list]
            types = [m.armor_type for m in meas_list]
            most_common_number = max(set(numbers), key=numbers.count)
            most_common_type = max(set(types), key=types.count)

            fused_armor = Armor()
            fused_armor.number = most_common_number
            fused_armor.type = most_common_type
            fused_armor.pose.position = Point(x=float(optimized_position[0]), y=float(optimized_position[1]), z=float(optimized_position[2]))
            fused_armor.pose.orientation = Quaternion(x=float(fused_orientation[0]), y=float(fused_orientation[1]), z=float(fused_orientation[2]), w=float(fused_orientation[3]))
            fused_armor.distance_to_image_center = float((optimized_position[:2] ** 2).sum() ** 0.5)
            fused_armors.append(fused_armor)

        # Debug: log clustering details when requested
        if self.console_debug:
            try:
                now_ns = self.get_clock().now().nanoseconds
                if (now_ns - self.last_console_debug_time.nanoseconds) > 1e9:
                    cluster_lines = [f"[armor_fusion debug] Clusters found: {len(clusters)}"]
                    for cid, meas in clusters.items():
                        nums = [m.number for m in meas]
                        cluster_lines.append(f"  cluster {cid}: {len(meas)} measurements, numbers={nums}")
                    cluster_lines.append(f"Fused armors produced: {len(fused_armors)}")
                    self.get_logger().info("\n".join(cluster_lines))
                    self.last_console_debug_time = self.get_clock().now()
            except Exception as e:
                self.get_logger().warn(f"Failed to produce cluster debug info: {e}")

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
