from typing import Optional
import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rm_interfaces.msg import Armor
from .types import ArmorMeasurement


def transform_to_base_link(tf_buffer: tf2_ros.Buffer, armor: Armor, source_frame: str, target_frame: str, timestamp) -> Optional[ArmorMeasurement]:
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = timestamp
        pose_stamped.pose = armor.pose
        transform = tf_buffer.lookup_transform(target_frame, source_frame, timestamp, timeout=rclpy.duration.Duration(seconds=0.1))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        position = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        ])
        orientation = np.array([
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        ])
        distance = np.linalg.norm(position)
        sigma = 0.01 + 0.001 * distance
        covariance = np.eye(3) * sigma**2
        return ArmorMeasurement(
            position=position,
            orientation=orientation,
            number=armor.number,
            armor_type=armor.type,
            camera_frame=source_frame,
            timestamp=timestamp.nanoseconds / 1e9,
            covariance=covariance
        )
    except Exception:
        return None
