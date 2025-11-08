from typing import Optional
import numpy as np
import traceback
import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rm_interfaces.msg import Armor
from armor_fusion.types import ArmorMeasurement


def transform_to_base_link(tf_buffer: tf2_ros.Buffer, armor: Armor, source_frame: str, target_frame: str, timestamp, logger=None) -> Optional[ArmorMeasurement]:
    """Transform an armor's pose into the target frame and return an ArmorMeasurement.

    This function is defensive: `armor.pose` may be a geometry_msgs/Pose (as defined in the message)
    or sometimes a PoseStamped-like object (some publishers incorrectly assign a PoseStamped).
    We handle both cases and log a full traceback when a transform fails.
    """
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = timestamp

        # armor.pose should be a Pose, but be tolerant if it's a PoseStamped-like object
        if hasattr(armor.pose, 'position') and hasattr(armor.pose, 'orientation'):
            # geometry_msgs/Pose
            pose_stamped.pose = armor.pose
        elif hasattr(armor.pose, 'pose'):
            # PoseStamped-like: copy inner pose
            pose_stamped.pose = armor.pose.pose
        else:
            if logger:
                logger.warning(f"Unknown armor.pose type when transforming from {source_frame} to {target_frame}: {type(armor.pose)}")
            return None

        # Try to get transform at the message timestamp first. If unavailable (common during
        # startup), fall back to the latest available transform (now) to avoid dropping all
        # measurements when frames become available slightly later.
        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, timestamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            # fallback to latest transform
            if logger:
                logger.debug(f"Transform not available at timestamp {timestamp} for {source_frame}->{target_frame}, falling back to latest transform")
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))

        # Use the stamped transformer to avoid type-dispatch issues in some tf2_geometry_msgs versions
        transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)

        pos = transformed_pose.pose.position
        ori = transformed_pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        orientation = np.array([ori.x, ori.y, ori.z, ori.w])

        distance = np.linalg.norm(position)
        sigma = 0.01 + 0.001 * distance
        covariance = np.eye(3) * sigma**2

        # Normalize timestamp to seconds (accept multiple timestamp types)
        ts_seconds = None
        try:
            ts_seconds = timestamp.nanoseconds / 1e9
        except Exception:
            try:
                ts_seconds = float(timestamp.sec) + float(timestamp.nanosec) / 1e9
            except Exception:
                try:
                    # rclpy.time.Time has to_msg()
                    msg = timestamp.to_msg()
                    ts_seconds = float(msg.sec) + float(msg.nanosec) / 1e9
                except Exception:
                    ts_seconds = None

        return ArmorMeasurement(
            position=position,
            orientation=orientation,
            number=armor.number,
            armor_type=armor.type,
            camera_frame=source_frame,
            timestamp=ts_seconds,
            covariance=covariance
        )
    except Exception as e:
        if logger:
            tb = traceback.format_exc()
            try:
                logger.warning(f"Failed to transform armor from {source_frame} to {target_frame}: {e}\n{tb}")
            except Exception:
                # Some older logger implementations provide warn instead of warning
                try:
                    logger.warn(f"Failed to transform armor from {source_frame} to {target_frame}: {e}\n{tb}")
                except Exception:
                    pass
        return None
