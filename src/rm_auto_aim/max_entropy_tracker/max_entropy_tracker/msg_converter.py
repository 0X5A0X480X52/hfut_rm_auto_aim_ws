"""
消息转换模块

将 ROS2 消息类型转换为跟踪器所需的数据结构
"""

import math
from typing import Optional
from geometry_msgs.msg import Quaternion, Pose
from rm_interfaces.msg import Armor

from .core.observation import ObservationData


def quaternion_to_yaw(orientation: Quaternion) -> float:
    """
    从四元数提取 yaw 角
    
    假设装甲板主要是绕 Z 轴旋转，使用简化的提取方法
    
    Args:
        orientation: 四元数 (x, y, z, w)
        
    Returns:
        yaw 角（弧度）
    """
    # 通用欧拉角提取 (ZYX 顺序)
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def quaternion_to_euler(orientation: Quaternion) -> tuple:
    """
    从四元数提取完整的欧拉角 (roll, pitch, yaw)
    
    Args:
        orientation: 四元数 (x, y, z, w)
        
    Returns:
        (roll, pitch, yaw) 弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
    cosr_cosp = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
    sinp = max(-1.0, min(1.0, sinp))  # 限制范围避免 asin 域错误
    pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def armor_to_observation(
    armor: Armor,
    timestamp: Optional[float] = None
) -> ObservationData:
    """
    将 Armor 消息转换为 ObservationData
    
    Args:
        armor: ROS2 Armor 消息
        timestamp: 可选时间戳（秒）
        
    Returns:
        ObservationData 观测数据
    """
    # 提取位置
    x = armor.pose.position.x
    y = armor.pose.position.y
    z = armor.pose.position.z
    
    # 提取 yaw
    yaw = quaternion_to_yaw(armor.pose.orientation)
    
    return ObservationData(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        timestamp=timestamp
    )


def pose_to_observation(
    pose: Pose,
    timestamp: Optional[float] = None
) -> ObservationData:
    """
    将 geometry_msgs/Pose 转换为 ObservationData
    
    Args:
        pose: ROS2 Pose 消息
        timestamp: 可选时间戳（秒）
        
    Returns:
        ObservationData 观测数据
    """
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    yaw = quaternion_to_yaw(pose.orientation)
    
    return ObservationData(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        timestamp=timestamp
    )


def get_robot_id_from_armor_number(armor_number: str) -> str:
    """
    从装甲板编号推断机器人ID
    
    Args:
        armor_number: 装甲板编号 ("1", "2", "3", "4", "5", "outpost", "base", "sentry")
        
    Returns:
        机器人ID字符串
    """
    # 编号到机器人ID的映射
    # 1-5 对应不同的步兵/英雄
    # outpost, base, sentry 是特殊目标
    return armor_number
