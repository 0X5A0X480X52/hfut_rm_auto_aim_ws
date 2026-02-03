"""
可视化模块

生成用于 RViz 显示的 MarkerArray，包括:
- 机器人中心位置（SPHERE）
- 预测的装甲板位置（CUBE）
- 速度方向（ARROW）

支持发布到不同坐标系：
- virtual_camera_frame: 直接使用 tracker 内部状态（虚拟坐标系）
- odom: 将 tracker 状态变换到世界坐标系后发布
"""

import numpy as np
from typing import Dict, Optional, TYPE_CHECKING
from builtin_interfaces.msg import Time as RosTime
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA

from .trackers.adaptive_armor_tracker import AdaptiveArmorTracker

if TYPE_CHECKING:
    from .tf_handler import TFHandler


# 颜色定义
COLOR_CENTER = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色 - 机器人中心
COLOR_ARMOR = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)   # 蓝色 - 装甲板
COLOR_VELOCITY = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # 黄色 - 速度
COLOR_TRACKING = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色 - 跟踪中
COLOR_TEMP_LOST = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # 橙色 - 临时丢失


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    将 yaw 角转换为四元数（绕 Z 轴旋转）
    
    Args:
        yaw: yaw 角（弧度）
        
    Returns:
        Quaternion 消息
    """
    q = Quaternion()
    q.w = np.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = np.sin(yaw / 2.0)
    return q


def build_tracker_markers(
    target_frame: str,
    trackers: Dict[str, AdaptiveArmorTracker],
    timestamp: RosTime,
    tf_handler: Optional['TFHandler'] = None
) -> MarkerArray:
    """
    为所有跟踪器构建可视化 Marker
    
    Args:
        target_frame: 目标坐标系（通常是 'odom'）
        trackers: robot_id -> tracker 字典
        timestamp: ROS 时间戳
        tf_handler: TF 处理器（已废弃，现在tracker直接在odom坐标系中工作）
        
    Returns:
        MarkerArray 消息
    """
    marker_array = MarkerArray()
    marker_id = 0
    
    for robot_id, tracker in trackers.items():
        if not tracker.is_initialized:
            continue
        
        # 获取跟踪器状态（已经在odom坐标系中）
        pos = tracker.get_center_position()
        yaw = tracker.get_yaw()
        r1, r2 = tracker.get_radii()
        dza = tracker.get_dza()
        state = tracker.get_state()
        
        # 获取速度（已经在odom坐标系中）
        vx = state.get('vx', 0.0)
        vy = state.get('vy', 0.0)
        vz = state.get('vz', 0.0)
        
        # 确定颜色
        if tracker.is_tracking:
            center_color = COLOR_TRACKING
        else:
            center_color = COLOR_TEMP_LOST
        
        # 1. 机器人中心 SPHERE
        center_marker = Marker()
        center_marker.header.frame_id = target_frame
        center_marker.header.stamp = timestamp
        center_marker.ns = f"robot_center_{robot_id}"
        center_marker.id = marker_id
        marker_id += 1
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        center_marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        center_marker.scale.x = 0.1
        center_marker.scale.y = 0.1
        center_marker.scale.z = 0.1
        center_marker.color = center_color
        center_marker.lifetime.sec = 0
        center_marker.lifetime.nanosec = 100000000  # 0.1秒
        marker_array.markers.append(center_marker)
        
        # 2. 文本标签 - 显示机器人ID
        text_marker = Marker()
        text_marker.header.frame_id = target_frame
        text_marker.header.stamp = timestamp
        text_marker.ns = f"robot_label_{robot_id}"
        text_marker.id = marker_id
        marker_id += 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(
            x=float(pos[0]),
            y=float(pos[1]),
            z=float(pos[2]) + 0.3
        )
        text_marker.text = f"ID:{robot_id}"
        text_marker.scale.z = 0.15
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 100000000
        marker_array.markers.append(text_marker)
        
        # 3. 预测的装甲板位置 CUBE（4面）
        # 注意：装甲板编号遵循UKF中的panel_angle定义
        # panel_angle相对center_yaw: 0, π/2, π, 3π/2
        # armor_yaw = center_yaw + panel_angle（径向方向：中心指向装甲板）
        for panel_idx in range(4):
            # 计算装甲板位置
            panel_angle = yaw + panel_idx * (np.pi / 2)  # 0, 90, 180, 270 度
            r = r1 if panel_idx % 2 == 0 else r2  # 交替使用 r1, r2
            
            # 装甲板中心高度偏移（upper/lower层）
            # panel_idx: 0->lower(r1), 1->upper(r2), 2->lower(r1), 3->upper(r2)
            armor_layer = 'upper' if panel_idx % 2 == 1 else 'lower'
            z_offset = dza if armor_layer == 'upper' else -dza
            
            # 装甲板位置 = 中心 + 半径 * 径向单位向量（center指向armor）
            armor_x = pos[0] + r * np.cos(panel_angle)
            armor_y = pos[1] + r * np.sin(panel_angle)
            armor_z = pos[2] + z_offset
            
            armor_marker = Marker()
            armor_marker.header.frame_id = target_frame
            armor_marker.header.stamp = timestamp
            armor_marker.ns = f"armor_{robot_id}"
            armor_marker.id = marker_id
            marker_id += 1
            armor_marker.type = Marker.CUBE
            armor_marker.action = Marker.ADD
            armor_marker.pose.position = Point(x=armor_x, y=armor_y, z=armor_z)
            # 装甲板朝向：法向量指向外侧
            armor_marker.pose.orientation = yaw_to_quaternion(panel_angle)
            # 装甲板尺寸：厚度 x 宽度 x 高度
            armor_marker.scale.x = 0.03  # 厚度
            armor_marker.scale.y = 0.135 if panel_idx % 2 == 0 else 0.23  # 宽度（小/大装甲板）
            armor_marker.scale.z = 0.125  # 高度
            armor_marker.color = COLOR_ARMOR
            armor_marker.lifetime.sec = 0
            armor_marker.lifetime.nanosec = 100000000
            marker_array.markers.append(armor_marker)
        
        # 4. 速度方向 ARROW
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        
        if speed > 0.1:  # 只有速度足够大才显示
            vel_marker = Marker()
            vel_marker.header.frame_id = target_frame
            vel_marker.header.stamp = timestamp
            vel_marker.ns = f"velocity_{robot_id}"
            vel_marker.id = marker_id
            marker_id += 1
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD
            
            # 箭头起点和终点
            start_point = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            scale = min(speed * 0.5, 1.0)  # 限制箭头长度
            end_point = Point(
                x=float(pos[0] + vx * scale / speed),
                y=float(pos[1] + vy * scale / speed),
                z=float(pos[2] + vz * scale / speed)
            )
            vel_marker.points = [start_point, end_point]
            vel_marker.scale.x = 0.03  # 箭头轴直径
            vel_marker.scale.y = 0.06  # 箭头头部直径
            vel_marker.scale.z = 0.0
            vel_marker.color = COLOR_VELOCITY
            vel_marker.lifetime.sec = 0
            vel_marker.lifetime.nanosec = 100000000
            marker_array.markers.append(vel_marker)
        
        # 5. Yaw 方向 ARROW（红色）
        yaw_marker = Marker()
        yaw_marker.header.frame_id = target_frame
        yaw_marker.header.stamp = timestamp
        yaw_marker.ns = f"yaw_{robot_id}"
        yaw_marker.id = marker_id
        marker_id += 1
        yaw_marker.type = Marker.ARROW
        yaw_marker.action = Marker.ADD
        
        yaw_length = 0.3
        start_point = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        end_point = Point(
            x=float(pos[0] + yaw_length * np.cos(yaw)),
            y=float(pos[1] + yaw_length * np.sin(yaw)),
            z=float(pos[2])
        )
        yaw_marker.points = [start_point, end_point]
        yaw_marker.scale.x = 0.02
        yaw_marker.scale.y = 0.04
        yaw_marker.scale.z = 0.0
        yaw_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        yaw_marker.lifetime.sec = 0
        yaw_marker.lifetime.nanosec = 100000000
        marker_array.markers.append(yaw_marker)
    
    return marker_array


def build_delete_all_markers(target_frame: str, timestamp: RosTime) -> MarkerArray:
    """
    创建删除所有 Marker 的消息
    
    Args:
        target_frame: 目标坐标系
        timestamp: ROS 时间戳
        
    Returns:
        MarkerArray 消息
    """
    marker_array = MarkerArray()
    
    delete_marker = Marker()
    delete_marker.header.frame_id = target_frame
    delete_marker.header.stamp = timestamp
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)
    
    return marker_array
