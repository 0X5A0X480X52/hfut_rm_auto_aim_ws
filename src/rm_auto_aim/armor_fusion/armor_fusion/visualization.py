from typing import Dict, List
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .types import ArmorMeasurement
from rm_interfaces.msg import Armor


def build_visualization_markers(target_frame: str, clusters: Dict[int, List[ArmorMeasurement]], fused_armors: List[Armor]):
    marker_array = MarkerArray()
    for cluster_id, measurements in clusters.items():
        for i, m in enumerate(measurements):
            marker = Marker()
            marker.header.frame_id = target_frame
            marker.ns = f'cluster_{cluster_id}_measurements'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(m.position[0]), y=float(m.position[1]), z=float(m.position[2]))
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker_array.markers.append(marker)
    for i, armor in enumerate(fused_armors):
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.ns = 'fused_targets'
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = armor.pose
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker_array.markers.append(marker)
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = 'fused_targets_text'
        text_marker.id = i
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose = armor.pose
        text_marker.pose.position.z += 0.2
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = armor.number
        marker_array.markers.append(text_marker)
    return marker_array
