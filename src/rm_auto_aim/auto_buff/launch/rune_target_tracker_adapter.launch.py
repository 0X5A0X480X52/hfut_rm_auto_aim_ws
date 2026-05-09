from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("auto_buff")
    param_file = os.path.join(pkg_share, "config", "buff_pose_estimator.yaml")
    return LaunchDescription([
        Node(
            package="auto_buff",
            executable="buff_pose_estimator_node",
            name="buff_pose_estimator",
            output="screen",
            parameters=[param_file],
        )
    ])
