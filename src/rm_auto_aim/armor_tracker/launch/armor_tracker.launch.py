import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('armor_tracker'),
        'config',
        'tracker_params.yaml'
    )

    armor_tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node_exe',
        name='armor_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([
        armor_tracker_node,
    ])
