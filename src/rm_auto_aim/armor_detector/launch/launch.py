"""
Launch file for armor_detector.

This launch file supports loading a YAML config from the package `share/armor_detector/config`
and allows overriding common topic names via launch arguments:
  - config_file: path to parameters YAML (default: <package_share>/config/armor_detector.yaml)
  - image_topic: name of the input image topic to subscribe (default: /image_raw)
  - camera_info_topic: name of the camera_info topic (default: /camera_info)
  - armors_topic: name of the published armors topic (default: /armor_detector/armors)
  - marker_topic: name of the published marker topic (default: /armor_detector/marker)
  - debug: enable debug publishers (true/false)

Usage examples:
  ros2 launch armor_detector launch.py
  ros2 launch armor_detector launch.py config_file:=/path/to/my.yaml image_topic:=/camera/color/image_raw

Note: this file assumes the package name is `armor_detector` and the executable registered
in CMake is `armor_detector_node` (this matches the package CMakeLists.txt in this repo).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Resolve package share for defaults
    pkg_share = get_package_share_directory('armor_detector')
    default_config = os.path.join(pkg_share, 'config', 'armor_detector.yaml')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to a YAML file with node parameters (ROS2 parameter file).'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/image_raw',
        description='Input image topic name (will remap node "image_raw" to this).'
    )

    camera_info_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera_info',
        description='Camera info topic name (will remap node "camera_info" to this).'
    )

    armors_topic_arg = DeclareLaunchArgument(
        'armors_topic', default_value='/armor_detector/armors',
        description='Published armors topic name (remaps "armor_detector/armors").'
    )

    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic', default_value='/armor_detector/marker',
        description='Published marker topic name (remaps "armor_detector/marker").'
    )

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Whether to enable debug publishers (string bool).'
    )

    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')
    camera_info = LaunchConfiguration('camera_info_topic')
    armors_topic = LaunchConfiguration('armors_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    debug = LaunchConfiguration('debug')

    # Node: armor_detector_node is registered in CMake as executable
    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        parameters=[config_file],
        remappings=[
            # remap the subscriptions
            ('image_raw', image_topic),
            ('camera_info', camera_info),
            # remap the publishers
            ('armor_detector/armors', armors_topic),
            ('armor_detector/marker', marker_topic),
            # debug image topics (image_transport publishers use these names)
            ('armor_detector/binary_img', 'armor_detector/binary_img'),
            ('armor_detector/number_img', 'armor_detector/number_img'),
            ('armor_detector/result_img', 'armor_detector/result_img'),
        ],
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(camera_info_arg)
    ld.add_action(armors_topic_arg)
    ld.add_action(marker_topic_arg)
    ld.add_action(debug_arg)
    ld.add_action(detector_node)

    return ld
