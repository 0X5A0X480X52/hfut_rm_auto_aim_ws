#!/usr/bin/env python3
"""
Launch file for multi-camera armor fusion node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('armor_fusion'),
            'config',
            'fusion_params.yaml'
        ]),
        description='Path to the fusion parameters YAML file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # Multi-camera fusion node
    fusion_node = Node(
        package='armor_fusion',
        executable='multi_camera_fusion_node.py',
        name='multi_camera_fusion',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 可以在这里添加话题重映射
        ],
        arguments=['--ros-args', '--log-level', 
                  ['DEBUG' if LaunchConfiguration('debug') else 'INFO']]
    )
    
    return LaunchDescription([
        config_file_arg,
        debug_arg,
        fusion_node,
    ])
