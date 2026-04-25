#!/usr/bin/env python3
"""
Max Entropy Tracker Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('max_entropy_tracker')
    
    # 参数文件路径
    default_params_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    
    # 声明 launch 参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameter file'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='Target coordinate frame (world frame)'
    )
    
    enable_csv_logging_arg = DeclareLaunchArgument(
        'enable_csv_logging',
        default_value='false',
        description='Enable CSV logging of observations'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Max Entropy Tracker 节点
    max_entropy_tracker_node = Node(
        package='max_entropy_tracker',
        executable='max_entropy_tracker_node',
        name='max_entropy_tracker_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
        ],
        remappings=[
            # 可以在这里添加话题重映射
            # ('/armor_detector/armors', '/your_custom_topic'),
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        target_frame_arg,
        enable_csv_logging_arg,
        debug_mode_arg,
        max_entropy_tracker_node,
    ])
