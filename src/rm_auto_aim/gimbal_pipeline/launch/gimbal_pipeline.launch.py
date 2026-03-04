#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GimbalPipeline 独立启动文件 (仅启动 gimbal_pipeline 节点本身)

用于调试或在其他 bringup 文件中嵌入使用:
    ros2 launch gimbal_pipeline gimbal_pipeline.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gimbal_pipeline')
    config_file = os.path.join(pkg_dir, 'config', 'gimbal_pipeline.yaml')

    declare_debug = DeclareLaunchArgument(
        'debug', default_value='false', description='Enable debug publishers')

    gimbal_pipeline_node = Node(
        package='gimbal_pipeline',
        executable='gimbal_pipeline_node',
        name='gimbal_pipeline',
        output='both',
        emulate_tty=True,
        parameters=[
            config_file,
            {'debug_mode': LaunchConfiguration('debug')},
        ],
        remappings=[
            ('cmd_gimbal', '/armor_solver/cmd_gimbal'),
        ],
    )

    return LaunchDescription([
        declare_debug,
        gimbal_pipeline_node,
    ])
