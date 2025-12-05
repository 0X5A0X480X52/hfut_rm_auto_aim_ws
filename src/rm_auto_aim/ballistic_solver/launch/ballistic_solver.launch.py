#!/usr/bin/env python3
#"""
# Created by Chengfu Zou
# Modified by Amatrix (HFUT RM SKT GROUP)
# Copyright (C) FYT Vision Group
#"""
# Copyright (C) FYT Vision Group. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
弹道解算服务节点启动文件

启动方式:
    ros2 launch ballistic_solver ballistic_solver.launch.py

带参数启动:
    ros2 launch ballistic_solver ballistic_solver.launch.py bullet_speed:=30.0

服务调用示例:
    ros2 service call /ballistic_solver/solve rm_interfaces/srv/SolveBallistic \
        "{target_position: {x: 5.0, y: 0.0, z: 1.0}, target_velocity: {x: 0.0, y: 0.0, z: 0.0}, bullet_speed: 28.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('ballistic_solver')
    
    # 参数文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'ballistic_solver.yaml')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the configuration file'
    )
    
    bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='28.0',
        description='Bullet speed in m/s'
    )
    
    compensator_type_arg = DeclareLaunchArgument(
        'compensator_type',
        default_value='resistance',
        description='Trajectory compensator type: ideal or resistance'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 创建节点
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node_exe',
        name='ballistic_solver',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'bullet_speed': LaunchConfiguration('bullet_speed'),
                'compensator_type': LaunchConfiguration('compensator_type'),
            }
        ],
        # 重映射话题/服务名称 (如果需要)
        # remappings=[
        #     ('~/solve', '/ballistic_solver/solve'),
        # ],
    )
    
    return LaunchDescription([
        config_file_arg,
        bullet_speed_arg,
        compensator_type_arg,
        use_sim_time_arg,
        ballistic_solver_node,
    ])
