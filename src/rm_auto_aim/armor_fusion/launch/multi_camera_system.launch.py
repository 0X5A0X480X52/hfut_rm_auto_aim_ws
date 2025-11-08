#!/usr/bin/env python3
"""
Complete multi-camera auto-aim system launch file
Launches multiple detectors, fusion node, and solver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    完整的多摄像头自瞄系统启动文件
    
    系统架构:
    Camera1 -> Detector1 ---|
                             |--> Fusion --> Solver --> Gimbal Control
    Camera2 -> Detector2 ---|
    """
    
    # Declare launch arguments
    enable_camera1 = DeclareLaunchArgument(
        'enable_camera1',
        default_value='true',
        description='Enable camera 1 detector'
    )
    
    enable_camera2 = DeclareLaunchArgument(
        'enable_camera2',
        default_value='true',
        description='Enable camera 2 detector'
    )
    
    enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz visualization'
    )

    # Allow passing a params file to configure fusion/solver/detectors
    system_params = DeclareLaunchArgument(
        'system_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('armor_fusion'),
            'config',
            'fusion_params.yaml'
        ]),
        description='Path to a YAML params file to configure fusion/solver/detectors'
    )
    
    target_color = DeclareLaunchArgument(
        'target_color',
        default_value='red',
        description='Target color: red or blue'
    )
    
    # Camera 1 detector node
    camera1_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera1_detector',
        namespace='camera1',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera1')),
        parameters=[{
            'debug': True,
            'target_frame': 'odom',
        }],
        remappings=[
            ('image_raw', '/camera1/image_raw'),
            ('camera_info', '/camera1/camera_info'),
            ('armor_detector/armors', '/camera1/armor_detector/armors'),
        ]
    )
    
    # Camera 2 detector node
    camera2_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera2_detector',
        namespace='camera2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera2')),
        parameters=[{
            'debug': True,
            'target_frame': 'odom',
        }],
        remappings=[
            ('image_raw', '/camera2/image_raw'),
            ('camera_info', '/camera2/camera_info'),
            ('armor_detector/armors', '/camera2/armor_detector/armors'),
        ]
    )
    
    # Fusion node
    fusion_node = Node(
        package='armor_fusion',
        executable='multi_camera_fusion_node.py',
        name='multi_camera_fusion',
        output='screen',
        parameters=[
            LaunchConfiguration('system_params'),
            {
                'enable_visualization': LaunchConfiguration('enable_visualization'),
            }
        ]
    )
    
    # Solver node (subscribing to fused armors)
    solver_node = Node(
        package='armor_solver',
        executable='armor_solver_node',
        name='armor_solver',
        output='screen',
        parameters=[{
            'debug': True,
            'target_frame': 'odom',
        }],
        remappings=[
            # Remap to subscribe to fusion output instead of single detector
            ('armor_detector/armors', 'armor_fusion/armors'),
        ]
    )
    
    return LaunchDescription([
        enable_camera1,
        enable_camera2,
        enable_visualization,
        target_color,
        system_params,
        camera1_detector,
        camera2_detector,
        fusion_node,
        solver_node,
    ])
