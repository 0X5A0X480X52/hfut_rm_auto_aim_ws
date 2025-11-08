#!/usr/bin/env python3
"""
Launch multi-camera system using real MindVision cameras (dual driver).

This launch includes the MindVision dual camera launch and starts the
detectors, fusion and solver nodes configured to use the camera namespaces
`camera_left` and `camera_right` produced by the driver.

Usage:
  ros2 launch armor_fusion multi_camera_system_with_mindvision.launch.py

You can override the mindvision params file with `params_file:=/path/to/dual_camera_params.yaml`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    mv_pkg = FindPackageShare('mindvision_camera')
    fusion_pkg = FindPackageShare('armor_fusion')

    default_params = PathJoinSubstitution([mv_pkg, 'config', 'dual_camera_params.yaml'])

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='mindvision dual camera params file'
    )

    declare_use_sensor_qos = DeclareLaunchArgument('use_sensor_data_qos', default_value='false', description='Use sensor data QoS for camera topics')
    declare_enable_viz = DeclareLaunchArgument('enable_visualization', default_value='true', description='Enable fusion visualization')

    # Include the mindvision dual camera driver (it launches namespaces camera_left and camera_right)
    include_mindvision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mv_pkg, 'launch', 'dual_camera_launch.py'])
        ),
        launch_arguments={'params_file': LaunchConfiguration('params_file'), 'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos')}.items()
    )

    # Start armor_detector under camera_left namespace - it will subscribe to camera_left/image_raw and camera_left/camera_info
    camera_left_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera_left_detector',
        namespace='camera_left',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug': True,
            'target_frame': 'odom',
        }],
    )

    # Start armor_detector under camera_right namespace (delayed start handled by driver already)
    camera_right_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera_right_detector',
        namespace='camera_right',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug': True,
            'target_frame': 'odom',
        }],
    )

    # Fusion node - subscribe to the detectors' armors topics under camera namespaces
    fusion_node = Node(
        package='armor_fusion',
        executable='multi_camera_fusion_node.py',
        name='multi_camera_fusion',
        output='screen',
        parameters=[{
            # detectors publish their armors to 'camera_left/armor_detector/armors' and 'camera_right/armor_detector/armors'
            'camera_topics': ['camera_left/armor_detector/armors', 'camera_right/armor_detector/armors'],
            'enable_visualization': LaunchConfiguration('enable_visualization'),
        }]
    )

    # Solver node subscribes to fused armors
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
            ('armor_detector/armors', 'armor_fusion/armors'),
        ]
    )

    ld = LaunchDescription()
    for decl in [declare_params, declare_use_sensor_qos, declare_enable_viz]:
        ld.add_action(decl)

    # Mindvision driver first
    ld.add_action(include_mindvision)

    # Then detectors and processing
    ld.add_action(camera_left_detector)
    ld.add_action(camera_right_detector)
    ld.add_action(fusion_node)
    ld.add_action(solver_node)

    return ld
