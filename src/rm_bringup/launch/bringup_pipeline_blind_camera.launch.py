#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GimbalPipeline with main camera and blind camera.

Pipeline:
  main_camera (hik/mindvision + armor_detector) --\
                                                     +--> armor_fusion --> /armor_fusion/armors --> gimbal_pipeline
  blind_camera_1 (usb_camera + armor_detector) ----/

Topic naming:
  Main camera (no namespace):
    - /image_raw
    - /camera_info
    - /armor_detector/armors

  Blind camera (namespace: blind_camera_1):
    - /blind_camera_1/image_raw
    - /blind_camera_1/camera_info
    - /blind_camera_1/armor_detector/armors
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Load launch params from launch_params_decoupled.yaml
    try:
        launch_params = yaml.safe_load(open(os.path.join(
            get_package_share_directory('rm_bringup'), 'config', 'launch_params_decoupled.yaml')))
    except Exception:
        launch_params = {
            'image_source': 'hik',
            'virtual_serial': True,
            'namespace': '',
            'debug': True,
            'odom2camera': {
                'xyz': '0.174275 0.000 0.086463',
                'rpy': '0.0 0.1396 -0.00'
            }
        }

    # Load blind camera params
    blind_camera_params_path = os.path.join(
        get_package_share_directory('usb_camera_driver'), 'config', 'blind_camera_1_params.yaml')
    try:
        with open(blind_camera_params_path, 'r', encoding='utf-8') as f:
            blind_camera_params_yaml = yaml.safe_load(f)
        blind_camera_params = blind_camera_params_yaml.get('/**', {}).get('ros__parameters', {})
    except Exception:
        blind_camera_params = {
            'xyz': '0.05 0.1 0.05',
            'rpy': '0.0 0.0 0.0'
        }

    main_camera_xyz = launch_params.get('odom2camera', {}).get('xyz', '0.174275 0.000 0.086463').strip('"')
    main_camera_rpy = launch_params.get('odom2camera', {}).get('rpy', '0.0 0.1396 -0.00').strip('"')
    blind_camera_xyz = blind_camera_params.get('xyz', '0.05 0.1 0.05').strip('"')
    blind_camera_rpy = blind_camera_params.get('rpy', '0.0 0.0 0.0').strip('"')

    def get_bringup_params(name):
        return os.path.join(
            get_package_share_directory('rm_bringup'),
            'config', 'node_params', f'{name}_params.yaml')

    def get_pkg_params(pkg_name, param_file):
        return os.path.join(get_package_share_directory(pkg_name), 'config', param_file)

    def get_usb_camera_params(name):
        return os.path.join(
            get_package_share_directory('usb_camera_driver'),
            'config', f'{name}_params.yaml')

    # Declare launch arguments
    declare_image_source = DeclareLaunchArgument(
        'image_source',
        default_value=str(launch_params.get('image_source', 'hik')),
        description='Image source: video | mindvision | hik'
    )
    declare_virtual_serial = DeclareLaunchArgument(
        'virtual_serial',
        default_value=str(launch_params.get('virtual_serial', True)).lower(),
        description='Use virtual serial instead of real serial'
    )
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value=str(launch_params.get('debug', True)).lower(),
        description='Enable debug mode'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=launch_params.get('namespace', ''),
        description='Namespace for all nodes'
    )
    declare_enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable fusion markers'
    )

    # Robot description with main camera and blind camera
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        os.path.join(
            get_package_share_directory('rm_robot_description'),
            'urdf', 'rm_gimbal_with_blind_camera.urdf.xacro'),
        ' main_camera_xyz:="', main_camera_xyz, '"',
        ' main_camera_rpy:="', main_camera_rpy, '"',
        ' blind_camera_1_xyz:="', blind_camera_xyz, '"',
        ' blind_camera_1_rpy:="', blind_camera_rpy, '"',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 1000.0
        }],
        output='screen',
    )

    # Ballistic solver
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node_exe',
        name='ballistic_solver',
        output='screen',
        emulate_tty=True,
        parameters=[get_pkg_params('ballistic_solver', 'ballistic_solver.yaml')],
    )

    # Armor fusion node
    fusion_node = Node(
        package='armor_fusion',
        executable='armor_fusion_node',
        name='armor_fusion',
        output='screen',
        emulate_tty=True,
        parameters=[
            get_pkg_params('armor_fusion', 'fusion_params.yaml'),
            {
                'camera_topics': [
                    '/armor_detector/armors',           # Main camera (no namespace)
                    '/blind_camera_1/armor_detector/armors',
                ],
                'camera_info_topics': [
                    '/camera_info',                     # Main camera
                    '/blind_camera_1/camera_info',
                ],
                'target_frame': 'odom',
                'output_topic': '/armor_fusion/armors',
                'enable_visualization': LaunchConfiguration('enable_visualization'),
                'console_debug': LaunchConfiguration('debug'),
            },
        ],
    )

    # Gimbal pipeline
    gimbal_pipeline_node = Node(
        package='gimbal_pipeline',
        executable='gimbal_pipeline_node',
        name='gimbal_pipeline',
        output='both',
        emulate_tty=True,
        parameters=[
            get_pkg_params('gimbal_pipeline', 'gimbal_pipeline.yaml'),
            {'debug_mode': LaunchConfiguration('debug')},
        ],
        remappings=[
            ('cmd_gimbal', '/armor_solver/cmd_gimbal'),
            ('armors', '/armor_fusion/armors'),
        ],
    )

    # Main camera + detector container
    def create_main_camera_detector_container(context):
        image_source = LaunchConfiguration('image_source').perform(context)
        image_source = image_source.lower() if image_source else 'hik'
        debug_enabled = LaunchConfiguration('debug').perform(context).lower() == 'true'

        if image_source == 'video':
            image_node = ComposableNode(
                package='video_player',
                plugin='video_player::VideoPlayerNode',
                name='video_player',
                parameters=[get_bringup_params('video_player')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        elif image_source == 'mindvision':
            image_node = ComposableNode(
                package='mindvision_camera',
                plugin='mindvision_camera::MVCameraNode',
                name='mv_camera',
                parameters=[get_bringup_params('mindvision_camera_driver')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        elif image_source == 'hik':
            image_node = ComposableNode(
                package='ros2_hik_camera',
                plugin='ros2_hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[get_bringup_params('hik_camera_driver')],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                    'env': {
                        'MVCAM_SDK_PATH': '/opt/MVS',
                        'MVCAM_COMMON_RUNENV': '/opt/MVS/lib'
                    }
                }]
            )
        else:
            image_node = ComposableNode(
                package='mindvision_camera',
                plugin='mindvision_camera::MVCameraNode',
                name='camera_driver',
                parameters=[get_bringup_params('camera_driver')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )

        # Main camera armor detector (no namespace, default topics)
        main_armor_detector_node = ComposableNode(
            package='armor_detector',
            plugin='fyt::auto_aim::ArmorDetectorNode',
            name='armor_detector',
            parameters=[
                get_bringup_params('armor_detector'),
                {'debug': debug_enabled},
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        container = ComposableNodeContainer(
            name='main_camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[image_node, main_armor_detector_node],
            output='both',
            emulate_tty=True,
        )
        return [container]

    # Blind camera + detector container
    def create_blind_camera_detector_container(context):
        debug_enabled = LaunchConfiguration('debug').perform(context).lower() == 'true'

        blind_camera_node = ComposableNode(
            package='usb_camera_driver',
            plugin='blind_vision::USBCameraNode',
            name='usb_camera',
            namespace='blind_camera_1',
            parameters=[get_usb_camera_params('blind_camera_1')],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

        # Blind camera armor detector (namespace: blind_camera_1)
        blind_armor_detector_node = ComposableNode(
            package='armor_detector',
            plugin='fyt::auto_aim::ArmorDetectorNode',
            name='armor_detector',
            namespace='blind_camera_1',
            parameters=[
                get_bringup_params('armor_detector'),
                {'debug': debug_enabled},
            ],
            # Remap to local topics within blind_camera_1 namespace
            remappings=[
                ('/image_raw', 'image_raw'),
                ('/camera_info', 'camera_info'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

        container = ComposableNodeContainer(
            name='blind_camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[blind_camera_node, blind_armor_detector_node],
            output='both',
            emulate_tty=True,
        )
        return [container]

    # Serial node
    def create_serial_node_action(context):
        virtual_serial = LaunchConfiguration('virtual_serial').perform(context).lower() == 'true'
        if virtual_serial:
            return [Node(
                package='rm_serial_driver',
                executable='virtual_serial_node',
                name='virtual_serial',
                output='both',
                emulate_tty=True,
                parameters=[get_bringup_params('virtual_serial')],
            )]
        else:
            return [Node(
                package='rm_serial_driver',
                executable='rm_serial_driver_node',
                name='serial_driver',
                output='both',
                emulate_tty=True,
                parameters=[get_bringup_params('serial_driver')],
            )]

    # Delayed startup
    delay_serial = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=create_serial_node_action)],
    )
    delay_ballistic = TimerAction(
        period=2.0,
        actions=[ballistic_solver_node],
    )
    delay_main_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_main_camera_detector_container)],
    )
    delay_blind_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_blind_camera_detector_container)],
    )
    delay_fusion = TimerAction(
        period=2.8,
        actions=[fusion_node],
    )
    delay_gimbal_pipeline = TimerAction(
        period=3.2,
        actions=[gimbal_pipeline_node],
    )

    push_namespace = PushRosNamespace(LaunchConfiguration('namespace'))

    return LaunchDescription([
        declare_image_source,
        declare_virtual_serial,
        declare_debug,
        declare_namespace,
        declare_enable_visualization,

        robot_state_publisher,
        push_namespace,

        delay_serial,
        delay_ballistic,
        delay_main_camera_detector,
        delay_blind_camera_detector,
        delay_fusion,
        delay_gimbal_pipeline,
    ])
