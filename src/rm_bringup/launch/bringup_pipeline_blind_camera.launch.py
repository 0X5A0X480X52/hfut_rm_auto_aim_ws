#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GimbalPipeline with main camera and blind camera (no camera intrinsics).

Pipeline:
  main_camera (hik/mindvision + armor_detector) --\
                                                     +--> gimbal_pipeline
  blind_camera_1 (usb_camera + blind_detector) -----/

Topic naming:
  Main camera (no namespace):
    - /image_raw
    - /camera_info
    - /armor_detector/armors

  Blind detector (namespace blind_camera_1):
    - Subscribes to: image_raw → /blind_camera_1/image_raw
    - Publishes to:  blinds   → /blind_camera_1/blinds

gimbal_pipeline subscribes to:
  - armors -> /armor_detector/armors  (main camera with PnP)
  - blinds -> /blind_camera_1/blinds  (blind camera, no PnP, via blind.topics param)
"""

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():
    # Load launch params from launch_params_decoupled.yaml
    try:
        launch_params = yaml.safe_load(open(os.path.join(
            get_package_share_directory('rm_bringup'), 'config', 'launch_params_decoupled.yaml')))
    except Exception:
        launch_params = {
            'image_source': 'video',
            'virtual_serial': True,
            'namespace': '',
            'odom2camera': {
                'xyz': '0 0 0',
                'rpy': '0 0 0'
            }
        }

    # ── 补盲相机在 gimbal_link 坐标系中的安装位姿 ──
    # 直接定义在 launch.py 中，不再从 blind_camera_*_params.yaml 读取，
    # 保证与 BlindDetector 中的偏移参数同源一致。
    blind_camera_xyz_default = '-0.175 0.0 0.086'
    blind_camera_rpy_default = '0.0 0.14 3.14159'
    blind_camera_2_xyz_default = '-0.175 0.05 0.086'
    blind_camera_2_rpy_default = '0.0 0.14 1.5708'
    blind_camera_3_xyz_default = '-0.175 -0.05 0.086'
    blind_camera_3_rpy_default = '0.0 0.14 -1.5708'

    # ── 补盲相机开关 (False 则不启动，可用于调试/屏蔽故障相机) ──
    enable_blind_camera_1 = True
    enable_blind_camera_2 = True
    enable_blind_camera_3 = True

    main_camera_xyz = launch_params.get('odom2camera', {}).get('xyz', '0.174275 0.000 0.086463')
    main_camera_rpy = launch_params.get('odom2camera', {}).get('rpy', '0.0 0.1396 -0.00')
    blind_camera_xyz = blind_camera_xyz_default
    blind_camera_rpy = blind_camera_rpy_default
    blind_camera_2_xyz = blind_camera_2_xyz_default
    blind_camera_2_rpy = blind_camera_2_rpy_default
    blind_camera_3_xyz = blind_camera_3_xyz_default
    blind_camera_3_rpy = blind_camera_3_rpy_default

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
        default_value=str(launch_params.get('image_source', 'video')),
        description='Image source: video | mindvision | hik'
    )
    declare_virtual_serial = DeclareLaunchArgument(
        'virtual_serial',
        default_value=str(launch_params.get('virtual_serial', True)).lower(),
        description='Use virtual serial instead of real serial'
    )
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug mode for all nodes'
    )
    declare_enable_blind = DeclareLaunchArgument(
        'enable_blind',
        default_value='true',
        description='下位机补盲功能开关 (true=启用, false=关闭). gimbal_pipeline 据此填 cmd.distance 哨兵值.'
    )
    declare_blind_camera_xyz = DeclareLaunchArgument(
        'blind_camera_xyz',
        default_value=blind_camera_xyz_default,
        description='补盲相机在 gimbal_link 坐标系中的安装位置 (xyz, m)'
    )
    declare_blind_camera_rpy = DeclareLaunchArgument(
        'blind_camera_rpy',
        default_value=blind_camera_rpy_default,
        description='补盲相机1在 gimbal_link 坐标系中的安装姿态 (rpy, rad)'
    )
    declare_blind_camera_2_xyz = DeclareLaunchArgument(
        'blind_camera_2_xyz',
        default_value=blind_camera_2_xyz_default,
        description='补盲相机2在 gimbal_link 坐标系中的安装位置 (xyz, m)'
    )
    declare_blind_camera_2_rpy = DeclareLaunchArgument(
        'blind_camera_2_rpy',
        default_value=blind_camera_2_rpy_default,
        description='补盲相机2在 gimbal_link 坐标系中的安装姿态 (rpy, rad)'
    )
    declare_blind_camera_3_xyz = DeclareLaunchArgument(
        'blind_camera_3_xyz',
        default_value=blind_camera_3_xyz_default,
        description='补盲相机3在 gimbal_link 坐标系中的安装位置 (xyz, m)'
    )
    declare_blind_camera_3_rpy = DeclareLaunchArgument(
        'blind_camera_3_rpy',
        default_value=blind_camera_3_rpy_default,
        description='补盲相机3在 gimbal_link 坐标系中的安装姿态 (rpy, rad)'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=launch_params.get('namespace', ''),
        description='Namespace for all nodes'
    )

    # ── URDF 机器人描述 (含补盲相机) ──
    # main_camera 参数来自 YAML 包含转义引号, blind_camera 参数为纯 Python 字符串。
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal_with_blind_camera.urdf.xacro'),
        ' main_camera_xyz:=', main_camera_xyz,
        ' main_camera_rpy:=', main_camera_rpy,
        ' blind_camera_1_xyz:="', blind_camera_xyz, '"',
        ' blind_camera_1_rpy:="', blind_camera_rpy, '"',
        ' blind_camera_2_xyz:="', blind_camera_2_xyz, '"',
        ' blind_camera_2_rpy:="', blind_camera_2_rpy, '"',
        ' blind_camera_3_xyz:="', blind_camera_3_xyz, '"',
        ' blind_camera_3_rpy:="', blind_camera_3_rpy, '"'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_gimbal_description, value_type=str),
            'publish_frequency': 1000.0
        }]
    )

    # ==================== 装甲板检测节点 (ComposableNode) ====================
    armor_detector_node = ComposableNode(
        package='armor_detector',
        plugin='fyt::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[get_bringup_params('armor_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ==================== 弹道解算服务 ====================
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node_exe',
        name='ballistic_solver',
        output='screen',
        emulate_tty=True,
        parameters=[get_pkg_params('ballistic_solver', 'ballistic_solver.yaml')],
    )

    # ==================== GimbalPipeline 统一节点 ====================
    gimbal_pipeline_node = Node(
        package='gimbal_pipeline',
        executable='gimbal_pipeline_node',
        name='gimbal_pipeline',
        output='both',
        emulate_tty=True,
        parameters=[
            get_pkg_params('gimbal_pipeline', 'gimbal_pipeline.yaml'),
            {
                'debug_mode': LaunchConfiguration('debug'),
                'enable_blind': LaunchConfiguration('enable_blind'),
                'blind.topics': [
                    '/blind_camera_1/blinds',
                    '/blind_camera_2/blinds',
                    '/blind_camera_3/blinds'
                ],
            },
        ],
        remappings=[
            ('cmd_gimbal', '/armor_solver/cmd_gimbal'),
            ('armors', '/armor_detector/armors'),
        ],
    )

    # ==================== 主相机 + 检测器 容器 ====================
    def create_camera_detector_container(context):
        image_source = LaunchConfiguration('image_source').perform(context)
        image_source = image_source.lower() if image_source else 'video'

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

        container = ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[image_node, armor_detector_node],
            output='both',
            emulate_tty=True,
        )
        return [container]

    # ==================== 补盲相机 + 补盲检测器 容器 ====================
    # Factory function — 返回一个 OpaqueFunction 兼容的创建函数
    def make_blind_camera_container_func(camera_prefix, container_name):
        def _create(context):
            debug_enabled = LaunchConfiguration('debug').perform(context).lower() == 'true'

            blind_camera_node = ComposableNode(
                package='usb_camera_driver',
                plugin='blind_vision::USBCameraNode',
                name='usb_camera_node',
                namespace=camera_prefix,
                parameters=[get_usb_camera_params(camera_prefix)],
                extra_arguments=[{'use_intra_process_comms': True}],
            )

            # Blind detector (no PnP, only estimates yaw/pitch from pixel positions)
            # 相机朝向直接通过 TF 查询 camera_optical_frame 获取，无需安装偏移参数
            blind_detector_node = ComposableNode(
                package='blind_detector',
                plugin='fyt::auto_aim::ArmorDetectorNode',
                name='blind_detector',
                namespace=camera_prefix,
                parameters=[
                    get_bringup_params('armor_detector'),
                    {
                        'camera_frame_id': f'{camera_prefix}_optical_frame',
                        'debug': debug_enabled,
                    },
                ],
                remappings=[
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )

            container = ComposableNodeContainer(
                name=container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[blind_camera_node, blind_detector_node],
                output='both',
                emulate_tty=True,
            )
            return [container]
        return _create

    # ==================== 串口节点 ====================
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

    # ==================== 延迟启动配置 ====================
    delay_serial = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=create_serial_node_action)],
    )
    delay_ballistic = TimerAction(
        period=2.0,
        actions=[ballistic_solver_node],
    )
    delay_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_camera_detector_container)],
    )
    delay_blind_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=make_blind_camera_container_func(
            'blind_camera_1', 'blind_camera_detector_container'))],
    )
    delay_blind_camera_detector_2 = TimerAction(
        period=2.5,
        actions=[OpaqueFunction(function=make_blind_camera_container_func(
            'blind_camera_2', 'blind_camera_detector_container_2'))],
    )
    delay_blind_camera_detector_3 = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=make_blind_camera_container_func(
            'blind_camera_3', 'blind_camera_detector_container_3'))],
    )
    delay_gimbal_pipeline = TimerAction(
        period=2.5,
        actions=[gimbal_pipeline_node],
    )

    # ==================== 命名空间 ====================
    push_namespace = PushRosNamespace(LaunchConfiguration('namespace'))

    # ==================== 构建启动描述 ====================
    ld_actions = [
        declare_image_source,
        declare_virtual_serial,
        declare_debug,
        declare_enable_blind,
        declare_blind_camera_xyz,
        declare_blind_camera_rpy,
        declare_blind_camera_2_xyz,
        declare_blind_camera_2_rpy,
        declare_blind_camera_3_xyz,
        declare_blind_camera_3_rpy,
        declare_namespace,

        robot_state_publisher,
        push_namespace,

        delay_serial,
        delay_ballistic,
        delay_camera_detector,
    ]

    # 按开关加入补盲相机（调试时可直接屏蔽故障相机）
    if enable_blind_camera_1:
        ld_actions.append(delay_blind_camera_detector)
    if enable_blind_camera_2:
        ld_actions.append(delay_blind_camera_detector_2)
    if enable_blind_camera_3:
        ld_actions.append(delay_blind_camera_detector_3)

    ld_actions.append(delay_gimbal_pipeline)

    return LaunchDescription(ld_actions)
