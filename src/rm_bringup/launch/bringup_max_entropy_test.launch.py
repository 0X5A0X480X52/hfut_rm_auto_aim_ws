#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Max Entropy Tracker 测试启动文件

数据流:
  装甲板检测 → max_entropy_tracker → target_selector → gimbal_controller
                                                              ↓
                                                     ballistic_solver (服务)

启动方式:
    ros2 launch rm_bringup bringup_max_entropy_test.launch.py

带参数启动:
    ros2 launch rm_bringup bringup_max_entropy_test.launch.py image_source:=video virtual_serial:=true debug:=true
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from launch.substitutions import LaunchConfiguration
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch import LaunchDescription

    # 尝试加载启动参数配置
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

    # 声明启动参数
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

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=launch_params.get('namespace', ''),
        description='Namespace for all nodes'
    )

    # URDF 机器人描述
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_gimbal_description, value_type=str),
                    'publish_frequency': 1000.0}]
    )

    def get_bringup_params(name):
        """获取 rm_bringup 中的参数文件路径"""
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    def get_pkg_params(pkg_name, param_file):
        """获取各包中的参数文件路径"""
        return os.path.join(get_package_share_directory(pkg_name), 'config', param_file)

    # ==================== 装甲板检测节点 ====================
    armor_detector_node = ComposableNode(
        package='armor_detector',
        plugin='fyt::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[get_bringup_params('armor_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ==================== Max Entropy Tracker 节点 ====================
    max_entropy_tracker_node = Node(
        package='max_entropy_tracker',
        executable='max_entropy_tracker_node',
        name='max_entropy_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[
            get_pkg_params('max_entropy_tracker', 'tracker_params.yaml'),
            {'debug_mode': LaunchConfiguration('debug')}
        ],
        remappings=[
            # 订阅装甲板检测结果
            ('/armor_detector/armors', '/armor_detector/armors'),
        ]
    )

    # ==================== 目标选择器 (target_selector) ====================
    target_selector_node = Node(
        package='target_selector',
        executable='target_selector_node',
        name='target_selector',
        output='screen',
        emulate_tty=True,
        parameters=[
            get_pkg_params('target_selector', 'target_selector.yaml'),
            {
                'debug': LaunchConfiguration('debug'),
                'topics.robots_sub': '/max_entropy_tracker/tracked_robots',
            }
        ],
        remappings=[
            # 订阅 TrackedRobots 从 max_entropy_tracker
            ('/robot_pose_estimator/robots', '/max_entropy_tracker/tracked_robots'),
        ]
    )

    # ==================== 弹道解算服务 (ballistic_solver) ====================
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node_exe',
        name='ballistic_solver',
        output='screen',
        emulate_tty=True,
        parameters=[
            get_pkg_params('ballistic_solver', 'ballistic_solver.yaml'),
        ],
    )

    # ==================== 云台控制器 (gimbal_controller) 改为独立节点以排查问题 ====================
    gimbal_controller_node = Node(
        package='gimbal_controller',
        executable='gimbal_controller_node',
        name='gimbal_controller',
        output='both',
        emulate_tty=True,
        parameters=[
            get_pkg_params('gimbal_controller', 'gimbal_controller.yaml'),
            {
                'strategy': 'current',
                'ballistic_mode': 'service',
                'bullet_speed': 20.0,
            }
        ],
        remappings=[
            ('tracked_robots', '/max_entropy_tracker/tracked_robots'),
            ('selected_target', '/target_selector/selected_target'),
            ('cmd_gimbal', '/armor_solver/cmd_gimbal'),
        ],
    )

    # ==================== 使用组件容器提高图像传输效率 ====================
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
                name='camera_driver',
                parameters=[get_bringup_params('camera_driver')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        elif image_source == 'hik':
            image_node = ComposableNode(
                package='ros2_hik_camera',
                plugin='ros2_hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[get_bringup_params('camera_driver')],
                extra_arguments=[{'use_intra_process_comms': True}]
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
            composable_node_descriptions=[
                image_node,
                armor_detector_node,
            ],
            output='both',
            emulate_tty=True,
        )
        return [container]

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
    # 串口节点延迟 1.5 秒启动
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=create_serial_node_action)],
    )

    # 弹道解算服务延迟 2.0 秒启动 (作为服务需要先准备好)
    delay_ballistic_solver = TimerAction(
        period=2.0,
        actions=[ballistic_solver_node],
    )

    # 相机和检测容器延迟 2.0 秒启动
    delay_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_camera_detector_container)],
    )

    # Max Entropy Tracker 延迟 2.5 秒启动 (等待检测器准备好)
    delay_max_entropy_tracker = TimerAction(
        period=2.5,
        actions=[max_entropy_tracker_node],
    )

    # 目标选择器延迟 3.0 秒启动 (等待跟踪器准备好)
    delay_target_selector = TimerAction(
        period=3.0,
        actions=[target_selector_node],
    )

    # 云台控制器延迟 3.5 秒启动 (等待目标选择器和弹道解算服务准备好)
    delay_gimbal_controller = TimerAction(
        period=3.5,
        actions=[gimbal_controller_node],
    )

    # ==================== 命名空间 ====================
    push_namespace = PushRosNamespace(LaunchConfiguration('namespace'))

    # ==================== 构建启动描述 ====================
    return LaunchDescription([
        # 声明参数
        declare_image_source,
        declare_virtual_serial,
        declare_debug,
        declare_namespace,

        # 机器人描述发布
        robot_gimbal_publisher,

        # 命名空间
        push_namespace,

        # 串口节点 (延迟启动)
        delay_serial_node,

        # 弹道解算服务 (延迟启动, 先于其他需要它的节点)
        delay_ballistic_solver,

        # 相机和检测容器 (延迟启动)
        delay_camera_detector,

        # === Max Entropy Tracker 测试节点链 ===
        # max_entropy_tracker (延迟启动)
        delay_max_entropy_tracker,

        # target_selector (延迟启动)
        delay_target_selector,

        # gimbal_controller (延迟启动)
        delay_gimbal_controller,
    ])
