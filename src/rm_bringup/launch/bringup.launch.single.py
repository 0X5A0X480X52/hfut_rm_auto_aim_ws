import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    # 设置参数
    SetParameter(name='rune', value=launch_params['rune']),
    
    # 机器人描述
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
    robot_navigation_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

    # 发布机器人状态
    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                     'publish_frequency': 1000.0}]
    )
    
    robot_navigation_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_navigation_description,}]
    )

    # 获取参数的路径
    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    # 图像
    if launch_params['video_play']: 
        image_node  = Node(
            package='mindvision_camera',
            executable='video_player_node',
            name='video_player',
            parameters=[get_params('video_player')],
            output='both',
        )
    else:
         image_node  = Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            name='camera_driver',
            parameters=[get_params('camera_driver')],
            output='both',
        )

    # 串口驱动
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[get_params('virtual_serial')],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('serial_driver')],
        )
        
    # 装甲板识别节点
    armor_detector_node = Node(
        package='armor_detector', 
        executable='armor_detector_node',
        name='armor_detector',
        parameters=[get_params('armor_detector')],
        output='both',
    )
    
    # 装甲板解算
    if launch_params['hero_solver']:
        armor_solver_node = Node(
            package='hero_armor_solver',
            executable='hero_armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
        )
    else:
        armor_solver_node = Node(
            package='armor_solver',
            executable='armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
        )

    # 延迟启动串口驱动节点
    delay_serial_node = TimerAction(
        period=3.0,  # 增加延迟时间
        actions=[serial_driver_node],
    )

    # 延迟启动装甲板解算节点
    delay_armor_solver_node = TimerAction(
        period=3.0,  # 增加延迟时间
        actions=[armor_solver_node],
    )
    
    # 延迟启动图像检测和装甲板识别节点
    delay_image_node = TimerAction(
        period=3.0,  # 增加延迟时间
        actions=[image_node],
    )

    delay_armor_detector_node = TimerAction(
        period=3.0,  # 增加延迟时间
        actions=[armor_detector_node],
    )

    # 将所有节点按顺序添加到 launch_description_list
    launch_description_list = [
        robot_gimbal_publisher,
        push_namespace := PushRosNamespace(launch_params['namespace']),
        delay_serial_node,
        delay_image_node,
        delay_armor_detector_node,
        delay_armor_solver_node
    ]

    if launch_params['navigation']:
        launch_description_list.append(robot_navigation_publisher)

    # 返回 LaunchDescription 对象
    return LaunchDescription(launch_description_list)
