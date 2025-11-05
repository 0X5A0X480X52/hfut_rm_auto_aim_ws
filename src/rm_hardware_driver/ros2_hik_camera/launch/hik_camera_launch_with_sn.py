import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    单相机启动文件，支持通过命令行参数指定序列号
    
    使用示例:
    # 不指定序列号，使用第一个检测到的相机
    ros2 launch ros2_hik_camera hik_camera_launch_with_sn.py
    
    # 指定序列号
    ros2 launch ros2_hik_camera hik_camera_launch_with_sn.py camera_sn:=A1B2C3D4E5F6
    """
    
    params_file = os.path.join(
        get_package_share_directory('ros2_hik_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://ros2_hik_camera/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=params_file,
            description='相机参数文件路径'
        ),
        DeclareLaunchArgument(
            name='camera_info_url',
            default_value=camera_info_url,
            description='相机内参文件路径'
        ),
        DeclareLaunchArgument(
            name='use_sensor_data_qos',
            default_value='false',
            description='是否使用 SensorDataQoS'
        ),
        DeclareLaunchArgument(
            name='camera_sn',
            default_value='',
            description='相机序列号（留空使用第一个相机）'
        ),
        
        Node(
            package='ros2_hik_camera',
            executable='ros2_hik_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'camera_info_url': LaunchConfiguration('camera_info_url'),
                    'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                    'camera_sn': LaunchConfiguration('camera_sn'),
                }
            ],
        )
    ])
