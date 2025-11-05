import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    列出所有检测到的相机信息的 launch 文件
    运行此文件可以查看所有连接的相机及其序列号
    """
    
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://mindvision_camera/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        
        # 不指定序列号，节点会列出所有相机信息并使用第一个
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        )
    ])
