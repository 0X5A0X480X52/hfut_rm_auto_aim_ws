import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动双相机的 launch 文件
    使用前请在 dual_camera_params.yaml 中配置正确的相机序列号
    """
    
    params_file = os.path.join(
        get_package_share_directory('ros2_hik_camera'), 'config', 'dual_camera_params.yaml')

    # 左相机配置
    camera_info_url_left = 'package://ros2_hik_camera/config/camera_info.yaml'
    
    # 右相机配置 (如果有不同的标定文件可以指定)
    camera_info_url_right = 'package://ros2_hik_camera/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file,
                              description='双相机参数配置文件路径'),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false',
                              description='是否使用 SensorDataQoS'),
        
        # 左相机节点
        Node(
            package='ros2_hik_camera',
            executable='ros2_hik_camera_node',
            name='hik_camera',
            namespace='camera_left',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': camera_info_url_left,
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),
        
        # 右相机节点
        Node(
            package='ros2_hik_camera',
            executable='ros2_hik_camera_node',
            name='hik_camera',
            namespace='camera_right',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': camera_info_url_right,
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        )
    ])
