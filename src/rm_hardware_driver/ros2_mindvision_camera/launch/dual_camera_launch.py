import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动双相机的 launch 文件
    使用前请在 dual_camera_params.yaml 中配置正确的相机序列号
    """
    
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'dual_camera_params.yaml')

        # 默认标定文件
    default_camera_info_left = 'package://mindvision_camera/config/camera_info.yaml'
    default_camera_info_right = 'package://mindvision_camera/config/camera_info.yaml'

    # 尝试从 YAML 文件中读取左右相机的 camera_info_url
    camera_info_left = default_camera_info_left
    camera_info_right = default_camera_info_right
    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
            camera_info_left = params.get('/camera_left', {}) \
                                     .get('mv_camera', {}) \
                                     .get('ros__parameters', {}) \
                                     .get('camera_info_url', default_camera_info_left)
            camera_info_right = params.get('/camera_right', {}) \
                                      .get('mv_camera', {}) \
                                      .get('ros__parameters', {}) \
                                      .get('camera_info_url', default_camera_info_right)
    except Exception as e:
        print(f"[Warning] 读取参数文件 {params_file} 失败，将使用默认 camera_info。Error: {e}")

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file,
                              description='双相机参数配置文件路径'),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false',
                              description='是否使用 SensorDataQoS'),
        
        # 左相机节点
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            name='mv_camera',
            namespace='camera_left',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': camera_info_left,
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),
        
        # 右相机节点 - 延迟2秒启动以避免USB带宽冲突
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='mindvision_camera',
                    executable='mindvision_camera_node',
                    name='mv_camera',
                    namespace='camera_right',
                    output='screen',
                    emulate_tty=True,
                    parameters=[LaunchConfiguration('params_file'), {
                        'camera_info_url': camera_info_right,
                        'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                    }],
                )
            ]
        )
    ])
