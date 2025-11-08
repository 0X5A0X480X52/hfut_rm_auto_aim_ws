import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('video_player')
    # default config directory (contains camera_info and params files)
    default_config_dir = os.path.join(pkg_dir, 'config', 'dual_01')

    # Launch arguments: left/right params files and generic overrides
    declare_params_left = DeclareLaunchArgument(
        'params_file_left',
        default_value=os.path.join(default_config_dir, 'params_left.yaml'),
        description='Params file for left video_player'
    )
    declare_params_right = DeclareLaunchArgument(
        'params_file_right',
        default_value=os.path.join(default_config_dir, 'params_right.yaml'),
        description='Params file for right video_player'
    )

    declare_fps = DeclareLaunchArgument('fps', default_value='30.0', description='Playback frame rate')
    declare_loop = DeclareLaunchArgument('loop_playback', default_value='true', description='Loop playback')
    declare_qos = DeclareLaunchArgument('use_sensor_data_qos', default_value='false', description='Use sensor data QoS')

    # default camera_info files
    default_caminfo_left = os.path.join(default_config_dir, 'camera_info_left.yaml')
    default_caminfo_right = os.path.join(default_config_dir, 'camera_info_right.yaml')

    # left node loads its params file
    left_node = Node(
        package='video_player',
        executable='video_player_node',
        name='video_player_left',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file_left')],
    )

    # right node loads its params file
    right_node = Node(
        package='video_player',
        executable='video_player_node',
        name='video_player_right',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file_right')],
    )

    ld = LaunchDescription()
    for decl in [declare_params_left, declare_params_right, declare_fps, declare_loop, declare_qos]:
        ld.add_action(decl)

    ld.add_action(left_node)
    ld.add_action(right_node)

    return ld
