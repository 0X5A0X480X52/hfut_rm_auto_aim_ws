#!/usr/bin/env python3
"""
Launch multi-camera system with two video_player nodes as simulated cameras.

This launch starts two instances of the `video_player` node (left/right) and
places them under the `camera1` and `camera2` namespaces respectively so that
`multi_camera_system.launch.py` (which expects `/camera1/...` and
`/camera2/...`) can consume the streams as if they were real cameras.

可通过 launch 参数覆盖左右两路的 params 文件路径（默认使用 video_player/config/dual_01）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # package share locations
    video_pkg = FindPackageShare('video_player')
    fusion_pkg = FindPackageShare('armor_fusion')

    default_params_dir = PathJoinSubstitution([video_pkg, 'config', 'dual_01'])

    declare_params_left = DeclareLaunchArgument(
        'params_file_left',
        default_value=PathJoinSubstitution([video_pkg, 'config', 'dual_01', 'params_left.yaml']),
        description='Params file for left video_player (will be launched in namespace camera1)'
    )

    declare_params_right = DeclareLaunchArgument(
        'params_file_right',
        default_value=PathJoinSubstitution([video_pkg, 'config', 'dual_01', 'params_right.yaml']),
        description='Params file for right video_player (will be launched in namespace camera2)'
    )

    # Optional: allow overriding fps/loop via launch args if desired
    declare_fps = DeclareLaunchArgument('fps', default_value='30.0', description='Playback frame rate')
    declare_loop = DeclareLaunchArgument('loop_playback', default_value='true', description='Loop playback')
    # Allow overriding the video file paths (defaults to workspace test video)
    declare_video_left = DeclareLaunchArgument(
        'video_path_left',
        default_value='/home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws/test_video/output.avi',
        description='Video file for left (camera1)'
    )
    declare_video_right = DeclareLaunchArgument(
        'video_path_right',
        default_value='/home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws/test_video/output.avi',
        description='Video file for right (camera2)'
    )

    # Optionally start a virtual serial node (similar to rm_bringup)
    declare_virtual_serial = DeclareLaunchArgument('use_virtual_serial', default_value='true', description='Start virtual serial node from rm_serial_driver')

    # Left (camera1) video_player node
    left_player = Node(
        package='video_player',
        executable='video_player_node',
        name='video_player_left',
        namespace='camera1',
        output='screen',
        emulate_tty=True,
        # load params file then override image_topic and camera_name so topics become /camera1/image_raw and /camera1/camera_info
    parameters=[LaunchConfiguration('params_file_left'),
            {'video_path': LaunchConfiguration('video_path_left'),
             'image_topic': 'image_raw', 'camera_name': 'camera1',
             'fps': LaunchConfiguration('fps'), 'loop_playback': LaunchConfiguration('loop_playback')}]
    )

    # Right (camera2) video_player node
    right_player = Node(
        package='video_player',
        executable='video_player_node',
        name='video_player_right',
        namespace='camera2',
        output='screen',
        emulate_tty=True,
    parameters=[LaunchConfiguration('params_file_right'),
            {'video_path': LaunchConfiguration('video_path_right'),
             'image_topic': 'image_raw', 'camera_name': 'camera2',
             'fps': LaunchConfiguration('fps'), 'loop_playback': LaunchConfiguration('loop_playback')}]
    )

    # helper to find params file in rm_bringup
    def get_params(name: str) -> str:
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', f'{name}_params.yaml')

    # virtual serial node (delayed start)
    virtual_serial_node = Node(
        package='rm_serial_driver',
        executable='virtual_serial_node',
        name='virtual_serial',
        output='both',
        emulate_tty=True,
        parameters=[get_params('virtual_serial')],
    )

    delay_virtual_serial = TimerAction(period=1.5, actions=[virtual_serial_node])

    # Include the existing multi-camera system launch
    include_multi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fusion_pkg, 'launch', 'multi_camera_system.launch.py'])
        ),
        # Ensure multi-camera system enables both cameras by default
        launch_arguments={'enable_camera1': 'true', 'enable_camera2': 'true'}.items()
    )

    ld = LaunchDescription()
    for decl in [declare_params_left, declare_params_right, declare_fps, declare_loop, declare_video_left, declare_video_right]:
        ld.add_action(decl)

    # Add video players first so topics exist when detectors start
    ld.add_action(left_player)
    ld.add_action(right_player)

    # Publish static transforms from base_link -> camera*_optical_frame so
    # detector (which performs TF lookups) can find transforms during testing.
    # We place small lateral offsets for the two cameras to avoid identical poses.
    camera1_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera1_static_tf',
        arguments=['0.1', '0.0', '0.0', '0', '0', '0', 'odom', 'camera1_optical_frame'],
        output='screen'
    )

    camera2_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera2_static_tf',
        arguments=['-0.1', '0.0', '0.0', '0', '0', '0', 'odom', 'camera2_optical_frame'],
        output='screen'
    )

    ld.add_action(camera1_static_tf)
    ld.add_action(camera2_static_tf)

    # Delay starting virtual serial to allow other infra to initialize
    ld.add_action(delay_virtual_serial)

    # Then launch the multi-camera system (detectors, fusion, solver)
    ld.add_action(include_multi)

    return ld
