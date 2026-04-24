#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Video player + armor_detector test launch for blind camera pipeline.

Uses a video file instead of a real USB camera to test the armor detection pipeline.
Both nodes share the 'blind_camera_1' namespace so downstream nodes (e.g. armor_fusion)
see the same topic layout as the real blind_camera.launch.py.

Usage:
    ros2 launch usb_camera_driver video_player_test.launch.py
    ros2 launch usb_camera_driver video_player_test.launch.py video_path:=/path/to/video.avi
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ws_dir = get_package_share_directory("usb_camera_driver")
    # install/usb_camera_driver/share/usb_camera_driver -> workspace root (4 levels up)
    ws_root = os.path.abspath(os.path.join(ws_dir, "..", "..", "..", ".."))
    default_video = os.path.join(ws_root, "test_video", "output.avi")

    armor_detector_pkg = get_package_share_directory("armor_detector")

    declare_video_path = DeclareLaunchArgument(
        "video_path",
        default_value=default_video,
        description="Path to the video file for playback",
    )
    declare_fps = DeclareLaunchArgument(
        "fps",
        default_value="30.0",
        description="Playback frame rate",
    )
    declare_loop_playback = DeclareLaunchArgument(
        "loop_playback",
        default_value="true",
        description="Loop video playback when finished",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="blind_camera_1",
        description="Namespace for video player and detector nodes",
    )
    declare_debug = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug mode for armor_detector",
    )

    video_player_node = ComposableNode(
        package="video_player",
        plugin="video_player::VideoPlayerNode",
        name="video_player",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {
                "video_path": LaunchConfiguration("video_path"),
                "loop_playback": LaunchConfiguration("loop_playback"),
                "fps": LaunchConfiguration("fps"),
                "camera_name": LaunchConfiguration("namespace"),
                "flip_image": False,
                "image_topic": "image_raw",
                "use_sensor_data_qos": False,
                "camera_info_url": "package://rm_bringup/config/camera_info.yaml",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="fyt::auto_aim::ArmorDetectorNode",
        name="armor_detector",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            os.path.join(armor_detector_pkg, "config", "armor_detector.yaml"),
            {"debug": LaunchConfiguration("debug")},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="video_test_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[video_player_node, armor_detector_node],
        output="both",
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_video_path,
        declare_fps,
        declare_loop_playback,
        declare_namespace,
        declare_debug,
        container,
    ])
