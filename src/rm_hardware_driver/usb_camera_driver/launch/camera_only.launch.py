#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch only the USB camera node (no armor_detector).
Publishes: <namespace>/image_raw, <namespace>/camera_info

Usage:
    ros2 launch usb_camera_driver camera_only.launch.py
    ros2 launch usb_camera_driver camera_only.launch.py camera_name:=blind_camera_1
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory("usb_camera_driver")

    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="blind_camera_1",
        description="Camera name (used to find <name>_params.yaml)",
    )

    def camera_config(name):
        return PathJoinSubstitution([pkg_dir, "config", [name, "_params.yaml"]])

    camera_node = ComposableNode(
        package="usb_camera_driver",
        plugin="blind_vision::USBCameraNode",
        name="usb_camera",
        namespace=LaunchConfiguration("camera_name"),
        parameters=[camera_config(LaunchConfiguration("camera_name"))],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[camera_node],
        output="both",
        emulate_tty=True,
    )

    republish_node = Node(
        package="image_transport",
        executable="republish",
        name="image_republish",
        namespace=LaunchConfiguration("camera_name"),
        arguments=["raw", "compressed"],
        remappings=[
            ("in/raw", "image_raw"),
            ("out", "image_raw"),
        ],
    )

    return LaunchDescription([
        declare_camera_name,
        container,
        republish_node,
    ])
