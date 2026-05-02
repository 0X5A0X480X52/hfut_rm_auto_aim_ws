#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Blind camera driver launch file.

Launches USB camera node and armor_detector for blind spot detection.
Both nodes run in the same component container with intra-process communication.

Usage:
    ros2 launch usb_camera_driver blind_camera.launch.py
    ros2 launch usb_camera_driver blind_camera.launch.py camera_name:=blind_camera_1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory("usb_camera_driver")
    bringup_pkg = get_package_share_directory("rm_bringup")

    # Declare launch arguments
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="blind_camera_1",
        description="Camera name for blind spot detection",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="blind_camera_1",
        description="Namespace for the camera and detector nodes",
    )
    declare_debug = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug mode for blind_detector",
    )

    def get_camera_config(name):
        return PathJoinSubstitution([pkg_dir, "config", [name, "_params.yaml"]])

    # Blind camera node
    blind_camera_node = ComposableNode(
        package="usb_camera_driver",
        plugin="blind_vision::USBCameraNode",
        name="usb_camera",
        namespace=LaunchConfiguration("namespace"),
        parameters=[get_camera_config(LaunchConfiguration("camera_name"))],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Blind detector node (light detection, no PnP)
    # Subscribes to: <namespace>/image_raw
    # Publishes to:  /blind_detector/blind (after remap)
    blind_detector_node = ComposableNode(
        package="blind_detector",
        plugin="fyt::auto_aim::ArmorDetectorNode",
        name="blind_detector",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            os.path.join(bringup_pkg, "config", "node_params", "armor_detector_params.yaml"),
            {
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_yaw": 180.0,
                "camera_pitch": 8.0,
                "h_fov": 60.0,
                "v_fov": 45.0,
                "debug": LaunchConfiguration("debug"),
            },
        ],
        remappings=[
            ([LaunchConfiguration("camera_name"), "_image_raw"], "image_raw"),
            (["blind_detector/", LaunchConfiguration("camera_name"), "/blind"], "/blind_detector/blind"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Container for blind camera + blind detector
    blind_camera_container = ComposableNodeContainer(
        name="blind_camera_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[blind_camera_node, blind_detector_node],
        output="both",
        emulate_tty=True,
    )

    # image_transport republish: adds compressed topic for foxglove visualization
    republish_node = Node(
        package="image_transport",
        executable="republish",
        name="image_republish",
        namespace=LaunchConfiguration("namespace"),
        arguments=["raw", "compressed"],
        remappings=[
            ("in/raw", "image_raw"),
            ("out", "image_raw"),
        ],
    )

    return LaunchDescription(
        [
            declare_camera_name,
            declare_namespace,
            declare_debug,
            blind_camera_container,
            republish_node,
        ]
    )
