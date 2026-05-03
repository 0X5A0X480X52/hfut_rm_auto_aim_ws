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
from launch_ros.actions import ComposableNodeContainer
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
    # Subscribes to: <namespace>/image_raw  (relative topic, resolved by namespace)
    # Publishes to:  <namespace>/blinds     (relative topic, resolved by namespace)
    blind_detector_node = ComposableNode(
        package="blind_detector",
        plugin="fyt::auto_aim::ArmorDetectorNode",
        name="blind_detector",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            os.path.join(bringup_pkg, "config", "node_params", "armor_detector_params.yaml"),
            {
                "h_fov": 60.0,
                "v_fov": 45.0,
                "debug": LaunchConfiguration("debug"),
            },
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

    return LaunchDescription(
        [
            declare_camera_name,
            declare_namespace,
            declare_debug,
            blind_camera_container,
        ]
    )
