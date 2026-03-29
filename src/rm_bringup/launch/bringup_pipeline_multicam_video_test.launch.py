#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Multi-camera pipeline test launch (video + virtual serial).

Pipeline:
  camera1(video_player + armor_detector) --\
                                          +--> armor_fusion --> /armor_detector/armors --> gimbal_pipeline --> /armor_solver/cmd_gimbal
  camera2(video_player + armor_detector) --/

This launch is test-only and does not modify the original bringup_pipeline.launch.py.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    default_video = "/home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws/test_video/output.avi"

    launch_params_path = os.path.join(
        get_package_share_directory("rm_bringup"),
        "config",
        "launch_params_multicam_video_test.yaml",
    )
    with open(launch_params_path, "r", encoding="utf-8") as f:
        launch_params = yaml.safe_load(f) or {}

    gimbal_pose = launch_params.get("gimbal", {})
    camera1_pose = launch_params.get("camera1", {})
    camera2_pose = launch_params.get("camera2", {})

    gimbal_xyz = str(gimbal_pose.get("xyz", "0 0 0"))
    gimbal_rpy = str(gimbal_pose.get("rpy", "0 0 0"))
    camera1_xyz = str(camera1_pose.get("xyz", "0.05 0.06 0.05"))
    camera1_rpy = str(camera1_pose.get("rpy", "0 0 0"))
    camera2_xyz = str(camera2_pose.get("xyz", "0.05 -0.06 0.05"))
    camera2_rpy = str(camera2_pose.get("rpy", "0 0 3.1415926"))

    def get_bringup_params(name):
        return os.path.join(
            get_package_share_directory("rm_bringup"),
            "config",
            "node_params",
            f"{name}_params.yaml",
        )

    def get_pkg_params(pkg_name, param_file):
        return os.path.join(get_package_share_directory(pkg_name), "config", param_file)

    declare_virtual_serial = DeclareLaunchArgument(
        "virtual_serial",
        default_value="true",
        description="Use virtual serial node",
    )
    declare_debug = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug mode",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all nodes",
    )
    declare_enable_visualization = DeclareLaunchArgument(
        "enable_visualization",
        default_value="true",
        description="Enable fusion markers",
    )
    declare_video_path_left = DeclareLaunchArgument(
        "video_path_left",
        default_value=default_video,
        description="Video path for camera1",
    )
    declare_video_path_right = DeclareLaunchArgument(
        "video_path_right",
        default_value=default_video,
        description="Video path for camera2",
    )
    declare_fps = DeclareLaunchArgument(
        "fps",
        default_value="30.0",
        description="Playback fps",
    )
    declare_loop_playback = DeclareLaunchArgument(
        "loop_playback",
        default_value="true",
        description="Loop video playback",
    )
    declare_robot_xacro = DeclareLaunchArgument(
        "robot_xacro",
        default_value=os.path.join(
            get_package_share_directory("rm_robot_description"),
            "urdf",
            "test_pipeline_dual_video.urdf.xacro",
        ),
        description="URDF xacro for dual-camera test",
    )
    declare_fusion_executable = DeclareLaunchArgument(
        "fusion_executable",
        default_value="armor_fusion_node",
        description="Fusion executable name",
    )

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        LaunchConfiguration("robot_xacro"),
        ' gimbal_xyz:="',
        gimbal_xyz,
        '"',
        ' gimbal_rpy:="',
        gimbal_rpy,
        '"',
        ' camera1_xyz:="',
        camera1_xyz,
        '"',
        ' camera1_rpy:="',
        camera1_rpy,
        '"',
        ' camera2_xyz:="',
        camera2_xyz,
        '"',
        ' camera2_rpy:="',
        camera2_rpy,
        '"',
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "publish_frequency": 1000.0,
            }
        ],
        output="screen",
    )

    ballistic_solver_node = Node(
        package="ballistic_solver",
        executable="ballistic_solver_node_exe",
        name="ballistic_solver",
        output="screen",
        emulate_tty=True,
        parameters=[get_pkg_params("ballistic_solver", "ballistic_solver.yaml")],
    )

    fusion_node = Node(
        package="armor_fusion",
        executable=LaunchConfiguration("fusion_executable"),
        name="multi_camera_fusion",
        output="screen",
        emulate_tty=True,
        parameters=[
            get_pkg_params("armor_fusion", "fusion_params.yaml"),
            {
                "camera_topics": [
                    "/camera1/armor_detector/armors",
                    "/camera2/armor_detector/armors",
                ],
                "camera_info_topics": [
                    "/camera1/camera_info",
                    "/camera2/camera_info",
                ],
                "target_frame": "odom",
                "output_topic": "/armor_detector/armors",
                "enable_visualization": LaunchConfiguration("enable_visualization"),
                "console_debug": LaunchConfiguration("debug"),
            },
        ],
    )

    gimbal_pipeline_node = Node(
        package="gimbal_pipeline",
        executable="gimbal_pipeline_node",
        name="gimbal_pipeline",
        output="both",
        emulate_tty=True,
        parameters=[
            get_pkg_params("gimbal_pipeline", "gimbal_pipeline.yaml"),
            {"debug_mode": LaunchConfiguration("debug")},
        ],
        remappings=[
            ("cmd_gimbal", "/armor_solver/cmd_gimbal"),
            ("camera_info", "/camera1/camera_info"),
        ],
    )

    def create_camera_detector_containers(context):
        fps = LaunchConfiguration("fps").perform(context)
        loop_playback = LaunchConfiguration("loop_playback").perform(context)
        video_path_left = LaunchConfiguration("video_path_left").perform(context)
        video_path_right = LaunchConfiguration("video_path_right").perform(context)
        debug_enabled = LaunchConfiguration("debug").perform(context).lower() == "true"

        cam1_video = ComposableNode(
            package="video_player",
            plugin="video_player::VideoPlayerNode",
            name="video_player",
            namespace="camera1",
            parameters=[
                {
                    "video_path": video_path_left,
                    "loop_playback": loop_playback.lower() == "true",
                    "fps": float(fps),
                    "camera_name": "camera1",
                    "flip_image": False,
                    "image_topic": "image_raw",
                    "use_sensor_data_qos": False,
                    "camera_info_url": "package://rm_bringup/config/camera_info.yaml",
                }
            ],
            remappings=[
                ("/image_raw", "image_raw"),
                ("/camera_info", "camera_info"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        cam1_detector = ComposableNode(
            package="armor_detector",
            plugin="fyt::auto_aim::ArmorDetectorNode",
            name="armor_detector",
            namespace="camera1",
            parameters=[
                get_bringup_params("armor_detector"),
                {"debug": debug_enabled},
            ],
            remappings=[
                ("/image_raw", "image_raw"),
                ("/camera_info", "camera_info"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        cam2_video = ComposableNode(
            package="video_player",
            plugin="video_player::VideoPlayerNode",
            name="video_player",
            namespace="camera2",
            parameters=[
                {
                    "video_path": video_path_right,
                    "loop_playback": loop_playback.lower() == "true",
                    "fps": float(fps),
                    "camera_name": "camera2",
                    "flip_image": False,
                    "image_topic": "image_raw",
                    "use_sensor_data_qos": False,
                    "camera_info_url": "package://rm_bringup/config/camera_info.yaml",
                }
            ],
            remappings=[
                ("/image_raw", "image_raw"),
                ("/camera_info", "camera_info"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        cam2_detector = ComposableNode(
            package="armor_detector",
            plugin="fyt::auto_aim::ArmorDetectorNode",
            name="armor_detector",
            namespace="camera2",
            parameters=[
                get_bringup_params("armor_detector"),
                {"debug": debug_enabled},
            ],
            remappings=[
                ("/image_raw", "image_raw"),
                ("/camera_info", "camera_info"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        cam1_container = ComposableNodeContainer(
            name="camera1_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[cam1_video, cam1_detector],
            output="both",
            emulate_tty=True,
        )

        cam2_container = ComposableNodeContainer(
            name="camera2_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[cam2_video, cam2_detector],
            output="both",
            emulate_tty=True,
        )

        return [cam1_container, cam2_container]

    def create_serial_node_action(context):
        virtual_serial = LaunchConfiguration("virtual_serial").perform(context).lower() == "true"
        if virtual_serial:
            return [
                Node(
                    package="rm_serial_driver",
                    executable="virtual_serial_node",
                    name="virtual_serial",
                    output="both",
                    emulate_tty=True,
                    parameters=[get_bringup_params("virtual_serial")],
                )
            ]

        return [
            Node(
                package="rm_serial_driver",
                executable="rm_serial_driver_node",
                name="serial_driver",
                output="both",
                emulate_tty=True,
                parameters=[get_bringup_params("serial_driver")],
            )
        ]

    delay_serial = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=create_serial_node_action)],
    )
    delay_ballistic = TimerAction(
        period=2.0,
        actions=[ballistic_solver_node],
    )
    delay_camera_detector = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_camera_detector_containers)],
    )
    delay_fusion = TimerAction(
        period=2.8,
        actions=[fusion_node],
    )
    delay_gimbal_pipeline = TimerAction(
        period=3.2,
        actions=[gimbal_pipeline_node],
    )

    push_namespace = PushRosNamespace(LaunchConfiguration("namespace"))

    return LaunchDescription(
        [
            declare_virtual_serial,
            declare_debug,
            declare_namespace,
            declare_enable_visualization,
            declare_video_path_left,
            declare_video_path_right,
            declare_fps,
            declare_loop_playback,
            declare_robot_xacro,
            declare_fusion_executable,
            robot_state_publisher,
            push_namespace,
            delay_serial,
            delay_ballistic,
            delay_camera_detector,
            delay_fusion,
            delay_gimbal_pipeline,
        ]
    )
