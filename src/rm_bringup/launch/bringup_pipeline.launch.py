#!/usr/bin/env python3

"""Launch the supported camera-to-gimbal pipeline for a robot profile."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


SUPPORTED_PROFILES = ("hero", "infantry_4_1", "leg", "uav", "video_test")
IMAGE_SOURCES = ("video", "mindvision", "hik")
SERIAL_MODES = ("real", "virtual")
DETECTOR_TYPES = ("armor_detector", "armor_detector_nn")
COMPOSITIONS = ("standalone", "container")


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            line = key_node.start_mark.line + 1
            raise RuntimeError(
                f"Duplicate YAML key {key!r} in {loader.name}:{line}"
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_yaml(path):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError(f"Required configuration does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.load(stream, Loader=_UniqueKeyLoader)
    if not isinstance(data, dict):
        raise RuntimeError(f"Configuration must contain a YAML mapping: {path}")
    return data


def _flatten_parameters(parameters, prefix=""):
    flattened = {}
    for key, value in parameters.items():
        if not isinstance(key, str):
            raise RuntimeError(f"ROS parameter key must be a string: {key!r}")
        full_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flattened.update(_flatten_parameters(value, full_key))
        else:
            flattened[full_key] = value
    return flattened


def _ros_parameters(path):
    document = _load_yaml(path)
    blocks = []
    for node_selector, node_config in document.items():
        if not isinstance(node_config, dict) or "ros__parameters" not in node_config:
            continue
        parameters = node_config["ros__parameters"]
        if not isinstance(parameters, dict):
            raise RuntimeError(
                f"ros__parameters must be a mapping for {node_selector!r}: {path}"
            )
        blocks.append(parameters)
    if len(blocks) != 1:
        raise RuntimeError(
            f"Expected exactly one ros__parameters block in {path}, found {len(blocks)}"
        )
    return _flatten_parameters(blocks[0])


def _merge_parameters(*layers):
    merged = {}
    for layer in layers:
        merged.update(layer)
    return merged


def _resolve(context, argument, defaults, allowed=None):
    value = LaunchConfiguration(argument).perform(context).strip()
    if value == "auto":
        value = str(defaults[argument]).strip()
    if allowed is not None and value not in allowed:
        choices = ", ".join(allowed)
        raise RuntimeError(f"Invalid {argument}={value!r}; expected one of: {choices}")
    return value


def _resolve_switch(context, argument, defaults):
    value = _resolve(context, argument, defaults, ("on", "off"))
    return value == "on"


def _package_config(package_name, filename):
    path = Path(get_package_share_directory(package_name)) / "config" / filename
    if not path.is_file():
        raise RuntimeError(f"Required package configuration does not exist: {path}")
    return str(path)


def _package_parameters(package_name, filename):
    return _ros_parameters(_package_config(package_name, filename))


def _profile_config(profile_root, filename):
    path = profile_root / filename
    if not path.is_file():
        raise RuntimeError(f"Required profile configuration does not exist: {path}")
    return str(path)


def _profile_parameters(profile_root, filename):
    return _ros_parameters(_profile_config(profile_root, filename))


def _optional_profile_config(profile_root, filename):
    path = profile_root / filename
    return str(path) if path.is_file() else None


def _optional_profile_parameters(profile_root, filename):
    path = _optional_profile_config(profile_root, filename)
    return _ros_parameters(path) if path else {}


def _camera_description(image_source, profile_root, debug):
    if image_source == "video":
        return {
            "package": "video_player",
            "executable": "video_player_node",
            "plugin": "video_player::VideoPlayerNode",
            "name": "video_player",
            "parameters": [_profile_parameters(profile_root, "video_player_params.yaml")],
            "environment": {},
        }
    if image_source == "mindvision":
        return {
            "package": "mindvision_camera",
            "executable": "mindvision_camera_node",
            "plugin": "mindvision_camera::MVCameraNode",
            "name": "mv_camera",
            "parameters": [
                _profile_parameters(profile_root, "mindvision_camera_driver_params.yaml")
            ],
            "environment": {},
        }
    return {
        "package": "ros2_hik_camera",
        "executable": "ros2_hik_camera_node",
        "plugin": "ros2_hik_camera::HikCameraNode",
        "name": "hik_camera",
        "parameters": [_profile_parameters(profile_root, "hik_camera_driver_params.yaml")],
        "environment": {
            "MVCAM_SDK_PATH": "/opt/MVS",
            "MVCAM_COMMON_RUNENV": "/opt/MVS/lib",
        },
    }


def _detector_description(detector_type, profile_root, debug):
    if detector_type == "armor_detector_nn":
        parameters = _merge_parameters(
            _package_parameters("armor_detector_nn", "armor_detector_nn.yaml"),
            _optional_profile_parameters(profile_root, "armor_detector_nn.yaml"),
            {"debug": debug},
        )
        return {
            "package": "armor_detector_nn",
            "executable": "armor_detector_nn_node",
            "plugin": "fyt::auto_aim::ArmorDetectorNNNode",
            "parameters": [parameters],
        }
    return {
        "package": "armor_detector",
        "executable": "armor_detector_node",
        "plugin": "fyt::auto_aim::ArmorDetectorNode",
        "parameters": [
            _merge_parameters(
                _profile_parameters(profile_root, "armor_detector_params.yaml"),
                {"debug": debug},
            )
        ],
    }


def _buff_parameters(package_name, profile_root, filename):
    return [
        _merge_parameters(
            _package_parameters(package_name, filename),
            _optional_profile_parameters(profile_root, filename),
        )
    ]


def _serial_parameters(profile_root, serial_mode, auto_buff):
    filename = (
        "serial_driver_params.yaml"
        if serial_mode == "real"
        else "virtual_serial_params.yaml"
    )
    return _merge_parameters(
        _profile_parameters(profile_root, filename),
        {"enable_auto_buff": auto_buff},
    )


def _gimbal_parameters(profile_root, debug, auto_buff):
    return _merge_parameters(
        _package_parameters("gimbal_pipeline", "gimbal_pipeline.yaml"),
        _profile_parameters(profile_root, "gimbal_pipeline.yaml"),
        {
            "debug_mode": debug,
            "external_targets.enable": auto_buff,
            "external_targets.buff.enable": auto_buff,
        },
    )


def _launch_setup(context):
    profile_name = LaunchConfiguration("profile").perform(context).strip()
    if profile_name not in SUPPORTED_PROFILES:
        choices = ", ".join(SUPPORTED_PROFILES)
        raise RuntimeError(f"Invalid profile={profile_name!r}; expected one of: {choices}")

    bringup_share = Path(get_package_share_directory("rm_bringup"))
    profile_root = bringup_share / "config" / profile_name
    profile_document = _load_yaml(profile_root / "profile.yaml")
    defaults = profile_document.get("profile")
    if not isinstance(defaults, dict):
        raise RuntimeError(f"profile.yaml has no 'profile' mapping: {profile_root}")

    image_source = _resolve(context, "image_source", defaults, IMAGE_SOURCES)
    serial_mode = _resolve(context, "serial_mode", defaults, SERIAL_MODES)
    detector_type = _resolve(context, "detector_type", defaults, DETECTOR_TYPES)
    composition = _resolve(context, "composition", defaults, COMPOSITIONS)
    auto_buff = _resolve_switch(context, "auto_buff", defaults)
    debug = _resolve_switch(context, "debug", defaults)
    namespace = LaunchConfiguration("namespace").perform(context).strip()
    if namespace == "auto":
        namespace = str(defaults.get("namespace", "")).strip()

    odom2camera = defaults.get("odom2camera")
    if not isinstance(odom2camera, dict) or not {"xyz", "rpy"} <= odom2camera.keys():
        raise RuntimeError(f"profile.odom2camera requires xyz and rpy: {profile_root}")

    description_file = (
        Path(get_package_share_directory("rm_robot_description"))
        / "urdf"
        / "rm_gimbal.urdf.xacro"
    )
    robot_description = Command(
        [
            "xacro ",
            str(description_file),
            " xyz:=\"",
            str(odom2camera["xyz"]),
            "\" rpy:=\"",
            str(odom2camera["rpy"]),
            "\"",
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "publish_frequency": 1000.0,
            }
        ],
        output="both",
    )

    camera = _camera_description(image_source, profile_root, debug)
    detector = _detector_description(detector_type, profile_root, debug)
    if composition == "container":
        camera_node = ComposableNode(
            package=camera["package"],
            plugin=camera["plugin"],
            name=camera["name"],
            parameters=camera["parameters"],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        detector_node = ComposableNode(
            package=detector["package"],
            plugin=detector["plugin"],
            name="armor_detector",
            parameters=detector["parameters"],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        camera_detector_actions = [
            ComposableNodeContainer(
                name="camera_detector_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[camera_node, detector_node],
                output="both",
                emulate_tty=True,
                additional_env=camera["environment"],
            )
        ]
    else:
        camera_detector_actions = [
            Node(
                package=camera["package"],
                executable=camera["executable"],
                name=camera["name"],
                parameters=camera["parameters"],
                output="both",
                emulate_tty=True,
                additional_env=camera["environment"],
            ),
            Node(
                package=detector["package"],
                executable=detector["executable"],
                name="armor_detector",
                parameters=detector["parameters"],
                output="both",
                emulate_tty=True,
            ),
        ]

    serial_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node" if serial_mode == "real" else "virtual_serial_node",
        name="serial_driver" if serial_mode == "real" else "virtual_serial",
        parameters=[_serial_parameters(profile_root, serial_mode, auto_buff)],
        output="both",
        emulate_tty=True,
    )

    gimbal_parameters = [_gimbal_parameters(profile_root, debug, auto_buff)]
    gimbal_node = Node(
        package="gimbal_pipeline",
        executable="gimbal_pipeline_node",
        name="gimbal_pipeline",
        parameters=gimbal_parameters,
        remappings=[("cmd_gimbal", "armor_solver/cmd_gimbal")],
        output="both",
        emulate_tty=True,
    )

    actions = [robot_state_publisher, serial_node, *camera_detector_actions, gimbal_node]
    if auto_buff:
        actions.extend(
            [
                Node(
                    package="auto_buff",
                    executable="buff_detector_node",
                    name="buff_detector",
                    parameters=_buff_parameters(
                        "auto_buff", profile_root, "buff_detector.yaml"
                    ),
                    output="both",
                    emulate_tty=True,
                ),
                Node(
                    package="auto_buff",
                    executable="buff_pose_estimator_node",
                    name="buff_pose_estimator",
                    parameters=_buff_parameters(
                        "auto_buff", profile_root, "buff_pose_estimator.yaml"
                    ),
                    output="both",
                    emulate_tty=True,
                ),
            ]
        )

    return [GroupAction(actions=[PushRosNamespace(namespace), *actions])]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="infantry_4_1",
                description="Robot profile: " + " | ".join(SUPPORTED_PROFILES),
            ),
            DeclareLaunchArgument(
                "image_source",
                default_value="auto",
                description="auto | video | mindvision | hik",
            ),
            DeclareLaunchArgument(
                "serial_mode", default_value="auto", description="auto | real | virtual"
            ),
            DeclareLaunchArgument(
                "detector_type",
                default_value="auto",
                description="auto | armor_detector | armor_detector_nn",
            ),
            DeclareLaunchArgument(
                "composition",
                default_value="auto",
                description="auto | standalone | container",
            ),
            DeclareLaunchArgument(
                "auto_buff", default_value="auto", description="auto | on | off"
            ),
            DeclareLaunchArgument(
                "debug", default_value="auto", description="auto | on | off"
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="auto",
                description="ROS namespace or auto for the profile default",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
