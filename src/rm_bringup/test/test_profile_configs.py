import importlib.util
import itertools
import json
import os
from pathlib import Path
import re

import pytest
from launch import LaunchContext
from launch.actions import GroupAction, TimerAction


os.environ.setdefault("ROS_LOG_DIR", "/tmp/rm_bringup_test_logs")

BRINGUP_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = BRINGUP_ROOT.parent
PROFILES = ("hero", "infantry_4_1", "leg", "uav", "video_test")
PROFILE_FILES = {
    "armor_detector_params.yaml",
    "camera_info.yaml",
    "gimbal_pipeline.yaml",
    "hik_camera_driver_params.yaml",
    "mindvision_camera_driver_params.yaml",
    "profile.yaml",
    "serial_driver_params.yaml",
    "video_player_params.yaml",
    "virtual_serial_params.yaml",
}
PROFILE_KEYS = {
    "image_source",
    "serial_mode",
    "detector_type",
    "composition",
    "auto_buff",
    "debug",
    "namespace",
    "odom2camera",
}


@pytest.fixture(scope="module")
def launch_module():
    path = BRINGUP_ROOT / "launch" / "bringup_pipeline.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_pipeline_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    package_roots = {
        "rm_bringup": BRINGUP_ROOT,
        "gimbal_pipeline": SRC_ROOT / "rm_auto_aim" / "gimbal_pipeline",
        "armor_detector_nn": SRC_ROOT / "rm_auto_aim" / "armor_detector_nn",
        "auto_buff": SRC_ROOT / "rm_auto_aim" / "auto_buff",
        "rm_robot_description": SRC_ROOT / "rm_robot_description",
        "ros2_hik_camera": SRC_ROOT / "rm_hardware_driver" / "ros2_hik_camera",
        "mindvision_camera": SRC_ROOT / "rm_hardware_driver" / "ros2_mindvision_camera",
        "video_player": SRC_ROOT / "rm_hardware_driver" / "video_player",
    }

    def package_share(package_name):
        return str(package_roots[package_name])

    module.get_package_share_directory = package_share
    module._test_package_roots = package_roots
    return module


def _context(**overrides):
    values = {
        "profile": "infantry_4_1",
        "image_source": "auto",
        "serial_mode": "auto",
        "detector_type": "auto",
        "composition": "auto",
        "auto_buff": "auto",
        "debug": "auto",
        "namespace": "auto",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _walk_actions(action):
    yield action
    for child in action.get_sub_entities():
        yield from _walk_actions(child)


def _package_urls(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from _package_urls(child)
    elif isinstance(value, list):
        for child in value:
            yield from _package_urls(child)
    elif isinstance(value, str) and value.startswith("package://"):
        yield value


def _declared_parameters(*paths):
    pattern = re.compile(r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"')
    declared = set()
    for path in paths:
        declared.update(pattern.findall(path.read_text(encoding="utf-8")))
    return declared


def _assert_declared(label, parameters, *source_paths):
    unknown = set(parameters) - _declared_parameters(*source_paths)
    assert not unknown, f"{label}: undeclared parameters: {sorted(unknown)}"


def test_profile_inventory_and_resource_paths(launch_module):
    config_root = BRINGUP_ROOT / "config"
    assert {path.name for path in config_root.iterdir()} == set(PROFILES)

    for profile in PROFILES:
        root = config_root / profile
        expected = set(PROFILE_FILES)
        if profile == "video_test":
            expected |= {
                "buff_detector.yaml",
                "buff_pose_estimator.yaml",
                "input.mp4",
            }
        assert {path.name for path in root.iterdir()} == expected

        document = launch_module._load_yaml(root / "profile.yaml")
        assert set(document) == {"profile"}
        assert set(document["profile"]) == PROFILE_KEYS
        assert set(document["profile"]["odom2camera"]) == {"xyz", "rpy"}

        for path in root.glob("*.yaml"):
            document = launch_module._load_yaml(path)
            for url in _package_urls(document):
                package, relative = url.removeprefix("package://").split("/", 1)
                assert (launch_module._test_package_roots[package] / relative).is_file(), url

    for package, filename in (
        ("armor_detector_nn", "armor_detector_nn.yaml"),
        ("auto_buff", "buff_detector.yaml"),
        ("auto_buff", "buff_pose_estimator.yaml"),
        ("gimbal_pipeline", "gimbal_pipeline.yaml"),
    ):
        document = launch_module._load_yaml(
            launch_module._package_config(package, filename)
        )
        for url in _package_urls(document):
            resource_package, relative = url.removeprefix("package://").split("/", 1)
            assert (
                launch_module._test_package_roots[resource_package] / relative
            ).is_file(), url


def test_all_configuration_leaves_are_declared(launch_module):
    armor_source = (
        SRC_ROOT / "rm_auto_aim" / "armor_detector" / "src" / "armor_detector_node.cpp"
    )
    nn_source = (
        SRC_ROOT
        / "rm_auto_aim"
        / "armor_detector_nn"
        / "src"
        / "armor_detector_nn_node.cpp"
    )
    hik_source = (
        SRC_ROOT
        / "rm_hardware_driver"
        / "ros2_hik_camera"
        / "src"
        / "ros2_hik_camera_node.cpp"
    )
    mindvision_source = (
        SRC_ROOT
        / "rm_hardware_driver"
        / "ros2_mindvision_camera"
        / "src"
        / "mv_camera_node.cpp"
    )
    video_source = (
        SRC_ROOT
        / "rm_hardware_driver"
        / "video_player"
        / "src"
        / "video_player_node.cpp"
    )
    serial_source = (
        SRC_ROOT
        / "rm_hardware_driver"
        / "rm_serial_driver"
        / "src"
        / "serial_driver_node.cpp"
    )
    virtual_source = (
        SRC_ROOT
        / "rm_hardware_driver"
        / "rm_serial_driver"
        / "src"
        / "virtual_serial_node.cpp"
    )
    buff_detector_source = (
        SRC_ROOT
        / "rm_auto_aim"
        / "auto_buff"
        / "buff_detector"
        / "src"
        / "buff_detector_node.cpp"
    )
    buff_pose_source = (
        SRC_ROOT
        / "rm_auto_aim"
        / "auto_buff"
        / "src"
        / "ros2"
        / "buff_pose_estimator_node.cpp"
    )

    nn_params = launch_module._package_parameters(
        "armor_detector_nn", "armor_detector_nn.yaml"
    )
    _assert_declared("armor_detector_nn", nn_params, nn_source)

    buff_detector = launch_module._package_parameters(
        "auto_buff", "buff_detector.yaml"
    )
    buff_pose = launch_module._package_parameters(
        "auto_buff", "buff_pose_estimator.yaml"
    )
    _assert_declared("buff_detector", buff_detector, buff_detector_source)
    _assert_declared("buff_pose_estimator", buff_pose, buff_pose_source)

    for profile in PROFILES:
        root = BRINGUP_ROOT / "config" / profile
        _assert_declared(
            f"{profile}/armor_detector",
            launch_module._profile_parameters(root, "armor_detector_params.yaml"),
            armor_source,
        )
        _assert_declared(
            f"{profile}/hik",
            launch_module._profile_parameters(root, "hik_camera_driver_params.yaml"),
            hik_source,
        )
        _assert_declared(
            f"{profile}/mindvision",
            launch_module._profile_parameters(
                root, "mindvision_camera_driver_params.yaml"
            ),
            mindvision_source,
        )
        _assert_declared(
            f"{profile}/video",
            launch_module._profile_parameters(root, "video_player_params.yaml"),
            video_source,
        )
        _assert_declared(
            f"{profile}/serial",
            launch_module._serial_parameters(root, "real", False),
            serial_source,
        )
        _assert_declared(
            f"{profile}/virtual_serial",
            launch_module._serial_parameters(root, "virtual", False),
            virtual_source,
        )

    video_root = BRINGUP_ROOT / "config" / "video_test"
    _assert_declared(
        "video_test/buff_detector",
        {
            **buff_detector,
            **launch_module._optional_profile_parameters(
                video_root, "buff_detector.yaml"
            ),
        },
        buff_detector_source,
    )
    _assert_declared(
        "video_test/buff_pose_estimator",
        {
            **buff_pose,
            **launch_module._optional_profile_parameters(
                video_root, "buff_pose_estimator.yaml"
            ),
        },
        buff_pose_source,
    )


def test_default_profile_parameter_tables(launch_module):
    tables = {}
    for profile in PROFILES:
        root = BRINGUP_ROOT / "config" / profile
        defaults = launch_module._load_yaml(root / "profile.yaml")["profile"]
        debug = defaults["debug"] == "on"
        auto_buff = defaults["auto_buff"] == "on"
        camera = launch_module._camera_description(
            defaults["image_source"], root, debug
        )
        detector = launch_module._detector_description(
            defaults["detector_type"], root, debug
        )
        tables[profile] = {
            "selection": defaults,
            "camera": camera["parameters"][0],
            "detector": detector["parameters"][0],
            "serial": launch_module._serial_parameters(
                root, defaults["serial_mode"], auto_buff
            ),
            "gimbal": launch_module._gimbal_parameters(
                root, debug, auto_buff
            ),
        }

    output = Path(os.environ["ROS_LOG_DIR"]) / "rm_bringup_final_parameters.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(tables, indent=2, sort_keys=True), encoding="utf-8"
    )
    assert output.is_file()


def test_detector_feature_switches_are_explicit(launch_module):
    for profile in PROFILES:
        params = launch_module._profile_parameters(
            BRINGUP_ROOT / "config" / profile,
            "armor_detector_params.yaml",
        )
        assert isinstance(params["use_pca"], bool)
        assert params["use_ba"] is True
        assert isinstance(params["use_pnp_refiner"], bool)
        assert params["pnp_refiner.mode"] in {
            "none",
            "single_xyz_yaw",
        }


def test_gimbal_parameters_are_declared_and_current(launch_module):
    source_root = SRC_ROOT / "rm_auto_aim" / "gimbal_pipeline" / "src"
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(source_root.glob("gimbal_pipeline*.cpp"))
    )
    declared = set(
        re.findall(r'declare_parameter(?:<[^>]+>)?\(\s*"([^"]+)"', source)
    )
    package_params = launch_module._package_parameters(
        "gimbal_pipeline", "gimbal_pipeline.yaml"
    )
    forbidden_prefixes = (
        "norm4_v2.",
        "norm4_v3.ukf_v2.",
        "outpost.v3.",
        "controller.current_position.",
        "controller.predicted_position.",
        "controller.state_machine.",
    )
    forbidden_keys = {
        "tracker.implementation",
        "outpost.use_tracker_v2",
        "outpost.use_tracker_v3",
        "selector.strategy",
        "controller.strategy",
        "controller.ballistic_mode",
    }

    for profile in PROFILES:
        profile_root = BRINGUP_ROOT / "config" / profile
        profile_params = launch_module._profile_parameters(
            profile_root, "gimbal_pipeline.yaml"
        )
        merged = {**package_params, **profile_params}
        unknown = set(merged) - declared
        assert not unknown, f"{profile}: undeclared parameters: {sorted(unknown)}"
        assert not (set(merged) & forbidden_keys)
        assert not any(
            key.startswith(forbidden_prefixes) for key in merged
        )
        assert merged["norm4_v3.backend_config.backend_type"] in {
            "ukf_v1",
            "inekf",
        }

        for key, value in merged.items():
            if key.endswith("topic") or key.endswith(".topic"):
                assert not (isinstance(value, str) and value.startswith("/")), (
                    profile,
                    key,
                    value,
                )


@pytest.mark.parametrize("profile", PROFILES)
def test_default_profile_launch_graph(launch_module, profile):
    actions = launch_module._launch_setup(_context(profile=profile))
    assert len(actions) == 1
    assert isinstance(actions[0], GroupAction)
    graph = list(_walk_actions(actions[0]))
    assert not any(isinstance(action, TimerAction) for action in graph)
    assert "ballistic_solver" not in repr(graph)


def test_launch_branch_matrix_and_buff_switch(launch_module):
    dimensions = itertools.product(
        launch_module.IMAGE_SOURCES,
        launch_module.SERIAL_MODES,
        launch_module.DETECTOR_TYPES,
        launch_module.COMPOSITIONS,
        ("on", "off"),
    )
    profile_root = BRINGUP_ROOT / "config" / "video_test"
    for image_source, serial_mode, detector_type, composition, auto_buff in dimensions:
        actions = launch_module._launch_setup(
            _context(
                profile="video_test",
                image_source=image_source,
                serial_mode=serial_mode,
                detector_type=detector_type,
                composition=composition,
                auto_buff=auto_buff,
            )
        )
        assert len(actions) == 1

        enabled = auto_buff == "on"
        serial = launch_module._serial_parameters(
            profile_root, serial_mode, enabled
        )
        gimbal = launch_module._gimbal_parameters(profile_root, True, enabled)
        assert serial["enable_auto_buff"] is enabled
        assert gimbal["external_targets.enable"] is enabled
        assert gimbal["external_targets.buff.enable"] is enabled


@pytest.mark.parametrize(
    ("argument", "value"),
    [
        ("profile", "unknown"),
        ("image_source", "usb"),
        ("serial_mode", "mock"),
        ("detector_type", "legacy"),
        ("composition", "process"),
        ("auto_buff", "true"),
        ("debug", "false"),
    ],
)
def test_invalid_launch_values_fail_fast(launch_module, argument, value):
    with pytest.raises(RuntimeError):
        launch_module._launch_setup(_context(**{argument: value}))
