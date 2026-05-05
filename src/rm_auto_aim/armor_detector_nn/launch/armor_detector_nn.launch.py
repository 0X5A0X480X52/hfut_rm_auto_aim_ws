import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("armor_detector_nn")
    config_path = os.path.join(pkg_share, "config", "armor_detector_nn.yaml")

    container = ComposableNodeContainer(
        name="armor_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="armor_detector_nn",
                plugin="fyt::auto_aim::ArmorDetectorNNNode",
                name="armor_detector",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
