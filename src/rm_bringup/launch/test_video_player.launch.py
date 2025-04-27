from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    import os

    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    image_node = ComposableNode(
        package='mindvision_camera',
        plugin='fyt::camera_driver::VideoPlayerNode',
        name='video_player',
        parameters=[get_params('video_player')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='only_video_player_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_node
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        container
    ])
