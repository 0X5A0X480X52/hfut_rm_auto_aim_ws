import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('robot_pose_estimator')
    
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'robot_pose_estimator.yaml'),
        description='Path to the configuration file'
    )
    
    # Get launch configurations
    debug = LaunchConfiguration('debug')
    config_file = LaunchConfiguration('config_file')
    
    # Robot pose estimator node
    robot_pose_estimator_node = Node(
        package='robot_pose_estimator',
        executable='robot_pose_estimator_node',
        name='robot_pose_estimator',
        output='screen',
        parameters=[
            config_file,
            {'debug': debug}
        ],
        remappings=[
            # Add any remappings here if needed
        ]
    )
    
    return LaunchDescription([
        debug_arg,
        config_file_arg,
        robot_pose_estimator_node
    ])
