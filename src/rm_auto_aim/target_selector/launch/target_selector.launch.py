import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('target_selector')
    
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode for visualization markers'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'target_selector.yaml'),
        description='Path to the configuration file'
    )
    
    strategy_arg = DeclareLaunchArgument(
        'strategy',
        default_value='min_yaw_deviation',
        description='Target selection strategy'
    )
    
    # Get launch configurations
    debug = LaunchConfiguration('debug')
    config_file = LaunchConfiguration('config_file')
    strategy = LaunchConfiguration('strategy')
    
    # Target selector node
    target_selector_node = Node(
        package='target_selector',
        executable='target_selector_node',
        name='target_selector',
        output='screen',
        parameters=[
            config_file,
            {
                'debug': debug,
                'strategy': strategy
            }
        ],
        remappings=[
            # Add any remappings here if needed
        ]
    )
    
    return LaunchDescription([
        debug_arg,
        config_file_arg,
        strategy_arg,
        target_selector_node
    ])
