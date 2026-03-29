#!/usr/bin/env python3
"""
Launch multi-camera system using real MindVision cameras (dual driver).

This launch includes the MindVision dual camera launch and starts the
detectors, fusion and solver nodes configured to use the camera namespaces
`camera_left` and `camera_right` produced by the driver.

Usage:
  ros2 launch armor_fusion multi_camera_system_with_mindvision.launch.py

You can override the mindvision params file with `params_file:=/path/to/dual_camera_params.yaml`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    mv_pkg = FindPackageShare('mindvision_camera')
    fusion_pkg = FindPackageShare('armor_fusion')

    default_params = PathJoinSubstitution([mv_pkg, 'config', 'dual_camera_params.yaml'])

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='mindvision dual camera params file'
    )

    declare_use_sensor_qos = DeclareLaunchArgument('use_sensor_data_qos', default_value='false', description='Use sensor data QoS for camera topics')
    declare_enable_viz = DeclareLaunchArgument('enable_visualization', default_value='true', description='Enable fusion visualization')
    declare_enable_tf = DeclareLaunchArgument('enable_robot_tf', default_value='true', description='Start robot_state_publisher and static TFs for cameras')
    declare_robot_xacro = DeclareLaunchArgument('robot_xacro', default_value='./src/rm_robot_description/urdf/test_dual_camera.urdf.xacro', description='Path to test robot xacro')
    declare_fusion_executable = DeclareLaunchArgument(
        'fusion_executable',
        default_value='armor_fusion_node',
        description='Fusion executable, e.g. armor_fusion_node or multi_camera_fusion_node.py')

    # Include the mindvision dual camera driver (it launches namespaces camera_left and camera_right)
    include_mindvision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mv_pkg, 'launch', 'dual_camera_launch.py'])
        ),
        launch_arguments={'params_file': LaunchConfiguration('params_file'), 'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos')}.items()
    )

    # robot_state_publisher from test URDF and mapping from URDF optical frames to camera1/camera2 frames
    # allow overriding the xacro path (use workspace src path by default)
    robot_desc = Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('robot_xacro')])
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # robot_desc is a Command substitution (xacro call). Wrap it as a string ParameterValue
        parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}],
    )

    # map camera_left_optical_frame -> camera1_optical_frame and right -> camera2_optical_frame
    static_tf_map_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_left',
        output='screen',
        arguments=['0','0','0','0','0','0','camera_left_optical_frame','camera1_optical_frame'],
    )
    static_tf_map_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_right',
        output='screen',
        arguments=['0','0','0','0','0','0','camera_right_optical_frame','camera2_optical_frame'],
    )

    # Start armor_detector under camera_left namespace - it will subscribe to camera_left/image_raw and camera_left/camera_info
    camera_left_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera_left_detector',
        namespace='camera_left',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('system_params'),
            {
                'debug': True,
                'target_frame': 'odom',
            }
        ],
    )

    # Start armor_detector under camera_right namespace (delayed start handled by driver already)
    camera_right_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='camera_right_detector',
        namespace='camera_right',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('system_params'),
            {
                'debug': True,
                'target_frame': 'odom',
            }
        ],
    )

    # Fusion node - subscribe to the detectors' armors topics under camera namespaces
    fusion_node = Node(
        package='armor_fusion',
        executable=LaunchConfiguration('fusion_executable'),
        name='multi_camera_fusion',
        output='screen',
        parameters=[
            LaunchConfiguration('system_params'),
            {
                # detectors publish their armors to 'camera_left/armor_detector/armors' and 'camera_right/armor_detector/armors'
                'camera_topics': ['camera_left/armor_detector/armors', 'camera_right/armor_detector/armors'],
                'enable_visualization': LaunchConfiguration('enable_visualization'),
                'output_topic': '/armor_detector/armors',
            }
        ]
    )

    # Solver node subscribes to fused armors
    solver_node = Node(
        package='armor_solver',
        executable='armor_solver_node',
        name='armor_solver',
        output='screen',
        parameters=[
            LaunchConfiguration('system_params'),
            {
                'debug': True,
                'target_frame': 'odom',
            }
        ]
    )

    ld = LaunchDescription()
    
    # add system params and other declared launch arguments
    declare_system_params = DeclareLaunchArgument(
        'system_params',
        default_value=PathJoinSubstitution([fusion_pkg, 'config', 'multi_camera_system_with_mindvision_params.yaml']),
        description='System-level params file for fusion/solver/detectors'
    )

    for decl in [declare_params, declare_use_sensor_qos, declare_enable_viz, declare_enable_tf, declare_robot_xacro, declare_fusion_executable, declare_system_params]:
        ld.add_action(decl)

    # Mindvision driver first
    ld.add_action(include_mindvision)

    # Start robot_state_publisher shortly after driver to ensure frames exist
    ld.add_action(TimerAction(period=0.5, actions=[rsp_node, static_tf_map_left, static_tf_map_right]))

    # Then detectors and processing
    ld.add_action(camera_left_detector)
    ld.add_action(camera_right_detector)
    ld.add_action(fusion_node)
    ld.add_action(solver_node)

    return ld
