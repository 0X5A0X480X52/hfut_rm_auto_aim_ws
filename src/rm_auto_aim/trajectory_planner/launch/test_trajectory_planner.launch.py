"""
Launch文件：启动trajectory_planner测试环境

包含：
1. trajectory_planner_tester 节点（模拟数据源）
2. trajectory_planner 节点（被测试对象）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('trajectory_planner')
    config_file = os.path.join(pkg_share, 'config', 'trajectory_planner.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_audit_arg = DeclareLaunchArgument(
        'enable_audit',
        default_value='false',
        description='Enable MPC audit logging'
    )
    
    # 测试节点参数
    tester_params = {
        'dt': 0.02,  # 50Hz
        'num_plates': 4,
        'base_omega': 2.0,
        'center_alpha': 0.1,
        'prediction_steps': 18,
        'measurement_noise_std': 0.035,  # 2度
        'enable_robots_pub': True,
    }
    
    # trajectory_planner 测试节点
    tester_node = Node(
        package='trajectory_planner',
        executable='test_trajectory_planner_node.py',
        name='trajectory_planner_tester',
        output='screen',
        parameters=[tester_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True,
    )
    
    # trajectory_planner 节点（延迟1秒启动，等待测试节点就绪）
    planner_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='trajectory_planner',
                executable='trajectory_planner_node.py',
                name='trajectory_planner',
                output='screen',
                parameters=[
                    config_file,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                emulate_tty=True,
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_audit_arg,
        tester_node,
        planner_node,
    ])
