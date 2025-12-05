# Created by Chengfu Zou
# Maintained by Chengfu Zou, Labor
# Modified by Amatrix (HFUT RM SKT GROUP)
# Copyright (C) FYT Vision Group
# Copyright (C) HFUT RM SKT GROUP. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import pytest
import rclpy
from rclpy.node import Node
from rm_interfaces.srv import SolveBallistic
from geometry_msgs.msg import Point, Vector3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing
import launch_testing.actions


def generate_test_description():
    # Launch the ballistic_solver node
    node = LaunchNode(
        package='ballistic_solver',
        executable='ballistic_solver_node_exe',
        name='ballistic_solver',
        output='screen',
    )

    return LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ])


@pytest.mark.launch_test
def test_static_target_service(launch_service, proc_info, proc_output):
    """Test the ballistic solver service with a static target"""
    rclpy.init()
    client_node = rclpy.create_node('ballistic_solver_test_client_static')
    client = client_node.create_client(SolveBallistic, '/ballistic_solver/solve')

    assert client.wait_for_service(timeout_sec=5.0), "Service '/ballistic_solver/solve' not available"

    req = SolveBallistic.Request()
    req.target_position.x = 5.0
    req.target_position.y = 0.0
    req.target_position.z = 1.0
    req.target_velocity.x = 0.0
    req.target_velocity.y = 0.0
    req.target_velocity.z = 0.0
    req.bullet_speed = 28.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)

    assert future.done()
    response = future.result()
    assert response is not None
    assert response.success is True
    assert response.flight_time > 0.0
    assert abs(response.pitch) < 1.0  # ~57 degrees

    client_node.destroy_node()
    rclpy.shutdown()


@pytest.mark.launch_test
def test_moving_target_service(launch_service, proc_info, proc_output):
    """Test the ballistic solver service with a moving target"""
    rclpy.init()
    client_node = rclpy.create_node('ballistic_solver_test_client_moving')
    client = client_node.create_client(SolveBallistic, '/ballistic_solver/solve')

    assert client.wait_for_service(timeout_sec=5.0), "Service '/ballistic_solver/solve' not available"

    req = SolveBallistic.Request()
    req.target_position.x = 5.0
    req.target_position.y = 0.0
    req.target_position.z = 1.0
    req.target_velocity.x = 0.0
    req.target_velocity.y = 0.8
    req.target_velocity.z = 0.0
    req.bullet_speed = 28.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)

    assert future.done()
    response = future.result()
    assert response is not None
    # For moving target, solver should succeed with reasonable values
    assert response.success is True
    assert response.flight_time > 0.0
    assert abs(response.pitch) < 1.0

    client_node.destroy_node()
    rclpy.shutdown()


@pytest.mark.launch_test
def test_invalid_target_service(launch_service, proc_info, proc_output):
    """Test the ballistic solver service with an invalid/too-close target"""
    rclpy.init()
    client_node = rclpy.create_node('ballistic_solver_test_client_invalid')
    client = client_node.create_client(SolveBallistic, '/ballistic_solver/solve')

    assert client.wait_for_service(timeout_sec=5.0), "Service '/ballistic_solver/solve' not available"

    req = SolveBallistic.Request()
    req.target_position.x = 0.0
    req.target_position.y = 0.0
    req.target_position.z = 0.0
    req.target_velocity.x = 0.0
    req.target_velocity.y = 0.0
    req.target_velocity.z = 0.0
    req.bullet_speed = 28.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)

    assert future.done()
    response = future.result()
    assert response is not None
    assert response.success is False

    client_node.destroy_node()
    rclpy.shutdown()
