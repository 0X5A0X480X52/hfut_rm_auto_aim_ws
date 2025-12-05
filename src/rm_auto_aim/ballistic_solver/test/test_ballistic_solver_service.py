import os
"""
Created by Chengfu Zou
Maintained by Chengfu Zou, Labor
Modified by Amatrix (HFUT RM SKT GROUP)
Copyright (C) FYT Vision Group
Copyright (C) HFUT RM SKT GROUP. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import time
import subprocess
import signal

import pytest
import rclpy
from rclpy.node import Node

from rm_interfaces.srv import SolveBallistic


def wait_for_service(client, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        if client.wait_for_service(timeout_sec=0.1):
            return True
    return False


def call_service_and_assert(client, pos, vel, bullet_speed=28.0, timeout=5.0):
    req = SolveBallistic.Request()
    req.target_position.x = pos[0]
    req.target_position.y = pos[1]
    req.target_position.z = pos[2]
    req.target_velocity.x = vel[0]
    req.target_velocity.y = vel[1]
    req.target_velocity.z = vel[2]
    req.bullet_speed = bullet_speed

    future = client.call_async(req)
    rclpy.spin_until_future_complete(client._node, future, timeout_sec=timeout)
    assert future.done(), "Service call timed out"
    response = future.result()
    assert response is not None
    return response


@pytest.mark.integration
def test_ballistic_solver_service_static(tmp_path):
    # Spawn the node executable
    pkg_dir = os.path.join(os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)[0], 'share', 'ballistic_solver')
    exe = os.path.join(pkg_dir.replace('/share/ballistic_solver', '/lib/ballistic_solver'), 'ballistic_solver_node_exe')
    proc = subprocess.Popen([exe], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        rclpy.init()
        node = rclpy.create_node('ballistic_solver_test_client_py')
        client = node.create_client(SolveBallistic, '/ballistic_solver/solve')
        assert wait_for_service(client, timeout=5.0), 'Service did not appear'

        resp = call_service_and_assert(client, (5.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        assert resp.success is True
        assert resp.flight_time > 0.0
        assert abs(resp.pitch) < 1.0
        node.destroy_node()
        rclpy.shutdown()
    finally:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)


@pytest.mark.integration
def test_ballistic_solver_service_moving(tmp_path):
    # Spawn the node executable
    pkg_dir = os.path.join(os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)[0], 'share', 'ballistic_solver')
    exe = os.path.join(pkg_dir.replace('/share/ballistic_solver', '/lib/ballistic_solver'), 'ballistic_solver_node_exe')
    proc = subprocess.Popen([exe], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        rclpy.init()
        node = rclpy.create_node('ballistic_solver_test_client_py_moving')
        client = node.create_client(SolveBallistic, '/ballistic_solver/solve')
        assert wait_for_service(client, timeout=5.0), 'Service did not appear'

        resp = call_service_and_assert(client, (6.0, 0.2, 1.0), (0.0, 1.2, 0.0), bullet_speed=28.0)
        assert resp.success is True
        assert resp.flight_time > 0.0
        assert abs(resp.pitch) < 1.0
        node.destroy_node()
        rclpy.shutdown()
    finally:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
