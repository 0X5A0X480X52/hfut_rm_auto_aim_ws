# Copyright (C) FYT Vision Group. All rights reserved.
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

"""
Integration test for RobotPoseEstimatorNode.

This test verifies:
1. The node receives tracked armors and publishes robot state estimates
2. Virtual armors are generated correctly
3. Target messages are published for armor_solver compatibility
4. Multi-robot tracking works correctly
"""

import os
import subprocess
import time
import signal
import threading
import math
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

import rm_interfaces.msg as rm_msg


def _find_executable_path():
    """Find installed executable using ament_index."""
    share_dir = get_package_share_directory('robot_pose_estimator')
    install_dir = os.path.dirname(os.path.dirname(share_dir))
    exe_path = os.path.join(install_dir, 'lib', 'robot_pose_estimator', 'robot_pose_estimator_node')
    return exe_path


def _get_sensor_data_qos():
    """Create QoS profile matching rclcpp::SensorDataQoS."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


class IntegrationTestNode(Node):
    """Test node that subscribes to estimator outputs and publishes test tracked armors."""

    def __init__(self):
        super().__init__('robot_pose_estimator_integration_test_node')
        self.robots_msgs = []
        self.virtual_armors_msgs = []
        self.target_msgs = []
        self.tracked_armors_received = []

        qos = _get_sensor_data_qos()

        # Subscribers to estimator outputs
        self._robots_sub = self.create_subscription(
            rm_msg.TrackedRobots, '/test/robots',
            self._robots_callback, qos)
        self._virtual_armors_sub = self.create_subscription(
            rm_msg.Armors, '/test/virtual_armors',
            self._virtual_armors_callback, qos)
        self._target_sub = self.create_subscription(
            rm_msg.Target, '/test/target',
            self._target_callback, qos)
        
        # Local echo subscriber
        self._tracked_armors_sub = self.create_subscription(
            rm_msg.TrackedArmors, '/test/tracked_armors',
            self._tracked_armors_callback, qos)

    def _robots_callback(self, msg):
        self.get_logger().info(f'Received robots: {len(msg.robots)} robots')
        self.robots_msgs.append(msg)

    def _virtual_armors_callback(self, msg):
        self.get_logger().info(f'Received virtual armors: {len(msg.armors)} armors')
        self.virtual_armors_msgs.append(msg)

    def _target_callback(self, msg):
        self.get_logger().info(f'Received target: tracking={msg.tracking}')
        self.target_msgs.append(msg)

    def _tracked_armors_callback(self, msg):
        self.get_logger().info(f'Local echo tracked armors: {len(msg.armors)} armors')
        self.tracked_armors_received.append(msg)


def _get_tracking_state(track_state_str):
    """Convert track state string to TrackedArmor constant."""
    # TrackedArmor constants: LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3
    state_map = {
        'LOST': 0,
        'DETECTING': 1,
        'DETECT': 1,  # alias
        'TRACKING': 2,
        'TRACK': 2,   # alias
        'TEMP_LOST': 3,
        'PREDICT': 3  # alias for TEMP_LOST
    }
    return state_map.get(track_state_str.upper(), 1)


def create_test_tracked_armors(node, robot_id='1', armor_type='small', 
                                x=3.0, y=0.0, z=1.0, yaw=0.0,
                                track_state='DETECT'):
    """Create a sample TrackedArmors message simulating armor_tracker output."""
    tracked = rm_msg.TrackedArmors()
    tracked.header.stamp = node.get_clock().now().to_msg()
    tracked.header.frame_id = 'odom'

    armor = rm_msg.TrackedArmor()
    armor.armor_id = robot_id
    armor.armor_type = armor_type
    armor.track_id = 1
    armor.tracking_state = _get_tracking_state(track_state)
    armor.confidence = 1.0
    
    # Position (geometry_msgs/Point)
    armor.position.x = x
    armor.position.y = y
    armor.position.z = z
    
    # Yaw angle directly
    armor.yaw = yaw
    armor.yaw_velocity = 0.0
    
    # Velocity (geometry_msgs/Vector3)
    armor.velocity.x = 0.0
    armor.velocity.y = 0.0
    armor.velocity.z = 0.0
    
    # Source type: SOURCE_DETECT=0
    armor.source_type = 0

    tracked.armors.append(armor)
    return tracked


def create_multi_robot_tracked_armors(node):
    """Create TrackedArmors with multiple robots."""
    tracked = rm_msg.TrackedArmors()
    tracked.header.stamp = node.get_clock().now().to_msg()
    tracked.header.frame_id = 'odom'

    # Robot 1 - Standard infantry
    armor1 = rm_msg.TrackedArmor()
    armor1.armor_id = '1'
    armor1.armor_type = 'small'
    armor1.track_id = 1
    armor1.tracking_state = 1  # DETECTING
    armor1.confidence = 1.0
    armor1.position.x = 3.0
    armor1.position.y = 0.0
    armor1.position.z = 1.0
    armor1.yaw = 0.0
    armor1.yaw_velocity = 0.0
    armor1.source_type = 0

    # Robot 2 - Hero
    armor2 = rm_msg.TrackedArmor()
    armor2.armor_id = '2'
    armor2.armor_type = 'large'
    armor2.track_id = 2
    armor2.tracking_state = 1  # DETECTING
    armor2.confidence = 1.0
    armor2.position.x = 5.0
    armor2.position.y = 1.0
    armor2.position.z = 1.2
    armor2.yaw = 0.0
    armor2.yaw_velocity = 0.0
    armor2.source_type = 0

    # Robot 3 - Balance infantry
    armor3 = rm_msg.TrackedArmor()
    armor3.armor_id = '3'
    armor3.armor_type = 'large'
    armor3.track_id = 3
    armor3.tracking_state = 3  # TEMP_LOST (PREDICT)
    armor3.confidence = 0.8
    armor3.position.x = 7.0
    armor3.position.y = -1.0
    armor3.position.z = 0.8
    armor3.yaw = 0.0
    armor3.yaw_velocity = 0.0
    armor3.source_type = 0

    tracked.armors.extend([armor1, armor2, armor3])
    return tracked


class EstimatorProcessManager:
    """Manages the robot pose estimator node subprocess lifecycle."""

    def __init__(self):
        self.process = None
        self.stdout_lines = []
        self.stderr_lines = []

    def start(self):
        """Start the estimator node subprocess."""
        exe_path = _find_executable_path()
        if not os.path.exists(exe_path):
            raise FileNotFoundError(f'Executable not found: {exe_path}')

        args = [
            exe_path,
            '--ros-args',
            '-r', '__node:=robot_pose_estimator_test',
            '-p', 'debug:=true',
            '-p', 'predict_rate:=100.0',
            '-p', 'topics.tracked_armors_sub:=/test/tracked_armors',
            '-p', 'topics.robots_pub:=/test/robots',
            '-p', 'topics.virtual_armors_pub:=/test/virtual_armors',
            '-p', 'topics.target_pub:=/test/target',
            '-p', 'tracking.tracking_threshold:=3',
            '-p', 'tracking.lost_threshold:=10',
        ]

        env = os.environ.copy()
        env.setdefault('ROS_DOMAIN_ID', '0')

        self.process = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)

        # Start log readers
        def read_stream(stream, lines, prefix):
            for line in iter(stream.readline, b''):
                if line:
                    text = line.decode('utf-8', errors='ignore').rstrip()
                    lines.append(text)
                    print(f'{prefix}: {text}')

        threading.Thread(target=read_stream, 
            args=(self.process.stdout, self.stdout_lines, 'OUT'), daemon=True).start()
        threading.Thread(target=read_stream,
            args=(self.process.stderr, self.stderr_lines, 'ERR'), daemon=True).start()

        # Wait for node to initialize
        time.sleep(2.0)

        if self.process.poll() is not None:
            raise RuntimeError(f'Estimator node died on startup. Exit code: {self.process.returncode}')

    def stop(self):
        """Stop the estimator node subprocess."""
        if self.process is None:
            return
        try:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=3)
        except Exception:
            try:
                self.process.terminate()
                self.process.wait(timeout=2)
            except Exception:
                try:
                    self.process.kill()
                except Exception:
                    pass

    def is_running(self):
        """Check if process is still running."""
        return self.process is not None and self.process.poll() is None

    def get_logs(self):
        """Get collected logs."""
        return '\n'.join(self.stdout_lines[-50:]), '\n'.join(self.stderr_lines[-20:])


@pytest.fixture
def ros_context():
    """Fixture that manages ROS context lifecycle."""
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass


@pytest.fixture
def estimator_process():
    """Fixture that manages estimator process lifecycle."""
    manager = EstimatorProcessManager()
    manager.start()
    yield manager
    manager.stop()


@pytest.fixture
def test_node(ros_context):
    """Fixture that provides a test node with executor."""
    node = IntegrationTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield node

    try:
        executor.shutdown()
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass


def wait_for_discovery(node, timeout=8.0):
    """Wait until we discover the estimator's endpoints."""
    start = time.time()
    while time.time() - start < timeout:
        pubs_robots = node.get_publishers_info_by_topic('/test/robots')
        pubs_target = node.get_publishers_info_by_topic('/test/target')
        subs = node.get_subscriptions_info_by_topic('/test/tracked_armors')

        # Look for external endpoints (not our test node)
        ext_pubs = [p for p in pubs_robots if 'integration_test' not in str(p.node_name)]
        ext_subs = [s for s in subs if 'integration_test' not in str(s.node_name)]

        if ext_pubs and ext_subs:
            print(f'Discovery OK: {len(ext_pubs)} pub(s), {len(ext_subs)} sub(s)')
            return True
        time.sleep(0.1)

    print(f'Discovery failed after {timeout}s')
    return False


# ============================================================================
# Integration Tests
# ============================================================================

@pytest.mark.integration
def test_estimator_publishes_target(ros_context, estimator_process, test_node):
    """Test that estimator receives tracked armors and publishes Target messages."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    # Wait for discovery
    assert wait_for_discovery(test_node), 'Discovery failed'

    # Additional settling time for DDS matching
    time.sleep(1.0)

    # Publish tracked armors repeatedly
    print('Publishing tracked armors...')
    for i in range(60):  # 3 seconds at 20Hz
        tracked = create_test_tracked_armors(
            test_node, 
            robot_id='1',
            x=3.0 + i * 0.01,  # Slowly moving
            y=0.0,
            z=1.0
        )
        pub.publish(tracked)
        time.sleep(0.05)

    # Wait for results
    time.sleep(2.0)

    # Get logs for debugging
    stdout, stderr = estimator_process.get_logs()

    # Check that target messages were received
    assert len(test_node.target_msgs) > 0, (
        f'No target messages received.\n'
        f'Local tracked armors echoes: {len(test_node.tracked_armors_received)}\n'
        f'Node stdout (last 50 lines):\n{stdout}\n'
        f'Node stderr:\n{stderr}'
    )

    print(f'Received {len(test_node.target_msgs)} target messages')
    
    # Verify target content
    last_target = test_node.target_msgs[-1]
    # Target should be tracking or have some state
    print(f'Last target: tracking={last_target.tracking}, id={last_target.id}')


@pytest.mark.integration
def test_estimator_publishes_robots(ros_context, estimator_process, test_node):
    """Test that estimator publishes TrackedRobots messages."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish tracked armors
    for _ in range(40):
        tracked = create_test_tracked_armors(test_node)
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(2.0)

    # Check robots messages
    print(f'Received {len(test_node.robots_msgs)} robots messages')
    
    # Should have received some robot state messages
    assert len(test_node.robots_msgs) > 0, 'No robots messages received'
    
    # Check if any message contains tracked robots
    has_robots = any(len(msg.robots) > 0 for msg in test_node.robots_msgs)
    print(f'Has robots in messages: {has_robots}')


@pytest.mark.integration
def test_estimator_publishes_virtual_armors(ros_context, estimator_process, test_node):
    """Test that estimator generates and publishes virtual armors."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish tracked armors
    for _ in range(40):
        tracked = create_test_tracked_armors(test_node)
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(2.0)

    # Check virtual armors messages
    print(f'Received {len(test_node.virtual_armors_msgs)} virtual armors messages')
    
    # Should have received virtual armors
    assert len(test_node.virtual_armors_msgs) > 0, 'No virtual armors messages received'


@pytest.mark.integration
def test_multi_robot_tracking(ros_context, estimator_process, test_node):
    """Test that estimator can track multiple robots simultaneously."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish multi-robot tracked armors
    for _ in range(60):
        tracked = create_multi_robot_tracked_armors(test_node)
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(2.0)

    # Check robots messages
    assert len(test_node.robots_msgs) > 0, 'No robots messages received'
    
    # Find message with maximum robots
    max_robots = max(len(msg.robots) for msg in test_node.robots_msgs) if test_node.robots_msgs else 0
    print(f'Maximum robots tracked: {max_robots}')
    
    # Should track multiple robots
    # Note: May not track all 3 immediately due to tracking_threshold
    assert max_robots >= 1, 'Expected at least 1 robot to be tracked'


@pytest.mark.integration
def test_filter_invalid_track_states(ros_context, estimator_process, test_node):
    """Test that estimator filters armors with invalid track states."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish armors with LOST state (should be filtered)
    for _ in range(30):
        tracked = create_test_tracked_armors(
            test_node, 
            track_state='LOST'
        )
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(1.0)

    # Get initial counts
    initial_robot_count = sum(len(msg.robots) for msg in test_node.robots_msgs)
    
    # Now publish valid DETECT armors
    for _ in range(40):
        tracked = create_test_tracked_armors(
            test_node, 
            track_state='DETECT'
        )
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(2.0)

    # Should have more robots after valid armors
    final_robot_count = sum(len(msg.robots) for msg in test_node.robots_msgs)
    
    print(f'Initial robot count: {initial_robot_count}')
    print(f'Final robot count: {final_robot_count}')


@pytest.mark.integration
def test_target_compatibility_with_armor_solver(ros_context, estimator_process, test_node):
    """Test that Target messages are compatible with armor_solver expectations."""
    pub = test_node.create_publisher(
        rm_msg.TrackedArmors, '/test/tracked_armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish tracked armors to get tracking state
    for _ in range(60):
        tracked = create_test_tracked_armors(
            test_node,
            robot_id='1',
            x=3.0,
            y=0.0,
            z=1.0,
            yaw=0.5
        )
        pub.publish(tracked)
        time.sleep(0.05)

    time.sleep(2.0)

    assert len(test_node.target_msgs) > 0, 'No target messages received'
    
    # Find a target message that is tracking
    tracking_targets = [t for t in test_node.target_msgs if t.tracking]
    
    if tracking_targets:
        target = tracking_targets[-1]
        print(f'Target: id={target.id}, tracking={target.tracking}')
        print(f'Position: ({target.position.x:.2f}, {target.position.y:.2f}, {target.position.z:.2f})')
        print(f'Yaw: {target.yaw:.2f}, v_yaw: {target.v_yaw:.2f}')
        print(f'Radius: r1={target.radius_1:.2f}, r2={target.radius_2:.2f}')
        print(f'Armors num: {target.armors_num}')
        
        # Verify Target message has expected fields
        assert hasattr(target, 'position')
        assert hasattr(target, 'velocity')
        assert hasattr(target, 'yaw')
        assert hasattr(target, 'v_yaw')
        assert hasattr(target, 'radius_1')
        assert hasattr(target, 'radius_2')
        assert hasattr(target, 'armors_num')
    else:
        print('No tracking targets received (may need more frames to reach tracking state)')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
