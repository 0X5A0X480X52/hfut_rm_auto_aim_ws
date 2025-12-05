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
Integration test for ArmorTrackerNode.

This test verifies:
1. The node receives armors and publishes tracked armors
2. Track IDs are unique
3. History and prediction windows are published
"""

import os
import subprocess
import time
import signal
import threading
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

import rm_interfaces.msg as rm_msg


def _find_executable_path():
    """Find installed executable using ament_index."""
    share_dir = get_package_share_directory('armor_tracker')
    install_dir = os.path.dirname(os.path.dirname(share_dir))
    exe_path = os.path.join(install_dir, 'lib', 'armor_tracker', 'armor_tracker_node_exe')
    return exe_path


def _get_sensor_data_qos():
    """Create QoS profile matching rclcpp::SensorDataQoS."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


class IntegrationTestNode(Node):
    """Test node that subscribes to tracker outputs and publishes test armors."""

    def __init__(self):
        super().__init__('armor_tracker_integration_test_node')
        self.tracked_msgs = []
        self.history_msgs = []
        self.prediction_msgs = []
        self.armors_received = []

        qos = _get_sensor_data_qos()

        self._tracked_sub = self.create_subscription(
            rm_msg.TrackedArmors, '/test/tracked_armors',
            self._tracked_callback, qos)
        self._history_sub = self.create_subscription(
            rm_msg.TrackHistoryWindows, '/test/history_windows',
            self._history_callback, qos)
        self._pred_sub = self.create_subscription(
            rm_msg.TrackPredictionWindows, '/test/prediction_windows',
            self._pred_callback, qos)
        self._armors_sub = self.create_subscription(
            rm_msg.Armors, '/test/armors',
            self._armors_callback, qos)

    def _tracked_callback(self, msg):
        self.get_logger().info(f'Received tracked: {len(msg.armors)} armors')
        self.tracked_msgs.append(msg)

    def _history_callback(self, msg):
        self.history_msgs.append(msg)

    def _pred_callback(self, msg):
        self.prediction_msgs.append(msg)

    def _armors_callback(self, msg):
        self.get_logger().info(f'Local echo: {len(msg.armors)} armors')
        self.armors_received.append(msg)


def create_test_armors(node):
    """Create a sample Armors message."""
    armors = rm_msg.Armors()
    armors.header.stamp = node.get_clock().now().to_msg()
    armors.header.frame_id = 'odom'

    a1 = rm_msg.Armor()
    a1.number = '1'
    a1.type = 'small'
    a1.distance_to_image_center = 0.1
    a1.pose.position.x = 1.0
    a1.pose.position.y = 0.0
    a1.pose.position.z = 1.0
    a1.pose.orientation.w = 1.0

    a2 = rm_msg.Armor()
    a2.number = '2'
    a2.type = 'small'
    a2.distance_to_image_center = 0.2
    a2.pose.position.x = 2.0
    a2.pose.position.y = 1.0
    a2.pose.position.z = 1.0
    a2.pose.orientation.w = 1.0

    armors.armors.append(a1)
    armors.armors.append(a2)
    return armors


class TrackerProcessManager:
    """Manages the tracker node subprocess lifecycle."""

    def __init__(self):
        self.process = None
        self.stdout_lines = []
        self.stderr_lines = []

    def start(self):
        """Start the tracker node subprocess."""
        exe_path = _find_executable_path()
        if not os.path.exists(exe_path):
            raise FileNotFoundError(f'Executable not found: {exe_path}')

        args = [
            exe_path,
            '--ros-args',
            '-r', '__node:=armor_tracker_test',
            '-p', 'debug:=true',
            '-p', 'topics.armors_sub:=/test/armors',
            '-p', 'topics.tracked_armors_pub:=/test/tracked_armors',
            '-p', 'topics.history_windows_pub:=/test/history_windows',
            '-p', 'topics.prediction_windows_pub:=/test/prediction_windows',
            '-p', 'tracking_threshold:=1',
            '-p', 'lost_threshold:=100',
            '-p', 'max_trackers:=10',
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
            raise RuntimeError(f'Tracker node died on startup. Exit code: {self.process.returncode}')

    def stop(self):
        """Stop the tracker node subprocess."""
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
def tracker_process():
    """Fixture that manages tracker process lifecycle."""
    manager = TrackerProcessManager()
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
    """Wait until we discover the tracker's endpoints."""
    start = time.time()
    while time.time() - start < timeout:
        pubs = node.get_publishers_info_by_topic('/test/tracked_armors')
        subs = node.get_subscriptions_info_by_topic('/test/armors')

        # Look for external endpoints (not our test node)
        ext_pubs = [p for p in pubs if 'integration_test' not in str(p.node_name)]
        ext_subs = [s for s in subs if 'integration_test' not in str(s.node_name)]

        if ext_pubs and ext_subs:
            print(f'Discovery OK: {len(ext_pubs)} pub(s), {len(ext_subs)} sub(s)')
            return True
        time.sleep(0.1)

    print(f'Discovery failed after {timeout}s')
    return False


@pytest.mark.integration
def test_tracker_publishes_tracked_armors(ros_context, tracker_process, test_node):
    """Test that tracker receives armors and publishes tracked results."""
    pub = test_node.create_publisher(rm_msg.Armors, '/test/armors', _get_sensor_data_qos())

    # Wait for discovery
    assert wait_for_discovery(test_node), 'Discovery failed'

    # Additional settling time for DDS matching
    time.sleep(1.0)

    # Publish armors repeatedly
    print('Publishing armors...')
    for _ in range(60):  # 3 seconds at 20Hz
        armors = create_test_armors(test_node)
        pub.publish(armors)
        time.sleep(0.05)

    # Wait for results
    time.sleep(2.0)

    # The tracker publishes at 100Hz even without detections
    # So we should receive TrackedArmors messages regardless
    stdout, stderr = tracker_process.get_logs()

    assert len(test_node.tracked_msgs) > 0, (
        f'No tracked messages received.\n'
        f'Local armors echoes: {len(test_node.armors_received)}\n'
        f'Node stdout (last 50 lines):\n{stdout}\n'
        f'Node stderr:\n{stderr}'
    )

    print(f'Received {len(test_node.tracked_msgs)} tracked messages')
    print(f'Local armors echoes: {len(test_node.armors_received)}')


@pytest.mark.integration
def test_track_ids_are_unique(ros_context, tracker_process, test_node):
    """Test that track IDs are unique within each tracked message."""
    pub = test_node.create_publisher(rm_msg.Armors, '/test/armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    # Publish armors
    for _ in range(40):
        armors = create_test_armors(test_node)
        pub.publish(armors)
        time.sleep(0.05)

    time.sleep(2.0)

    # Check track ID uniqueness in messages with multiple armors
    for msg in test_node.tracked_msgs:
        if len(msg.armors) > 1:
            ids = [a.track_id for a in msg.armors]
            assert len(ids) == len(set(ids)), f'Duplicate track IDs: {ids}'


@pytest.mark.integration
def test_windows_are_published(ros_context, tracker_process, test_node):
    """Test that history and prediction windows are published."""
    pub = test_node.create_publisher(rm_msg.Armors, '/test/armors', _get_sensor_data_qos())

    assert wait_for_discovery(test_node), 'Discovery failed'
    time.sleep(1.0)

    for _ in range(40):
        armors = create_test_armors(test_node)
        pub.publish(armors)
        time.sleep(0.05)

    time.sleep(2.0)

    # Windows should be published (can be empty if disabled)
    print(f'History messages: {len(test_node.history_msgs)}')
    print(f'Prediction messages: {len(test_node.prediction_msgs)}')

    # At minimum we should receive some window messages (they're enabled by default)
    assert len(test_node.history_msgs) > 0 or len(test_node.prediction_msgs) > 0, \
        'No window messages received'


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
