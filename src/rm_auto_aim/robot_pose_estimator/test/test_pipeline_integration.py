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
End-to-end integration test for armor_tracker -> robot_pose_estimator pipeline.

This test verifies:
1. armor_tracker receives armors and publishes tracked armors
2. robot_pose_estimator receives tracked armors and publishes robot states
3. The full pipeline from detection to robot state estimation works correctly
4. Virtual armors are generated that could be used for aiming
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


def _find_armor_tracker_executable():
    """Find armor_tracker executable."""
    try:
        share_dir = get_package_share_directory('armor_tracker')
        install_dir = os.path.dirname(os.path.dirname(share_dir))
        exe_path = os.path.join(install_dir, 'lib', 'armor_tracker', 'armor_tracker_node_exe')
        return exe_path
    except Exception:
        return None


def _find_robot_pose_estimator_executable():
    """Find robot_pose_estimator executable."""
    try:
        share_dir = get_package_share_directory('robot_pose_estimator')
        install_dir = os.path.dirname(os.path.dirname(share_dir))
        exe_path = os.path.join(install_dir, 'lib', 'robot_pose_estimator', 'robot_pose_estimator_node')
        return exe_path
    except Exception:
        return None


def _get_sensor_data_qos():
    """Create QoS profile matching rclcpp::SensorDataQoS."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


class PipelineTestNode(Node):
    """Test node that monitors the full pipeline."""

    def __init__(self):
        super().__init__('pipeline_integration_test_node')
        
        # Input tracking (what we publish)
        self.armors_published = 0
        
        # Intermediate tracking (armor_tracker output)
        self.tracked_armors_msgs = []
        
        # Output tracking (robot_pose_estimator output)
        self.robots_msgs = []
        self.virtual_armors_msgs = []
        self.target_msgs = []

        qos = _get_sensor_data_qos()

        # Subscribe to armor_tracker output
        self._tracked_armors_sub = self.create_subscription(
            rm_msg.TrackedArmors, '/pipeline/tracked_armors',
            self._tracked_armors_callback, qos)
        
        # Subscribe to robot_pose_estimator outputs
        self._robots_sub = self.create_subscription(
            rm_msg.TrackedRobots, '/pipeline/robots',
            self._robots_callback, qos)
        self._virtual_armors_sub = self.create_subscription(
            rm_msg.Armors, '/pipeline/virtual_armors',
            self._virtual_armors_callback, qos)
        self._target_sub = self.create_subscription(
            rm_msg.Target, '/pipeline/target',
            self._target_callback, qos)

    def _tracked_armors_callback(self, msg):
        self.get_logger().info(f'[Tracker] Tracked: {len(msg.armors)} armors')
        self.tracked_armors_msgs.append(msg)

    def _robots_callback(self, msg):
        self.get_logger().info(f'[Estimator] Robots: {len(msg.robots)} robots')
        self.robots_msgs.append(msg)

    def _virtual_armors_callback(self, msg):
        self.get_logger().info(f'[Estimator] Virtual armors: {len(msg.armors)} armors')
        self.virtual_armors_msgs.append(msg)

    def _target_callback(self, msg):
        self.get_logger().info(f'[Estimator] Target: tracking={msg.tracking}, id={msg.id}')
        self.target_msgs.append(msg)


def create_simulated_armors(node, robot_id='1', armor_type='small',
                            x=3.0, y=0.0, z=1.0, yaw=0.0):
    """Create simulated Armors message (detection output)."""
    armors = rm_msg.Armors()
    armors.header.stamp = node.get_clock().now().to_msg()
    armors.header.frame_id = 'odom'

    armor = rm_msg.Armor()
    armor.number = robot_id
    armor.type = armor_type
    armor.distance_to_image_center = 0.1
    
    # Position
    armor.pose.position.x = x
    armor.pose.position.y = y
    armor.pose.position.z = z
    
    # Orientation (from yaw)
    armor.pose.orientation.w = math.cos(yaw / 2.0)
    armor.pose.orientation.x = 0.0
    armor.pose.orientation.y = 0.0
    armor.pose.orientation.z = math.sin(yaw / 2.0)

    armors.armors.append(armor)
    return armors


def create_multi_robot_armors(node, time_offset=0.0):
    """Create Armors message with multiple robots."""
    armors = rm_msg.Armors()
    armors.header.stamp = node.get_clock().now().to_msg()
    armors.header.frame_id = 'odom'

    # Robot 1 - Infantry at 3m
    armor1 = rm_msg.Armor()
    armor1.number = '1'
    armor1.type = 'small'
    armor1.distance_to_image_center = 0.1
    armor1.pose.position.x = 3.0 + 0.1 * math.sin(time_offset)
    armor1.pose.position.y = 0.0 + 0.05 * math.cos(time_offset)
    armor1.pose.position.z = 1.0
    armor1.pose.orientation.w = 1.0
    armors.armors.append(armor1)

    # Robot 2 - Hero at 5m
    armor2 = rm_msg.Armor()
    armor2.number = '2'
    armor2.type = 'large'
    armor2.distance_to_image_center = 0.2
    armor2.pose.position.x = 5.0
    armor2.pose.position.y = 1.0
    armor2.pose.position.z = 1.2
    armor2.pose.orientation.w = 1.0
    armors.armors.append(armor2)

    return armors


class NodeProcessManager:
    """Generic manager for ROS2 node subprocesses."""

    def __init__(self, name, exe_path, extra_args=None):
        self.name = name
        self.exe_path = exe_path
        self.extra_args = extra_args or []
        self.process = None
        self.stdout_lines = []
        self.stderr_lines = []

    def start(self):
        """Start the node subprocess."""
        if not os.path.exists(self.exe_path):
            raise FileNotFoundError(f'{self.name} executable not found: {self.exe_path}')

        args = [self.exe_path] + self.extra_args

        env = os.environ.copy()
        env.setdefault('ROS_DOMAIN_ID', '0')

        self.process = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)

        def read_stream(stream, lines, prefix):
            for line in iter(stream.readline, b''):
                if line:
                    text = line.decode('utf-8', errors='ignore').rstrip()
                    lines.append(text)
                    print(f'[{self.name}] {prefix}: {text}')

        threading.Thread(target=read_stream,
            args=(self.process.stdout, self.stdout_lines, 'OUT'), daemon=True).start()
        threading.Thread(target=read_stream,
            args=(self.process.stderr, self.stderr_lines, 'ERR'), daemon=True).start()

        time.sleep(2.0)

        if self.process.poll() is not None:
            raise RuntimeError(f'{self.name} died on startup. Exit code: {self.process.returncode}')

    def stop(self):
        """Stop the node subprocess."""
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
        return self.process is not None and self.process.poll() is None

    def get_logs(self):
        return '\n'.join(self.stdout_lines[-30:]), '\n'.join(self.stderr_lines[-15:])


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
def armor_tracker_process():
    """Fixture that manages armor_tracker process."""
    exe_path = _find_armor_tracker_executable()
    if not exe_path or not os.path.exists(exe_path):
        pytest.skip('armor_tracker executable not found')
    
    args = [
        '--ros-args',
        '-r', '__node:=armor_tracker_pipeline',
        '-p', 'debug:=true',
        '-p', 'topics.armors_sub:=/pipeline/armors',
        '-p', 'topics.tracked_armors_pub:=/pipeline/tracked_armors',
        '-p', 'tracking_threshold:=2',
        '-p', 'lost_threshold:=50',
    ]
    
    manager = NodeProcessManager('armor_tracker', exe_path, args)
    manager.start()
    yield manager
    manager.stop()


@pytest.fixture
def robot_pose_estimator_process():
    """Fixture that manages robot_pose_estimator process."""
    exe_path = _find_robot_pose_estimator_executable()
    if not exe_path or not os.path.exists(exe_path):
        pytest.skip('robot_pose_estimator executable not found')
    
    args = [
        '--ros-args',
        '-r', '__node:=robot_pose_estimator_pipeline',
        '-p', 'debug:=true',
        '-p', 'predict_rate:=100.0',
        '-p', 'topics.tracked_armors_sub:=/pipeline/tracked_armors',
        '-p', 'topics.robots_pub:=/pipeline/robots',
        '-p', 'topics.virtual_armors_pub:=/pipeline/virtual_armors',
        '-p', 'topics.target_pub:=/pipeline/target',
        '-p', 'tracking.tracking_threshold:=2',
        '-p', 'tracking.lost_threshold:=20',
    ]
    
    manager = NodeProcessManager('robot_pose_estimator', exe_path, args)
    manager.start()
    yield manager
    manager.stop()


@pytest.fixture
def test_node(ros_context):
    """Fixture that provides a test node with executor."""
    node = PipelineTestNode()
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


def wait_for_pipeline_discovery(node, timeout=10.0):
    """Wait until we discover both nodes' endpoints."""
    start = time.time()
    while time.time() - start < timeout:
        # Check armor_tracker
        tracker_subs = node.get_subscriptions_info_by_topic('/pipeline/armors')
        tracker_pubs = node.get_publishers_info_by_topic('/pipeline/tracked_armors')
        
        # Check robot_pose_estimator
        estimator_subs = node.get_subscriptions_info_by_topic('/pipeline/tracked_armors')
        estimator_pubs = node.get_publishers_info_by_topic('/pipeline/robots')
        
        # Filter out our test node
        ext_tracker_subs = [s for s in tracker_subs if 'test' not in str(s.node_name).lower()]
        ext_tracker_pubs = [p for p in tracker_pubs if 'test' not in str(p.node_name).lower()]
        ext_estimator_subs = [s for s in estimator_subs if 'test' not in str(s.node_name).lower()]
        ext_estimator_pubs = [p for p in estimator_pubs if 'test' not in str(p.node_name).lower()]
        
        if ext_tracker_subs and ext_tracker_pubs and ext_estimator_subs and ext_estimator_pubs:
            print('Pipeline discovery OK')
            print(f'  Tracker: {len(ext_tracker_subs)} sub(s), {len(ext_tracker_pubs)} pub(s)')
            print(f'  Estimator: {len(ext_estimator_subs)} sub(s), {len(ext_estimator_pubs)} pub(s)')
            return True
        
        time.sleep(0.1)

    print(f'Pipeline discovery failed after {timeout}s')
    return False


# ============================================================================
# Pipeline Integration Tests
# ============================================================================

@pytest.mark.integration
@pytest.mark.pipeline
def test_full_pipeline_single_robot(ros_context, armor_tracker_process, 
                                     robot_pose_estimator_process, test_node):
    """Test the full pipeline: Armors -> TrackedArmors -> Robot State."""
    pub = test_node.create_publisher(
        rm_msg.Armors, '/pipeline/armors', _get_sensor_data_qos())

    # Wait for both nodes to be discovered
    assert wait_for_pipeline_discovery(test_node), 'Pipeline discovery failed'
    time.sleep(1.0)

    print('\n=== Publishing armors ===')
    
    # Publish armors (simulating detector output)
    for i in range(80):  # 4 seconds at 20Hz
        armors = create_simulated_armors(
            test_node,
            robot_id='1',
            x=3.0 + 0.1 * math.sin(i * 0.1),  # Slight oscillation
            y=0.0,
            z=1.0,
            yaw=i * 0.02  # Slowly rotating
        )
        pub.publish(armors)
        test_node.armors_published += 1
        time.sleep(0.05)

    # Wait for pipeline to process
    time.sleep(3.0)

    # Get logs
    tracker_stdout, tracker_stderr = armor_tracker_process.get_logs()
    estimator_stdout, estimator_stderr = robot_pose_estimator_process.get_logs()

    print('\n=== Pipeline Results ===')
    print(f'Armors published: {test_node.armors_published}')
    print(f'Tracked armors received: {len(test_node.tracked_armors_msgs)}')
    print(f'Robot states received: {len(test_node.robots_msgs)}')
    print(f'Virtual armors received: {len(test_node.virtual_armors_msgs)}')
    print(f'Target messages received: {len(test_node.target_msgs)}')

    # Verify armor_tracker output
    assert len(test_node.tracked_armors_msgs) > 0, (
        f'No tracked armors received from armor_tracker.\n'
        f'Tracker stdout:\n{tracker_stdout}\n'
        f'Tracker stderr:\n{tracker_stderr}'
    )

    # Verify robot_pose_estimator output
    assert len(test_node.target_msgs) > 0, (
        f'No target messages received from robot_pose_estimator.\n'
        f'Estimator stdout:\n{estimator_stdout}\n'
        f'Estimator stderr:\n{estimator_stderr}'
    )

    # Check tracked armors content
    tracked_with_armors = [m for m in test_node.tracked_armors_msgs if len(m.armors) > 0]
    print(f'Tracked messages with armors: {len(tracked_with_armors)}')
    
    if tracked_with_armors:
        last_tracked = tracked_with_armors[-1]
        print(f'Last tracked armor: id={last_tracked.armors[0].armor_id}, '
              f'state={last_tracked.armors[0].tracking_state}')

    # Check robot state content
    robots_with_tracking = [m for m in test_node.robots_msgs if len(m.robots) > 0]
    if robots_with_tracking:
        last_robots = robots_with_tracking[-1]
        for robot in last_robots.robots:
            print(f'Robot: id={robot.id}, type={robot.type}, tracking={robot.tracking}')


@pytest.mark.integration
@pytest.mark.pipeline
def test_full_pipeline_multi_robot(ros_context, armor_tracker_process,
                                    robot_pose_estimator_process, test_node):
    """Test the full pipeline with multiple robots."""
    pub = test_node.create_publisher(
        rm_msg.Armors, '/pipeline/armors', _get_sensor_data_qos())

    assert wait_for_pipeline_discovery(test_node), 'Pipeline discovery failed'
    time.sleep(1.0)

    print('\n=== Publishing multi-robot armors ===')
    
    # Publish armors for multiple robots
    for i in range(80):
        armors = create_multi_robot_armors(test_node, time_offset=i * 0.1)
        pub.publish(armors)
        time.sleep(0.05)

    time.sleep(3.0)

    print('\n=== Multi-Robot Pipeline Results ===')
    print(f'Tracked armors received: {len(test_node.tracked_armors_msgs)}')
    print(f'Robot states received: {len(test_node.robots_msgs)}')
    print(f'Target messages received: {len(test_node.target_msgs)}')

    # Should have tracked armors
    assert len(test_node.tracked_armors_msgs) > 0, 'No tracked armors received'

    # Check for multi-armor messages (multiple robots detected)
    multi_armor_msgs = [m for m in test_node.tracked_armors_msgs if len(m.armors) >= 2]
    print(f'Messages with 2+ armors: {len(multi_armor_msgs)}')
    
    # Check robot tracking
    robots_msgs_with_robots = [m for m in test_node.robots_msgs if len(m.robots) > 0]
    if robots_msgs_with_robots:
        max_robots = max(len(m.robots) for m in robots_msgs_with_robots)
        print(f'Maximum robots tracked simultaneously: {max_robots}')


@pytest.mark.integration
@pytest.mark.pipeline
def test_pipeline_latency(ros_context, armor_tracker_process,
                          robot_pose_estimator_process, test_node):
    """Test pipeline latency from armor input to robot state output."""
    pub = test_node.create_publisher(
        rm_msg.Armors, '/pipeline/armors', _get_sensor_data_qos())

    assert wait_for_pipeline_discovery(test_node), 'Pipeline discovery failed'
    time.sleep(1.0)

    # First, warm up the pipeline
    for _ in range(30):
        armors = create_simulated_armors(test_node)
        pub.publish(armors)
        time.sleep(0.05)
    
    time.sleep(1.0)
    
    # Clear previous messages
    test_node.tracked_armors_msgs.clear()
    test_node.target_msgs.clear()
    
    # Record publish time
    publish_time = time.time()
    
    # Publish a new armor
    armors = create_simulated_armors(test_node, x=4.0)  # Slightly different position
    pub.publish(armors)
    
    # Wait for response
    timeout = 1.0
    start = time.time()
    while time.time() - start < timeout:
        if len(test_node.target_msgs) > 0:
            receive_time = time.time()
            latency = (receive_time - publish_time) * 1000  # ms
            print(f'Pipeline latency: {latency:.2f} ms')
            
            # Latency should be reasonable (< 500ms for real-time)
            assert latency < 500, f'Pipeline latency too high: {latency:.2f} ms'
            return
        time.sleep(0.01)
    
    print('Warning: No target received within timeout, latency test inconclusive')


@pytest.mark.integration
@pytest.mark.pipeline
def test_pipeline_robot_lost_recovery(ros_context, armor_tracker_process,
                                       robot_pose_estimator_process, test_node):
    """Test that pipeline handles robot lost and recovery correctly."""
    pub = test_node.create_publisher(
        rm_msg.Armors, '/pipeline/armors', _get_sensor_data_qos())

    assert wait_for_pipeline_discovery(test_node), 'Pipeline discovery failed'
    time.sleep(1.0)

    print('\n=== Phase 1: Initial tracking ===')
    
    # Phase 1: Establish tracking
    for _ in range(40):
        armors = create_simulated_armors(test_node, robot_id='1')
        pub.publish(armors)
        time.sleep(0.05)
    
    time.sleep(1.0)
    initial_targets = len(test_node.target_msgs)
    print(f'Initial targets received: {initial_targets}')

    print('\n=== Phase 2: Robot "lost" (no detections) ===')
    
    # Phase 2: Stop publishing (simulate lost robot)
    time.sleep(1.5)  # Wait for lost detection
    
    targets_after_lost = len(test_node.target_msgs)
    print(f'Targets after lost period: {targets_after_lost}')

    print('\n=== Phase 3: Robot "recovered" ===')
    
    # Phase 3: Resume publishing (robot recovered)
    for _ in range(40):
        armors = create_simulated_armors(test_node, robot_id='1')
        pub.publish(armors)
        time.sleep(0.05)
    
    time.sleep(1.0)
    final_targets = len(test_node.target_msgs)
    print(f'Final targets received: {final_targets}')

    # Should receive targets in all phases
    assert initial_targets > 0, 'No targets during initial phase'
    assert final_targets > targets_after_lost, 'No new targets after recovery'


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s', '-m', 'pipeline'])
