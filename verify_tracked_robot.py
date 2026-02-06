#!/usr/bin/env python3
"""
验证 TrackedRobot 消息的完整性
"""
import rclpy
from rclpy.node import Node
from rm_interfaces.msg import TrackedRobots
import sys

class TrackedRobotVerifier(Node):
    def __init__(self):
        super().__init__('tracked_robot_verifier')
        self.sub = self.create_subscription(
            TrackedRobots,
            '/max_entropy_tracker/tracked_robots',
            self.callback,
            10
        )
        self.count = 0
        self.get_logger().info("Waiting for TrackedRobots messages...")
    
    def callback(self, msg: TrackedRobots):
        self.count += 1
        
        for robot in msg.robots:
            print(f"\n{'='*60}")
            print(f"[Message #{self.count}] Robot ID: {robot.robot_id}")
            print(f"{'='*60}")
            print(f"Robot Type: {robot.robot_type}")
            print(f"Track State: {robot.track_state}")
            print(f"\n--- Position & Motion ---")
            print(f"Center Position: ({robot.center_position.x:.3f}, {robot.center_position.y:.3f}, {robot.center_position.z:.3f})")
            print(f"Center Velocity: ({robot.center_velocity.x:.3f}, {robot.center_velocity.y:.3f}, {robot.center_velocity.z:.3f})")
            print(f"Center Acceleration: ({robot.center_acceleration.x:.3f}, {robot.center_acceleration.y:.3f}, {robot.center_acceleration.z:.3f})")
            print(f"Yaw: {robot.yaw:.3f} rad")
            print(f"Yaw Velocity: {robot.yaw_velocity:.3f} rad/s")
            print(f"Yaw Acceleration: {robot.yaw_acceleration:.3f} rad/s²")
            
            print(f"\n--- Geometry ---")
            print(f"Radius: r1={robot.radius:.3f}, r2={robot.radius_2:.3f}")
            print(f"Height Offset: d_za={robot.d_za:.3f}, d_zc={robot.d_zc:.3f}")
            print(f"Number of Armors: {robot.num_armors}")
            
            print(f"\n--- Armors Offset (Robot Frame) ---")
            print(f"Armors Offset Length: {len(robot.armors_offset)}")
            if len(robot.armors_offset) != robot.num_armors:
                print(f"❌ ERROR: armors_offset length ({len(robot.armors_offset)}) != num_armors ({robot.num_armors})")
            else:
                print(f"✓ armors_offset length matches num_armors")
            
            for i, pose in enumerate(robot.armors_offset):
                print(f"  Armor {i}: pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
            
            print(f"\n--- Covariance Matrix ---")
            print(f"Covariance Dimension: {robot.covariance_dim}")
            print(f"Covariance Data Length: {len(robot.state_covariance)}")
            if robot.covariance_dim > 0:
                expected_len = robot.covariance_dim ** 2
                if len(robot.state_covariance) != expected_len:
                    print(f"❌ ERROR: covariance length ({len(robot.state_covariance)}) != dim² ({expected_len})")
                else:
                    print(f"✓ Covariance length matches dim²")
                    # Print diagonal elements
                    print(f"  Diagonal elements (variance):")
                    for i in range(robot.covariance_dim):
                        idx = i * robot.covariance_dim + i
                        print(f"    State[{i}]: {robot.state_covariance[idx]:.6f}")
            else:
                print("  (Covariance not available)")
            
            print(f"\n--- Tracking Info ---")
            print(f"Bound Armor IDs: {robot.bound_armor_ids}")
            if len(robot.bound_armor_ids) == 0:
                print(f"❌ WARNING: bound_armor_ids is empty")
            else:
                print(f"✓ bound_armor_ids: {robot.bound_armor_ids}")
            
            print(f"Confidence: {robot.confidence:.2f}")
            print(f"Is Visible: {robot.is_visible}")
            print(f"Visible Armor Count: {robot.visible_armor_count}")
            if robot.is_visible and robot.visible_armor_count == 0:
                print(f"❌ WARNING: is_visible=True but visible_armor_count=0")
            else:
                print(f"✓ Visibility info consistent")
            
            print(f"{'='*60}\n")

def main():
    rclpy.init()
    node = TrackedRobotVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
