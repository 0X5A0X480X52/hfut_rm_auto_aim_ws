#!/usr/bin/env python3
"""
Test script for armor fusion node
Publishes simulated armor detections from two cameras to test the fusion
"""

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import Armor, Armors
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math


class FusionTestPublisher(Node):
    """发布模拟的装甲板检测数据用于测试融合功能"""
    
    def __init__(self):
        super().__init__('fusion_test_publisher')
        
        # 创建两个发布者，模拟两个摄像头
        self.pub1 = self.create_publisher(
            Armors, 
            '/camera1/armor_detector/armors', 
            10
        )
        self.pub2 = self.create_publisher(
            Armors, 
            '/camera2/armor_detector/armors', 
            10
        )
        
        # 定时发布
        self.timer = self.create_timer(0.1, self.publish_test_data)
        self.count = 0
        
        self.get_logger().info('Fusion test publisher started')
        self.get_logger().info('Publishing simulated detections from two cameras')
    
    def create_armor(self, x, y, z, number='1', armor_type='small'):
        """创建装甲板消息"""
        armor = Armor()
        armor.number = number
        armor.type = armor_type
        armor.pose = Pose()
        armor.pose.position = Point(x=x, y=y, z=z)
        armor.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        armor.distance_to_image_center = math.sqrt(x**2 + y**2)
        return armor
    
    def publish_test_data(self):
        """发布测试数据"""
        self.count += 1
        
        # 模拟场景：有两个目标
        # 目标1: 在 (3.0, 1.0, 0.5) 位置
        # 目标2: 在 (4.0, -1.0, 0.3) 位置
        
        # 添加一些噪声
        noise_scale = 0.05
        
        # Camera 1 的观测（能看到两个目标，但有噪声）
        msg1 = Armors()
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg1.header.frame_id = 'camera1_optical_frame'
        
        # 目标1的观测（加噪声）
        noise1 = np.random.randn(3) * noise_scale
        armor1_cam1 = self.create_armor(
            3.0 + noise1[0], 
            1.0 + noise1[1], 
            0.5 + noise1[2], 
            '1', 
            'small'
        )
        msg1.armors.append(armor1_cam1)
        
        # 目标2的观测（加噪声）
        noise2 = np.random.randn(3) * noise_scale
        armor2_cam1 = self.create_armor(
            4.0 + noise2[0], 
            -1.0 + noise2[1], 
            0.3 + noise2[2], 
            '2', 
            'large'
        )
        msg1.armors.append(armor2_cam1)
        
        # Camera 2 的观测（也能看到两个目标，但有不同的噪声）
        msg2 = Armors()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = 'camera2_optical_frame'
        
        # 目标1的观测（加噪声）
        noise3 = np.random.randn(3) * noise_scale
        armor1_cam2 = self.create_armor(
            3.0 + noise3[0], 
            1.0 + noise3[1], 
            0.5 + noise3[2], 
            '1', 
            'small'
        )
        msg2.armors.append(armor1_cam2)
        
        # 目标2的观测（加噪声）
        noise4 = np.random.randn(3) * noise_scale
        armor2_cam2 = self.create_armor(
            4.0 + noise4[0], 
            -1.0 + noise4[1], 
            0.3 + noise4[2], 
            '2', 
            'large'
        )
        msg2.armors.append(armor2_cam2)
        
        # 发布
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        
        if self.count % 50 == 0:  # 每5秒打印一次
            self.get_logger().info(
                f'Published test data: '
                f'Camera1={len(msg1.armors)} armors, '
                f'Camera2={len(msg2.armors)} armors'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FusionTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
