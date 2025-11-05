#!/usr/bin/env python3
"""
Simple helper: play a ros2 bag using subprocess and record /image_raw into mp4 using the same recorder logic.
Usage: bag_to_mp4 <bag_path> [--out output.mp4] [--fps 30]

This script will spawn `ros2 bag play <bag_path>` and simultaneously subscribe to the topic and write frames.
When the bag playback process exits, the recorder stops and file is closed.
"""
import argparse
import subprocess
import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BagRecorder(Node):
    def __init__(self, out_path, fps, topic='/image_raw', codec='mp4v'):
        super().__init__('bag_to_mp4_recorder')
        self.bridge = CvBridge()
        self.out_path = out_path
        self.fps = fps
        self.topic = topic
        self.codec = codec
        self.writer = None
        self.lock = threading.Lock()
        qos = rclpy.qos.QoSProfile(depth=10)
        self.sub = self.create_subscription(Image, self.topic, self.cb, qos)
        self.get_logger().info(f"BagRecorder will write to {self.out_path}")
        self.frame_count = 0

    def cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            try:
                self.get_logger().error(f"cv_bridge conversion failed: {e}")
            except Exception:
                pass
            return
        h, w = cv_img.shape[:2]
        with self.lock:
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*self.codec)
                out_dir = os.path.dirname(self.out_path)
                if out_dir and not os.path.exists(out_dir):
                    os.makedirs(out_dir, exist_ok=True)
                self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
                if not self.writer.isOpened():
                    try:
                        self.get_logger().error(f"Failed to open VideoWriter {self.out_path}")
                    except Exception:
                        pass
                    self.writer = None
                    return
                try:
                    self.get_logger().info(f"Started VideoWriter: {self.out_path}")
                except Exception:
                    pass
            if self.writer:
                self.writer.write(cv_img)
                self.frame_count += 1

    def stop_and_release(self):
        with self.lock:
            if self.writer:
                try:
                    self.get_logger().info(f"Stopping writer, wrote {self.frame_count} frames")
                except Exception:
                    pass
                try:
                    self.writer.release()
                except Exception as ex:
                    try:
                        self.get_logger().warn(f"Error releasing: {ex}")
                    except Exception:
                        pass
                self.writer = None


def main():
    parser = argparse.ArgumentParser(description='Convert ros2 bag -> mp4 by playing bag and subscribing to /image_raw')
    parser.add_argument('bag', help='path to ros2 bag directory')
    parser.add_argument('--out', default=os.path.expanduser('~/bag_output.mp4'))
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--topic', default='/image_raw')
    parser.add_argument('--codec', default='mp4v')
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"Bag path does not exist: {args.bag}")
        return

    rclpy.init()
    node = BagRecorder(args.out, args.fps, topic=args.topic, codec=args.codec)

    play_cmd = ['ros2', 'bag', 'play', args.bag]
    print(f"Starting ros2 bag play: {' '.join(play_cmd)}")
    proc = subprocess.Popen(play_cmd)

    try:
        while proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.001)
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        proc.terminate()
    finally:
        node.stop_and_release()
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            proc.terminate()

if __name__ == '__main__':
    main()
