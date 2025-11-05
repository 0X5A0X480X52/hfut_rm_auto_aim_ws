#!/usr/bin/env python3
"""
Simple ROS2 recorder node: subscribes to /image_raw and writes MP4 using OpenCV VideoWriter.
Defaults to writing BGR8 images as MP4 (mp4v). Use parameters to change path/fps/topic/codec.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        # parameters
        self.declare_parameter('out_path', os.path.expanduser('~/camera_output.mp4'))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('codec', 'mp4v')  # fourcc
        self.declare_parameter('auto_start', True)

        self.out_path = self.get_parameter('out_path').get_parameter_value().string_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.codec = self.get_parameter('codec').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.writer = None
        self.lock = threading.Lock()
        self.frame_count = 0

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.sub = self.create_subscription(Image, self.topic, self.image_cb, qos_profile)
        self.get_logger().info(f"Recorder subscribed to {self.topic}, will write to {self.out_path}")

    def image_cb(self, msg: Image):
        try:
            # Convert to cv image (BGR8 expected)
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
                # init writer
                fourcc = cv2.VideoWriter_fourcc(*self.codec)
                # ensure output dir exists
                out_dir = os.path.dirname(self.out_path)
                if out_dir and not os.path.exists(out_dir):
                    os.makedirs(out_dir, exist_ok=True)
                self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
                if not self.writer.isOpened():
                    try:
                        self.get_logger().error(f"Failed to open VideoWriter with path {self.out_path}")
                    except Exception:
                        pass
                    self.writer = None
                    return
                try:
                    self.get_logger().info(f"Started VideoWriter: {self.out_path} ({w}x{h} @ {self.fps}fps, codec={self.codec})")
                except Exception:
                    pass

            if self.writer:
                self.writer.write(cv_img)
                self.frame_count += 1

    def destroy_node(self):
        with self.lock:
            if self.writer:
                try:
                    self.get_logger().info(f"Releasing VideoWriter, wrote {self.frame_count} frames")
                except Exception:
                    pass
                try:
                    self.writer.release()
                except Exception:
                    try:
                        self.get_logger().warn("Error releasing writer")
                    except Exception:
                        pass
                self.writer = None
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
