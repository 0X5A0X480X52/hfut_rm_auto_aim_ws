#!/usr/bin/env python3
"""
Multi-topic synchronized recorder for ROS2.

Subscribes to multiple image topics and writes each topic to its own MP4 file.
A synchronized frame is written only when every topic has an image whose
timestamps are within `sync_tolerance` seconds of each other. This ensures
that each written frame across files is time-aligned for stereo testing.

Parameters (ROS2 params):
 - topics: list of topic names to subscribe to (default ['/image_raw'])
 - out_paths: list of output file paths (same length as topics). If a single
   path is provided and contains '{topic}', that placeholder will be replaced
   by a sanitized topic name. If omitted, outputs default to '~/topicname.mp4'.
 - fps: target frames per second for output files (default: 30.0)
 - codec: fourcc codec for VideoWriter (default: 'mp4v')
 - sync_tolerance: maximum allowed time difference (seconds) between the
   earliest and latest image in a synchronized set (default: 0.02s)

Usage example:
 ros2 run image_raw_recoder multi_recorder --ros-args -p topics:=[/left/image_raw,/right/image_raw] -p fps:=30.0

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
import time
from typing import List


def _topic_to_filename(topic: str) -> str:
    # sanitize topic into a safe filename
    return topic.strip('/').replace('/', '_') or 'topic'


class MultiRecorder(Node):
    def __init__(self):
        super().__init__('multi_recorder')
        # declare parameters
        self.declare_parameter('topics', ['/image_raw'])
        self.declare_parameter('out_paths', [])
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('codec', 'mp4v')
        self.declare_parameter('sync_tolerance', 0.02)
        self.declare_parameter('queue_size', 10)

        topics = self.get_parameter('topics').get_parameter_value().string_array_value
        # If topics param is empty, fall back to single default
        if not topics:
            topics = ['/image_raw']

        self.topics: List[str] = list(topics)
        self.out_paths_param = self.get_parameter('out_paths').get_parameter_value().string_array_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.codec = self.get_parameter('codec').get_parameter_value().string_value
        self.sync_tolerance = float(self.get_parameter('sync_tolerance').get_parameter_value().double_value)
        self.queue_size = int(self.get_parameter('queue_size').get_parameter_value().integer_value)

        # Build out_paths for each topic
        self.out_paths: List[str] = []
        if self.out_paths_param and len(self.out_paths_param) >= len(self.topics):
            self.out_paths = list(self.out_paths_param[:len(self.topics)])
        else:
            # try template replacement or default naming
            template = self.out_paths_param[0] if self.out_paths_param else ''
            for t in self.topics:
                if template and '{topic}' in template:
                    path = template.replace('{topic}', _topic_to_filename(t))
                else:
                    path = os.path.expanduser(f'~/{_topic_to_filename(t)}.mp4')
                self.out_paths.append(path)

        if len(self.out_paths) != len(self.topics):
            raise RuntimeError('Number of out_paths must match number of topics')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # per-topic latest messages (msg, timestamp_float)
        self.latest = {t: None for t in self.topics}
        # video writers
        self.writers = {t: None for t in self.topics}
        # frame sizes stored when writer initialized
        self.sizes = {t: None for t in self.topics}

        qos = rclpy.qos.QoSProfile(depth=self.queue_size)
        self.subs = []
        for t in self.topics:
            sub = self.create_subscription(Image, t, lambda msg, topic=t: self.image_cb(topic, msg), qos)
            self.subs.append(sub)

        self.last_written_time = 0.0
        self.frame_count = 0
        self.get_logger().info(f"MultiRecorder subscribed to {self.topics}, outputs: {self.out_paths}")

    def _msg_time(self, msg: Image) -> float:
        # header.stamp: builtin_interfaces/Time
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def image_cb(self, topic: str, msg: Image):
        try:
            ts = self._msg_time(msg)
        except Exception:
            # some images may not have header - use wall time
            ts = time.time()

        with self.lock:
            self.latest[topic] = (msg, ts)

            # check if all topics have a recent message
            if any(self.latest[t] is None for t in self.topics):
                return

            times = [self.latest[t][1] for t in self.topics]
            t_min = min(times)
            t_max = max(times)
            if t_max - t_min > self.sync_tolerance:
                # not synchronized enough yet
                return
            if t_max <= self.last_written_time:
                # nothing newer than last written
                return

            # we have a synchronized set to write
            for idx, t in enumerate(self.topics):
                msg_i, ts_i = self.latest[t]
                try:
                    cv_img = self.bridge.imgmsg_to_cv2(msg_i, desired_encoding='bgr8')
                except Exception as e:
                    # skip on conversion error
                    self.get_logger().error(f"cv_bridge conversion failed for {t}: {e}")
                    return

                h, w = cv_img.shape[:2]
                if self.writers[t] is None:
                    # init writer for this topic
                    fourcc = cv2.VideoWriter_fourcc(*self.codec)
                    out_dir = os.path.dirname(self.out_paths[idx])
                    if out_dir and not os.path.exists(out_dir):
                        os.makedirs(out_dir, exist_ok=True)
                    writer = cv2.VideoWriter(self.out_paths[idx], fourcc, self.fps, (w, h))
                    if not writer.isOpened():
                        self.get_logger().error(f"Failed to open VideoWriter for {self.out_paths[idx]}")
                        self.writers[t] = None
                        return
                    self.writers[t] = writer
                    self.sizes[t] = (w, h)
                    self.get_logger().info(f"Started writer for {t} -> {self.out_paths[idx]} ({w}x{h})")

                # if size mismatch, skip writing this frame (or could resize)
                if self.sizes[t] != (w, h):
                    self.get_logger().warning(f"Size changed for {t}, skipping frame (was {self.sizes[t]}, now {(w,h)})")
                    return

                # write frame
                try:
                    self.writers[t].write(cv_img)
                except Exception as e:
                    self.get_logger().error(f"Error writing frame for {t}: {e}")
                    return

            # advance last_written_time and counters
            self.last_written_time = t_max
            self.frame_count += 1

    def destroy_node(self):
        with self.lock:
            for t, w in self.writers.items():
                if w:
                    try:
                        self.get_logger().info(f"Releasing writer for {t}")
                    except Exception:
                        pass
                    try:
                        w.release()
                    except Exception:
                        pass
            self.writers = {t: None for t in self.topics}
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MultiRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
