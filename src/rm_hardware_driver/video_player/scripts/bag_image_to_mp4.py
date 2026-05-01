#!/usr/bin/env python3
import os
import sqlite3
import argparse

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def get_topic_id(cursor, topic_name):
    cursor.execute("SELECT id, type FROM topics WHERE name = ?", (topic_name,))
    result = cursor.fetchone()
    if result is None:
        raise RuntimeError(f"Topic not found: {topic_name}")

    topic_id, topic_type = result
    if topic_type != "sensor_msgs/msg/Image":
        raise RuntimeError(f"Topic type is {topic_type}, not sensor_msgs/msg/Image")

    return topic_id


def find_db3_file(bag_dir):
    for file in os.listdir(bag_dir):
        if file.endswith(".db3"):
            return os.path.join(bag_dir, file)
    raise RuntimeError(f"No .db3 file found in {bag_dir}")


def convert_bag_to_mp4(bag_dir, topic, output, fps):
    db3_path = find_db3_file(bag_dir)

    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    topic_id = get_topic_id(cursor, topic)

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,),
    )

    bridge = CvBridge()
    writer = None
    frame_count = 0

    for timestamp, data in cursor:
        msg = deserialize_message(data, Image)
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(output, fourcc, fps, (width, height))

            if not writer.isOpened():
                raise RuntimeError(f"Failed to open video writer: {output}")

            print(f"Video size: {width}x{height}")
            print(f"FPS: {fps}")

        writer.write(frame)
        frame_count += 1

    if writer is not None:
        writer.release()

    conn.close()

    print(f"Done. Saved {frame_count} frames to {output}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", help="ROS2 bag directory")
    parser.add_argument(
        "--topic",
        default="/camera/image_raw",
        help="Image topic name",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="output.mp4",
        help="Output mp4 file",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Output video FPS",
    )

    args = parser.parse_args()

    rclpy.init()
    convert_bag_to_mp4(args.bag_dir, args.topic, args.output, args.fps)
    rclpy.shutdown()


if __name__ == "__main__":
    main()