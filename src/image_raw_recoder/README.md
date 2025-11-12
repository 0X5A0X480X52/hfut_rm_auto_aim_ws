image_raw_recoder
=================

A small ROS2 Python package that provides two ways to produce MP4 from camera image topic:

1. recorder_node: subscribes to `/image_raw` and writes MP4 in real-time using OpenCV VideoWriter. (default)
2. bag_to_mp4: helper that runs `ros2 bag play <bag>` and simultaneously subscribes to `/image_raw` to write MP4.

3. multi_recorder: subscribe to multiple image topics and write each to its own MP4 file, only writing frames when all topics have synchronized timestamps within a tolerance. Useful for stereo camera testing.

Example:

```bash
# record two camera topics simultaneously to separate files (topics param is an array)
ros2 run image_raw_recoder multi_recorder --ros-args -p topics:=[/left/image_raw,/right/image_raw] -p fps:=30.0

# use template for output paths; {topic} will be replaced with sanitized topic name
ros2 run image_raw_recoder multi_recorder --ros-args -p topics:=[/left/image_raw,/right/image_raw] -p out_paths:=["~/videos/{topic}.mp4","~/videos/{topic}.mp4"] -p sync_tolerance:=0.02
```

Install
-------

From the workspace root:

```bash
colcon build --packages-select image_raw_recoder
source install/setup.bash
```

Run recorder_node (default method):

```bash
ros2 run image_raw_recoder recorder_node --ros-args -p out_path:=~/camera_output.mp4 -p fps:=30.0 -p topic:=/image_raw
```

Record ros topics to bag and then convert via bag_to_mp4:

```bash
# record first (in another terminal)
ros2 bag record /image_raw /camera_info -o my_recording

# then convert
ros2 run image_raw_recoder bag_to_mp4 <path_to_bag_directory> --out ~/bag_output.mp4 --fps 30
```

Notes
-----

- The node expects BGR8-encoded images on the topic. If your camera publishes `rgb8` or another encoding, adjust the cv_bridge conversion in the code.
- For H.264 proper encoding, ensure your OpenCV build supports libx264; otherwise use `mp4v` or perform ffmpeg post-processing.
- The package depends on `cv_bridge` and `rclpy`.
