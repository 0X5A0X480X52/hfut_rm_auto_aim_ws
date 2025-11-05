# Video Player 快速入门

## 1. 简介

`video_player` 是一个 ROS2 虚拟摄像头包，用于从视频文件读取图像并发布到 `/image_raw` 话题。这对于调试计算机视觉算法非常有用，无需实际的摄像头硬件。

## 2. 快速开始

### 2.1 基本使用

```bash
# Source ROS2 环境
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash

# 启动节点（需要指定视频路径）
ros2 launch video_player video_player_launch.py video_path:=/path/to/your/video.mp4
```

### 2.2 使用便捷脚本

```bash
# 使用提供的脚本启动
cd /home/amatrix/hfut_rm_auto_aim_ws/src/rm_hardware_driver/video_player/scripts
./run_video_player.sh /path/to/video.mp4 30 true
```

参数说明：
- 参数1: 视频文件路径（必需）
- 参数2: 播放帧率（可选，默认30）
- 参数3: 是否循环播放（可选，默认true）

## 3. 查看视频流

### 3.1 使用 rqt_image_view

```bash
# 在新终端中
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

然后在 rqt 界面选择 `/image_raw` 话题。

### 3.2 使用命令行查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看图像话题详情
ros2 topic info /image_raw

# 查看相机信息话题
ros2 topic echo /camera_info
```

## 4. 常用启动参数

### 4.1 自定义帧率

```bash
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/video.mp4 \
  fps:=60.0
```

### 4.2 禁用循环播放

```bash
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/video.mp4 \
  loop_playback:=false
```

### 4.3 翻转图像

```bash
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/video.mp4 \
  flip_image:=true
```

### 4.4 使用自定义相机标定

```bash
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/video.mp4 \
  camera_info_url:=file:///path/to/custom_camera_info.yaml
```

## 5. 与其他节点集成

`video_player` 发布的话题与 `ros2_mindvision_camera` 完全兼容，可以直接替换使用：

```bash
# 原来使用 mindvision 摄像头
ros2 launch mindvision_camera mv_launch.py

# 现在可以用 video_player 替换进行测试
ros2 launch video_player video_player_launch.py video_path:=/path/to/test.mp4
```

## 6. 录制测试视频

如果需要录制实际摄像头的视频用于后续测试：

```bash
# 录制话题到 bag 文件
ros2 bag record /image_raw /camera_info

# 或者直接从摄像头录制视频
# (使用 OpenCV 或其他工具)
```

## 7. 调试技巧

### 7.1 查看节点日志

```bash
ros2 run video_player video_player_node \
  --ros-args \
  -p video_path:=/path/to/video.mp4 \
  --log-level debug
```

### 7.2 监控发布频率

```bash
ros2 topic hz /image_raw
```

### 7.3 检查图像尺寸

```bash
ros2 topic echo /camera_info | grep -A 2 "width\|height"
```

## 8. 支持的视频格式

由于使用 OpenCV，支持常见的所有视频格式：
- MP4 (推荐)
- AVI
- MOV
- MKV
- WEBM
- FLV

## 9. 性能建议

1. **帧率设置**: 根据实际需要设置合适的帧率，过高的帧率会增加 CPU 负担
2. **视频分辨率**: 使用与实际应用接近的分辨率
3. **循环播放**: 对于长时间测试，建议启用循环播放
4. **QoS 设置**: 根据订阅者的需求选择合适的 QoS 配置

## 10. 故障排除

### 问题：视频无法打开
**解决方案**: 
- 检查文件路径是否正确
- 确认文件格式被 OpenCV 支持
- 检查文件是否损坏

### 问题：图像不显示
**解决方案**:
- 使用 `ros2 topic list` 确认话题存在
- 检查 QoS 设置是否匹配
- 查看节点日志排查错误

### 问题：帧率不稳定
**解决方案**:
- 降低播放帧率
- 检查系统负载
- 确认视频文件没有损坏

## 11. 更多信息

详细文档请参考: [README.md](../README.md)
