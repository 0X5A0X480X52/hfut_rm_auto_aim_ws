# Video Player 测试示例

## 前提条件

确保你有一个测试视频文件。如果没有，可以使用以下方法之一获取：

### 方法 1: 从真实摄像头录制

如果你有 mindvision 摄像头：

```bash
# 终端 1: 启动摄像头
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
ros2 launch mindvision_camera mv_launch.py

# 终端 2: 录制视频（使用 OpenCV 或 ffmpeg）
# 或者使用 ROS2 bag:
ros2 bag record /image_raw /camera_info -o test_recording
```

### 方法 2: 使用测试视频

下载或使用任何 MP4 视频文件进行测试。

## 测试步骤

### 测试 1: 基本功能测试

```bash
# 启动 video_player
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
ros2 launch video_player video_player_launch.py video_path:=/path/to/test.mp4

# 预期输出:
# [INFO] Starting VideoPlayerNode!
# [INFO] Video file opened: /path/to/test.mp4
# [INFO] Resolution: 1280x1024
# [INFO] Original FPS: 30.00
# [INFO] Playback FPS: 30.00
# [INFO] Loop playback: true
# [INFO] Starting video playback!
```

### 测试 2: 查看图像

在新终端中：

```bash
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在 rqt 窗口选择 `/image_raw` 话题，应该能看到视频播放。

### 测试 3: 验证话题

```bash
# 列出话题
ros2 topic list
# 应该看到:
# /image_raw
# /camera_info

# 查看图像话题频率
ros2 topic hz /image_raw
# 应该显示接近设置的 fps

# 查看图像信息
ros2 topic echo /camera_info --once
```

### 测试 4: 自定义帧率

```bash
# 以 60 fps 播放
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/test.mp4 \
  fps:=60.0

# 验证帧率
ros2 topic hz /image_raw
# 应该显示接近 60 Hz
```

### 测试 5: 禁用循环播放

```bash
# 单次播放
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/test.mp4 \
  loop_playback:=false

# 视频播放结束后节点会自动停止
```

### 测试 6: 图像翻转

```bash
# 翻转图像
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/test.mp4 \
  flip_image:=true

# 在 rqt_image_view 中查看，图像应该上下左右都翻转了
```

### 测试 7: 使用便捷脚本

```bash
cd /home/amatrix/hfut_rm_auto_aim_ws/src/rm_hardware_driver/video_player/scripts
./run_video_player.sh /path/to/test.mp4 30 true
```

### 测试 8: 与下游节点集成

假设你有一个目标检测节点订阅 `/image_raw`：

```bash
# 终端 1: 启动 video_player
ros2 launch video_player video_player_launch.py video_path:=/path/to/test.mp4

# 终端 2: 启动你的目标检测节点
ros2 run your_package detector_node
```

video_player 应该能够无缝替代真实摄像头。

## 常见测试场景

### 场景 1: 替代真实摄像头进行调试

```bash
# 原来的启动命令（使用真实摄像头）
# ros2 launch mindvision_camera mv_launch.py

# 现在使用 video_player 替代
ros2 launch video_player video_player_launch.py \
  video_path:=~/Videos/recorded_test.mp4
```

### 场景 2: 回归测试

```bash
# 使用固定的测试视频进行自动化测试
ros2 launch video_player video_player_launch.py \
  video_path:=~/test_data/regression_test.mp4 \
  loop_playback:=false
```

### 场景 3: 性能测试

```bash
# 测试高帧率处理能力
ros2 launch video_player video_player_launch.py \
  video_path:=~/Videos/high_fps_test.mp4 \
  fps:=120.0
```

## 验证清单

- [ ] 节点正常启动
- [ ] 视频文件成功打开
- [ ] `/image_raw` 话题正常发布
- [ ] `/camera_info` 话题正常发布
- [ ] 帧率符合预期
- [ ] rqt_image_view 能正常显示图像
- [ ] 循环播放正常工作
- [ ] 图像翻转功能正常
- [ ] 能与下游节点正常集成
- [ ] 节点关闭时资源正常释放

## 故障排除

### 问题 1: 找不到 video_player 包

```bash
# 解决方案: 重新 source 环境
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
```

### 问题 2: 视频文件无法打开

```bash
# 检查文件是否存在
ls -lh /path/to/test.mp4

# 检查文件权限
chmod 644 /path/to/test.mp4

# 尝试用 OpenCV 直接测试
python3 -c "import cv2; cap = cv2.VideoCapture('/path/to/test.mp4'); print('OK' if cap.isOpened() else 'FAIL')"
```

### 问题 3: rqt_image_view 看不到图像

```bash
# 检查 QoS 设置
ros2 topic info /image_raw -v

# 尝试使用 sensor_data QoS
ros2 launch video_player video_player_launch.py \
  video_path:=/path/to/test.mp4 \
  use_sensor_data_qos:=true
```

## 性能基准

在标准配置下（1280x1024, 30fps），预期性能：

- CPU 使用率: < 10%
- 内存使用: < 100MB
- 延迟: < 50ms
- 帧率稳定性: ±2%

## 下一步

测试通过后，可以：

1. 将 video_player 集成到你的视觉处理管道
2. 创建自己的测试视频集
3. 编写自动化测试脚本
4. 配置自定义的相机参数

## 反馈

如果遇到问题或有改进建议，请：

1. 检查日志输出
2. 查阅 README.md 和 QUICKSTART.md
3. 查看 CHANGELOG.md 了解已知问题
