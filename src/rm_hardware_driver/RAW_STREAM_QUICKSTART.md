# YAW 原始数据流功能使用说明

## 快速开始

### 1. 配置

编辑相应的配置文件启用录制：

**HikCamera**:
```yaml
# src/rm_hardware_driver/ros2_hik_camera/config/camera_params.yaml
raw_stream:
  enabled: true                    # 启用录制
  path: /tmp/hik_camera.yaw       # 输出路径
  interval: 1                      # 每帧保存（1=每帧，2=每2帧）
```

**MindVision**:
```yaml
# src/rm_hardware_driver/ros2_mindvision_camera/config/camera_params.yaml
raw_stream:
  enabled: true                    # 启用录制
  path: /tmp/mv_camera.yaw        # 输出路径
  interval: 1                      # 每帧保存
```

### 2. 启动相机

```bash
# HikCamera
ros2 launch ros2_hik_camera hik_camera_launch.py

# MindVision
ros2 launch mindvision_camera mv_launch.py
```

### 3. 解码录制的文件

```bash
# 生成视频 (默认)
python3 src/rm_hardware_driver/ros2_hik_camera/scripts/decode_yaw.py /tmp/hik_camera.yaw

# 导出图像序列
python3 src/rm_hardware_driver/ros2_hik_camera/scripts/decode_yaw.py /tmp/hik_camera.yaw --mode images

# 查看帮助
python3 src/rm_hardware_driver/ros2_hik_camera/scripts/decode_yaw.py --help
```

## 动态控制

运行时修改参数（无需重启节点）：

```bash
# 启用录制
ros2 param set /hik_camera raw_stream.enabled true

# 禁用录制
ros2 param set /hik_camera raw_stream.enabled false

# 更改输出路径（会停止当前录制）
ros2 param set /hik_camera raw_stream.path /home/user/data/$(date +%Y%m%d_%H%M%S).yaw

# 调整采样间隔（不影响相机真实帧率）
ros2 param set /hik_camera raw_stream.interval 2  # 每2帧保存一次
```

## 重要说明

1. **采样间隔**: `interval` 参数只控制录制时的采样频率，不影响相机的实际帧率
2. **磁盘空间**: 原始数据占用空间大，建议使用 `/tmp` 或确保有足够空间
3. **性能**: 使用独立线程异步写入，不会阻塞相机采集

## 详细文档

完整功能说明和高级用法请参考: [YAW_RAW_STREAM_GUIDE.md](YAW_RAW_STREAM_GUIDE.md)
