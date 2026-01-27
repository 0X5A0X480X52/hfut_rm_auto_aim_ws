# YAW 格式原始数据流保存功能

## 功能说明

为 HikCamera 和 MindVision 相机驱动添加了原始数据流录制功能，可以将相机的原始 RAW 数据保存为 YAW 格式文件，用于离线分析、调试和数据集制作。

## YAW 文件格式

YAW (Yet Another raW) 是一种简单的二进制容器格式：

### 文件结构
```
[ASCII 头部]
YAWFMT
WIDTH <宽度>
HEIGHT <高度>
PIXELTYPE <像素类型> (HikCamera) 或 MEDIATYPE <媒体类型> (MindVision)
FPS <帧率>
INTERVAL <采样间隔>
===DATA===

[二进制帧数据]
每帧: <8字节时间戳(微秒,小端)><4字节数据长度(小端)><原始数据>
...

[ASCII 尾部]
===META===
FRAMES <总帧数>
```

## 配置参数

### HikCamera (`ros2_hik_camera/config/camera_params.yaml`)

```yaml
/hik_camera:
  ros__parameters:
    # ... 其他参数 ...
    
    raw_stream:
      enabled: false              # 是否启用原始流录制
      path: /tmp/hik_camera.yaw   # 输出文件路径
      interval: 1                 # 采样间隔 (1=每帧, 2=每2帧, ...)
```

### MindVision (`ros2_mindvision_camera/config/camera_params.yaml`)

```yaml
/mv_camera:
  ros__parameters:
    # ... 其他参数 ...
    
    raw_stream:
      enabled: false              # 是否启用原始流录制
      path: /tmp/mv_camera.yaw    # 输出文件路径
      interval: 1                 # 采样间隔 (1=每帧, 2=每2帧, ...)
```

## 参数说明

- **enabled**: 设置为 `true` 启用原始流录制
- **path**: YAW 文件保存路径（建议使用 `/tmp` 或专用数据目录）
- **interval**: 采样频率控制
  - `1`: 保存每一帧
  - `2`: 每两帧保存一次
  - `N`: 每 N 帧保存一次
  - **注意**: 此参数仅控制录制采样率，不影响相机的实际帧率

## 使用方法

### 1. 启用录制

编辑配置文件，设置 `raw_stream.enabled: true`:

```bash
# HikCamera
nano src/rm_hardware_driver/ros2_hik_camera/config/camera_params.yaml

# MindVision
nano src/rm_hardware_driver/ros2_mindvision_camera/config/camera_params.yaml
```

### 2. 启动相机节点

```bash
# 重新编译 (如果修改了代码)
colcon build --packages-select ros2_hik_camera ros2_mindvision_camera

# 启动 HikCamera
ros2 launch ros2_hik_camera hik_camera_launch.py

# 或启动 MindVision
ros2 launch ros2_mindvision_camera mv_launch.py
```

### 3. 运行时动态控制

可以通过 ROS 参数动态启用/禁用录制：

```bash
# 启用录制
ros2 param set /hik_camera raw_stream.enabled true

# 禁用录制
ros2 param set /hik_camera raw_stream.enabled false

# 更改输出路径 (会停止当前录制)
ros2 param set /hik_camera raw_stream.path /home/user/data/capture.yaw

# 调整采样间隔
ros2 param set /hik_camera raw_stream.interval 2
```

## 解码 YAW 文件

两个相机包都提供了 `decode_yaw.py` 解码脚本。

### 基本用法

```bash
# 解码为视频 (默认模式)
python3 src/rm_hardware_driver/ros2_hik_camera/scripts/decode_yaw.py /tmp/hik_camera.yaw

# 指定输出目录和文件名
python3 decode_yaw.py /tmp/hik_camera.yaw --out_dir ./output --out_video video.mp4

# 解码为图像序列
python3 decode_yaw.py /tmp/hik_camera.yaw --mode images --format png

# 导出原始帧数据
python3 decode_yaw.py /tmp/hik_camera.yaw --mode raw
```

### 完整参数

```bash
python3 decode_yaw.py <input.yaw> [选项]

必需参数:
  input.yaw          输入的 YAW 文件

可选参数:
  --out_dir DIR      输出目录 (默认: decoded)
  --mode MODE        解码模式: ffmpeg|images|raw (默认: ffmpeg)
  --out_video FILE   输出视频文件名 (默认: output.mp4)
  --save_frames      ffmpeg 模式下同时保存图像
  --fps FPS          覆盖帧率 (0=使用文件头中的值)
  --format FMT       图像格式: png|jpg (默认: png)
```

### 解码模式

1. **ffmpeg 模式** (默认)
   - 使用 ffmpeg 编码为 H.264 视频
   - 如果 ffmpeg 不可用，回退到 OpenCV VideoWriter
   - 输出: `output.mp4`

2. **images 模式**
   - 保存每一帧为独立图像
   - 输出: `frame_000000.png`, `frame_000001.png`, ...

3. **raw 模式**
   - 导出原始帧数据（无解码）
   - 输出: `frame_000000.raw`, `frame_000001.raw`, ...

## 性能考虑

### 存储空间

原始数据文件较大，请确保有足够磁盘空间：

- **BayerRG8 格式** (HikCamera): ~width × height 字节/帧
- **RGB24 格式** (MindVision 处理后): ~width × height × 3 字节/帧

示例 (640×480, 30fps, 1分钟):
- Bayer: ~640 × 480 × 30 × 60 ≈ 553 MB
- RGB24: ~640 × 480 × 3 × 30 × 60 ≈ 1.66 GB

### 采样间隔优化

如果磁盘空间受限或不需要完整帧率，使用 `interval` 参数降低采样率：

```yaml
raw_stream:
  interval: 2  # 15fps (相机30fps时)
  interval: 3  # 10fps (相机30fps时)
```

### 异步写入

录制使用独立线程和队列缓冲，不会阻塞相机采集线程。队列大小：256 帧。

## 故障排除

### 问题：队列已满，丢帧

```
WARN: Raw stream queue full, dropping frames
```

**原因**: 磁盘写入速度不足

**解决**:
1. 增大 `interval` 降低采样率
2. 使用更快的存储设备 (SSD)
3. 降低相机帧率

### 问题：解码失败

```
Warning: Could not decode frame X
```

**原因**: 帧数据格式不匹配或损坏

**解决**:
1. 使用 `--mode raw` 导出原始数据检查
2. 确认相机像素格式设置正确
3. 检查文件是否完整 (查看 `===META===` 部分)

### 问题：ffmpeg 编码失败

**解决**:
1. 安装 ffmpeg: `sudo apt install ffmpeg`
2. 或使用 OpenCV 回退: 脚本会自动尝试
3. 使用 `--mode images` 保存为图像序列

## 技术细节

### 线程安全

- 使用 `std::mutex` 和 `std::condition_variable` 保护队列
- 独立工作线程进行文件写入
- 优雅关闭机制避免数据丢失

### 时间戳

每帧包含微秒级时间戳 (`std::chrono::system_clock`)，可用于:
- 帧率分析
- 时间同步
- 性能测试

### 兼容性

- **HikCamera**: 记录 `MV_FRAME_OUT_INFO_EX` 中的原始数据
- **MindVision**: 记录 `tSdkFrameHead` 中的原始数据
- 两种格式可用同一解码脚本处理

## 示例工作流程

### 1. 录制调试数据

```bash
# 启用录制
ros2 param set /hik_camera raw_stream.enabled true
ros2 param set /hik_camera raw_stream.path /tmp/debug_$(date +%Y%m%d_%H%M%S).yaw

# 运行测试...
# (录制自动进行)

# 停止录制
ros2 param set /hik_camera raw_stream.enabled false
```

### 2. 离线分析

```bash
# 解码为视频查看
python3 decode_yaw.py /tmp/debug_*.yaw --out_dir analysis

# 提取特定帧为图像
python3 decode_yaw.py /tmp/debug_*.yaw --mode images --format png
```

### 3. 制作数据集

```bash
# 每5帧采样一次录制
ros2 param set /hik_camera raw_stream.interval 5

# 解码为标注友好的PNG序列
python3 decode_yaw.py capture.yaw --mode images --format png --out_dir dataset/images
```

## 相关文件

### HikCamera
- 源码: `src/rm_hardware_driver/ros2_hik_camera/src/ros2_hik_camera_node.cpp`
- 头文件: `src/rm_hardware_driver/ros2_hik_camera/include/ros2_hik_camera/hik_camera_node.hpp`
- 配置: `src/rm_hardware_driver/ros2_hik_camera/config/camera_params.yaml`
- 解码器: `src/rm_hardware_driver/ros2_hik_camera/scripts/decode_yaw.py`

### MindVision
- 源码: `src/rm_hardware_driver/ros2_mindvision_camera/src/mv_camera_node.cpp`
- 配置: `src/rm_hardware_driver/ros2_mindvision_camera/config/camera_params.yaml`
- 解码器: `src/rm_hardware_driver/ros2_mindvision_camera/scripts/decode_yaw.py`
