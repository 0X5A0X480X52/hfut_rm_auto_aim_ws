# HIKVision 相机多相机配置指南

## 步骤 1: 查找相机序列号

连接所有相机后，运行以下命令查看所有相机信息：

```bash
ros2 launch ros2_hik_camera list_cameras_launch.py
```

输出示例：
```
[INFO] [hik_camera]: Starting HikCameraNode!
[INFO] [hik_camera]: Enumerate status = 0
[INFO] [hik_camera]: Found camera count = 2
[INFO] [hik_camera]: Camera 0: MV-CA016-10UC (SN: CA016ABC12345)
[INFO] [hik_camera]: Camera 1: MV-CA016-10UC (SN: CA016DEF67890)
[INFO] [hik_camera]: No camera_sn specified, using first camera (SN: CA016ABC12345)
```

## 步骤 2: 配置相机参数

将序列号填入配置文件 `config/dual_camera_params.yaml`：

```yaml
/camera_left:
  ros__parameters:
    camera_name: camera_left
    camera_sn: "CA016ABC12345"  # 从步骤1获取的序列号
    
    exposure_time: 5000.0
    gain: 8.0
    frame_rate: 10.0
    flip_image: false

/camera_right:
  ros__parameters:
    camera_name: camera_right
    camera_sn: "CA016DEF67890"  # 从步骤1获取的序列号
    
    exposure_time: 5000.0
    gain: 8.0
    frame_rate: 10.0
    flip_image: false
```

## 步骤 3: 启动多相机

```bash
ros2 launch ros2_hik_camera dual_camera_launch.py
```

## 步骤 4: 查看图像

使用 rqt 查看图像：

```bash
rqt_image_view
```

选择话题：
- `/camera_left/image_raw`
- `/camera_right/image_raw`

## 自定义配置

### 添加更多相机

1. 在配置文件中添加新的命名空间和参数
2. 在 `dual_camera_launch.py` 中添加新的 Node 配置

示例 - 添加第三个相机：

```python
# 第三个相机节点
Node(
    package='ros2_hik_camera',
    executable='ros2_hik_camera_node',
    name='hik_camera',
    namespace='camera_center',
    output='screen',
    emulate_tty=True,
    parameters=[LaunchConfiguration('params_file'), {
        'camera_info_url': camera_info_url_center,
        'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
    }],
)
```

### 单独启动某个相机

```bash
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args \
  -r __ns:=/my_camera \
  -p camera_sn:="YOUR_CAMERA_SN" \
  -p camera_name:=my_camera \
  -p exposure_time:=5000.0 \
  -p gain:=8.0 \
  -p frame_rate:=10.0
```

## 检查话题

```bash
# 列出所有图像话题
ros2 topic list | grep image

# 查看相机信息
ros2 topic echo /camera_left/camera_info

# 查看图像话题信息
ros2 topic info /camera_left/image_raw

# 查看帧率
ros2 topic hz /camera_left/image_raw

# 查看带宽使用
ros2 topic bw /camera_left/image_raw
```

## 性能优化

### USB 带宽管理

当使用多个 USB 相机时：

1. **使用不同的 USB 控制器**：将相机连接到不同的 USB 根集线器
   ```bash
   # 查看 USB 拓扑
   lsusb -t
   ```

2. **调整帧率**：降低单个相机的帧率以分配带宽
   ```yaml
   frame_rate: 10.0  # 对于双相机，建议从 10fps 开始
   ```

3. **调整分辨率**：如果支持，可以降低分辨率

### GigE 网络优化

对于 GigE 相机：

1. **网络配置**：
   - 使用千兆网卡
   - 增加接收缓冲区大小
   - 关闭防火墙或添加例外

2. **MTU 设置**：
   ```bash
   # 设置网卡 MTU 为 9000（Jumbo Frames）
   sudo ifconfig eth0 mtu 9000
   ```

## 相机标定

为每个相机单独进行标定：

```bash
# 左相机标定
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args --remap image:=/camera_left/image_raw \
  --remap camera:=/camera_left

# 右相机标定
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args --remap image:=/camera_right/image_raw \
  --remap camera:=/camera_right
```

## 常见问题

### Q: 启动时提示 "Camera with SN 'xxx' not found!"

**A:** 检查序列号是否正确，可以使用 `list_cameras_launch.py` 重新查看所有相机序列号。

### Q: 两个节点都连接到同一个相机

**A:** 确保每个节点的配置文件中 `camera_sn` 参数设置了不同的序列号。

### Q: 图像帧率很低或丢帧

**A:** 
1. 检查 USB 带宽是否充足
2. 将相机连接到不同的 USB 控制器
3. 降低帧率或分辨率
4. 检查 CPU 使用率

### Q: 相机无法同时启动

**A:**
1. 检查系统资源（CPU、内存、USB 带宽）
2. 确认每个相机使用不同的序列号
3. 查看日志输出的错误信息

### Q: WSL2 性能问题

**A:** WSL2 的 USB 虚拟化严重限制了性能。建议：
- 使用 Windows 原生 ROS2
- 安装 Ubuntu 双系统
- 参考主 README 的 WSL2 限制说明

## 调试技巧

### 查看详细日志

```bash
ros2 launch ros2_hik_camera dual_camera_launch.py --ros-args --log-level debug
```

### 检查相机连接状态

```bash
# USB 相机
lsusb | grep -i hik

# GigE 相机
arp -a | grep -i hik
```

### 监控系统资源

```bash
# CPU 使用率
htop

# USB 带宽（需要安装 usbview）
sudo usbview
```

## 最佳实践

1. **命名规范**：使用有意义的命名空间，如 `/camera_front`, `/camera_back`, `/camera_left`, `/camera_right`

2. **参数文件管理**：为不同场景创建不同的配置文件

3. **启动顺序**：先启动一个相机验证正常工作，再添加其他相机

4. **日志记录**：保存启动日志以便问题排查

5. **备份标定数据**：相机标定数据应该定期备份

## 参考资料

- [HIKVision MVS SDK 文档](https://www.hikrobotics.com/cn/machinevision/service/download)
- [ROS2 image_transport 文档](https://github.com/ros-perception/image_common)
- [camera_calibration 教程](http://wiki.ros.org/camera_calibration)
