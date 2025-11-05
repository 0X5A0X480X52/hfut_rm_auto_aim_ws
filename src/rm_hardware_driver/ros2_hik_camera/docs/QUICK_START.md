# HIKVision 多相机快速开始指南

## 🚀 快速开始

### 步骤 1: 查看相机序列号

```bash
# Source 你的工作空间
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash

# 列出所有连接的相机
ros2 launch ros2_hik_camera list_cameras_launch.py
```

你会看到类似这样的输出：
```
[INFO] [hik_camera]: Found camera count = 2
[INFO] [hik_camera]: Camera 0: MV-CA016-10UC (SN: CA016ABC12345)
[INFO] [hik_camera]: Camera 1: MV-CA016-10UC (SN: CA016DEF67890)
```

记下这些序列号！

### 步骤 2: 配置相机参数

编辑配置文件：
```bash
nano ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/config/dual_camera_params.yaml
```

将序列号替换为你的实际序列号：
```yaml
/camera_left:
  ros__parameters:
    camera_sn: "CA016ABC12345"  # ← 替换这里
    ...

/camera_right:
  ros__parameters:
    camera_sn: "CA016DEF67890"  # ← 替换这里
    ...
```

### 步骤 3: 启动双相机

```bash
ros2 launch ros2_hik_camera dual_camera_launch.py
```

### 步骤 4: 查看图像

在新终端中：
```bash
# 查看左相机
rqt_image_view /camera_left/image_raw

# 或查看右相机
rqt_image_view /camera_right/image_raw
```

## 📋 常用命令

### 启动单个相机（指定序列号）

```bash
ros2 launch ros2_hik_camera hik_camera_launch_with_sn.py camera_sn:=CA016ABC12345
```

### 启动单个相机（默认第一个）

```bash
ros2 launch ros2_hik_camera hik_camera_launch.py
```

### 手动启动特定相机到自定义命名空间

```bash
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args \
  -r __ns:=/my_camera \
  -p camera_sn:="YOUR_SN" \
  -p camera_name:=my_camera \
  -p exposure_time:=5000.0 \
  -p gain:=8.0 \
  -p frame_rate:=10.0
```

## 🔍 检查话题

```bash
# 列出所有图像话题
ros2 topic list | grep image

# 查看相机信息
ros2 topic echo /camera_left/camera_info

# 查看图像话题信息
ros2 topic info /camera_left/image_raw
```

## ⚙️ 动态调整参数

```bash
# 查看可用参数
ros2 param list /camera_left/hik_camera

# 修改曝光时间
ros2 param set /camera_left/hik_camera exposure_time 8000.0

# 修改增益
ros2 param set /camera_left/hik_camera gain 12.0

# 修改帧率
ros2 param set /camera_left/hik_camera frame_rate 15.0
```

## 📊 性能监控

```bash
# 查看帧率
ros2 topic hz /camera_left/image_raw

# 查看带宽使用
ros2 topic bw /camera_left/image_raw

# 查看延迟
ros2 topic delay /camera_left/image_raw
```

## 🛠️ 故障排查

### 相机未检测到
```bash
# 检查 USB 设备
lsusb | grep -i hik

# 检查权限
ls -l /dev/video*
```

### 序列号不匹配
```bash
# 重新扫描相机
ros2 launch ros2_hik_camera list_cameras_launch.py
```

### 节点启动失败
```bash
# 查看详细日志
ros2 launch ros2_hik_camera dual_camera_launch.py --ros-args --log-level debug
```

### 帧率低/丢帧
```bash
# 检查 USB 拓扑（确保使用不同的 USB 控制器）
lsusb -t

# 降低帧率
ros2 param set /camera_left/hik_camera frame_rate 5.0
```

## 💡 提示

1. **USB 带宽**: 
   - 单 USB 3.0 控制器带宽约 400 MB/s
   - 建议将相机连接到不同的 USB 控制器
   - 从低帧率（10fps）开始测试

2. **命名空间**: 
   - 使用描述性命名空间：`/camera_front`, `/camera_back`
   - 避免使用通用名称如 `/camera1`, `/camera2`

3. **参数调整**: 
   - 可以在运行时通过 rqt 的 Dynamic Reconfigure 插件调整参数
   - 参数修改实时生效，无需重启节点

4. **标定**: 
   - 每个相机需要单独进行标定
   - 标定数据保存在 `config/` 目录

5. **WSL2 用户**:
   - WSL2 的 USB 性能受限（10-15 fps）
   - 建议使用 Windows 原生 ROS2 或双系统

## 📚 更多信息

- 详细配置指南: `docs/multi_camera_guide.md`
- 主 README: `README.md`
- WSL2 限制说明: `WSL2_USB_LIMITATIONS.md`

## 🎯 典型应用场景

### 双目视觉系统
```bash
# 启动左右相机
ros2 launch ros2_hik_camera dual_camera_launch.py

# 启动立体视觉处理
ros2 launch stereo_image_proc stereo_image_proc.launch.py \
  left_namespace:=/camera_left \
  right_namespace:=/camera_right
```

### 多视角监控
```bash
# 修改 dual_camera_launch.py 添加更多相机
# 使用不同的命名空间：/camera_front, /camera_back, /camera_left, /camera_right
```

### 全景拼接
```bash
# 启动多个相机
# 使用图像拼接算法进行全景合成
```
