# ROS2 HIKVision Camera Driver

这是一个用于HIKVision工业相机的ROS2驱动程序包，参考了ros2_mindvision_camera的实现方式。

## 功能特性

- 支持HIKVision USB和GigE相机
- 实时图像发布到ROS2话题
- 支持动态参数调整（曝光时间、增益、帧率等）
- 支持相机标定信息加载
- 使用rclcpp_components实现组件化架构

## 依赖项

- ROS2 (Humble/Foxy或更高版本)
- HIKVision MVS SDK
- OpenCV
- image_transport
- camera_info_manager

## ⚠️ WSL2性能限制说明

**如果您在WSL2环境中使用此驱动，请注意：**

WSL2通过USBIPD虚拟化USB设备，存在严重的性能瓶颈：
- Windows原生环境：支持 **249 fps** ✅
- WSL2环境：仅支持 **10-15 fps** ❌

**原因**：WSL2的USB over IP机制将USB 3.0性能降级到30-50 MB/s（理论400 MB/s）

**解决方案**：
1. **推荐**：使用Windows原生ROS2 → 支持完整帧率
2. **推荐**：安装Ubuntu双系统 → 最佳性能
3. **可选**：使用Docker + USB直通 → 50-100 fps
4. **临时**：继续使用WSL2 → 仅适合10fps以下的应用

详细分析请查看：[WSL2_USB_LIMITATIONS.md](./WSL2_USB_LIMITATIONS.md)

测试您的USB性能：
```bash
./scripts/test_wsl2_usb_performance.sh
```

## 安装HIKVision SDK

1. 确保HIKVision MVS SDK已安装在系统中
2. SDK头文件已包含在 `include/ros2_hik_camera/inc/` 目录中
3. SDK库文件位于 `/opt/MVS/lib/64` (64位系统) 或 `/opt/MVS/lib/32` (32位系统)
4. 如果SDK库安装在其他位置，请修改 `CMakeLists.txt` 中的 `HIK_SDK_DIR` 变量

## 编译

```bash
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select ros2_hik_camera
source install/setup.bash
```

## 使用方法

### 启动相机节点

#### 单相机模式

```bash
ros2 launch ros2_hik_camera hik_camera_launch.py
```

#### 多相机模式

**1. 查看所有连接的相机信息和序列号**

```bash
ros2 launch ros2_hik_camera list_cameras_launch.py
```

这将列出所有检测到的相机及其序列号（SN）。

**2. 配置多相机参数**

编辑 `config/dual_camera_params.yaml` 文件，为每个相机指定序列号：

```yaml
/camera_left:
  ros__parameters:
    camera_name: camera_left
    camera_sn: "YOUR_LEFT_CAMERA_SN"  # 替换为实际序列号
    ...

/camera_right:
  ros__parameters:
    camera_name: camera_right
    camera_sn: "YOUR_RIGHT_CAMERA_SN"  # 替换为实际序列号
    ...
```

**3. 启动多个相机**

```bash
ros2 launch ros2_hik_camera dual_camera_launch.py
```

这将同时启动两个相机节点，分别在 `/camera_left` 和 `/camera_right` 命名空间下。

**注意事项：**

- `camera_sn` 参数留空时，节点将使用第一个检测到的相机
- 每个相机必须指定唯一的序列号以避免冲突
- 可以通过修改 launch 文件添加更多相机节点

### 查看图像

使用rqt_image_view查看图像：

```bash
# 单相机模式
ros2 run rqt_image_view rqt_image_view

# 多相机模式 - 查看左相机
rqt_image_view /camera_left/image_raw

# 多相机模式 - 查看右相机
rqt_image_view /camera_right/image_raw
```

或使用rviz2：

```bash
rviz2
```

在rviz2中添加Image显示，订阅相应的话题。

### 动态参数调整

实时调整曝光时间：

```bash
# 单相机模式
ros2 param set /hik_camera exposure_time 5000.0

# 多相机模式
ros2 param set /camera_left/hik_camera exposure_time 5000.0
ros2 param set /camera_right/hik_camera exposure_time 5000.0
```

实时调整增益：

```bash
ros2 param set /hik_camera gain 10.0
```

实时调整帧率：

```bash
ros2 param set /hik_camera frame_rate 30.0
```

实时调整图像分辨率：

```bash
# 设置为1920x1080
ros2 param set /hik_camera image_width 1920
ros2 param set /hik_camera image_height 1080

# 设置为1280x720
ros2 param set /hik_camera image_width 1280
ros2 param set /hik_camera image_height 720

# 注意：必须同时设置width和height，且都为非零值才会生效
```

## 配置文件

### camera_params.yaml

相机运行参数配置文件，包括：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_name` | string | "hik_camera" | 相机名称 |
| `camera_sn` | string | "" | 相机序列号（空为自动检测第一个相机） |
| `image_width` | int | 0 | 图像宽度（0表示使用相机默认分辨率） |
| `image_height` | int | 0 | 图像高度（0表示使用相机默认分辨率） |
| `exposure_time` | double | 5000.0 | 曝光时间（微秒） |
| `gain` | double | 8.0 | 增益值（0~16） |
| `frame_rate` | double | 30.0 | 帧率（fps） |
| `flip_image` | bool | false | 是否翻转图像 |

**分辨率设置说明：**
- 当 `image_width` 和 `image_height` 都设置为0时，使用相机默认分辨率
- 只有当两个参数都非零时，才会设置分辨率
- 设置的分辨率不能超过相机支持的最大分辨率
- 设置分辨率时会自动将 OffsetX 和 OffsetY 重置为 0（ROI 从左上角开始）
- 可以通过动态参数在运行时调整分辨率

### camera_info.yaml

相机标定信息文件，使用camera_calibration工具进行标定后生成。

## 话题

### 发布话题

- `/image_raw` (sensor_msgs/Image): 原始图像数据
- `/camera_info` (sensor_msgs/CameraInfo): 相机标定信息

## 与mindvision_camera的对比

本驱动程序的实现参考了ros2_mindvision_camera，主要相似之处：

1. **组件化架构**: 使用rclcpp_components实现可组合节点
2. **参数管理**: 支持运行时动态参数调整
3. **图像发布**: 使用image_transport发布图像和相机信息
4. **线程模型**: 使用独立线程进行图像采集

主要区别：

1. **SDK差异**: HIKVision SDK API与MindVision SDK不同
2. **像素格式转换**: HIKVision需要显式的像素格式转换
3. **参数范围**: 根据HIKVision相机的实际能力调整参数范围

## 相机标定

使用ROS2的camera_calibration工具进行标定：

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args --remap image:=/image_raw
```

标定完成后，点击"Save"保存标定结果到配置文件。

## 故障排除

### 找不到相机

- 检查相机USB/网络连接
- 确认HIKVision SDK正确安装
- 运行SDK自带的工具确认相机可被识别

### 编译错误

- 确认所有ROS2依赖已安装
- 检查HIKVision SDK路径是否正确
- 确认OpenCV已安装

### 运行时错误

- 检查USB权限设置
- 确认相机未被其他程序占用
- 查看日志输出定位问题

## License

MIT License

## 作者

amatrix

## 参考

- [ros2_mindvision_camera](https://github.com/chenjunnn/ros2_mindvision_camera)
- HIKVision MVS SDK Documentation
