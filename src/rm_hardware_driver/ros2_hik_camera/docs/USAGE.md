# ros2_hik_camera 使用指南

## 快速开始

### 1. 编译包

```bash
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select ros2_hik_camera
source install/setup.bash
```

或使用提供的构建脚本：

```bash
cd ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera
./build.sh
```

### 2. 测试安装

运行测试脚本验证包是否正确安装：

```bash
cd ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera
./test.sh
```

### 3. 连接相机

- USB相机：直接通过USB连接
- GigE相机：通过网线连接，确保网络配置正确

### 4. 启动相机节点

使用launch文件启动（推荐）：

```bash
source ~/hfut_rm_auto_aim_ws/install/setup.bash
ros2 launch ros2_hik_camera hik_camera_launch.py
```

或直接运行节点：

```bash
source ~/hfut_rm_auto_aim_ws/install/setup.bash
ros2 run ros2_hik_camera ros2_hik_camera_node
```

### 5. 查看图像

在另一个终端中：

```bash
# 使用rqt_image_view
ros2 run rqt_image_view rqt_image_view

# 或查看话题列表
ros2 topic list

# 查看图像话题信息
ros2 topic info /image_raw

# 查看图像数据
ros2 topic echo /image_raw --no-arr
```

## 参数配置

### 运行时参数调整

查看所有参数：

```bash
ros2 param list /hik_camera
```

获取参数值：

```bash
ros2 param get /hik_camera exposure_time
```

设置参数值：

```bash
# 曝光时间（微秒）
ros2 param set /hik_camera exposure_time 5000.0

# 增益（0~16）
ros2 param set /hik_camera gain 10.0

# 帧率（fps）
ros2 param set /hik_camera frame_rate 30.0

# 翻转图像
ros2 param set /hik_camera flip_image true
```

### 配置文件修改

编辑配置文件：

```bash
nano ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/config/camera_params.yaml
```

修改后重新编译并启动：

```bash
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select ros2_hik_camera
source install/setup.bash
ros2 launch ros2_hik_camera hik_camera_launch.py
```

## 相机标定

### 1. 安装标定工具

```bash
sudo apt install ros-humble-camera-calibration  # 或对应的ROS2版本
```

### 2. 打印标定板

使用8x6的棋盘格标定板，每个方格25mm。

### 3. 运行标定

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    image:=/image_raw \
    camera:=/hik_camera
```

### 4. 标定过程

1. 在不同角度、位置、距离下移动标定板
2. 等待X、Y、Size、Skew进度条变绿
3. 点击"Calibrate"按钮进行计算
4. 点击"Save"保存标定结果
5. 将生成的文件复制到配置目录：

```bash
cp /tmp/calibrationdata.tar.gz ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/config/
cd ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/config/
tar -xzf calibrationdata.tar.gz
mv ost.yaml camera_info.yaml
```

## 常见问题

### 问题1: 找不到相机

**现象**：
```
No camera found!
```

**解决方法**：
1. 检查相机USB连接
2. 检查USB权限：
   ```bash
   lsusb
   # 如果看到相机设备但无权限，添加udev规则
   sudo nano /etc/udev/rules.d/99-mvusb.rules
   # 添加：SUBSYSTEM=="usb", ATTR{idVendor}=="xxxx", MODE="0666"
   # 重新加载规则
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
3. 运行HIKVision官方工具确认相机可见

### 问题2: 编译错误

**现象**：
```
fatal error: MvCameraControl.h: No such file or directory
```

**解决方法**：
确认头文件在正确位置：
```bash
ls ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/include/ros2_hik_camera/inc/
```

应该能看到：
- MvCameraControl.h
- CameraParams.h
- MvErrorDefine.h
- 等文件

### 问题3: 运行时库找不到

**现象**：
```
error while loading shared libraries: libMvCameraControl.so
```

**解决方法**：
添加库路径到LD_LIBRARY_PATH：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64
# 或永久添加到~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64' >> ~/.bashrc
source ~/.bashrc
```

### 问题4: 相机被其他程序占用

**现象**：
```
Failed to open device, status = xxx
```

**解决方法**：
1. 关闭其他使用相机的程序（如HIKVision MVS软件）
2. 重新插拔相机USB
3. 重启节点

### 问题5: 图像质量问题

**图像太暗**：
```bash
ros2 param set /hik_camera exposure_time 10000.0
ros2 param set /hik_camera gain 12.0
```

**图像太亮**：
```bash
ros2 param set /hik_camera exposure_time 1000.0
ros2 param set /hik_camera gain 4.0
```

**帧率不够**：
```bash
ros2 param set /hik_camera frame_rate 60.0
```

## 高级用法

### 使用组合模式

在其他节点中加载相机组件：

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_hik_camera',
                plugin='ros2_hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[{
                    'exposure_time': 5000.0,
                    'gain': 8.0,
                }],
            ),
        ],
    )
    return LaunchDescription([container])
```

### 多相机同时使用

修改launch文件支持多个相机：

```python
Node(
    package='ros2_hik_camera',
    executable='ros2_hik_camera_node',
    name='camera_left',
    namespace='left',
    parameters=[...],
),
Node(
    package='ros2_hik_camera',
    executable='ros2_hik_camera_node',
    name='camera_right',
    namespace='right',
    parameters=[...],
),
```

## 调试技巧

### 启用详细日志

```bash
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args --log-level debug
```

### 录制图像数据

```bash
ros2 bag record /image_raw /camera_info
```

### 回放数据

```bash
ros2 bag play <bag_file>
```

## 性能优化

1. **降低图像分辨率**：如果不需要全分辨率
2. **调整帧率**：根据应用需求设置合适的帧率
3. **使用QoS**：对于高速图像流，启用sensor_data QoS
4. **图像压缩**：使用compressed_image_transport减少网络负载

## 相关链接

- [HIKVision MVS SDK文档](https://www.hikrobotics.com/)
- [ROS2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [camera_info_manager](https://github.com/ros-perception/image_common)
