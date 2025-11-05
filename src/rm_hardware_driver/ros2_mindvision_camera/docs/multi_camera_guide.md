# MindVision 相机多相机配置指南

## 步骤 1: 查找相机序列号

连接所有相机后，运行以下命令查看所有相机信息：

```bash
ros2 launch mindvision_camera list_cameras_launch.py
```

输出示例：
```
[INFO] [mv_camera]: Starting MVCameraNode!
[INFO] [mv_camera]: Enumerate state = 0
[INFO] [mv_camera]: Found camera count = 2
[INFO] [mv_camera]: Camera 0: MV-U300 (SN: A1B2C3D4E5F6)
[INFO] [mv_camera]: Camera 1: MV-U300 (SN: F6E5D4C3B2A1)
[INFO] [mv_camera]: No camera_sn specified, using first camera: MV-U300 (SN: A1B2C3D4E5F6)
```

## 步骤 2: 配置相机参数

将序列号填入配置文件 `config/dual_camera_params.yaml`：

```yaml
/camera_left:
  ros__parameters:
    camera_name: camera_left
    camera_sn: "A1B2C3D4E5F6"  # 从步骤1获取的序列号
    
    exposure_time: 3500
    analog_gain: 64
    ...

/camera_right:
  ros__parameters:
    camera_name: camera_right
    camera_sn: "F6E5D4C3B2A1"  # 从步骤1获取的序列号
    
    exposure_time: 3500
    analog_gain: 64
    ...
```

## 步骤 3: 启动多相机

```bash
ros2 launch mindvision_camera dual_camera_launch.py
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

### 单独启动某个相机

```bash
ros2 run mindvision_camera mindvision_camera_node --ros-args \
  -r __ns:=/my_camera \
  -p camera_sn:="YOUR_CAMERA_SN" \
  -p camera_name:=my_camera
```

## 常见问题

**Q: 启动时提示 "Camera with SN 'xxx' not found!"**

A: 检查序列号是否正确，可以使用 `list_cameras_launch.py` 重新查看所有相机序列号。

**Q: 两个节点都连接到同一个相机**

A: 确保每个节点的配置文件中 `camera_sn` 参数设置了不同的序列号。

**Q: 如何获取相机的标定参数？**

A: 参考主 README.md 中的标定章节，为每个相机单独进行标定。
