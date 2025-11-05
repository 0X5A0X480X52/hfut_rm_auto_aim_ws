# 图像分辨率设置功能

## 概述

`ros2_mindvision_camera` 包现在支持通过 ROS2 参数设置相机的图像分辨率。

## 功能特性

- ✅ 支持通过配置文件设置图像分辨率
- ✅ 支持运行时动态修改分辨率
- ✅ 默认使用相机原生分辨率（参数为 0）
- ✅ 自动验证分辨率范围
- ✅ 支持单相机和双相机模式

## 使用方法

### 1. 在配置文件中设置分辨率

编辑 `config/camera_params.yaml` 或 `config/dual_camera_params.yaml`：

```yaml
ros__parameters:
  # 图像分辨率设置（设置为0则使用相机默认分辨率）
  image_width: 1920   # 图像宽度
  image_height: 1080  # 图像高度
```

### 2. 通过 launch 参数设置

```bash
ros2 launch mindvision_camera mv_launch.py
```

### 3. 运行时动态修改分辨率

```bash
# 修改宽度
ros2 param set /mv_camera image_width 1920

# 修改高度
ros2 param set /mv_camera image_height 1080
```

### 4. 查看当前分辨率设置

```bash
ros2 param get /mv_camera image_width
ros2 param get /mv_camera image_height
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `image_width` | int | `0` | 图像宽度（0表示使用相机默认） |
| `image_height` | int | `0` | 图像高度（0表示使用相机默认） |

## 注意事项

1. **必须同时设置宽度和高度**：只设置一个参数不会生效
2. **默认行为**：当参数为 0 时，保持相机原生分辨率
3. **分辨率范围**：支持的分辨率范围取决于相机型号，系统会自动验证
4. **性能影响**：更高的分辨率会增加数据传输量和处理负担

## 常见分辨率示例

### 1920x1080 (Full HD)
```yaml
image_width: 1920
image_height: 1080
```

### 1280x720 (HD)
```yaml
image_width: 1280
image_height: 720
```

### 640x480 (VGA)
```yaml
image_width: 640
image_height: 480
```

### 使用相机默认分辨率
```yaml
image_width: 0
image_height: 0
```

## 双相机配置示例

在 `config/dual_camera_params.yaml` 中为每个相机独立设置分辨率：

```yaml
/camera_left:
  mv_camera:
    ros__parameters:
      camera_name: camera_left
      camera_sn: "041170020075"
      image_width: 1920
      image_height: 1080
      # ... 其他参数

/camera_right:
  mv_camera:
    ros__parameters:
      camera_name: camera_right
      camera_sn: "042081320100"
      image_width: 1920
      image_height: 1080
      # ... 其他参数
```

## 故障排除

### 问题：设置分辨率后没有生效

**解决方法**：
1. 确认同时设置了 `image_width` 和 `image_height` 为非零值
2. 检查日志输出，确认分辨率是否在相机支持的范围内
3. 尝试重启节点

### 问题：日志显示"Failed to set image resolution"

**可能原因**：
- 设置的分辨率超出相机支持范围
- 相机硬件不支持该分辨率

**解决方法**：
1. 查看节点日志中显示的支持范围
2. 尝试使用相机默认分辨率（设置为 0）
3. 参考相机规格手册确认支持的分辨率

## 技术实现细节

### SDK 函数调用

使用 MindVision SDK 的以下函数：
- `CameraGetImageResolution()` - 获取当前分辨率
- `CameraSetImageResolution()` - 设置新分辨率
- `CameraGetCapability()` - 获取相机能力（分辨率范围）

### 设置时机

分辨率在以下时机设置：
1. 节点初始化时（在 `CameraPlay()` 之前）
2. 运行时通过参数服务器修改时

### 代码位置

- 参数声明：`src/mv_camera_node.cpp::declareParameters()`
- 运行时修改：`src/mv_camera_node.cpp::parametersCallback()`
- 配置文件：`config/camera_params.yaml`, `config/dual_camera_params.yaml`

## 更新日志

- **2025-10-12**: 添加图像分辨率设置功能
  - 支持配置文件设置
  - 支持运行时动态修改
  - 更新文档说明
