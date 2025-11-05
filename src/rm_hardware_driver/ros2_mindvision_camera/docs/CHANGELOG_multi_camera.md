# MindVision 相机驱动多相机支持更新说明

## 修改内容

本次更新为 `ros2_mindvision_camera` 包添加了多相机支持功能。

### 1. 代码修改

**文件**: `src/mv_camera_node.cpp`

主要变更：
- 修改相机枚举逻辑，支持检测多个相机（最多10个）
- 添加 `camera_sn` 参数，允许通过序列号指定要使用的相机
- 当 `camera_sn` 为空时，默认使用第一个检测到的相机
- 启动时会打印所有检测到的相机信息（名称和序列号）

### 2. 新增配置文件

- **`config/dual_camera_params.yaml`**: 双相机配置示例
  - 为左右两个相机分别配置参数
  - 每个相机使用独立的命名空间
  - 需要填入实际的相机序列号

### 3. 新增 Launch 文件

- **`launch/list_cameras_launch.py`**: 列出所有连接的相机及序列号
- **`launch/dual_camera_launch.py`**: 同时启动两个相机
- **`launch/mv_launch_with_sn.py`**: 单相机启动，支持命令行指定序列号

### 4. 文档更新

- **`README.md`**: 添加多相机使用说明
- **`docs/multi_camera_guide.md`**: 详细的多相机配置指南

## 使用方法

### 查看所有相机

```bash
ros2 launch mindvision_camera list_cameras_launch.py
```

### 单相机模式（默认）

```bash
# 使用第一个相机
ros2 launch mindvision_camera mv_launch.py

# 指定序列号
ros2 launch mindvision_camera mv_launch_with_sn.py camera_sn:=YOUR_SN
```

### 双相机模式

1. 运行 `list_cameras_launch.py` 获取序列号
2. 编辑 `config/dual_camera_params.yaml`，填入序列号
3. 运行 `ros2 launch mindvision_camera dual_camera_launch.py`

## 技术细节

### 相机选择逻辑

```cpp
// 1. 枚举所有相机
tSdkCameraDevInfo t_camera_enum_list[10];
int i_camera_counts = 10;
CameraEnumerateDevice(t_camera_enum_list, &i_camera_counts);

// 2. 获取序列号参数
std::string camera_sn = this->declare_parameter("camera_sn", "");

// 3. 选择相机
if (camera_sn.empty()) {
    // 使用第一个相机
    selected_camera = &t_camera_enum_list[0];
} else {
    // 根据序列号查找
    for (int i = 0; i < i_camera_counts; i++) {
        if (std::string(t_camera_enum_list[i].acSn) == camera_sn) {
            selected_camera = &t_camera_enum_list[i];
            break;
        }
    }
}
```

### 多相机架构

每个相机运行在独立的节点中：
- 不同的命名空间（如 `/camera_left`, `/camera_right`）
- 独立的参数配置
- 通过序列号避免冲突

### 话题结构

单相机模式：
- `/image_raw`
- `/camera_info`

多相机模式：
- `/camera_left/image_raw`
- `/camera_left/camera_info`
- `/camera_right/image_raw`
- `/camera_right/camera_info`

## 兼容性

- ✅ 完全向后兼容，不指定序列号时行为与原代码一致
- ✅ 原有的 `mv_launch.py` 仍然可用
- ✅ 原有的配置文件格式仍然有效
- ✅ 支持动态参数调整

## 注意事项

1. **序列号格式**: 从相机枚举获取的字符串，通常是类似 "A1B2C3D4E5F6" 的格式
2. **节点命名**: 使用多相机时建议为每个节点设置不同的命名空间
3. **资源占用**: 每个相机节点会占用独立的 CPU 和内存资源
4. **USB 带宽**: 确保 USB 总线有足够带宽支持多个相机同时工作

## 测试建议

1. 先运行 `list_cameras_launch.py` 确认相机可以被检测到
2. 逐个启动相机节点，确认每个都能正常工作
3. 检查日志确认每个节点连接到正确的相机
4. 使用 `rqt_image_view` 验证图像输出

## 故障排查

**问题**: Camera with SN 'xxx' not found!
- **原因**: 序列号不匹配或相机未连接
- **解决**: 运行 `list_cameras_launch.py` 检查实际序列号

**问题**: 两个节点连接到同一相机
- **原因**: 序列号配置错误
- **解决**: 确保配置文件中每个相机的序列号不同

**问题**: 图像帧率低
- **原因**: USB 带宽不足
- **解决**: 使用不同的 USB 控制器或降低分辨率/帧率
