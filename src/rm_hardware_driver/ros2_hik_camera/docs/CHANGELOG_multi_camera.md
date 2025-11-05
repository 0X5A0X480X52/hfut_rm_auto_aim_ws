# HIKVision 相机驱动多相机支持更新说明

## 修改内容

本次更新为 `ros2_hik_camera` 包添加了多相机支持功能。

### 1. 代码修改

**文件**: `src/ros2_hik_camera_node.cpp`

主要变更：
- **构造函数**：添加相机信息打印逻辑，显示所有检测到的相机名称和序列号
- **initCamera() 函数**：
  - 添加 `camera_sn` 参数支持
  - 实现根据序列号查找并选择相机的逻辑
  - 支持 GigE 和 USB 两种设备类型的序列号读取
  - 当 `camera_sn` 为空时，默认使用第一个检测到的相机

### 2. 新增配置文件

- **`config/dual_camera_params.yaml`**: 双相机配置示例
  - 为左右两个相机分别配置参数
  - 每个相机使用独立的命名空间 (`/camera_left`, `/camera_right`)
  - 需要填入实际的相机序列号

- **更新 `config/camera_params.yaml`**: 添加 `camera_sn` 参数说明

### 3. 新增 Launch 文件

- **`launch/list_cameras_launch.py`**: 列出所有连接的相机及序列号
- **`launch/dual_camera_launch.py`**: 同时启动两个相机
- **`launch/hik_camera_launch_with_sn.py`**: 单相机启动，支持命令行指定序列号

### 4. 文档更新

- **`README.md`**: 添加多相机使用说明
- **`docs/multi_camera_guide.md`**: 详细的多相机配置指南
- **`docs/QUICK_START.md`**: 快速开始指南
- **`docs/CHANGELOG_multi_camera.md`**: 本文档

## 使用方法

### 查看所有相机

```bash
ros2 launch ros2_hik_camera list_cameras_launch.py
```

### 单相机模式（默认）

```bash
# 使用第一个相机
ros2 launch ros2_hik_camera hik_camera_launch.py

# 指定序列号
ros2 launch ros2_hik_camera hik_camera_launch_with_sn.py camera_sn:=YOUR_SN
```

### 双相机模式

1. 运行 `list_cameras_launch.py` 获取序列号
2. 编辑 `config/dual_camera_params.yaml`，填入序列号
3. 运行 `ros2 launch ros2_hik_camera dual_camera_launch.py`

## 技术细节

### 相机选择逻辑

```cpp
// 1. 获取序列号参数
std::string camera_sn = this->declare_parameter("camera_sn", "");

// 2. 选择相机
if (camera_sn.empty()) {
    // 使用第一个相机
    selected_device = device_list_.pDeviceInfo[0];
} else {
    // 根据序列号查找
    for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
        // 根据设备类型读取序列号
        if (dev_info->nTLayerType == MV_GIGE_DEVICE) {
            sn = std::string(reinterpret_cast<char*>(
                dev_info->SpecialInfo.stGigEInfo.chSerialNumber));
        } else if (dev_info->nTLayerType == MV_USB_DEVICE) {
            sn = std::string(reinterpret_cast<char*>(
                dev_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
        }
        
        if (sn == camera_sn) {
            selected_device = dev_info;
            break;
        }
    }
}
```

### 设备类型支持

支持两种设备类型：
- **MV_GIGE_DEVICE**: GigE 网络相机
- **MV_USB_DEVICE**: USB 3.0 相机

每种类型的序列号存储在不同的结构体成员中：
- GigE: `dev_info->SpecialInfo.stGigEInfo.chSerialNumber`
- USB: `dev_info->SpecialInfo.stUsb3VInfo.chSerialNumber`

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
- ✅ 原有的 `hik_camera_launch.py` 仍然可用
- ✅ 原有的配置文件格式仍然有效
- ✅ 支持动态参数调整

## 注意事项

1. **序列号格式**: 
   - GigE 相机：通常是 16 字符的字符串
   - USB 相机：可能是 64 字符的字符串
   - 从相机枚举获取的实际字符串

2. **节点命名**: 使用多相机时建议为每个节点设置不同的命名空间

3. **资源占用**: 每个相机节点会占用独立的 CPU 和内存资源

4. **USB 带宽**: 
   - 单个 USB 3.0 控制器带宽约 400 MB/s
   - 建议将多个相机连接到不同的 USB 控制器
   - 可以使用 `lsusb -t` 查看 USB 拓扑结构

5. **GigE 带宽**:
   - 千兆网络带宽约 125 MB/s
   - 建议使用 Jumbo Frames (MTU 9000)
   - 可以调整相机的 PacketSize 参数

## 与 MindVision 相机的对比

| 特性 | ros2_mindvision_camera | ros2_hik_camera |
|------|------------------------|-----------------|
| 多相机支持 | ✅ 已实现 | ✅ 已实现 |
| 序列号字段 | `acSn` (32 字符) | `chSerialNumber` (16/64 字符) |
| 设备类型 | 单一类型 | GigE + USB |
| 实现方式 | 相同架构 | 相同架构 |

## 测试建议

1. 先运行 `list_cameras_launch.py` 确认相机可以被检测到
2. 逐个启动相机节点，确认每个都能正常工作
3. 检查日志确认每个节点连接到正确的相机
4. 使用 `rqt_image_view` 验证图像输出
5. 监控系统资源使用情况（CPU、内存、带宽）

## 故障排查

**问题**: Camera with SN 'xxx' not found!
- **原因**: 序列号不匹配或相机未连接
- **解决**: 运行 `list_cameras_launch.py` 检查实际序列号

**问题**: 两个节点连接到同一相机
- **原因**: 序列号配置错误或为空
- **解决**: 确保配置文件中每个相机的序列号不同且正确

**问题**: 图像帧率低或丢帧
- **原因**: USB/网络带宽不足
- **解决**: 
  - 使用不同的 USB 控制器
  - 降低分辨率/帧率
  - 对于 GigE，启用 Jumbo Frames

**问题**: WSL2 性能低
- **原因**: WSL2 的 USB 虚拟化限制
- **解决**: 使用 Windows 原生 ROS2 或 Ubuntu 双系统

## 性能指标

### USB 相机
- 单相机：最高 249 fps (原生环境)
- 双相机：建议 10-30 fps 每个
- WSL2：10-15 fps

### GigE 相机
- 单相机：取决于网络配置，通常 50-100 fps
- 双相机：需要不同的网卡或网络分段

## 后续改进计划

- [ ] 添加相机自动发现和配置生成工具
- [ ] 支持相机热插拔
- [ ] 添加相机同步触发支持
- [ ] 优化多相机性能
- [ ] 添加更多的相机参数支持

## 参考资料

- HIKVision MVS SDK 文档
- ROS2 rclcpp_components 文档
- image_transport 文档
- camera_info_manager 文档
