# ros2_hik_camera 多相机支持功能完成

## ✅ 实现完成

已成功为 `ros2_hik_camera` 包添加了完整的多相机支持功能，与 `ros2_mindvision_camera` 的实现保持一致的架构。

## 📝 修改总结

### 1. **代码修改** (`src/ros2_hik_camera_node.cpp`)

#### 构造函数增强
- ✅ 添加相机信息打印，显示所有检测到的相机
- ✅ 同时支持 GigE 和 USB 两种设备类型
- ✅ 打印每个相机的型号和序列号

#### initCamera() 函数重构
- ✅ 添加 `camera_sn` 参数支持
- ✅ 实现基于序列号的相机选择逻辑
- ✅ 区分 GigE 和 USB 设备的序列号读取
- ✅ 默认行为：序列号为空时使用第一个相机（向后兼容）

### 2. **配置文件**

新增文件：
- ✅ `config/dual_camera_params.yaml` - 双相机配置模板
- ✅ 更新 `config/camera_params.yaml` - 添加 camera_sn 参数

### 3. **Launch 文件**

新增文件：
- ✅ `launch/list_cameras_launch.py` - 列出所有相机
- ✅ `launch/dual_camera_launch.py` - 启动双相机
- ✅ `launch/hik_camera_launch_with_sn.py` - 支持序列号参数的单相机启动

### 4. **文档**

新增/更新文档：
- ✅ 更新 `README.md` - 添加多相机使用说明
- ✅ `docs/multi_camera_guide.md` - 详细配置指南
- ✅ `docs/QUICK_START.md` - 快速开始指南
- ✅ `docs/CHANGELOG_multi_camera.md` - 变更日志

## 🚀 使用方法

### 快速开始

```bash
# 1. 查看所有相机序列号
ros2 launch ros2_hik_camera list_cameras_launch.py

# 2. 编辑配置文件填入序列号
nano ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/config/dual_camera_params.yaml

# 3. 启动双相机
ros2 launch ros2_hik_camera dual_camera_launch.py

# 4. 查看图像
rqt_image_view /camera_left/image_raw
rqt_image_view /camera_right/image_raw
```

### 单相机模式

```bash
# 默认使用第一个相机
ros2 launch ros2_hik_camera hik_camera_launch.py

# 指定序列号
ros2 launch ros2_hik_camera hik_camera_launch_with_sn.py camera_sn:=CA016ABC12345
```

## 🔧 技术特性

### 设备支持
- ✅ GigE 网络相机
- ✅ USB 3.0 相机
- ✅ 自动检测设备类型
- ✅ 支持最多 256 个设备（SDK 限制）

### 序列号处理
- GigE 相机：`chSerialNumber` (16 字符)
- USB 相机：`chSerialNumber` (64 字符)
- 自动根据设备类型读取正确的序列号字段

### 架构优势
- **独立节点**：每个相机运行在独立的 ROS2 节点中
- **命名空间隔离**：通过命名空间避免话题冲突
- **参数独立**：每个相机有独立的参数配置
- **动态调整**：支持运行时修改相机参数

## 📊 与 MindVision 相机对比

| 特性 | MindVision | HIKVision |
|------|-----------|-----------|
| 多相机支持 | ✅ | ✅ |
| 序列号参数 | ✅ `camera_sn` | ✅ `camera_sn` |
| 设备类型 | 单一 | GigE + USB |
| 序列号字段 | `acSn` | `chSerialNumber` |
| 实现方式 | 一致 | 一致 |
| Launch 文件 | 一致 | 一致 |

## ✨ 关键改进

### 1. **智能相机识别**
```cpp
// 打印所有检测到的相机
for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
    // 自动识别设备类型（GigE 或 USB）
    // 提取并显示相机名称和序列号
}
```

### 2. **灵活的相机选择**
```cpp
// 支持两种模式：
// 1. 不指定序列号 -> 使用第一个相机（默认）
// 2. 指定序列号 -> 查找并使用匹配的相机
```

### 3. **完善的错误处理**
- ✅ 序列号不匹配时给出明确错误
- ✅ 相机未找到时列出所有可用相机
- ✅ 设备打开失败时正确清理资源

## 🎯 应用场景

### 双目视觉
```bash
ros2 launch ros2_hik_camera dual_camera_launch.py
# 配合 stereo_image_proc 进行立体视觉处理
```

### 多视角监控
```bash
# 支持 3+ 相机，只需在 launch 文件中添加更多节点
```

### 全景拼接
```bash
# 多个相机从不同角度采集图像进行拼接
```

## 📈 性能考虑

### USB 带宽
- 单 USB 3.0 控制器：~400 MB/s
- 建议：将相机连接到不同的 USB 控制器
- 工具：`lsusb -t` 查看 USB 拓扑

### GigE 带宽
- 千兆网：~125 MB/s
- 优化：启用 Jumbo Frames (MTU 9000)
- 建议：使用专用网卡或网络分段

### 帧率建议
- 双相机 USB：10-30 fps 每个
- 双相机 GigE：30-60 fps 每个
- WSL2 环境：5-15 fps（受限）

## ⚠️ 注意事项

1. **序列号必须唯一**：每个相机节点必须配置不同的序列号
2. **命名空间必须不同**：避免话题冲突
3. **资源管理**：注意 CPU、内存和带宽使用
4. **WSL2 限制**：USB 性能严重受限，建议原生环境

## 🔍 调试工具

```bash
# 查看相机信息
ros2 launch ros2_hik_camera list_cameras_launch.py

# 检查话题
ros2 topic list | grep image

# 监控帧率
ros2 topic hz /camera_left/image_raw

# 查看带宽
ros2 topic bw /camera_left/image_raw

# 详细日志
ros2 launch ros2_hik_camera dual_camera_launch.py --ros-args --log-level debug
```

## 📚 文档位置

所有文档位于 `src/rm_hardware_driver/ros2_hik_camera/docs/`：
- `QUICK_START.md` - 快速开始
- `multi_camera_guide.md` - 详细配置指南
- `CHANGELOG_multi_camera.md` - 技术细节和变更日志

## ✅ 测试状态

- ✅ 代码编译通过
- ✅ Launch 文件安装正确
- ✅ 配置文件安装正确
- ✅ 文档完整
- ⚠️ 需要实际硬件测试（需要多个 HIKVision 相机）

## 🎉 完成！

`ros2_hik_camera` 现在具有与 `ros2_mindvision_camera` 相同的多相机支持能力！

可以开始使用多相机功能了。如有问题，请参考文档或查看日志输出。
