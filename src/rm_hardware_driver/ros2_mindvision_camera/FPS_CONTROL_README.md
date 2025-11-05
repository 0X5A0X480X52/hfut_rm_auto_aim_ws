# MindVision 相机 FPS 控制说明

## ✅ 已实现功能

### 智能帧率控制

代码会自动检测相机类型并选择合适的控制方式：

1. **网口相机 (GigE)** - 使用 `CameraSetFrameRate()`
   - 支持精确设置帧率（Hz）
   - 例如：10fps, 30fps, 60fps

2. **USB相机** - 自动回退到 `CameraSetFrameSpeed()`
   - 使用预设的速度档位
   - 0=Low, 1=Normal, 2=High, 3=Super
   - **推荐双相机使用 Low 模式 (0)** 以降低USB带宽占用

## 配置方法

编辑 `config/dual_camera_params.yaml`:

```yaml
/camera_left:
  mv_camera:
    ros__parameters:
      camera_sn: "041170020075"
      frame_rate: 10    # 网口相机会使用这个值
      frame_speed: 0    # USB相机会自动使用这个值 (Low模式)
      exposure_time: 3500
      # ... 其他参数
```

## 运行时动态调整

```bash
# USB相机 - 调整帧速度模式
ros2 param set /camera_left/mv_camera frame_speed 0  # Low模式
ros2 param set /camera_left/mv_camera frame_speed 1  # Normal模式

# 网口相机 - 调整帧率
ros2 param set /camera_left/mv_camera frame_rate 30  # 30fps
```

## 测试结果

### 左相机 ✅
- 成功初始化
- 自动检测为USB相机，使用 `frame_speed=0` (Low模式)
- 图像正常发布

### 右相机 ❌  
- 成功初始化并设置为Low模式
- **但无法获取图像（超时错误-12）**
- 这不是帧率设置的问题，而是USB带宽或硬件问题

## 已知问题和解决方案

### 问题：右相机超时 (Error -12)

**可能原因：**
1. WSL2 USB带宽限制
2. 两个相机连接到同一个USB控制器
3. 右相机硬件问题

**建议解决方法：**

1. **单独测试右相机**
   ```bash
   # 修改配置让左相机使用右相机的序列号
   camera_sn: "042081320100"
   ```

2. **检查USB拓扑**
   ```bash
   lsusb -t
   ```
   确保两个相机连接到不同的USB控制器

3. **使用原生Linux而非WSL2**
   WSL2的USB支持不如原生Linux稳定

4. **尝试更低的曝光时间**
   降低 `exposure_time` 可以减少每帧的数据量

## 技术细节

### 代码实现

```cpp
// 优先尝试网口相机的帧率设置
int status = CameraSetFrameRate(h_camera_, frame_rate);

if (status == -4) {  // NOT_SUPPORTED
    // USB相机不支持，回退到帧速度模式
    CameraSetFrameSpeed(h_camera_, frame_speed);
}
```

### 参数范围

- `frame_rate`: 0-200 (0表示最大帧率)
- `frame_speed`: 0-3 (取决于相机支持的模式数)

## 相关文件

- `src/mv_camera_node.cpp` - 主要实现
- `config/dual_camera_params.yaml` - 参数配置
- `launch/dual_camera_launch.py` - 启动文件（包含2秒延迟）
- `DUAL_CAMERA_SETUP.md` - 详细排查指南
