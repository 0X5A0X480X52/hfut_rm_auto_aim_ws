# MindVision 双相机配置指南

## 已完成的功能

### 1. 智能帧率控制
实现了自适应帧率控制，支持网口相机和USB相机：

#### 网口相机 (GigE)
使用 `frame_rate` 参数直接设置帧率（Hz）：
- 支持精确的帧率控制（如 10fps, 30fps, 60fps等）
- `<=0` 表示使用最大帧率

#### USB相机
自动回退到 `frame_speed` 模式控制：
- `0` - **Low** (低速模式，推荐用于双相机，降低USB带宽需求) ✅
- `1` - **Normal** (普通模式)
- `2` - **High** (高速模式，需要更高的传输带宽)
- `3` - **Super** (超高速模式，需要最高的传输带宽)

**配置方式：**
在 `config/dual_camera_params.yaml` 中设置：
```yaml
/camera_left:
  mv_camera:
    ros__parameters:
      frame_rate: 10    # 网口相机使用（优先尝试）
      frame_speed: 0    # USB相机使用（自动回退）
```

**动态调整：**
```bash
# 网口相机
ros2 param set /camera_left/mv_camera frame_rate 30

# USB相机
ros2 param set /camera_left/mv_camera frame_speed 1
```

### 2. 触发模式修复
添加了 `CameraSetTriggerMode(h_camera_, 0)` 来显式设置相机为连续采集模式。

### 3. 序列号配置
修复了参数文件中序列号的格式问题：
- 移除了左相机序列号的前导空格
- 添加了正确的 YAML 层级结构（namespace/node_name/ros__parameters）

## 当前问题

### 右相机图像获取超时
**现象：**
- 左相机工作正常
- 右相机初始化成功，但无法获取图像（`CameraGetImageBuffer` 返回 -12 TIMEOUT）
- 延迟启动也无法解决问题

**可能原因：**
1. **USB带宽不足** - WSL2 对USB设备的带宽分配有限制
2. **相机硬件问题** - 右相机（bubing, SN: 042081320100）可能有硬件故障
3. **USB控制器限制** - 两个相机可能连接到同一个USB控制器

## 问题排查步骤

### 1. 检查USB带宽和拓扑
```bash
# 查看USB设备连接情况
lsusb -t

# 查看USB设备详细信息
lsusb -v | grep -A 10 "041170020075\|042081320100"
```

**建议：** 将两个相机连接到不同的USB控制器/端口上。

### 2. 单独测试右相机
临时修改 `config/dual_camera_params.yaml`，让左相机使用右相机的序列号：
```yaml
/camera_left:
  mv_camera:
    ros__parameters:
      camera_sn: "042081320100"  # 临时使用右相机
```

然后只启动左相机节点：
```bash
ros2 run mindvision_camera mindvision_camera_node --ros-args \
  -r __ns:=/camera_left \
  --params-file $(ros2 pkg prefix mindvision_camera)/share/mindvision_camera/config/dual_camera_params.yaml
```

如果右相机单独可以工作，说明是USB带宽问题。

### 3. 降低分辨率或图像质量
如果相机支持，可以通过MindVision SDK设置较低的分辨率来减少带宽需求。

查看 `CameraDefine.h` 中的分辨率设置函数。

### 4. 增加超时时间
修改 `mv_camera_node.cpp` 中的超时参数：
```cpp
// 当前超时设置为 1000ms
int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);

// 可以尝试增加到 3000ms
int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 3000);
```

### 5. WSL2 USB设备穿透问题
WSL2 的USB设备支持可能不稳定。检查：

```bash
# 查看WSL2的USB设备
lsusb

# 如果使用 usbipd 工具，检查设备绑定状态
# 在Windows PowerShell中：
usbipd list
```

确保相机在WSL2中正确识别。

### 6. 使用 MindVision 官方工具测试
使用 MindVision 提供的相机测试工具验证硬件：
```bash
cd Camera/
# 运行官方测试程序
```

## 替代方案

### 方案A：使用软件触发模式
如果USB带宽确实不足，可以考虑使用软件触发模式，手动控制两个相机交替采集：

1. 设置触发模式为软件触发（mode=1）
2. 交替发送软触发信号
3. 这样可以避免两个相机同时传输数据

### 方案B：降低帧率
已经添加的 `frame_speed=0` (Low模式) 是最低设置。如果还不够，可能需要：
- 降低曝光时间
- 减少数据传输量

### 方案C：使用原生Linux而非WSL2
WSL2 的USB支持不如原生Linux稳定。如果可能，建议在原生Linux环境下测试。

## 配置文件示例

`config/dual_camera_params.yaml`:
```yaml
/camera_left:
  mv_camera:
    ros__parameters:
      camera_name: camera_left
      camera_sn: "041170020075"
      exposure_time: 3500
      analog_gain: 64
      frame_speed: 0  # 低速模式，降低带宽
      rgb_gain:
        r: 100
        g: 100
        b: 100
      saturation: 128
      gamma: 100
      flip_image: false

/camera_right:
  mv_camera:
    ros__parameters:
      camera_name: camera_right
      camera_sn: "042081320100"
      exposure_time: 3500
      analog_gain: 64
      frame_speed: 0  # 低速模式，降低带宽
      rgb_gain:
        r: 100
        g: 100
        b: 100
      saturation: 128
      gamma: 100
      flip_image: false
```

## 已修改的文件

1. `src/mv_camera_node.cpp`
   - 添加触发模式设置
   - 添加帧速度参数支持
   - 添加参数动态调整回调

2. `config/dual_camera_params.yaml`
   - 修复序列号格式
   - 添加 frame_speed 参数
   - 修复 YAML 层级结构

3. `launch/dual_camera_launch.py`
   - 添加右相机延迟启动（2秒）

## 参考资料

- MindVision SDK 文档
- CameraApi.h - `CameraSetFrameSpeed()` 函数说明
- CameraDefine.h - `emSdkFrameSpeed` 枚举定义
