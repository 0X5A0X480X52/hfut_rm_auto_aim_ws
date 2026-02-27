# max_entropy_tracker（C++ 实现）

基于最大熵 UKF 的机器人装甲板位姿估计器，C++17 实现。是 Python 版本 `max_entropy_tracker_python` 的完全等价 C++ 重写，作为 ROS 2 可加载组件（`rclcpp_components`）提供。

---

## 功能概述

- **双半径自旋 UKF**：针对 4 面装甲板机器人，支持两种不同半径（r1/r2）交替出现的几何结构
- **Yaw 分解**：`yaw = k·π + δ`，将连续旋转分解为整数周期 k 与残差 δ，避免角度跳变
- **最大熵 Panel 关联**：利用最大熵原理，从多个候选 panel 中选择最优关联
- **组合过程模型**：支持 CV / CA / Singer 平动模型，运行时可配置
- **单/双观测自适应**：单观测时冻结结构参数（r1/r2/dza），双观测时启用几何约束更新
- **ROS 2 组件节点**：注册为 `fyt::auto_aim::MaxEntropyTrackerNode`，支持零拷贝 intra-process 通信

---

## 构建方式

```bash
# 在工作空间根目录执行
colcon build --packages-select max_entropy_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**依赖**：ROS 2 Humble、Eigen3、rclcpp_components、rm_interfaces、tf2_ros、tf2_geometry_msgs

---

## 目录结构

```
max_entropy_tracker/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── tracker_params.yaml          # 默认运行参数
├── include/max_entropy_tracker/
│   ├── core/
│   │   ├── config.hpp               # UnifiedConfig：全局参数结构体
│   │   └── observation.hpp          # ObservationData：单帧观测数据
│   ├── utils/
│   │   ├── angle_utils.hpp          # 角度归一化、Yaw 分解/合成
│   │   ├── sigma_points.hpp         # Scaled UT Sigma 点生成器
│   │   └── constraints.hpp          # 状态约束、协方差正定性修复
│   ├── filters/
│   │   ├── base_ukf.hpp             # UKF 抽象基类（Sigma 点、Kalman 更新）
│   │   ├── dual_radius_spin_ukf.hpp # 双半径自旋 UKF 头文件
│   │   └── process_models/
│   │       ├── base.hpp             # StateLayout + ProcessModelComponent 接口
│   │       ├── translation.hpp      # CV / CA / Singer 平动模型
│   │       ├── rotation.hpp         # CV / CA 旋转模型（δ、δ̇）
│   │       ├── structural.hpp       # 结构参数随机游走（r1、r2、dza）
│   │       └── composite.hpp        # 组合模型 + 状态索引 DynamicStateIndex
│   ├── association/
│   │   ├── panel_associator.hpp     # Panel ID 关联（yaw → panel_id）
│   │   ├── height_identifier.hpp    # 装甲板高度层级识别（上/下）
│   │   └── oscillation_detector.hpp # r1/r2 参数震荡检测
│   ├── trackers/
│   │   ├── base_tracker.hpp         # 跟踪器状态机基类（INITIALIZING / TRACKING / TEMP_LOST / LOST）
│   │   └── adaptive_armor_tracker.hpp
│   ├── tracker_manager.hpp          # 多机器人跟踪器生命周期管理
│   ├── msg_converter.hpp            # 装甲板消息 ↔ ObservationData 转换
│   ├── tf_handler.hpp               # TF2 坐标变换封装
│   └── max_entropy_tracker_node.hpp # ROS 2 节点头文件
└── src/
    ├── filters/
    │   └── dual_radius_spin_ukf.cpp # 双半径 UKF 实现（~537 行）
    ├── trackers/
    │   └── adaptive_armor_tracker.cpp
    └── max_entropy_tracker_node.cpp # 节点实现：参数声明、回调、消息发布
```

---

## 状态向量

状态维度随过程模型动态变化，通过 `DynamicStateIndex` 按名称访问：

| 名称 | 说明 | CV | CA/Singer |
|------|------|:---:|:---:|
| X / VX | 中心 X 位置/速度 | ✓ | ✓ |
| Y / VY | 中心 Y 位置/速度 | ✓ | ✓ |
| Z / VZ | 中心 Z 位置/速度 | ✓ | ✓ |
| AX / AY / AZ | 加速度 | — | ✓ |
| DELTA | 残差角 δ ∈ (−π/2, π/2) | ✓ | ✓ |
| DELTA_RATE | δ 角速度 | ✓ | ✓ |
| R1 | 偶数 panel 半径 | ✓ | ✓ |
| R2 | 奇数 panel 半径 | ✓ | ✓ |
| DZA | 上/下层装甲板高度差 | ✓ | ✓ |

---

## 过程模型

通过参数 `motion.translation_model` 在运行时选择：

| 值 | 类型 | 说明 |
|----|------|------|
| `CV` | `CVTranslation` | 匀速模型，过程噪声作用于速度 |
| `CA` | `CATranslation` | 匀加速模型，过程噪声作用于加速度 |
| `Singer` | `SingerTranslation` | Singer 机动目标模型，含机动时间常数 α |

旋转子模型固定为 `CVRotation`（δ 匀速游走）；结构参数（r1/r2/dza）使用 `StructuralModel` 随机游走。

---

## ROS 2 接口

### 订阅

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/armor_detector/armors` | `rm_interfaces/msg/Armors` | BEST_EFFORT | 装甲板检测结果 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/max_entropy_tracker/target` | `rm_interfaces/msg/Target` | 当前最优跟踪目标（供云台控制器使用） |
| `/max_entropy_tracker/tracked_robots` | `rm_interfaces/msg/TrackedRobots` | 所有跟踪中机器人状态列表 |
| `/max_entropy_tracker/markers` | `visualization_msgs/msg/MarkerArray` | 可视化标记（debug 模式） |

---

## 参数说明

主要参数位于 `config/tracker_params.yaml`：

```yaml
target_frame: "odom"              # 跟踪坐标系
source_frame: "camera_optical_frame"
predict_rate: 100.0               # 预测频率 (Hz)
debug_mode: false

ukf:
  alpha: 0.001                    # Sigma 点扩散系数
  obs_noise_pos: 0.05             # 单观测位置噪声
  dual_obs_noise_pos: 0.01        # 双观测位置噪声

motion:
  translation_model: "CA"         # CV | CA | Singer
  ca_process_noise_acc: 1.0
  singer_alpha: 0.5

tracker:
  tracking_thres: 2               # 进入 TRACKING 所需帧数
  lost_thres: 8                   # 进入 LOST 所需丢失帧数
  max_match_distance: 2.0         # 关联最大距离 (m)
```

---

## 启动

```bash
# 直接运行可执行文件
ros2 run max_entropy_tracker max_entropy_tracker_node

# 使用 bringup 启动文件（默认 C++ 版本）
ros2 launch rm_bringup bringup_max_entropy_test.launch.py tracker_impl:=cpp
```

---

## 与 Python 版本的对应关系

| Python 模块 | C++ 等价文件 |
|-------------|-------------|
| `core/config.py` | `core/config.hpp` |
| `core/observation.py` | `core/observation.hpp` |
| `utils/angle_utils.py` | `utils/angle_utils.hpp` |
| `utils/sigma_points.py` | `utils/sigma_points.hpp` |
| `utils/constraints.py` | `utils/constraints.hpp` |
| `filters/base_ukf.py` | `filters/base_ukf.hpp` |
| `filters/dual_radius_spin_ukf.py` | `filters/dual_radius_spin_ukf.hpp/cpp` |
| `filters/process_models/` | `filters/process_models/` |
| `association/panel_associator.py` | `association/panel_associator.hpp` |
| `association/height_identifier.py` | `association/height_identifier.hpp` |
| `association/oscillation_detector.py` | `association/oscillation_detector.hpp` |
| `trackers/base_tracker.py` | `trackers/base_tracker.hpp` |
| `trackers/adaptive_armor_tracker.py` | `trackers/adaptive_armor_tracker.hpp/cpp` |
| `tracker_manager.py` | `tracker_manager.hpp` |
| `msg_converter.py` | `msg_converter.hpp` |
| `tf_handler.py` | `tf_handler.hpp` |
| `max_entropy_tracker_node.py` | `max_entropy_tracker_node.hpp/cpp` |
