# max_entropy_tracker_python（Python 实现）

基于最大熵 UKF 的机器人装甲板位姿估计器，Python 实现。源代码位于同级目录 `max_entropy_tracker/max_entropy_tracker/`，本包（`max_entropy_tracker_python`）通过符号链接将其封装为独立的 `ament_python` ROS 2 包，与 C++ 包 `max_entropy_tracker` 共存于同一工作空间。

---

## 包结构

```
max_entropy_tracker_python/
├── package.xml                   # ament_python 包声明
├── setup.py                      # 安装入口，注册 max_entropy_tracker_node 可执行文件
├── setup.cfg
├── resource/
│   └── max_entropy_tracker_python
├── max_entropy_tracker/          # 符号链接 → ../max_entropy_tracker/max_entropy_tracker/
├── config/                       # 符号链接 → ../max_entropy_tracker/config/
└── launch/                       # 符号链接 → ../max_entropy_tracker/launch/
```

> **注意**：`max_entropy_tracker/`、`config/`、`launch/` 均为符号链接，实际源文件在 `../max_entropy_tracker/` 目录下维护，两个包共享同一份代码和配置。

---

## Python 模块结构

```
max_entropy_tracker/              # Python 包（实际源码）
├── __init__.py
├── max_entropy_tracker_node.py   # ROS 2 节点入口（main()）
├── tracker_manager.py            # 多机器人跟踪器管理
├── msg_converter.py              # 消息 ↔ ObservationData 转换
├── tf_handler.py                 # TF2 坐标变换
├── visualization.py              # Markers 可视化
│
├── core/
│   ├── config.py                 # UnifiedConfig：全局参数数据类
│   └── observation.py            # ObservationData：单帧观测
│
├── utils/
│   ├── angle_utils.py            # 角度归一化、Yaw 分解/合成
│   ├── sigma_points.py           # Scaled UT Sigma 点生成器
│   └── constraints.py            # 状态约束、协方差正定性修复
│
├── filters/
│   ├── base_ukf.py               # UKF 抽象基类
│   ├── dual_radius_spin_ukf.py   # 双半径自旋 UKF
│   └── process_models/
│       ├── base.py               # StateLayout + ProcessModelComponent
│       ├── translation.py        # CV / CA / Singer 平动模型
│       ├── rotation.py           # CV / CA 旋转模型
│       ├── structural.py         # 结构参数随机游走
│       └── composite.py          # 组合模型 + 工厂函数
│
├── association/
│   ├── panel_associator.py       # Panel ID 关联（yaw → panel_id）
│   ├── height_identifier.py      # 装甲板高度层级识别
│   └── oscillation_detector.py   # r1/r2 参数震荡检测
│
└── trackers/
    ├── base_tracker.py           # 状态机基类（INITIALIZING / TRACKING / TEMP_LOST / LOST）
    └── adaptive_armor_tracker.py # 自适应装甲板跟踪器
```

---

## 核心算法

### 双半径自旋 UKF（DualRadiusSpinUKF）

动态维度状态向量（11D for CV，14D for CA/Singer），通过 `StateLayout` 按名称索引：

| 名称 | 说明 |
|------|------|
| X / VX | 中心 X 位置/速度 |
| Y / VY | 中心 Y 位置/速度 |
| Z / VZ | 中心 Z 位置/速度 |
| AX / AY / AZ | 加速度（CA/Singer 模式） |
| DELTA | 残差角 δ ∈ (−π/2, π/2) |
| DELTA_RATE | δ 角速度 |
| R1 | 偶数 panel 半径 |
| R2 | 奇数 panel 半径 |
| DZA | 上/下层装甲板高度差 |

**Yaw 分解**：`yaw = k·π + δ`，k ∈ {0, 1}，最大熵原理选择最优 k。

### 自适应装甲板跟踪器（AdaptiveArmorTracker）

| 组件 | 作用 |
|------|------|
| `DualRadiusSpinUKF` | 核心状态估计 |
| `PanelAssociator` | 将观测 yaw 匹配到最近的 panel |
| `HeightIdentifier` | 识别装甲板是上层还是下层 |
| `OscillationDetector` | 检测 r1/r2 震荡，触发参数重置 |

---

## 构建方式

```bash
colcon build --packages-select max_entropy_tracker_python
source install/setup.bash
```

**依赖**：ROS 2 Humble、rclpy、numpy、rm_interfaces、tf2_ros、tf2_geometry_msgs

---

## ROS 2 接口

### 订阅

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/armor_detector/armors` | `rm_interfaces/msg/Armors` | BEST_EFFORT | 装甲板检测结果 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/max_entropy_tracker/target` | `rm_interfaces/msg/Target` | 当前最优跟踪目标 |
| `/max_entropy_tracker/tracked_robots` | `rm_interfaces/msg/TrackedRobots` | 所有跟踪中机器人状态 |
| `/max_entropy_tracker/markers` | `visualization_msgs/msg/MarkerArray` | 可视化标记（debug 模式） |

---

## 启动

```bash
# 直接运行
ros2 run max_entropy_tracker_python max_entropy_tracker_node

# 使用 bringup 启动文件
ros2 launch rm_bringup bringup_max_entropy_test.launch.py tracker_impl:=python
```

---

## 与 C++ 版本的关系

| 特性 | Python (`max_entropy_tracker_python`) | C++ (`max_entropy_tracker`) |
|------|---------------------------------------|------------------------------|
| 构建类型 | `ament_python` | `ament_cmake` |
| 运行方式 | 独立进程 (`ros2 run`) | 独立可执行文件或可加载组件 |
| 调试便利性 | 高（运行时修改代码即生效） | 低（需重新编译） |
| 运行性能 | 受 Python GIL 限制 | 原生 C++17，-O3 优化 |
| 参数文件 | 共享 `config/tracker_params.yaml` | 共享同一份 yaml |
| 算法等价性 | ✓ 完全等价 | ✓ 完全等价 |

在 `launch_params_decoupled.yaml` 中设置 `tracker_impl: python` 或 `tracker_impl: cpp` 来切换启动版本。
