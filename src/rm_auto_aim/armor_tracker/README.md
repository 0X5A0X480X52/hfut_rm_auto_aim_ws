# Armor Tracker

## 概述

多装甲板跟踪包,为每个检测到的装甲板维护独立的EKF跟踪器。

## 功能特性

- ✅ 多目标跟踪: 同时跟踪多个装甲板
- ✅ 数据关联: 自动将检测结果与跟踪器匹配
- ✅ EKF滤波: 使用扩展卡尔曼滤波器进行状态估计
- ✅ 状态机管理: LOST/DETECTING/TRACKING/TEMP_LOST
- ✅ 可视化: RViz可视化跟踪结果

## 订阅话题

- `/armor_detector/armors` (rm_interfaces/msg/Armors)
  - 装甲板检测结果

## 发布话题

- `/armor_tracker/tracked_armors` (rm_interfaces/msg/TrackedArmors)
  - 所有被跟踪装甲板的状态列表

- `/armor_tracker/markers` (visualization_msgs/msg/MarkerArray) [调试模式]
  - 可视化标记

## 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `debug` | bool | true | 是否启用调试模式 |
| `max_match_distance` | double | 0.2 | 最大匹配距离(米) |
| `max_match_yaw_diff` | double | 1.0 | 最大匹配yaw角差(弧度) |
| `tracking_threshold` | int | 5 | 进入TRACKING状态的帧数 |
| `lost_threshold` | int | 10 | 进入LOST状态的帧数 |
| `max_trackers` | int | 20 | 最大跟踪器数量 |
| `ekf.sigma2_q_xyz` | double | 20.0 | 位置过程噪声方差 |
| `ekf.sigma2_q_yaw` | double | 1.0 | yaw角过程噪声方差 |
| `ekf.r_xyz` | double | 0.05 | 位置测量噪声方差 |
| `ekf.r_yaw` | double | 0.02 | yaw角测量噪声方差 |

## 使用方法

```bash
# 启动跟踪器
ros2 launch armor_tracker armor_tracker.launch.py

# 查看跟踪结果
ros2 topic echo /armor_tracker/tracked_armors

# RViz可视化
rviz2
# 添加 MarkerArray, Topic: /armor_tracker/markers
```

## 架构说明

### 核心类

1. **ArmorTrackerNode**: 主节点,管理多个跟踪器
2. **SingleArmorTracker**: 单个装甲板跟踪器
3. **ArmorEKF**: 扩展卡尔曼滤波器

### 状态向量

```
State: [x, vx, y, vy, z, vz, yaw, vyaw]
```

- `x, y, z`: 3D位置
- `vx, vy, vz`: 3D速度
- `yaw`: yaw角
- `vyaw`: yaw角速度

### 观测向量

```
Measurement: [x, y, z, yaw]
```

## 待优化

- [ ] 使用匈牙利算法优化数据关联
- [ ] 支持装甲板融合(同一机器人的多个装甲板)
- [ ] 自适应噪声参数
