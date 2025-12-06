# Robot Pose Estimator

机器人姿态估计模块，基于 `armor_detector` 的检测结果，估计机器人的中心位置、姿态和几何参数。

## 功能概述

1. **装甲板分组**: 将检测到的装甲板按机器人ID分组
2. **机器人类型识别**: 根据装甲板ID和类型识别机器人类型（平衡步兵、标准机器人、英雄、前哨站等）
3. **EKF状态估计**: 使用扩展卡尔曼滤波估计机器人中心位置、速度、yaw角和旋转半径
4. **虚拟装甲板生成**: 生成所有装甲板（包括被遮挡的）的估计位置
5. **Target兼容输出**: 输出与原 `armor_solver` 兼容的 Target 消息

## 数据流

```
                       /armor_detector/armors (Armors)
                                 |
                    +------------+-----------+
                    |                        |
                    v                        v
          [robot_pose_estimator]        [armor_tracker]
                    |                        ^
                    +---> /robot_pose_estimator/virtual_armors (Armors)
                    |                        |
                    +------------------------+
                    |
                    +---> /robot_pose_estimator/robots (TrackedRobots)
                    |
                    +---> /robot_pose_estimator/target (Target)
```

**注意**: 为避免反馈回路，`robot_pose_estimator` 直接订阅 `armor_detector` 的检测结果，
而不是 `armor_tracker` 的跟踪结果。虚拟装甲板单向流向 `armor_tracker`。

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/armor_detector/armors` | `rm_interfaces/Armors` | 检测到的装甲板列表（直接从detector获取） |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot_pose_estimator/robots` | `rm_interfaces/TrackedRobots` | 机器人状态列表 |
| `/robot_pose_estimator/virtual_armors` | `rm_interfaces/Armors` | 虚拟装甲板位置（发送给armor_tracker） |
| `/robot_pose_estimator/target` | `rm_interfaces/Target` | 兼容 armor_solver 的目标消息 |
| `/robot_pose_estimator/markers` | `visualization_msgs/MarkerArray` | 可视化标记 (debug模式) |

## 可视化

在 debug 模式下，会发布以下可视化标记到 `/robot_pose_estimator/markers`:

- **robot_centers**: 机器人中心位置（绿色球体）
- **robot_ids**: 机器人ID和类型文本
- **predicted_armors**: 预测的装甲板位置（蓝色方块，类似 armor_solver 的可视化风格）

## 输入过滤

仅使用状态为以下状态的装甲板作为输入：
- `DETECTING (1)`: 检测中
- `TRACKING (2)`: 跟踪中
- `TEMP_LOST (3)`: 临时丢失（预测）

状态为 `LOST (0)` 的装甲板将被忽略。

## 机器人类型识别

| 装甲板ID | 装甲板类型 | 机器人类型 | 装甲板数量 |
|----------|----------|------------|----------|
| 3, 4, 5 | large | BALANCE_2 | 2 |
| 1 | * | STANDARD_4 | 4 |
| 2 | * | HERO_4 | 4 |
| outpost | * | OUTPOST_3 | 3 |
| base | * | BASE | 4 |
| 其他 | * | STANDARD_4 | 4 |

## 状态向量

EKF使用10维状态向量：
```
[xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc]
```
- `xc, yc, zc`: 机器人中心位置
- `vxc, vyc, vzc`: 机器人中心速度
- `yaw`: 机器人朝向角
- `v_yaw`: yaw角速度
- `r`: 旋转半径
- `d_zc`: z方向偏移

## 观测向量

4维观测向量：
```
[xa, ya, za, yaw_a]
```
- `xa, ya, za`: 装甲板位置
- `yaw_a`: 装甲板朝向角

## 配置参数

### EKF参数
```yaml
ekf:
  sigma2_q_xyz: 0.05    # 位置过程噪声
  sigma2_q_yaw: 1.0     # yaw过程噪声
  sigma2_q_r: 0.05      # 半径过程噪声
  r_xyz: 0.05           # 位置观测噪声
  r_yaw: 0.02           # yaw观测噪声
```

### 跟踪参数
```yaml
tracking:
  max_match_distance: 0.5   # 最大匹配距离(m)
  max_match_yaw_diff: 1.0   # 最大yaw差(rad)
  tracking_threshold: 5     # 进入跟踪状态的帧数
  lost_threshold: 100       # 丢失阈值(帧数)
```

### 机器人几何参数
```yaml
robot:
  balance_radius: 0.15      # 平衡步兵半径(m)
  standard_radius: 0.23     # 标准机器人半径(m)
  hero_radius: 0.28         # 英雄机器人半径(m)
  outpost_radius: 0.26      # 前哨站半径(m)
```

## 编译

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws

# 编译依赖
colcon build --packages-select rm_interfaces rm_utils

# 编译 robot_pose_estimator
colcon build --packages-select robot_pose_estimator

# 刷新环境
source install/setup.bash
```

## 运行

```bash
# 使用默认配置
ros2 launch robot_pose_estimator robot_pose_estimator.launch.py

# 启用调试模式
ros2 launch robot_pose_estimator robot_pose_estimator.launch.py debug:=true

# 使用自定义配置
ros2 launch robot_pose_estimator robot_pose_estimator.launch.py \
    config_file:=/path/to/custom_config.yaml
```

## 与 armor_solver 的关系

本模块从 `armor_solver` 中提取了以下功能：
- EKF状态估计逻辑
- 装甲板位置计算
- 机器人类型识别
- 装甲板跳变处理

并提供了与 `armor_solver` 兼容的 Target 消息输出，可直接替代原有的跟踪输出。

## 架构说明

```
robot_pose_estimator/
├── include/robot_pose_estimator/
│   ├── robot_types.hpp              # 类型定义
│   ├── robot_tracker.hpp            # 单机器人跟踪器
│   ├── armor_grouper.hpp            # 装甲板分组器
│   ├── virtual_armor_generator.hpp  # 虚拟装甲板生成器
│   ├── robot_pose_estimator_core.hpp# 核心估计器
│   └── robot_pose_estimator_node.hpp# ROS2节点
├── src/
│   ├── robot_tracker.cpp
│   ├── armor_grouper.cpp
│   ├── virtual_armor_generator.cpp
│   ├── robot_pose_estimator_core.cpp
│   └── robot_pose_estimator_node.cpp
├── config/
│   └── robot_pose_estimator.yaml
├── launch/
│   └── robot_pose_estimator.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 待实现功能

- [ ] 多机器人跟踪优化
- [ ] 机器人类型自适应识别
- [ ] 旋转半径在线估计
- [ ] 与 target_selector 集成

## License

Apache-2.0
