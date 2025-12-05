# Target Selector

## 概述

目标选择器包,基于策略模式从机器人列表中选择击打目标。支持多种选择策略,并通过 Action 和 Service 与 `trajectory_planner` 进行交互。

## 功能特性

- ✅ **策略模式**: 可扩展的目标选择策略框架
- ✅ **最小 Yaw 偏差策略**: 选择与参考方向偏差最小的目标
- ✅ **滞回机制**: 防止目标频繁切换
- ✅ **Action 接口**: 控制 trajectory_planner 的轨迹规划
- ✅ **Service 接口**: 设置 trajectory_planner 的目标机器人
- ✅ **可视化**: 支持 RViz 可视化标记

## 架构

```
                              ┌─────────────────────┐
                              │   TrackedRobots     │
                              │  (from estimator)   │
                              └──────────┬──────────┘
                                         │
                                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                      TargetSelectorNode                         │
│  ┌───────────────┐   ┌───────────────────────────────────────┐ │
│  │ SelectionConfig│   │         SelectionStrategy             │ │
│  │ - reference_yaw│   │  ┌─────────────────────────────────┐ │ │
│  │ - max_deviation│   │  │   MinYawDeviationStrategy       │ │ │
│  │ - hysteresis   │   │  │   (可扩展更多策略)               │ │ │
│  └───────────────┘   │  └─────────────────────────────────┘ │ │
│                       └───────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                         │
                    ┌────────────────────┼────────────────────┐
                    │                    │                    │
                    ▼                    ▼                    ▼
           SelectedTarget        SetTargetRobot        TrajectoryPlan
              (Topic)              (Service)             (Action)
                                         │                    │
                                         └────────────────────┘
                                                   │
                                                   ▼
                                        ┌─────────────────────┐
                                        │  trajectory_planner │
                                        └─────────────────────┘
```

## 接口

### 订阅话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/robot_pose_estimator/robots` | `rm_interfaces/TrackedRobots` | 被跟踪的机器人列表 |

### 发布话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/target_selector/selected_target` | `rm_interfaces/SelectedTarget` | 选中的打击目标 |
| `/target_selector/markers` | `visualization_msgs/MarkerArray` | 可视化标记 (debug模式) |

### 服务客户端

| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/trajectory_planner/set_target_robot` | `rm_interfaces/SetTargetRobot` | 设置目标机器人ID |

### Action 客户端

| Action名 | Action类型 | 描述 |
|----------|------------|------|
| `/trajectory_planner/trajectory_plan` | `rm_interfaces/TrajectoryPlan` | 轨迹规划控制 |

## 消息定义

### SelectedTarget.msg
```
std_msgs/Header header
string robot_id           # 选中的机器人ID
float64 confidence        # 选择置信度 [0,1]
string selection_strategy # 使用的选择策略名称
```

### SetTargetRobot.srv
```
# Request
string robot_id           # 目标机器人ID
---
# Response
bool success              # 是否成功设置
string message            # 状态消息
string previous_robot_id  # 之前的目标ID
```

### TrajectoryPlan.action
```
# Goal
string robot_id           # 目标机器人ID
float64 reference_yaw     # 参考yaw方向
bool enable_tracking      # 是否启用跟踪
---
# Result
bool success
string message
float64 total_tracking_time
---
# Feedback
std_msgs/Header header
string current_robot_id
float64 yaw_error
float64 pitch_error
float64 distance
bool target_locked
uint8 planning_status     # 0=IDLE, 1=PLANNING, 2=TRACKING, 3=LOST_TARGET
```

## 选择策略

### MinYawDeviationStrategy (默认)

选择机器人中心与参考 yaw 方向偏差最小的目标。

**适用场景:**
- 云台快速锁定最近角度目标
- 减少云台旋转量
- 对正前方目标优先级更高

**算法:**
1. 根据置信度、距离、最大偏差过滤候选目标
2. 计算每个候选目标相对于参考方向的 yaw 偏差
3. 考虑滞回机制,避免目标频繁切换
4. 选择偏差最小的目标

### 扩展新策略

继承 `SelectionStrategy` 基类,实现 `selectTarget()` 方法:

```cpp
class MyStrategy : public SelectionStrategy {
public:
  std::optional<SelectionResult> selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) override {
    // 实现自定义选择逻辑
  }
  
  std::string getName() const override { return "MyStrategy"; }
  std::string getDescription() const override { return "My custom strategy"; }
};
```

## 参数配置

```yaml
# 选择策略
strategy: "min_yaw_deviation"

# 参考 yaw 方向 (rad)
reference_yaw: 0.0

# 选择约束
max_yaw_deviation: 3.14159  # 最大 yaw 偏差 (rad)
max_distance: 10.0          # 最大距离 (m)
min_confidence: 0.3         # 最小置信度

# 滞回阈值 (防止目标频繁切换)
hysteresis_threshold: 0.1

# TF 坐标系
gimbal_frame: "gimbal_link"
odom_frame: "odom"

# 话题配置
topics:
  robots_sub: "/robot_pose_estimator/robots"
  selected_target_pub: "/target_selector/selected_target"
  markers_pub: "/target_selector/markers"
  set_target_service: "/trajectory_planner/set_target_robot"
  trajectory_action: "/trajectory_planner/trajectory_plan"

# 调试模式
debug: false
```

## 使用方法

### 编译

```bash
cd ~/your_ws
colcon build --packages-select rm_interfaces target_selector
source install/setup.bash
```

### 运行

```bash
# 使用默认配置
ros2 launch target_selector target_selector.launch.py

# 启用调试模式
ros2 launch target_selector target_selector.launch.py debug:=true

# 指定配置文件
ros2 launch target_selector target_selector.launch.py \
  config_file:=/path/to/config.yaml
```

### 查看选择结果

```bash
ros2 topic echo /target_selector/selected_target
```

### RViz 可视化

1. 启动时设置 `debug:=true`
2. 在 RViz 中添加 MarkerArray
3. 设置 Topic: `/target_selector/markers`

## 数据流

```
robot_pose_estimator/robots
         │
         ▼
  TargetSelectorNode
         │
    ┌────┴────┐
    ▼         ▼
Strategy   Service/Action
Selection    Clients
    │         │
    ▼         ▼
selected_  trajectory_
 target     planner
```

## 文件结构

```
target_selector/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── target_selector.yaml
├── launch/
│   └── target_selector.launch.py
├── include/target_selector/
│   ├── selection_strategy.hpp       # 策略基类
│   ├── target_selector_node.hpp     # 节点头文件
│   └── strategies/
│       └── min_yaw_deviation_strategy.hpp
└── src/
    ├── selection_strategy.cpp       # 策略基类实现
    ├── target_selector_node.cpp     # 节点实现
    └── strategies/
        └── min_yaw_deviation_strategy.cpp
```

## 待实现功能

- [ ] 更多选择策略
  - [ ] NearestStrategy: 选择最近目标
  - [ ] PriorityStrategy: 基于优先级选择
  - [ ] ThreatStrategy: 基于威胁度评估
- [ ] 裁判系统数据集成 (血量、比赛状态等)
- [ ] 动态参数调整
- [ ] 单元测试

## License

Apache-2.0
