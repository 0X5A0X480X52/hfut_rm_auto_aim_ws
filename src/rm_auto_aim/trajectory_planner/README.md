# Trajectory Planner

## 概述

基于MPC的云台轨迹规划器 Python 实现。该包实现了模块化的轨迹规划系统，用于RoboMaster自动瞄准系统中的云台控制。

## 架构

```
                    ┌────────────────────────────────────────────────────────┐
                    │              TrajectoryPlannerNode                      │
                    │                                                         │
   TrackedRobots ──►│  ┌──────────────┐    ┌──────────────────┐              │
(robot_pose_estimator)│  │TargetManager │◄──►│SetTargetRobot Srv│◄── target_selector
                    │  └──────┬───────┘    └──────────────────┘              │
                    │         │                                               │
TrackPredictionWindows│  ┌────▼─────────┐   ┌─────────────────┐              │
   (armor_tracker) ──►│  │TargetPredictor│──►│  MPCController  │              │
                    │  └──────────────┘   └────────┬────────┘              │
                    │                              │                        │
                    │  ┌──────────────┐   ┌────────▼────────┐              │
   JointStates ────►│  │ GimbalModel  │◄──┤                 │              │
                    │  └──────────────┘   │                 │              │
                    │                      │    Control      │              │
                    │  ┌──────────────┐   │    Output       │──► GimbalCmd
                    │  │BallisticClient│◄──┤                 │   (yaw, pitch,
                    │  └──────────────┘   └─────────────────┘    fire_advice)
                    │         │                                               │
                    │         ▼                                               │
                    │  ballistic_solver                                       │
                    │     (service)                                           │
                    └────────────────────────────────────────────────────────┘
```

## 功能特性

- ✅ **QP-MPC控制**: 基于二次规划的模型预测控制
- ✅ **三阶云台模型**: 角度-角速度-角加速度状态空间模型
- ✅ **目标轨迹预测**: 从 armor_tracker 预测窗口获取目标轨迹
- ✅ **目标机器人管理**: 通过 Service 接收 target_selector 的目标设置
- ✅ **弹道解算集成**: 调用 ballistic_solver 服务计算 pitch
- ✅ **开火建议**: 基于误差、距离、置信度的开火决策
- ✅ **Action接口**: 支持 TrajectoryPlan Action 用于跟踪控制
- ✅ **模块化设计**: 各模块解耦，易于扩展和测试

## 模块说明

### GimbalModel (gimbal_model.py)
云台三阶动力学模型
- 状态: [θ, ω, α] (角度、角速度、角加速度)
- 控制输入: jerk (角加加速度)
- 支持状态约束和角度边界

### MPCController (mpc_controller.py)
基于二次规划的MPC控制器
- 预测时域优化
- 跟踪误差、控制能量、平滑性多目标优化
- 状态和控制约束处理

### TargetPredictor (target_predictor.py)
目标预测器
- 从 armor_tracker 预测窗口提取目标轨迹
- 支持多种装甲板选择策略
- 提供目标yaw角轨迹供MPC使用

### TargetManager (target_manager.py)
目标管理器
- 处理 target_selector 的目标设置请求
- 维护 robot_pose_estimator 的机器人信息
- 提供目标装甲板过滤功能

### BallisticClient (ballistic_client.py)
弹道解算客户端
- 调用 ballistic_solver 服务
- 支持同步/异步调用
- 提供简单pitch估算fallback

## 接口

### 订阅话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/armor_tracker/prediction_windows` | `TrackPredictionWindows` | 装甲板预测轨迹 |
| `/robot_pose_estimator/robots` | `TrackedRobots` | 机器人信息 |
| `/joint_states` | `JointState` | 当前云台状态 |

### 发布话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/trajectory_planner/gimbal_cmd` | `GimbalCmd` | 云台控制指令 |
| `/trajectory_planner/trajectory` | `GimbalTrajectory` | 规划轨迹 |
| `/trajectory_planner/markers` | `MarkerArray` | 可视化标记 (debug模式) |

### 服务

| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `~/set_target_robot` | `SetTargetRobot` | 设置目标机器人ID |

### Action

| Action名 | Action类型 | 描述 |
|----------|------------|------|
| `~/trajectory_plan` | `TrajectoryPlan` | 轨迹规划控制 |

### 服务客户端

| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/ballistic_solver/solve` | `SolveBallistic` | 弹道解算 |

## GimbalCmd 消息

```msg
std_msgs/Header header
float64 pitch           # 目标pitch角 (rad)
float64 yaw             # 目标yaw角 (rad)
float64 yaw_diff        # yaw误差 (rad)
float64 pitch_diff      # pitch误差 (rad)
float64 distance        # 目标距离 (m)
bool fire_advice        # 开火建议
```

## 参数配置

```yaml
# 控制频率
control_rate: 100.0  # Hz

# 开火阈值
fire_yaw_threshold: 0.05      # rad (~3°)
fire_pitch_threshold: 0.05    # rad
fire_distance_min: 1.0        # m
fire_distance_max: 8.0        # m
fire_confidence_threshold: 0.5

# 云台配置
gimbal:
  dt: 0.01                    # 时间步长
  omega_max_deg: 360.0        # 最大角速度
  theta_min_deg: -90.0        # 角度下限
  theta_max_deg: 90.0         # 角度上限

# MPC配置
mpc:
  prediction_horizon: 18      # 预测步数
  q_theta: 100.0              # 角度误差权重
  q_omega: 10.0               # 角速度误差权重
  q_alpha: 1.0                # 角加速度误差权重
  r_control: 0.01             # 控制输入权重
  s_smooth: 5.0               # 平滑性权重

# 目标预测器
predictor:
  min_confidence: 0.3
  max_distance: 10.0
  selection_strategy: "nearest_yaw"
```

## 使用方法

### 编译

```bash
cd ~/your_ws
colcon build --packages-select rm_interfaces trajectory_planner
source install/setup.bash
```

### 运行

```bash
# 使用默认配置
ros2 launch trajectory_planner trajectory_planner.launch.py

# 启用调试模式
ros2 launch trajectory_planner trajectory_planner.launch.py debug:=true

# 指定配置文件
ros2 launch trajectory_planner trajectory_planner.launch.py \
  config_file:=/path/to/config.yaml
```

### 设置目标

```bash
# 通过 service 设置目标机器人
ros2 service call /trajectory_planner/set_target_robot \
  rm_interfaces/srv/SetTargetRobot "{robot_id: '3'}"

# 清除目标
ros2 service call /trajectory_planner/set_target_robot \
  rm_interfaces/srv/SetTargetRobot "{robot_id: ''}"
```

### 查看输出

```bash
# 查看云台控制指令
ros2 topic echo /trajectory_planner/gimbal_cmd

# 查看规划轨迹
ros2 topic echo /trajectory_planner/trajectory
```

## 数据流

```
armor_tracker/prediction_windows ───┐
                                    │
robot_pose_estimator/robots ────────┼──► TrajectoryPlannerNode
                                    │           │
joint_states ───────────────────────┘           │
                                                ▼
target_selector ──SetTargetRobot Srv──► TargetManager
                                                │
                                                ▼
                                        TargetPredictor
                                                │
                                                ▼
                                         MPCController
                                                │
                                    ┌───────────┼───────────┐
                                    ▼           ▼           ▼
                             yaw_optimal  BallisticClient  GimbalModel
                                    │           │
                                    └─────┬─────┘
                                          ▼
                                     GimbalCmd
                                  (yaw, pitch, fire_advice)
```

## 文件结构

```
trajectory_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── trajectory_planner.yaml
├── launch/
│   └── trajectory_planner.launch.py
├── scripts/
│   └── trajectory_planner_node.py
├── trajectory_planner/
│   ├── __init__.py
│   ├── gimbal_model.py           # 云台动力学模型
│   ├── mpc_controller.py         # MPC控制器
│   ├── target_predictor.py       # 目标预测器
│   ├── target_manager.py         # 目标管理器
│   ├── ballistic_client.py       # 弹道解算客户端
│   └── trajectory_planner_node.py # 主节点
└── demo/
    └── test05_qp_mpc.py          # MPC演示代码
```

## 依赖

- Python >= 3.8
- numpy
- scipy
- rclpy
- rm_interfaces

## 与其他包的交互

### target_selector
- 通过 `SetTargetRobot` service 接收目标机器人ID
- target_selector 选择目标后调用该 service 通知 trajectory_planner

### armor_tracker
- 订阅 `TrackPredictionWindows` 获取装甲板预测轨迹
- 预测窗口包含装甲板的位置、速度、yaw等预测序列

### robot_pose_estimator
- 订阅 `TrackedRobots` 获取机器人信息
- 根据目标机器人ID获取其绑定的装甲板列表

### ballistic_solver
- 调用 `SolveBallistic` service 计算 pitch 角
- 给定目标位置、速度和子弹速度，返回发射角度

## License

Apache-2.0
