# Armor Solver 解耦设计方案

## 当前架构分析

当前 `armor_solver` 包集成了以下功能:
1. **Tracker (armor_tracker)**: 基于 EKF 的单目标跟踪器
2. **ArmorFliter**: 装甲板筛选器
3. **Solver**: 轨迹解算和云台指令生成
4. **ArmorSolverNode**: 主节点,协调各模块

### 存在的问题
- 职责不清晰,Tracker、姿态估计、轨迹规划混杂在一起
- 难以扩展到多机器人跟踪场景
- 缺少显式的机器人级姿态估计
- 目标选择逻辑嵌入在 Solver 中

---

## 优化后的解耦方案

### 包结构设计

```
rm_auto_aim/
├── armor_tracker/              # 1. 跟踪器包
├── robot_pose_estimator/       # 2. 机器人姿态估计器包
├── target_selector/            # 3. 目标选择器包
├── trajectory_planner/         # 4. 轨迹规划器包
├── ballistic_solver/           # 5. 弹道解算器包
├── gimbal_controller/          # 6. 云台控制器包
└── armor_solver/               # 原包保持不变
```

---

## 1. armor_tracker (跟踪器包)

### 功能
- 接收 PnP 解算的装甲板列表,为每个装甲板维护独立的跟踪器
- 使用 EKF 进行状态估计和预测
- 输出所有被跟踪装甲板的状态列表

### 输入
- 话题: `/armor_detector/armors` (rm_interfaces::msg::Armors)
  - PnP 解算得到的装甲板 3D 位置、姿态、ID、类型

### 输出
- 话题: `/armor_tracker/tracked_armors` (rm_interfaces::msg::TrackedArmors)
  - 所有被跟踪装甲板的状态列表
  - 每个装甲板包含: 位置、速度、ID、置信度、跟踪状态

### 核心类
```cpp
class ArmorTracker {
  // 单个装甲板跟踪器 (基于原 Tracker 类)
  - std::unique_ptr<ExtendedKalmanFilter> ekf_;
  - TrackingState state_; // LOST, DETECTING, TRACKING, TEMP_LOST
  - std::string armor_id_;
  - Eigen::VectorXd state_; // 位置、速度
  - int detect_count_, lost_count_;
};

class MultiArmorTrackerNode {
  // 多装甲板跟踪管理器
  - std::vector<std::unique_ptr<ArmorTracker>> trackers_;
  - void updateTrackers(const Armors& detections);
  - void associateDetections(const Armors& detections); // 数据关联
  - void pruneTrackers(); // 清理丢失的跟踪器
};
```

### 参数
```yaml
armor_tracker:
  max_match_distance: 0.2       # 数据关联最大距离
  max_match_yaw_diff: 1.0       # 最大yaw差
  tracking_threshold: 5         # 进入TRACKING状态的帧数
  lost_time_threshold: 0.3      # 丢失时间阈值
  max_trackers: 20              # 最大跟踪器数量
  ekf:
    sigma2_q_xyz: 0.05
    sigma2_q_yaw: 1.0
    r_xyz: 0.05
    r_yaw: 0.02
```

### 接口消息定义
```msg
# TrackedArmor.msg
std_msgs/Header header
string armor_id                  # 装甲板ID: "1"~"5", "outpost", "base"
geometry_msgs/Point position     # 3D位置
geometry_msgs/Vector3 velocity   # 3D速度
float64 yaw                      # yaw角
float64 yaw_velocity             # yaw角速度
float64 confidence               # 跟踪置信度 [0,1]
uint8 tracking_state             # LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3
builtin_interfaces/Time last_detected_time

# TrackedArmors.msg
std_msgs/Header header
TrackedArmor[] armors
```

---

## 2. robot_pose_estimator (机器人姿态估计器包)

### 功能
- 将相同 ID 的装甲板绑定到同一机器人实例
- 基于装甲板配置估计机器人的 URDF 结构(Balance/Standard/Hero/Sentry/Outpost)
- 估计机器人中心位置、姿态、旋转半径
- 生成被遮挡装甲板的虚拟检测,反馈给跟踪器

### 输入
- 话题: `/armor_tracker/tracked_armors` (rm_interfaces::msg::TrackedArmors)

### 输出
- 话题: `/robot_pose_estimator/robots` (rm_interfaces::msg::TrackedRobots)
  - 机器人列表,每个包含中心位置、yaw、旋转半径、装甲板配置
- 话题: `/robot_pose_estimator/virtual_armors` (rm_interfaces::msg::Armors)
  - 基于机器人姿态估计的虚拟装甲板位置,反馈给 tracker

### 核心类
```cpp
enum class RobotType {
  BALANCE_2,    // 平衡步兵: 2装甲板
  STANDARD_4,   // 标准机器人: 4装甲板
  HERO_4,       // 英雄: 4装甲板
  OUTPOST_3,    // 前哨站: 3装甲板
  SENTRY,       // 哨兵
  BASE          // 基地
};

struct RobotConfig {
  RobotType type;
  int num_armors;
  double radius;              // 旋转半径
  std::vector<double> angles; // 装甲板角度分布
  double height_offset;       // 中心高度偏移
};

class RobotPoseEstimator {
  - std::string robot_id_;
  - RobotConfig config_;
  - Eigen::Vector3d center_position_;
  - Eigen::Vector3d center_velocity_;
  - double yaw_, yaw_velocity_;
  - std::vector<std::string> bound_armor_ids_;
  
  - void estimateRobotType(const TrackedArmors& armors);
  - void updateCenterPose(const TrackedArmors& armors);
  - std::vector<Armor> generateVirtualArmors();
  - void bindArmorsToURDF();
};

class RobotPoseEstimatorNode {
  - std::map<std::string, std::unique_ptr<RobotPoseEstimator>> robots_;
  - void updateRobots(const TrackedArmors& armors);
  - void groupArmorsByRobot(const TrackedArmors& armors);
};
```

### 参数
```yaml
robot_pose_estimator:
  robot_configs:
    balance:
      num_armors: 2
      radius: 0.15
      angles: [0, 180]
      height_offset: 0.0
    standard:
      num_armors: 4
      radius: 0.23
      angles: [0, 90, 180, 270]
      height_offset: 0.125
    hero:
      num_armors: 4
      radius: 0.28
      angles: [0, 90, 180, 270]
      height_offset: 0.125
    outpost:
      num_armors: 3
      radius: 0.26
      angles: [0, 120, 240]
      height_offset: 0.0
  
  virtual_armor_generation: true
  binding_distance_threshold: 0.5  # 装甲板绑定距离阈值
  robot_timeout: 2.0               # 机器人超时时间
```

### 接口消息定义
```msg
# TrackedRobot.msg
std_msgs/Header header
string robot_id                    # "1"~"5", "outpost", "base"
uint8 robot_type                   # BALANCE_2=0, STANDARD_4=1, HERO_4=2, OUTPOST_3=3, etc.
geometry_msgs/Point center_position
geometry_msgs/Vector3 center_velocity
float64 yaw
float64 yaw_velocity
float64 radius                     # 旋转半径
string[] bound_armor_ids           # 绑定的装甲板ID列表
float64 confidence

# TrackedRobots.msg
std_msgs/Header header
TrackedRobot[] robots
```

---

## 3. target_selector (目标选择器包)

### 功能
- 基于策略从机器人列表中选择击打目标
- 支持多种选择策略: 最近、最弱、优先级、威胁度评估

### 输入
- 话题: `/robot_pose_estimator/robots` (rm_interfaces::msg::TrackedRobots)
- 话题: `/game_status` (可选,用于优先级决策)

### 输出
- 话题: `/target_selector/selected_target` (rm_interfaces::msg::SelectedTarget)
  - 选中的机器人ID

### 核心类
```cpp
enum class SelectionStrategy {
  NEAREST,        // 最近目标
  HIGHEST_THREAT, // 威胁度最高
  PRIORITY_BASED, // 基于优先级
  LOWEST_HP       // 血量最低(需要裁判系统)
};

class TargetSelector {
  - SelectionStrategy strategy_;
  - std::map<std::string, int> priority_map_; // ID -> 优先级
  
  - std::string selectTarget(const TrackedRobots& robots);
  - double calculateThreat(const TrackedRobot& robot);
  - double calculateDistance(const TrackedRobot& robot);
};
```

### 参数
```yaml
target_selector:
  selection_strategy: "priority_based"  # nearest/highest_threat/priority_based
  priority_map:
    "outpost": 1
    "sentry": 2
    "3": 3  # Balance
    "4": 3  # Balance
    "5": 3  # Balance
    "1": 4  # Standard
    "2": 5  # Hero
    "base": 6
  
  threat_weights:
    distance_weight: 0.4
    velocity_weight: 0.3
    type_weight: 0.3
```

### 接口消息定义
```msg
# SelectedTarget.msg
std_msgs/Header header
string robot_id
float64 confidence
```

---

## 4. trajectory_planner (轨迹规划器包)

### 功能
- 基于选中的机器人和跟踪器数据,使用 MPC 规划云台运动轨迹
- 考虑云台运动学约束
- 预测未来一段时间的最优轨迹

### 输入
- 话题: `/target_selector/selected_target` (rm_interfaces::msg::SelectedTarget)
- 话题: `/robot_pose_estimator/robots` (rm_interfaces::msg::TrackedRobots)
- 话题: `/armor_tracker/tracked_armors` (rm_interfaces::msg::TrackedArmors)
- 话题: `/joint_states` (当前云台状态)

### 输出
- 话题: `/trajectory_planner/trajectory` (rm_interfaces::msg::GimbalTrajectory)
  - 包含未来 N 个时间步的 pitch/yaw 轨迹

### 核心类
```cpp
class MPCController {
  - int horizon_;                // 预测时域
  - Eigen::MatrixXd Q_, R_;      // 代价函数权重
  - void setupOptimizationProblem();
  - std::vector<GimbalState> solve(const RobotState& target);
};

class TrajectoryPlannerNode {
  - std::unique_ptr<MPCController> mpc_;
  - void planTrajectory(const SelectedTarget& target,
                       const TrackedRobots& robots,
                       const TrackedArmors& armors);
};
```

### 参数
```yaml
trajectory_planner:
  mpc:
    prediction_horizon: 10      # 预测步数
    control_horizon: 5
    dt: 0.01                    # 时间步长
    
    # 代价函数权重
    tracking_weight: 1.0
    control_effort_weight: 0.1
    smoothness_weight: 0.5
    
    # 约束
    max_pitch_velocity: 4.0     # rad/s
    max_yaw_velocity: 6.0
    max_pitch_acceleration: 10.0
    max_yaw_acceleration: 15.0
```

### 接口消息定义
```msg
# GimbalState.msg
float64 pitch
float64 yaw
float64 pitch_velocity
float64 yaw_velocity

# GimbalTrajectory.msg
std_msgs/Header header
GimbalState[] trajectory
float64 dt
```

---

## 5. ballistic_solver (弹道解算器包)

### 功能
- 给定目标3D位置,计算考虑重力和空气阻力的弹道
- 输出击打该位置所需的云台 pitch 和 yaw

### 输入
- 服务: `/ballistic_solver/solve` (rm_interfaces::srv::SolveBallistic)
  - 请求: 目标3D位置、子弹速度
  - 响应: pitch, yaw, 飞行时间

### 输出
- 提供服务接口

### 核心类
```cpp
class BallisticSolver {
  - double bullet_speed_;
  - double gravity_;
  - double air_resistance_;
  
  - bool solvePitchYaw(const Eigen::Vector3d& target_pos,
                      double& pitch, double& yaw,
                      double& flight_time);
  - Eigen::Vector3d predictTargetPosition(const Eigen::Vector3d& pos,
                                         const Eigen::Vector3d& vel,
                                         double flight_time);
};
```

### 参数
```yaml
ballistic_solver:
  bullet_speed: 28.0      # m/s
  gravity: 9.8            # m/s^2
  air_resistance: 0.001
  max_iterations: 50      # 迭代求解最大次数
  convergence_threshold: 0.001
```

### 接口服务定义
```srv
# SolveBallistic.srv
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity
float64 bullet_speed
---
float64 pitch
float64 yaw
float64 flight_time
bool success
```

---

## 6. gimbal_controller (云台控制器包)

### 功能
- 接收 MPC 规划的轨迹
- 执行轨迹跟踪控制
- 发布云台控制指令给底层硬件

### 输入
- 话题: `/trajectory_planner/trajectory` (rm_interfaces::msg::GimbalTrajectory)
- 话题: `/joint_states` (当前云台状态反馈)

### 输出
- 话题: `/cmd_gimbal` (rm_interfaces::msg::GimbalCmd)

### 核心类
```cpp
class GimbalController {
  - PIDController pitch_pid_;
  - PIDController yaw_pid_;
  - void trackTrajectory(const GimbalTrajectory& trajectory);
};
```

### 参数
```yaml
gimbal_controller:
  pitch_pid:
    kp: 10.0
    ki: 0.1
    kd: 1.0
  yaw_pid:
    kp: 8.0
    ki: 0.1
    kd: 0.8
  
  rate: 100  # Hz
```

---

## 数据流图

```
armor_detector/armors
         |
         v
  [armor_tracker]
         |
         v
  tracked_armors -----------------> [robot_pose_estimator]
         ^                                    |
         |                                    v
         |                            robots + virtual_armors
   virtual_armors                             |
         |                                    v
         +----------------------------> [target_selector]
                                              |
                                              v
                                      selected_target
                                              |
                                              v
                                   [trajectory_planner] <--- joint_states
                                              |
                                              v
                                    gimbal_trajectory
                                              |
                                              v
                                   [gimbal_controller] <--- joint_states
                                              |
                                              v
                                         cmd_gimbal
```

---

## 优势分析

### 1. **职责清晰**
- 每个包负责单一职责,符合单一职责原则
- 便于独立测试和调试

### 2. **可扩展性强**
- 可以轻松添加新的目标选择策略
- 可以替换不同的轨迹规划算法(PID -> MPC -> LQR)
- 支持多机器人跟踪

### 3. **可复用性高**
- `ballistic_solver` 可用于其他需要弹道解算的场景
- `armor_tracker` 可用于其他跟踪任务
- `gimbal_controller` 可用于其他云台控制场景

### 4. **易于维护**
- 模块间耦合度低,修改一个模块不影响其他模块
- 清晰的接口定义

### 5. **支持分布式部署**
- 每个包都是独立的 ROS2 节点
- 可以部署在不同的计算资源上

---

## 实施建议

### Phase 1: 基础架构搭建
1. 创建各个包的基础结构
2. 定义所有接口消息和服务
3. 实现 `armor_tracker` (基于原 Tracker 重构)

### Phase 2: 核心功能实现
4. 实现 `robot_pose_estimator`
5. 实现 `ballistic_solver`
6. 实现简单版 `target_selector` (最近目标策略)

### Phase 3: 高级功能
7. 实现 `trajectory_planner` (先用简单 PID,再升级 MPC)
8. 实现 `gimbal_controller`
9. 完善 `target_selector` (多策略支持)

### Phase 4: 集成测试
10. 端到端测试
11. 性能优化
12. 与原 `armor_solver` 对比验证

---

## 与原 armor_solver 的映射关系

| 原模块 | 新包 |
|--------|------|
| Tracker | armor_tracker |
| ArmorFliter | robot_pose_estimator (部分) + target_selector (部分) |
| Solver::getArmorPositions | robot_pose_estimator |
| Solver::selectBestArmor | target_selector + trajectory_planner |
| TrajectoryCompensator | ballistic_solver |
| Solver::solve (云台指令计算) | gimbal_controller |
| EKF | armor_tracker (复用) |

---

## 兼容性考虑

为了保证平滑过渡,建议:
1. **保持原 `armor_solver` 不动**,新包独立开发
2. **提供适配层**: 创建一个 `armor_solver_bridge` 节点,兼容原有接口
3. **支持配置切换**: 通过 launch 文件选择使用新架构或旧架构

```python
# multi_camera_system.launch.py (新架构)
def generate_launch_description():
    return LaunchDescription([
        Node(package='armor_tracker', ...),
        Node(package='robot_pose_estimator', ...),
        Node(package='target_selector', ...),
        Node(package='trajectory_planner', ...),
        Node(package='ballistic_solver', ...),
        Node(package='gimbal_controller', ...),
    ])
```

---

## 下一步行动

1. **Review 此设计文档**,确认架构合理性
2. **创建包结构**
3. **定义消息接口** (在 `rm_interfaces` 中)
4. **实现 armor_tracker** (优先级最高)

有任何问题或需要调整的地方,请随时反馈!
