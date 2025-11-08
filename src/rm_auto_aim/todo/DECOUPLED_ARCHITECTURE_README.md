# RoboMaster Auto Aim - 解耦架构实现进度

## 架构概览

```
装甲板检测 → 装甲板跟踪 → 机器人姿态估计 → 目标选择 → 轨迹规划 → 云台控制
                  ↑              ↓
                  └─── 虚拟装甲板 ───┘
```

## 包状态

| 包名 | 状态 | 进度 | 说明 |
|------|------|------|------|
| `rm_interfaces` | ✅ 完成 | 100% | 新增7个消息类型,1个服务类型 |
| `armor_tracker` | ✅ 完成 | 100% | 多装甲板跟踪器,基于EKF |
| `robot_pose_estimator` | ⚠️ 待实现 | 0% | 机器人姿态估计 |
| `target_selector` | ⚠️ 待实现 | 0% | 目标选择器 |
| `trajectory_planner` | ⚠️ 待实现 | 0% | MPC轨迹规划 |
| `ballistic_solver` | ⚠️ 待实现 | 0% | 弹道解算 |
| `gimbal_controller` | ⚠️ 待实现 | 0% | 云台控制器 |
| `armor_solver` | 🔒 保留 | - | 原包保持不变 |

## 新增消息类型

### 1. TrackedArmor.msg
单个被跟踪装甲板的状态信息
- 位置、速度、yaw角
- 跟踪状态: LOST/DETECTING/TRACKING/TEMP_LOST
- 置信度

### 2. TrackedArmors.msg
所有被跟踪装甲板的列表

### 3. TrackedRobot.msg
单个被跟踪机器人的状态信息
- 机器人类型: BALANCE_2/STANDARD_4/HERO_4/OUTPOST_3等
- 中心位置、速度、yaw角
- 旋转半径、装甲板数量

### 4. TrackedRobots.msg
所有被跟踪机器人的列表

### 5. SelectedTarget.msg
选中的击打目标
- 机器人ID
- 选择策略

### 6. GimbalState.msg
云台状态 (pitch/yaw及其速度、加速度)

### 7. GimbalTrajectory.msg
云台运动轨迹 (MPC规划结果)

### 8. SolveBallistic.srv (服务)
弹道解算服务
- 请求: 目标位置、速度、子弹速度
- 响应: pitch、yaw、飞行时间

## 编译和测试

### 编译

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws

# 编译新的消息类型
colcon build --packages-select rm_interfaces

# 编译 armor_tracker
colcon build --packages-select armor_tracker

# 编译所有新包
colcon build --packages-select rm_interfaces armor_tracker robot_pose_estimator target_selector trajectory_planner ballistic_solver gimbal_controller

# 刷新环境
source install/setup.bash
```

### 测试 armor_tracker

```bash
# 启动 armor_tracker
ros2 launch armor_tracker armor_tracker.launch.py

# 查看话题
ros2 topic list | grep armor_tracker

# 查看跟踪结果
ros2 topic echo /armor_tracker/tracked_armors

# RViz可视化
rviz2
# 添加 MarkerArray, Topic: /armor_tracker/markers
```

## 下一步开发计划

### Phase 1: 基础功能 (已完成)
- [x] 定义消息接口
- [x] 实现 armor_tracker 包
- [x] 创建其他包的基础结构

### Phase 2: 核心功能
- [ ] 实现 robot_pose_estimator
  - [ ] 装甲板分组逻辑
  - [ ] 机器人类型识别
  - [ ] 中心位置估计
  - [ ] 虚拟装甲板生成
- [ ] 实现 ballistic_solver (复用 TrajectoryCompensator)
  - [ ] 弹道方程求解
  - [ ] 重力和空气阻力补偿
- [ ] 实现简单版 target_selector
  - [ ] 最近目标策略
  - [ ] 优先级策略

### Phase 3: 高级功能
- [ ] 实现 trajectory_planner
  - [ ] PID控制器 (先实现简单版本)
  - [ ] MPC控制器 (后续升级)
- [ ] 实现 gimbal_controller
  - [ ] 轨迹跟踪
  - [ ] PID调节
- [ ] 完善 target_selector
  - [ ] 威胁度评估
  - [ ] 多策略支持

### Phase 4: 集成测试
- [ ] 端到端测试
- [ ] 性能对比 (新架构 vs 原 armor_solver)
- [ ] 参数调优
- [ ] 文档完善

## 数据流图

```
/armor_detector/armors (Armors)
         |
         v
  [armor_tracker]
         |
         v
  /armor_tracker/tracked_armors (TrackedArmors)
         |
         v
  [robot_pose_estimator]
         |
         +---> /robot_pose_estimator/robots (TrackedRobots)
         |
         +---> /robot_pose_estimator/virtual_armors (Armors) --+
                                                                 |
                                                                 v
                                                          [armor_tracker]
                                                                 |
                                                                 v
  [target_selector] <--- /robot_pose_estimator/robots
         |
         v
  /target_selector/selected_target (SelectedTarget)
         |
         v
  [trajectory_planner] <--- /armor_tracker/tracked_armors
         |                  /joint_states
         v
  /trajectory_planner/trajectory (GimbalTrajectory)
         |
         v
  [gimbal_controller] <--- /joint_states
         |
         v
  /cmd_gimbal (GimbalCmd)
```

## 与原 armor_solver 的对比

| 特性 | 原 armor_solver | 新架构 |
|------|----------------|--------|
| 多目标跟踪 | ❌ 单目标 | ✅ 多目标 |
| 模块化 | ❌ 耦合紧密 | ✅ 高度解耦 |
| 可扩展性 | ⚠️ 有限 | ✅ 易于扩展 |
| 可复用性 | ❌ 低 | ✅ 高 |
| 测试难度 | ⚠️ 困难 | ✅ 容易 |
| MPC支持 | ❌ 无 | ✅ 有 (规划中) |
| 机器人级跟踪 | ⚠️ 隐式 | ✅ 显式 |

## 兼容性说明

- ✅ 保持原 `armor_solver` 包不变
- ✅ 新包独立开发,互不影响
- ✅ 可通过 launch 文件切换新旧架构
- ✅ 消息接口向后兼容

## 开发者指南

### 添加新功能

1. 在对应包的 `include/` 下添加头文件
2. 在 `src/` 下添加实现文件
3. 更新 `CMakeLists.txt`
4. 更新参数文件 `config/`
5. 更新 launch 文件
6. 更新 README

### 调试技巧

```bash
# 查看所有话题
ros2 topic list

# 监听特定话题
ros2 topic echo /armor_tracker/tracked_armors

# 查看节点信息
ros2 node info /armor_tracker

# 查看参数
ros2 param list /armor_tracker
ros2 param get /armor_tracker max_match_distance

# 可视化TF树
ros2 run tf2_tools view_frames
evince frames.pdf
```

## 参考资料

- [设计文档](./DECOUPLING_DESIGN.md)
- [armor_tracker README](./src/rm_auto_aim/armor_tracker/README.md)
- [原 armor_solver README](./src/rm_auto_aim/armor_solver/README.md)

## 贡献者

- Initial design: 2025-11-08

## License

Apache-2.0
