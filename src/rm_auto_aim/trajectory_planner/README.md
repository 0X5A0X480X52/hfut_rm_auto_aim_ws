# Trajectory Planner

## 概述

基于MPC的云台轨迹规划器。

## 计划功能

- [ ] MPC控制器实现
- [ ] 预测时域优化
- [ ] 云台运动学约束
- [ ] 代价函数设计
  - [ ] 跟踪误差
  - [ ] 控制能量
  - [ ] 平滑性
- [ ] 实时优化求解

## 输入

- `/target_selector/selected_target` (rm_interfaces/msg/SelectedTarget)
- `/robot_pose_estimator/robots` (rm_interfaces/msg/TrackedRobots)
- `/armor_tracker/tracked_armors` (rm_interfaces/msg/TrackedArmors)
- `/joint_states` (当前云台状态)

## 输出

- `/trajectory_planner/trajectory` (rm_interfaces/msg/GimbalTrajectory)

## 状态

⚠️ **待实现** - 当前仅创建了包结构,功能尚未实现
