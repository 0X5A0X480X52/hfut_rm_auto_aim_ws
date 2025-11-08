# Target Selector

## 概述

目标选择器,基于策略从机器人列表中选择击打目标。

## 计划功能

- [ ] 多种选择策略
  - [ ] NEAREST: 最近目标
  - [ ] HIGHEST_THREAT: 威胁度最高
  - [ ] PRIORITY_BASED: 基于优先级
  - [ ] LOWEST_HP: 血量最低(需要裁判系统)
- [ ] 威胁度评估
- [ ] 优先级配置
- [ ] 目标切换逻辑

## 输入

- `/robot_pose_estimator/robots` (rm_interfaces/msg/TrackedRobots)
- `/game_status` (可选,用于优先级决策)

## 输出

- `/target_selector/selected_target` (rm_interfaces/msg/SelectedTarget)

## 状态

⚠️ **待实现** - 当前仅创建了包结构,功能尚未实现
