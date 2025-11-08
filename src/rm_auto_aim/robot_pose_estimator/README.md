# Robot Pose Estimator

## 概述

机器人姿态估计器,将相同ID的装甲板绑定到同一机器人,估计机器人中心位置和姿态。

## 计划功能

- [ ] 装甲板分组: 将同ID装甲板绑定到机器人实例
- [ ] 机器人类型识别: Balance/Standard/Hero/Outpost/Sentry
- [ ] URDF结构估计: 估计旋转半径和装甲板配置
- [ ] 虚拟装甲板生成: 基于机器人姿态生成被遮挡装甲板的估计位置
- [ ] 机器人中心跟踪: 维护机器人中心位置和速度

## 输入

- `/armor_tracker/tracked_armors` (rm_interfaces/msg/TrackedArmors)

## 输出

- `/robot_pose_estimator/robots` (rm_interfaces/msg/TrackedRobots)
- `/robot_pose_estimator/virtual_armors` (rm_interfaces/msg/Armors)

## 状态

⚠️ **待实现** - 当前仅创建了包结构,功能尚未实现
