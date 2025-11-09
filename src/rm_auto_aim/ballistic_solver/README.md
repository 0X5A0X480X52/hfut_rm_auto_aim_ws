# Ballistic Solver

## 概述

弹道解算器,计算考虑重力和空气阻力的弹道轨迹。

## 计划功能

- [ ] 弹道方程求解
- [ ] 重力补偿
- [ ] 空气阻力模型
- [ ] 迭代求解算法
- [ ] 目标运动预测

## 服务接口

- `/ballistic_solver/solve` (rm_interfaces/srv/SolveBallistic)
  - 请求: 目标位置、目标速度、子弹速度
  - 响应: pitch、yaw、飞行时间

## 参数

- `bullet_speed`: 子弹速度 (m/s)
- `gravity`: 重力加速度 (m/s²)
- `air_resistance`: 空气阻力系数
- `max_iterations`: 迭代求解最大次数
- `convergence_threshold`: 收敛阈值

## 状态

⚠️ **待实现** - 当前仅创建了包结构,功能尚未实现

## 备注

可以复用 `armor_solver` 中的 `TrajectoryCompensator` 相关代码
