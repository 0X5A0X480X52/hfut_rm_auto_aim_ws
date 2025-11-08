# Gimbal Controller

## 概述

云台控制器,执行轨迹跟踪控制。

## 计划功能

- [ ] PID控制器实现
- [ ] 轨迹跟踪
- [ ] 前馈控制
- [ ] 控制指令输出

## 输入

- `/trajectory_planner/trajectory` (rm_interfaces/msg/GimbalTrajectory)
- `/joint_states` (sensor_msgs/msg/JointState)

## 输出

- `/cmd_gimbal` (rm_interfaces/msg/GimbalCmd)

## 参数

- `pitch_pid.kp/ki/kd`: Pitch轴PID参数
- `yaw_pid.kp/ki/kd`: Yaw轴PID参数
- `rate`: 控制频率 (Hz)

## 状态

⚠️ **待实现** - 当前仅创建了包结构,功能尚未实现
