# 快速启动指南

## 编译

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws

# 编译所有新包
colcon build --packages-select \
  rm_interfaces \
  armor_tracker \
  robot_pose_estimator \
  target_selector \
  trajectory_planner \
  ballistic_solver \
  gimbal_controller

# 或者只编译已实现的包
colcon build --packages-select rm_interfaces armor_tracker

# 刷新环境
source install/setup.bash
```

## 测试 armor_tracker

### 方式1: 使用launch文件

```bash
# 启动armor_tracker
ros2 launch armor_tracker armor_tracker.launch.py
```

### 方式2: 直接运行节点

```bash
ros2 run armor_tracker armor_tracker_node_exe
```

### 查看输出

```bash
# 新开终端,source环境
source install/setup.bash

# 查看话题
ros2 topic list | grep armor

# 查看跟踪结果
ros2 topic echo /armor_tracker/tracked_armors

# 查看节点信息
ros2 node info /armor_tracker

# 查看参数
ros2 param list /armor_tracker
```

### RViz可视化

```bash
# 启动RViz
rviz2

# 手动添加显示项:
# 1. Fixed Frame 设置为 "odom"
# 2. Add -> MarkerArray
#    Topic: /armor_tracker/markers
```

## 与现有系统集成测试

### 完整测试流程

```bash
# Terminal 1: 启动相机和检测器
ros2 launch armor_detector armor_detector.launch.py

# Terminal 2: 启动armor_tracker (新架构)
source install/setup.bash
ros2 launch armor_tracker armor_tracker.launch.py

# Terminal 3: 监控输出
source install/setup.bash
ros2 topic echo /armor_tracker/tracked_armors

# Terminal 4: RViz可视化
rviz2
```

### 使用视频播放器测试

```bash
# Terminal 1: 启动视频播放器系统
ros2 launch armor_fusion multi_camera_system_with_videoplayer.launch.py

# Terminal 2: 启动armor_tracker
source install/setup.bash
ros2 launch armor_tracker armor_tracker.launch.py

# Terminal 3: 查看结果
source install/setup.bash
ros2 topic echo /armor_tracker/tracked_armors
```

## 调试

### 检查消息类型

```bash
# 查看新消息类型
ros2 interface show rm_interfaces/msg/TrackedArmor
ros2 interface show rm_interfaces/msg/TrackedArmors
ros2 interface show rm_interfaces/msg/TrackedRobot
ros2 interface show rm_interfaces/srv/SolveBallistic
```

### 动态参数调整

```bash
# 查看当前参数
ros2 param get /armor_tracker max_match_distance

# 修改参数
ros2 param set /armor_tracker max_match_distance 0.3
ros2 param set /armor_tracker tracking_threshold 3
ros2 param set /armor_tracker debug true
```

### 查看日志

```bash
# 实时查看日志
ros2 run armor_tracker armor_tracker_node_exe --ros-args --log-level debug

# 或者在launch中设置
# output='screen' emulate_tty=True
```

## 常见问题

### Q1: 编译失败,找不到消息类型
**A**: 确保先编译 rm_interfaces
```bash
colcon build --packages-select rm_interfaces
source install/setup.bash
colcon build --packages-select armor_tracker
```

### Q2: 没有看到可视化标记
**A**: 检查:
1. debug参数是否为true
2. RViz的Fixed Frame是否设置为"odom"
3. Topic是否正确: /armor_tracker/markers

### Q3: 没有跟踪输出
**A**: 检查:
1. armor_detector是否正常发布数据
2. 话题名称是否正确: /armor_detector/armors
3. 查看日志: ros2 topic echo /armor_tracker/tracked_armors

## 性能监控

```bash
# 查看话题频率
ros2 topic hz /armor_tracker/tracked_armors

# 查看话题带宽
ros2 topic bw /armor_tracker/tracked_armors

# 查看CPU和内存使用
htop
# 搜索 armor_tracker
```

## 下一步

1. 继续实现 robot_pose_estimator
2. 实现 ballistic_solver
3. 实现 target_selector
4. 端到端测试完整系统

## 文档

- [设计文档](DECOUPLING_DESIGN.md)
- [架构说明](DECOUPLED_ARCHITECTURE_README.md)
- [实施总结](IMPLEMENTATION_SUMMARY.md)
- [armor_tracker文档](src/rm_auto_aim/armor_tracker/README.md)
