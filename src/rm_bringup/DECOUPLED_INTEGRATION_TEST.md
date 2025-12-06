# 解耦架构集成测试指南

## 架构概览

```
装甲板检测 → armor_tracker → robot_pose_estimator → target_selector → trajectory_planner
(armor_detector)     ↑              ↓                                       ↓
                     └─── virtual_armors ───┘                      ballistic_solver (服务)
```

### 各模块职责

| 模块 | 职责 |
|------|------|
| `armor_detector` | 检测装甲板，输出 3D 位置和姿态 |
| `armor_tracker` | 多装甲板跟踪，维护 EKF 状态估计 |
| `robot_pose_estimator` | 机器人级别姿态估计，生成虚拟装甲板 |
| `target_selector` | 目标选择策略，选择击打目标 |
| `trajectory_planner` | MPC 轨迹规划，输出云台控制指令 |
| `ballistic_solver` | 弹道解算服务，计算 pitch/yaw 角 |

### 话题数据流

```
/armor_detector/armors (Armors)
        │
        ▼
/armor_tracker/tracked_armors (TrackedArmors)
        │
        ▼
/robot_pose_estimator/robots (TrackedRobots)
/robot_pose_estimator/virtual_armors (Armors) ─────────┐
        │                                              │
        ▼                                              │
/target_selector/selected_target (SelectedTarget)      │
        │                                              │
        ▼                                              │
/trajectory_planner/gimbal_cmd (GimbalCmd)             │
        │                                              │
        └──────────────────────────────────────────────┘
            (virtual_armors 反馈给 armor_tracker)
```

## 编译

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws

# 编译所有相关包
colcon build --packages-select \
  rm_interfaces \
  rm_utils \
  armor_detector \
  armor_tracker \
  robot_pose_estimator \
  target_selector \
  trajectory_planner \
  ballistic_solver \
  rm_bringup

# 刷新环境
source install/setup.bash
```

## 启动

### 方式一：完整集成测试 (推荐)

```bash
# 使用默认参数启动
ros2 launch rm_bringup bringup_decoupled.launch.py

# 使用视频播放和虚拟串口
ros2 launch rm_bringup bringup_decoupled.launch.py video_play:=true virtual_serial:=true

# 开启调试模式
ros2 launch rm_bringup bringup_decoupled.launch.py debug:=true
```

### 方式二：分步启动 (调试用)

```bash
# Terminal 1: 串口
ros2 run rm_serial_driver virtual_serial_node

# Terminal 2: 相机/视频 + 检测器
ros2 launch rm_bringup test_video_player.launch.py

# Terminal 3: 弹道解算服务
ros2 launch ballistic_solver ballistic_solver.launch.py

# Terminal 4: 装甲板跟踪器
ros2 launch armor_tracker armor_tracker.launch.py

# Terminal 5: 机器人姿态估计器
ros2 launch robot_pose_estimator robot_pose_estimator.launch.py debug:=true

# Terminal 6: 目标选择器
ros2 launch target_selector target_selector.launch.py debug:=true

# Terminal 7: 轨迹规划器
ros2 launch trajectory_planner trajectory_planner.launch.py debug:=true
```

## 监控和调试

### 查看话题

```bash
# 查看所有相关话题
ros2 topic list | grep -E "armor|robot|target|trajectory|ballistic"

# 查看跟踪结果
ros2 topic echo /armor_tracker/tracked_armors

# 查看机器人状态
ros2 topic echo /robot_pose_estimator/robots

# 查看选中的目标
ros2 topic echo /target_selector/selected_target

# 查看云台控制指令
ros2 topic echo /trajectory_planner/gimbal_cmd
```

### RViz 可视化

```bash
rviz2
```

添加以下显示项：
1. `MarkerArray`: `/armor_tracker/markers`
2. `MarkerArray`: `/robot_pose_estimator/markers`
3. `MarkerArray`: `/target_selector/markers`
4. `MarkerArray`: `/trajectory_planner/markers`

### 服务调用测试

```bash
# 测试弹道解算服务
ros2 service call /ballistic_solver/solve rm_interfaces/srv/SolveBallistic \
  "{target_position: {x: 5.0, y: 0.0, z: 1.0}, target_velocity: {x: 0.0, y: 0.0, z: 0.0}, bullet_speed: 28.0}"
```

### 动态参数调整

```bash
# 查看节点参数
ros2 param list /armor_tracker
ros2 param list /robot_pose_estimator
ros2 param list /target_selector
ros2 param list /trajectory_planner

# 修改参数
ros2 param set /armor_tracker max_match_distance 0.3
ros2 param set /target_selector debug true
```

## 常见问题

### Q1: 节点找不到消息类型

确保先编译 `rm_interfaces` 并刷新环境：
```bash
colcon build --packages-select rm_interfaces
source install/setup.bash
```

### Q2: 某个节点启动失败

检查该节点的依赖是否已正确编译：
```bash
colcon build --packages-select <package_name> --symlink-install
```

### Q3: 话题没有数据

1. 检查上游节点是否正常运行
2. 使用 `ros2 topic info <topic>` 查看发布者和订阅者
3. 使用 `ros2 node info <node>` 查看节点的话题连接

### Q4: 与原 armor_solver 对比测试

使用原架构：
```bash
ros2 launch rm_bringup bringup.launch.py
```

使用解耦架构：
```bash
ros2 launch rm_bringup bringup_decoupled.launch.py
```

## 配置文件

| 文件 | 说明 |
|------|------|
| `config/launch_params_decoupled.yaml` | 解耦架构启动参数 |
| `config/node_params/*.yaml` | 各节点参数配置 |

各包自带的配置文件：
- `armor_tracker/config/tracker_params.yaml`
- `robot_pose_estimator/config/robot_pose_estimator.yaml`
- `target_selector/config/target_selector.yaml`
- `trajectory_planner/config/trajectory_planner.yaml`
- `ballistic_solver/config/ballistic_solver.yaml`
