# Trajectory Planner 测试指南

## 概述

`test_trajectory_planner_node.py` 是一个专门用于测试 trajectory_planner 包的ROS2节点。它模拟上游数据源（armor_tracker），生成测试话题数据，并验证 trajectory_planner 的行为。

## 功能特性

### 1. 模拟数据发布

- **TrackPredictionWindows** (`/tracker/prediction_windows`)
  - 模拟4个装甲板的预测轨迹
  - 整体匀加速运动 + 相对旋转
  - 包含18步预测（可配置）
  - 添加观测噪声模拟真实环境

- **JointState** (`/joint_states`)
  - 模拟云台当前状态反馈
  - 简单的目标跟踪模拟

- **TrackedRobots** (`/tracker/robots`, 可选)
  - 模拟机器人中心运动信息
  - 4装甲板标准机器人类型

### 2. 性能监控

- 订阅 GimbalCmd (`/trajectory_planner/gimbal_cmd`)
- 统计控制指令频率
- 计算跟踪误差
- 定期打印性能报告（每5秒）

### 3. 运动模型（参考test05_qp_mpc.py）

- **装甲板运动**:
  - 相对旋转: 基础角速度 2.0 rad/s (约114°/s)
  - 整体运动: 匀加速度 0.1 rad/s² (约5.7°/s²)
  - 初始中心角速度: 0.5 rad/s
  - 距离: 2.5m（固定）

- **观测噪声**:
  - 标准差: 2° (0.035 rad)
  - 仅在当前时刻添加噪声

## 使用方法

### 方式1: 使用Launch文件（推荐）

```bash
# 编译（如果首次使用）
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select trajectory_planner --symlink-install

# 刷新环境
source install/setup.bash

# 启动测试（同时启动测试节点和trajectory_planner）
ros2 launch trajectory_planner test_trajectory_planner.launch.py

# 可选：启用审计日志
ros2 launch trajectory_planner test_trajectory_planner.launch.py enable_audit:=true
```

### 方式2: 手动启动

**终端1 - 启动测试节点**:
```bash
source install/setup.bash
ros2 run trajectory_planner test_trajectory_planner_node.py
```

**终端2 - 启动trajectory_planner**:
```bash
source install/setup.bash
ros2 run trajectory_planner trajectory_planner_node
```

### 方式3: 自定义参数

```bash
ros2 run trajectory_planner test_trajectory_planner_node.py \
  --ros-args \
  -p dt:=0.01 \
  -p num_plates:=4 \
  -p base_omega:=3.0 \
  -p center_alpha:=0.2 \
  -p prediction_steps:=20 \
  -p measurement_noise_std:=0.05 \
  -p enable_robots_pub:=true
```

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dt` | float | 0.02 | 发布周期(秒)，对应50Hz |
| `num_plates` | int | 4 | 装甲板数量 |
| `base_omega` | float | 2.0 | 基础旋转角速度(rad/s) |
| `center_alpha` | float | 0.1 | 中心角加速度(rad/s²) |
| `prediction_steps` | int | 18 | 预测步数 |
| `measurement_noise_std` | float | 0.035 | 观测噪声标准差(rad) |
| `enable_robots_pub` | bool | true | 是否发布机器人信息 |

## 监控与调试

### 1. 查看话题

```bash
# 查看所有话题
ros2 topic list

# 监控预测窗口发布
ros2 topic hz /tracker/prediction_windows
ros2 topic echo /tracker/prediction_windows --no-arr

# 监控控制指令输出
ros2 topic hz /trajectory_planner/gimbal_cmd
ros2 topic echo /trajectory_planner/gimbal_cmd
```

### 2. 检查节点状态

```bash
# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /trajectory_planner_tester
ros2 node info /trajectory_planner
```

### 3. 实时日志

测试节点每5秒自动打印统计信息：
```
======================================================================
运行时间: 25.3s | 迭代: 1265
装甲板状态:
  中心角度: 45.23° | 中心角速度: 14.56°/s
Trajectory Planner 性能:
  收到指令数: 1245
  平均频率: 49.8 Hz
  平均跟踪误差: 2.34°
======================================================================
```

### 4. 可视化（使用rqt）

```bash
# 安装rqt（如果未安装）
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# 启动rqt
rqt

# 在rqt中：
# - Plugins -> Topics -> Topic Monitor (查看话题列表)
# - Plugins -> Visualization -> Plot (绘制实时曲线)
#   - 添加 /trajectory_planner/gimbal_cmd/yaw
#   - 添加 /trajectory_planner/gimbal_cmd/pitch
```

## 验证检查清单

### ✅ 基本功能

- [ ] 测试节点成功启动并发布话题
- [ ] trajectory_planner接收到prediction_windows
- [ ] trajectory_planner接收到joint_states
- [ ] trajectory_planner发布gimbal_cmd指令
- [ ] 控制指令频率 > 30 Hz

### ✅ 跟踪性能

- [ ] 平均跟踪误差 < 5°
- [ ] 云台能跟随装甲板中心运动
- [ ] 目标切换正常（4个装甲板之间）

### ✅ 边界约束

- [ ] 云台角度在配置的范围内（默认±30°）
- [ ] 无异常错误或警告日志
- [ ] 角速度、角加速度受限

### ✅ 实时性能

- [ ] 平均求解时间 < 20ms（50Hz）
- [ ] 无明显延迟或卡顿
- [ ] CPU使用率合理

## 故障排查

### 问题1: 测试节点启动但trajectory_planner无响应

**可能原因**:
- trajectory_planner未启动
- 话题名称不匹配
- QoS配置不兼容

**解决方法**:
```bash
# 检查话题连接
ros2 topic info /tracker/prediction_windows
ros2 topic info /trajectory_planner/gimbal_cmd

# 检查节点日志
ros2 node list
```

### 问题2: 收到指令但频率很低

**可能原因**:
- trajectory_planner参数配置问题
- 系统负载过高
- 缺少必要的依赖

**解决方法**:
```bash
# 查看trajectory_planner日志
ros2 run trajectory_planner trajectory_planner_node --ros-args --log-level debug

# 检查系统负载
top
```

### 问题3: 跟踪误差很大

**可能原因**:
- MPC参数未调优
- 预测模型不匹配
- 观测噪声过大

**解决方法**:
- 调整测试节点的 `measurement_noise_std` 参数
- 检查 trajectory_planner.yaml 中的MPC权重配置
- 增加预测步数 `prediction_steps`

## 进阶测试

### 1. 压力测试

```bash
# 增加装甲板旋转速度
ros2 run trajectory_planner test_trajectory_planner_node.py \
  --ros-args -p base_omega:=5.0

# 增加中心加速度
ros2 run trajectory_planner test_trajectory_planner_node.py \
  --ros-args -p center_alpha:=0.5
```

### 2. 噪声鲁棒性测试

```bash
# 增大观测噪声（5度）
ros2 run trajectory_planner test_trajectory_planner_node.py \
  --ros-args -p measurement_noise_std:=0.087

# 极端噪声（10度）
ros2 run trajectory_planner test_trajectory_planner_node.py \
  --ros-args -p measurement_noise_std:=0.175
```

### 3. 延迟测试

可以在测试节点中添加人工延迟来测试trajectory_planner的时间鲁棒性。

### 4. 审计模式验证

```bash
# 启用审计日志
# 在 trajectory_planner.yaml 中设置 audit_mode: true

# 运行测试
ros2 launch trajectory_planner test_trajectory_planner.launch.py

# 测试结束后可视化
python3 src/rm_auto_aim/trajectory_planner/scripts/visualize_audit_v2.py /tmp/mpc_audit.csv
```

## 对比参考

test05_qp_mpc.py (离线仿真) vs test_trajectory_planner_node.py (ROS2测试):

| 特性 | test05_qp_mpc.py | test_trajectory_planner_node.py |
|------|------------------|----------------------------------|
| 运行环境 | 独立Python脚本 | ROS2节点 |
| 数据源 | 内部模拟 | ROS2话题发布 |
| 控制器 | 内置QP-MPC | 外部trajectory_planner节点 |
| 可视化 | matplotlib离线图表 | rqt实时监控 |
| 用途 | 算法验证 | 系统集成测试 |

## 常见用例

### 用例1: 快速验证功能

```bash
ros2 launch trajectory_planner test_trajectory_planner.launch.py
# 观察5-10秒，检查日志输出正常即可
```

### 用例2: 长时间稳定性测试

```bash
ros2 launch trajectory_planner test_trajectory_planner.launch.py
# 运行30分钟以上，监控内存泄漏和性能衰减
```

### 用例3: 性能基准测试

```bash
# 启用审计日志记录详细性能数据
# 修改 config/trajectory_planner.yaml: audit_mode: true
ros2 launch trajectory_planner test_trajectory_planner.launch.py

# 运行5分钟
# Ctrl+C 停止

# 分析性能
python3 src/rm_auto_aim/trajectory_planner/scripts/visualize_audit_v2.py /tmp/mpc_audit.csv
```

## 扩展开发

如需修改测试场景，可以编辑 `test_trajectory_planner_node.py`：

- `_update_plate_states()`: 修改装甲板运动模型
- `_predict_plate_state()`: 调整预测逻辑和噪声
- `_gimbal_cmd_callback()`: 添加自定义验证逻辑

## 相关文件

- 测试节点: `scripts/test_trajectory_planner_node.py`
- 启动文件: `launch/test_trajectory_planner.launch.py`
- 配置文件: `config/trajectory_planner.yaml`
- 参考实现: `demo/test05_qp_mpc.py`
- 审计可视化: `scripts/visualize_audit_v2.py`

## 支持

如有问题，请检查：
1. ROS2版本是否为Humble
2. 所有依赖包是否已安装
3. workspace是否正确编译
4. 环境变量是否正确source
