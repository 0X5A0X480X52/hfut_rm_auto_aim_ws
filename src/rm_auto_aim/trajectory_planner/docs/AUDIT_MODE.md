# MPC审计模式使用指南

## 概述

MPC审计模式为trajectory_planner提供了完整的输入输出日志记录和可视化功能，用于分析和调试MPC控制器的行为。

## 功能特性

### 1. CSV日志记录
- **每帧完整数据**：记录MPC每帧的输入、输出、内部状态
- **缓冲写入**：100帧批量写入，避免影响实时性能
- **统计摘要**：自动生成求解成功率、平均耗时等统计信息

### 2. 交互式可视化
- **2D鸟瞰图**：显示机器人、目标位置、预测轨迹
- **Yaw-时间曲线**：对比目标yaw vs 实际yaw vs 预测yaw
- **控制序列图**：显示MPC输出的jerk控制序列
- **误差历史**：跟踪yaw/pitch误差随时间变化
- **求解时间监控**：分析优化器性能
- **时间滑动条**：逐帧查看MPC状态

## 使用方法

### 启用审计模式

编辑配置文件 `trajectory_planner.yaml`：

```yaml
trajectory_planner:
  ros__parameters:
    # 启用审计模式
    audit_mode: true
    audit_log_path: "/tmp/mpc_audit.csv"
```

### 运行系统

```bash
cd ~/hfut_rm_auto_aim_ws
source install/setup.bash

# 启动trajectory planner
ros2 run trajectory_planner trajectory_planner_node \
  --ros-args --params-file src/rm_auto_aim/trajectory_planner/config/trajectory_planner.yaml
```

**注意**：审计模式会记录大量数据，建议运行1-2分钟后停止（Ctrl+C），避免文件过大。

### 可视化审计日志

```bash
cd ~/hfut_rm_auto_aim_ws
python3 src/rm_auto_aim/trajectory_planner/scripts/visualize_audit.py /tmp/mpc_audit.csv
```

## 日志文件格式

CSV文件包含以下列（共155列）：

### 基础信息
- `timestamp`: 绝对时间戳
- `frame_id`: 帧序号
- `relative_time`: 相对时间（秒）

### 当前状态（3列）
- `current_theta`: 当前yaw角度
- `current_omega`: 当前yaw角速度
- `current_alpha`: 当前yaw角加速度

### 目标轨迹（18列）
- `target_theta_0` ~ `target_theta_17`: 目标yaw轨迹（N步）

### MPC输出（19列）
- `u_optimal`: 即时控制命令（jerk）
- `U_seq_0` ~ `U_seq_17`: 完整控制序列（N步）

### 预测轨迹（57列）
- `pred_theta_0` ~ `pred_theta_18`: 预测yaw（N+1步）
- `pred_omega_0` ~ `pred_omega_18`: 预测角速度（N+1步）
- `pred_alpha_0` ~ `pred_alpha_18`: 预测角加速度（N+1步）

### 求解信息（2列）
- `solve_time_ms`: 求解耗时（毫秒）
- `solver_status`: 求解状态（'solved', 'solved_inaccurate', 'failed'）

### 目标信息（7列）
- `target_track_id`: 目标跟踪ID
- `target_x`, `target_y`, `target_z`: 目标3D位置
- `target_distance`: 目标距离
- `target_confidence`: 目标置信度
- `target_yaw_from_origin`: 目标方位角

### 开火判断（3列）
- `yaw_error`: yaw误差
- `pitch_error`: pitch误差
- `fire_advice`: 开火建议（0/1）

## 统计摘要文件

日志关闭时自动生成 `*_summary.txt`，包含：
- 总帧数
- 求解成功/失败次数
- 平均/最大/最小求解时间
- 记录时长

## 性能考虑

- **缓冲写入**：每100帧批量写入一次，避免频繁I/O
- **控制频率**：100Hz控制循环不受日志记录影响
- **文件大小**：1分钟约6000帧，文件大小约10-15MB
- **建议时长**：建议记录1-2分钟用于分析，避免文件过大

## 应用场景

1. **调参优化**：观察不同权重参数对轨迹的影响
2. **性能分析**：检查求解时间是否满足实时要求（<10ms）
3. **跟踪精度**：分析yaw误差分布，评估控制精度
4. **故障诊断**：定位求解失败或异常行为的原因
5. **算法改进**：验证新的约束或目标函数的效果

## 示例分析流程

1. **启用审计模式**运行系统1分钟
2. **停止系统**保存日志
3. **查看统计摘要**了解整体性能
4. **可视化分析**：
   - 检查预测轨迹是否平滑
   - 观察控制输入是否合理
   - 分析误差峰值出现时刻
   - 对比目标与实际轨迹差异
5. **调整参数**并重复测试

## 故障排查

### 日志文件未生成
- 检查 `audit_mode: true` 是否生效
- 检查日志路径是否有写权限
- 查看ROS日志确认审计模式已启用

### 可视化脚本报错
- 确认安装了matplotlib：`pip3 install matplotlib pandas`
- 检查CSV文件是否完整（文件末尾可能未flush）

### 求解时间过长
- 观察solver_status列，检查是否频繁失败
- 可能需要调整MPC参数（降低prediction_horizon）
- 检查是否有其他进程占用CPU

## 扩展建议

未来可以添加：
- HDF5格式支持（更高效存储大数据）
- 实时监控界面（ROS2 rqt插件）
- 自动性能基准测试
- 误差分布直方图
- 频谱分析（控制频率特性）


## 候选目标记录

审计模式现在会将每帧的所有候选目标点及其预测轨迹写入单独的CSV文件：

- 文件名: `<audit_log_path>_candidates.csv`（例如 `/tmp/mpc_audit_candidates.csv`）
- 每行包含: `frame_id`, `candidate_idx`, `track_id`, `target_x/y/z`, `distance`, `confidence`, followed by `target_theta_0..target_theta_{N-1}` and `pos_x_i,pos_y_i,pos_z_i` for `N` steps

该文件便于分析在目标选择阶段被过滤或错选的候选点与其预测轨迹。
