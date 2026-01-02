# MPC审计模式使用说明

## 概述

MPC审计模式允许记录trajectory_planner节点的MPC控制器每帧的输入、输出和内部状态数据，用于离线分析、调试和性能优化。

## 功能特性

### 1. 数据记录

审计模式记录两个CSV文件：

#### 主日志文件 (`mpc_audit.csv`)

记录每帧的MPC运行数据，包括：

- **时间信息**: timestamp, frame_id, relative_time
- **云台状态**: current_theta, current_omega, current_alpha
- **目标轨迹**: target_theta_0 ~ target_theta_17 (N=18步)
- **MPC输出**: u_optimal (最优控制), U_seq_0 ~ U_seq_17 (控制序列)
- **预测轨迹**: pred_theta/omega/alpha_0 ~ _18 (N+1=19步状态预测)
- **求解信息**: solve_time_ms, solver_status
- **目标信息**: target_track_id, target_x/y/z, target_distance, target_confidence, target_yaw_from_origin

**坐标系与转换**: 所有记录的目标位置/方位角默认使用上游预测器提供的原始坐标（通常为相机或机器人原点坐标），审计器会自动将关键字段转换并额外保存为**云台坐标系**（以当前云台yaw为参考）：`target_x_gimbal`, `target_y_gimbal`, `target_z_gimbal`, `target_yaw_gimbal`。候选目标也包含等效的`pos_x_gimbal_*/pos_y_gimbal_*/pos_z_gimbal_*`和`target_theta_gimbal_*`列，且每条记录包含所使用的 `gimbal_yaw` 以便可追溯。

- **开火判断**: yaw_error, pitch_error, fire_advice

#### 候选目标日志文件 (`mpc_audit_candidates.csv`)

记录每帧所有候选目标的信息，包括：

- **基本信息**: frame_id, candidate_idx, track_id, armor_id, armor_type
- **位置信息**: target_x/y/z, distance, confidence
- **遮挡过滤**: armor_yaw, view_yaw, facing_diff, is_occluded, filter_reason
- **预测轨迹**: target_theta_0 ~ _17 (yaw轨迹), pos_x/y/z_0 ~_17 (位置轨迹)

### 2. 可视化工具

提供两个可视化脚本：

#### visualize_audit.py (基础版)

- 5个子图：鸟瞰图、yaw-time曲线、控制序列、误差历史、求解时间
- 交互式slider切换帧
- 显示选中目标和候选目标位置

#### visualize_audit_v2.py (增强版)

**新增功能**：

- **彩色目标区分**: 每个track_id分配唯一颜色
- **预测轨迹显示**: 显示每个候选目标的未来位置预测
- **姿态可视化**: 用箭头显示目标yaw方向（armor_yaw或view_yaw）
- **过滤标签**: 显示遮挡过滤条件、置信度、距离、filter_reason
- **MPC预测可视化**: 显示MPC预测的云台yaw终点方向
- **选中目标标记**: 用绿色星号标记当前选中目标

**鸟瞰图图例**：

- 红圆点: 机器人本体位置(0,0)
- 红箭头: 当前云台yaw方向
- 蓝虚线: MPC预测yaw最终方向
- 品红点划线: 目标yaw参考轨迹终点
- 绿星号: 当前选中目标
- 彩色圆点: 候选目标（每个track_id不同颜色）
- 彩色箭头: 目标姿态方向
- 彩色轨迹: 目标位置预测
- 文本框: 目标ID、置信度、距离、过滤状态

## 使用方法

### 1. 启用审计模式

编辑配置文件 `config/trajectory_planner.yaml`:

```yaml
trajectory_planner:
  ros__parameters:
    # ... 其他参数 ...
    
    # 审计模式配置
    audit_mode: true
    audit_log_path: "/tmp/mpc_audit.csv"  # 或指定其他路径
```

### 2. 运行节点

```bash
# 启动trajectory_planner节点
ros2 launch rm_bringup bringup.launch.py

# 或单独运行
ros2 run trajectory_planner trajectory_planner_node --ros-args --params-file config/trajectory_planner.yaml
```

节点运行时会自动在指定路径创建：

- `mpc_audit.csv` - 主日志
- `mpc_audit_candidates.csv` - 候选目标日志
- `mpc_audit_summary.txt` - 统计摘要

### 3. 停止记录

停止节点（Ctrl+C），日志会自动flush到磁盘。

### 4. 可视化分析

#### 使用基础版可视化

```bash
cd src/rm_auto_aim/trajectory_planner/scripts
python3 visualize_audit.py /tmp/mpc_audit.csv
```

#### 使用增强版可视化（推荐）

```bash
cd src/rm_auto_aim/trajectory_planner/scripts
python3 visualize_audit_v2.py /tmp/mpc_audit.csv
```

#### 测试可视化（使用模拟数据）

```bash
cd src/rm_auto_aim/trajectory_planner/scripts
python3 generate_test_audit.py
python3 visualize_audit_v2.py /tmp/mpc_audit_test.csv
```

### 5. 数据分析

可以使用pandas直接读取CSV进行自定义分析：

```python
import pandas as pd
import numpy as np

# 读取主日志
df = pd.read_csv('/tmp/mpc_audit.csv')

# 分析求解时间
print(f"平均求解时间: {df['solve_time_ms'].mean():.3f} ms")
print(f"最大求解时间: {df['solve_time_ms'].max():.3f} ms")

# 分析跟踪误差
print(f"平均yaw误差: {np.abs(df['yaw_error']).mean():.4f} rad")

# 求解成功率
success_rate = (df['solver_status'] == 'solved').sum() / len(df) * 100
print(f"求解成功率: {success_rate:.2f}%")

# 读取候选目标日志
df_cand = pd.read_csv('/tmp/mpc_audit_candidates.csv')

# 分析遮挡过滤
print(f"\n遮挡过滤统计:")
print(df_cand['filter_reason'].value_counts())
```

## 遮挡过滤说明

遮挡过滤通过以下条件判断装甲板是否被遮挡：

1. **armor_yaw**: 装甲板自身姿态角（来自上游armor_tracker）
2. **view_yaw**: 从机器人原点看装甲板的方位角 = atan2(y, x)
3. **facing_diff**: 装甲板法向与视线方向的夹角差异

   ```
   facing_diff = |abs(armor_yaw - view_yaw) - π|
   ```

4. **判断逻辑**:
   - 如果 `facing_diff > max_facing_angle_deviation` (默认1.05弧度≈60°)，则认为被遮挡
   - `filter_reason` 字段记录过滤原因：
     - `"valid"`: 有效候选
     - `"occluded(facing_diff=XX°>YY°)"`: 因遮挡被过滤
     - `"low_confidence(XX<YY)"`: 置信度过低
     - `"too_far(XXm>YYm)"`: 距离过远
     - `"no_data"`: 缺少必要数据

**注意**: 如果上游armor_tracker未提供armor_yaw数据（state.yaw=0），则所有目标的armor_yaw都为0，此时facing_diff计算可能不准确。可视化工具会优先显示armor_yaw箭头，若为0则fallback显示view_yaw箭头。

## 性能考虑

- **缓冲机制**: 使用100帧缓冲，批量写入磁盘，减少I/O开销
- **非阻塞**: 日志记录不影响实时控制性能
- **磁盘空间**:
  - 主CSV: 约 150列 × 帧数 × 20字节/单元 ≈ 3KB/帧
  - 候选CSV: 约 (15基础列 + 4×18预测列) × 候选数/帧 × 20字节 ≈ 1.5KB/候选/帧
  - 建议为长时间记录预留充足磁盘空间

## 配置参数

在 `trajectory_planner.yaml` 中：

```yaml
trajectory_planner:
  ros__parameters:
    # 审计模式
    audit_mode: false              # 是否启用审计记录
    audit_log_path: "/tmp/mpc_audit.csv"  # 主日志路径
    
    # 遮挡过滤参数（影响候选目标记录）
    predictor:
      enable_occlusion_check: true         # 是否启用遮挡检查
      max_facing_angle_deviation: 1.05     # 最大允许facing_diff (弧度)
      min_confidence: 0.5                  # 最小置信度阈值
      max_distance: 10.0                   # 最大距离阈值 (米)
```

## 故障排查

### 问题1: 没有生成候选目标CSV

**可能原因**:

- audit_mode未启用
- 没有候选目标数据（target_predictor未接收到预测）

**解决方法**:

- 检查 `audit_mode: true`
- 检查 armor_tracker 是否正常发布预测

### 问题2: 可视化时报错"找不到列"

**可能原因**:

- CSV格式版本不匹配
- 数据记录不完整

**解决方法**:

- 删除旧的CSV文件，重新记录
- 确保使用匹配的visualize脚本版本

### 问题3: 所有armor_yaw都是0

**可能原因**:

- 上游armor_tracker未提供装甲板姿态数据（state.yaw字段为0）

**解决方法**:

- 检查armor_tracker配置和实现
- 可视化工具会自动fallback到view_yaw显示

### 问题4: MPC预测轨迹不显示

**可能原因**:

- pred_theta列数据为nan
- MPC求解失败

**解决方法**:

- 检查 `solver_status` 列是否为 "solved"
- 检查MPC配置参数是否合理

## 文件结构

```
trajectory_planner/
├── trajectory_planner/
│   ├── audit_logger.py           # 审计日志记录器
│   ├── mpc_controller.py         # MPC控制器（返回诊断信息）
│   ├── target_predictor.py       # 目标预测器（提供候选信息）
│   └── trajectory_planner_node.py  # 主节点（集成审计功能）
├── scripts/
│   ├── visualize_audit.py        # 基础可视化工具
│   ├── visualize_audit_v2.py     # 增强可视化工具（推荐）
│   └── generate_test_audit.py    # 测试数据生成器
├── config/
│   └── trajectory_planner.yaml   # 配置文件
└── README_AUDIT.md               # 本文档
```

## 版本信息

- **审计系统版本**: 2.0
- **主要更新**:
  - 2.0: 添加候选目标记录、遮挡过滤信息、增强可视化
  - 1.0: 基础MPC输入输出记录

## 参考资料

- MPC控制器文档: [mpc_controller.py](trajectory_planner/mpc_controller.py)
- 目标预测器文档: [target_predictor.py](trajectory_planner/target_predictor.py)
- ROS2参数配置: [trajectory_planner.yaml](config/trajectory_planner.yaml)
