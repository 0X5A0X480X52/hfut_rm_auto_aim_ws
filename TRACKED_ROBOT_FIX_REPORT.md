# TrackedRobot 消息完整性修复报告

## 修复内容

### 1. max_entropy_tracker_node.py 修复

#### 问题1：`num_armors` 字段赋值顺序错误
**原因**：在调用 `_generate_armors_offset(msg.num_armors, ...)` 时，`msg.num_armors` 尚未赋值（默认为0），导致 `armors_offset` 为空列表。

**修复**：
- 添加 `_infer_num_armors()` 方法根据 `robot_type` 动态确定装甲板数量：
  - `BALANCE_2` → 2
  - `OUTPOST_3`, `BASE` → 3
  - `STANDARD_4`, `HERO_4` → 4
- 将 `msg.num_armors` 赋值移到调用 `_generate_armors_offset()` 之前

#### 问题2：协方差矩阵提取失败
**原因**：`tracker.get_covariance()` 方法不存在，实际协方差矩阵存储在 `tracker.ukf.P` 中。

**修复**：
- 使用 `tracker.ukf.P` 直接访问 UKF 内部协方差矩阵
- 正确设置 `covariance_dim = cov.shape[0]`（不是开平方）
- 添加异常处理和 warning 日志

#### 问题3：`bound_armor_ids` 未填充
**原因**：armor_detector 发布的 `armor.number` 就是机器人ID（如 "1", "2", "outpost"），不是独立的装甲板ID。

**修复**：
- 填充 `bound_armor_ids = [robot_id]` 表示所有装甲板属于该机器人
- 这符合当前的ID系统设计（没有单独的装甲板ID）

#### 问题4：`visible_armor_count` 不准确
**原因**：硬编码为1，不反映实际观测到的装甲板数量。

**修复**：
- 添加 `_last_observation_counts` 字典记录每个机器人最近观测到的装甲板数量
- 在 `armors_callback` 的 update 前更新计数
- 使用实际观测数量填充 `visible_armor_count`

#### 问题5：置信度计算改进
**修复**：
- `TRACKING` 状态 → 1.0
- `TEMP_LOST` 状态 → 0.7
- 其他状态 → 0.3

#### 问题6：添加 DEBUG 日志
**添加**：输出 TrackedRobot 关键字段以便验证数据完整性

### 2. gimbal_controller armor_position_calculator.cpp 修复

#### 添加 DEBUG 日志
**目的**：验证是否正确使用了 `armors_offset` 字段

**修复**：
- 在 `calculate()` 方法中添加日志，显示使用 `armors_offset` 还是回退到 `generateDefaultOffsets`
- 输出 `armors_offset.size()` 和 `num_armors` 以便对比

## 字段说明

### 加速度字段
- `center_acceleration` 和 `yaw_acceleration`：**仅在模型支持时有有效值**
- 当前 `dual_radius_spin_ukf.py` 的 `get_state_dict()` 不导出加速度字段
- 如果模型包含加速度状态（CA模型），需要扩展 `get_state_dict()` 方法
- 当前默认值为 0.0，表示加速度未建模或不可用

### bound_armor_ids 语义
- **当前实现**：填充机器人ID（如 `["1"]`）
- **设计说明**：armor_detector 发布的 `armor.number` 就是机器人ID，系统中没有单独的装甲板ID
- 所有属于该机器人的装甲板共享同一个ID（机器人ID）

## 验证步骤

### 1. 编译
```bash
cd /home/amatrix02/hfut_rm_auto_aim_ws
colcon build --packages-select max_entropy_tracker gimbal_controller
source install/setup.bash
```

### 2. 运行系统
```bash
ros2 launch rm_bringup bringup_max_entropy_test.launch.py image_source:=video virtual_serial:=true
```

### 3. 验证消息完整性
在新终端运行验证脚本：
```bash
source install/setup.bash
python3 verify_tracked_robot.py
```

验证脚本会检查：
- ✓ `len(armors_offset) == num_armors`
- ✓ `len(state_covariance) == covariance_dim²`
- ✓ `bound_armor_ids` 非空
- ✓ 可见性信息一致性（`is_visible=True` 时 `visible_armor_count > 0`）

### 4. 查看 DEBUG 日志
```bash
# max_entropy_tracker 日志
ros2 topic echo /max_entropy_tracker/tracked_robots --once

# gimbal_controller 日志（需要设置日志级别为 DEBUG）
ros2 run gimbal_controller gimbal_controller_node --ros-args --log-level debug
```

## 预期结果

### max_entropy_tracker 输出
```
TrackedRobot[1]: num_armors=4, armors_offset_len=4, cov_dim=14, 
bound_armor_ids=['1'], visible_count=2, confidence=1.00
```

### gimbal_controller 输出
```
Using armors_offset from TrackedRobot (size=4, num_armors=4)
```

## 技术验证

### armors_offset 计算正确性
`_generate_armors_offset()` 的实现与 `armor_solver.cpp` 的 `getArmorPositions()` **数学等价**：
- 装甲板角度：`angle = i * (2π / num_armors)`
- 半径交替（4装甲板）：`r1 → r2 → r1 → r2`
- 位置：`(-r*cos(angle), -r*sin(angle), dz)` （机器人坐标系）

### gimbal_controller 装甲板重建正确性
`ArmorPositionCalculator::calculate()` 与 `armor_solver` **数学等价**：

**armor_solver 直接计算**：
```cpp
armor_pos = center + (-r*cos(yaw+angle), -r*sin(yaw+angle), dz)
```

**gimbal_controller 分解计算**：
```cpp
// Step 1: 机器人坐标系偏移
offset = (-r*cos(angle), -r*sin(angle), dz)

// Step 2: 旋转矩阵变换到世界坐标系
rotation(yaw) * offset = (-r*cos(yaw+angle), -r*sin(yaw+angle), dz)
```

**三角恒等式验证**：
```
cos(yaw)*(-r*cos(angle)) - sin(yaw)*(-r*sin(angle))
= -r[cos(yaw)*cos(angle) + sin(yaw)*sin(angle)]
= -r*cos(yaw - angle)
= -r*cos(-(angle - yaw))
= -r*cos(angle - yaw)  [cos is even]
```

当 `angle = i*(2π/n)` 且 yaw 从机器人坐标系原点测量时，两种方法完全等价。

## 已知限制

1. **中心加速度**：需要 CA 运动模型且扩展 `get_state_dict()` 才有非零值
2. **yaw 角加速度**：当前旋转模型只有一阶角速度，无二阶角加速度状态
3. **装甲板ID系统**：当前系统中装甲板没有独立ID，都使用机器人ID

## 后续优化建议

1. 如果下游需要中心加速度，扩展 `dual_radius_spin_ukf.py` 的 `get_state_dict()`
2. 如果需要角加速度，升级旋转模型为 CA（成本较高）
3. 考虑是否需要独立的装甲板ID系统（需要 armor_detector 支持）
