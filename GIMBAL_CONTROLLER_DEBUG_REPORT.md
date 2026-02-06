# Gimbal Controller 调试报告

## 问题现象
`/trajectory_planner/gimbal_cmd` 输出始终为固定值：
- `yaw_diff=-1.575`
- `pitch_diff=-0.705`
- `dist=3.65`
- `fire=0`

## 根本原因分析

### 1. **目标选择器未发布消息** ✅ 主要问题
从日志中发现：`target=no` 表示 `gimbal_controller` 没有收到 `selected_target` 消息。

**原因**：
- `target_selector` 订阅的是 `/robot_pose_estimator/robots`
- 但在 launch 文件中 remapping 为 `/max_entropy_tracker/tracked_robots`
- 可能存在话题名称不匹配或者 `target_selector` 内部配置问题

**解决方案**：
检查 `target_selector` 的配置文件中 `topics.robots_sub` 参数是否正确设置。

### 2. **弹道解算服务超时** ⚠️ 性能问题
频繁出现：`Ballistic solver timeout after 10 ms`

**原因**：
- Timer 回调频率 250Hz（每 4ms 一次）
- 弹道解算服务响应时间 > 10ms
- 在 timer 回调中同步等待 future 会阻塞

**解决方案**：
- 已将超时时间缩短到 10ms 以避免阻塞过久
- 超时后使用本地 fallback 计算（`LocalTrajectoryCompensator`）
- **建议**：改为异步调用或使用缓存机制

### 3. **潜在死锁风险** ⚠️ 已缓解
**问题**：在 250Hz 的 timer 回调中同步等待 service future

**已采取措施**：
- 将超时从默认 100ms 缩短到 10ms
- 添加了服务可用性快速检查（5ms 超时）
- 超时后立即使用本地计算 fallback

**长期改进建议**：
- 使用异步 service 调用，不在 timer 回调中阻塞
- 或将弹道计算移到单独的线程/executor

## 添加的调试日志

### 1. 订阅回调日志
```cpp
- trackedRobotsCallback: 记录接收到的机器人数量和状态
- selectedTargetCallback: 记录选中的目标ID
```

### 2. Timer 回调日志
```cpp
- 每 50 次（约 5Hz）打印状态摘要
- 记录 enable、robots、target 是否可用
- 记录每次发布的命令值
```

### 3. 策略执行日志
```cpp
- 记录找到的目标机器人信息
- 记录策略执行前后的状态
- 记录计算出的命令参数
```

### 4. 弹道解算日志
```cpp
- 记录服务调用参数
- 记录调用结果或超时
- 警告超时情况
```

## 下一步诊断步骤

### 1. 检查话题连接
运行诊断脚本：
```bash
cd /home/amatrix02/hfut_rm_auto_aim_ws
chmod +x diagnose_topics.sh
source install/setup.bash
./diagnose_topics.sh
```

### 2. 检查 target_selector 配置
查看文件：
```
src/rm_auto_aim/target_selector/config/target_selector.yaml
```
确认 `topics.robots_sub` 参数值

### 3. 测试弹道解算服务
```bash
# 测试服务是否响应
ros2 service call /ballistic_solver/solve rm_interfaces/srv/SolveBallistic \
  "{target_position: {x: 3.0, y: 0.0, z: 0.5}, target_velocity: {x: 0.0, y: 0.0, z: 0.0}, bullet_speed: 20.0}"
```

## 修复建议优先级

### 高优先级 🔴
1. **修复 target_selector 话题连接**
   - 确保 `target_selector` 正确订阅 `/max_entropy_tracker/tracked_robots`
   - 检查 remapping 配置

### 中优先级 🟡
2. **优化弹道解算调用**
   - 考虑异步调用或缓存机制
   - 或提高 ballistic_solver 服务响应速度

### 低优先级 🟢
3. **添加更多防护**
   - 增加消息时间戳检查（避免使用过期数据）
   - 添加数据有效性验证

## 当前实现状态

✅ 已完成：
- 添加详细调试日志
- 修复潜在死锁问题（缩短超时）
- 实现 marker 可视化

⏳ 待处理：
- target_selector 话题连接问题
- 弹道解算性能优化
