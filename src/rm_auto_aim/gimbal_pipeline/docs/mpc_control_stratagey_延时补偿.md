# MPC 控制策略：延时补偿流程

好的，我们可以把你想要的完整 MPC 参考轨迹生成和开火判断流程整理成一个 **伪代码流程**，涵盖以下要素：

* 每个 dt 预测
* 子弹飞行时间补偿
* 弹道补偿
* 每个 dt 的选板（最小运动量）
* 生成 MPC 参考轨迹
* 开火建议计算（fire_advice）

---

## 伪代码流程

```cpp
// 输入:
//   target_robot: Tracker输出的目标状态
//   current_yaw, current_pitch: 当前云台角度
//   N: 预测步数
//   dt: MPC 时间步长
//   controller_delay, prediction_delay: 延迟参数
//   processing_delay: 当前时间与目标观测时间的差
//   bullet_speed: 子弹初速度
//   local_compensator: 弹道解算器
//   armor_selector: 装甲板选择器
//   fire_advisor: 开火判断器

// 初始化
X_ref = zeros(nx * N)        // nx = 状态维度 (yaw, pitch, yaw_dot, pitch_dot)
fire_advice = false

// 获取延迟信息
processing_delay = current_time - tracker_timestamp
total_base_delay = processing_delay + controller_delay + prediction_delay

for k = 0 to N-1:

    // 1. 计算参考时间
    t_ahead = (k + 1) * dt
    t_predict = total_base_delay + t_ahead

    // 2. 预测目标中心位置（匀速假设或 tracker 提供接口）
    pred_pos = target_pos + target_velocity * t_predict

    // 3. 子弹飞行时间补偿（迭代 1~2 次即可）
    t_flight = local_compensator.getFlyingTime(pred_pos)
    pred_pos_flight = target_pos + target_velocity * (t_predict + t_flight)
    t_flight = local_compensator.getFlyingTime(pred_pos_flight)
    pred_pos_flight = target_pos + target_velocity * (t_predict + t_flight)

    // 4. 预测可选装甲板位置
    candidate_armors = position_calculator.calculatePredicted(pred_pos_flight)

    // 5. 遍历每个装甲板
    best_error = INF
    for armor in candidate_armors:
        // 弹道解算
        ballistic = local_compensator.compensate(armor.position)
        
        // 预测 yaw_dot / pitch_dot
        yaw_dot_ref = (ballistic.yaw - prev_yaw_ref) / dt
        pitch_dot_ref = (ballistic.pitch - prev_pitch_ref) / dt

        // 计算延迟补偿后的云台姿态
        yaw_delayed   = prev_yaw_ref + yaw_dot_ref * effective_ctrl_delay
        pitch_delayed = prev_pitch_ref + pitch_dot_ref * effective_ctrl_delay

        // 与当前姿态的误差
        yaw_diff = normalize_angle(ballistic.yaw - yaw_delayed)
        pitch_diff = ballistic.pitch - pitch_delayed
        error = yaw_diff^2 + pitch_diff^2

        if error < best_error:
            best_error = error
            best_ballistic = ballistic
            yaw_dot_selected = yaw_dot_ref
            pitch_dot_selected = pitch_dot_ref

    // 6. 填充参考轨迹
    X_ref.segment(k * nx, nx) = [best_ballistic.yaw, best_ballistic.pitch,
                                 yaw_dot_selected, pitch_dot_selected]

    prev_yaw_ref = best_ballistic.yaw
    prev_pitch_ref = best_ballistic.pitch

// 7. FireAdvisor 判断（使用延迟补偿后的云台姿态）
yaw_now_for_fire   = current_yaw + yaw_dot_selected * controller_delay
pitch_now_for_fire = current_pitch + pitch_dot_selected * controller_delay

fire_advice = fire_advisor.shouldFire(
    yaw_now_for_fire,
    pitch_now_for_fire,
    best_ballistic.yaw,
    best_ballistic.pitch,
    distance_to_target
)
```

---

### ✅ 特点说明

1. **每个 dt 上选板**

   * 保留原有逻辑，对每个候选装甲板都进行弹道补偿，选误差最小的作为参考。

2. **延迟补偿**

   * `yaw_now_for_fire` / `pitch_now_for_fire` 采用 `controller_delay` + `processing_delay` 补偿后的云台姿态进行开火判断。

3. **飞行时间迭代补偿**

   * 在参考轨迹生成时，每个 dt 的目标位置都迭代补偿子弹飞行时间。

4. **参考轨迹的时间间隔仍为 dt**

   * MPC 输入序列仍然是 dt 均匀采样，飞行时间补偿只体现在 yaw/pitch 值上，而不会破坏序列均匀性。
