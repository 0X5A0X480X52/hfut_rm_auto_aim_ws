# gimbal_pipeline 典型问题调参指北

本文面向当前 `src/rm_auto_aim/gimbal_pipeline` 实现，聚焦 6 类高频问题：

1. 抖动
2. 丢锁
3. 错板
4. 慢半拍
5. 快半拍
6. 误开火

目标是给出“先看哪里、先调什么、如何判定有效”的可执行流程，避免盲目扫参数。

---

## 0. 调参总原则（强烈建议先做）

1. 一次只改一组参数，单次改动幅度控制在 10%~20%。
2. 先解决坐标/时延一致性，再调滤波器；否则会出现“调什么都不稳定”。
3. 优先顺序：
   - 关联是否正确（track 能不能绑对）
   - 状态估计是否稳定（UKF/模型）
   - 控制与开火时延是否匹配（delay/fire）
4. 保留 A/B 记录：每次修改记录“参数->现象变化->是否回滚”。

---

## 1. 抖动（Yaw/Pitch 高频抖动、来回摆）

### 1.1 典型现象
- 云台在目标附近左右来回小幅摆动
- `TRACKING` 状态正常，但输出 `yaw_diff/pitch_diff` 高频波动

### 1.2 实现链路定位
- 观测融合与状态估计：`ukf.*`, `motion.*`, `spin.*`
- 目标选择：`selector.*`
- 输出保护：`controller.output_filter.*`
- 平滑器：`smoother.*`

### 1.3 优先调参顺序
1. `controller.output_filter.enable_rate_limiter` 确保开启，再调：
   - `max_yaw_rate`
   - `max_pitch_rate`
2. 观测噪声：
   - `ukf.obs_noise_yaw`
   - `ukf.obs_noise_pos`
3. 若是选板抖动导致，调：
   - `selector.hysteresis_threshold`
   - `selector.sticky_lock_frames`
4. 仍抖动再考虑：
   - `smoother.enable_yaw_smooth`
   - `smoother.yaw_min_cutoff`, `smoother.yaw_beta`

### 1.4 调大/调小方向
- `obs_noise_yaw` 增大：更稳、更不敏感；过大可能转向滞后
- `max_yaw_rate` 减小：抑制抖动；过小可能跟不上急转
- `hysteresis_threshold` 增大：减少频繁切换；过大可能锁错目标

---

## 2. 丢锁（目标频繁 TEMP_LOST/LOST）

### 2.1 典型现象
- 目标短暂遮挡后无法恢复
- 跟踪状态在 `TRACKING <-> TEMP_LOST` 频繁切换

### 2.2 实现链路定位
- 跟踪状态机阈值：`tracker.tracking_thres/lost_thres/temp_lost_thres`
- 关联门限：`tracker.max_match_distance/max_match_yaw_diff`
- 超时：`tracker_timeout`

### 2.3 优先调参顺序
1. 提高容错：
   - `tracker.temp_lost_thres`
   - `tracker.lost_thres`
2. 放宽关联门限（小步）：
   - `tracker.max_match_distance`
   - `tracker.max_match_yaw_diff`
3. 检查是否预测不足：
   - `predict_rate`
   - `motion.translation_model`（必要时切 `Singer`）

### 2.4 风险提示
- 门限放太宽会从“丢锁”变成“错绑”。
- 建议先拉高 `temp_lost_thres`，再谨慎动 `max_match_*`。

---

## 3. 错板（同车内部装甲板 ID 绑定错误）

### 3.1 典型现象
- 应该 `0123` 的连续相位，变成 `0101` 或 `2323`
- 高低板在切换时发生反转（低板被重构为高板）

### 3.2 实现链路定位
- 跳变绑定：`tracker.jump_binding_*`
- 周期证据：`tracker.periodic_binding_*`
- 板错配审计：`panel_mismatch.*`
- Outpost 场景：`outpost.binding_*`, `outpost.z_offset_*`

### 3.3 优先调参顺序（Norm4/Adaptive通用）
1. 提升切板稳定性：
   - `tracker.jump_binding_confirm_frames` ↑
   - `tracker.jump_binding_switch_cooldown` ↑
2. 提高“切板证据质量”门槛：
   - `tracker.jump_binding_cost_margin_min` ↑
   - `tracker.jump_binding_confidence_floor` ↑
3. 若匀速旋转稳定场景，可尝试：
   - `tracker.periodic_binding_enable: true`
   - 小幅调 `tracker.periodic_binding_weight`
4. 打开审计（不自动纠偏）：
   - `panel_mismatch.enable: true`
   - `panel_mismatch.apply_correction: false`

### 3.4 风险提示
- 过度抬高门槛会导致“几乎不切板”，表现为跟踪迟钝。
- `periodic_binding` 在噪声大/非稳态旋转下可能引入错误先验。

---

## 4. 慢半拍（落后目标）

### 4.1 典型现象
- 准星总追在目标后面
- 目标加速后明显跟不上，停止后还在“补偿”

### 4.2 实现链路定位
- 统一延时链：`controller.delay.*`
- solver 预测：`controller.solver.prediction_delay/max_prediction_time`
- 控制器策略：`controller.strategy`
- MPC 时域：`controller.mpc.N`, `controller.mpc.dt`

### 4.3 优先调参顺序
1. 先调统一延时链：
   - `controller.delay.prediction_extra_s` ↑
   - `controller.delay.control_latency_s` ↑（若链路存在延时）
2. 再调 solver 预测：
   - `controller.solver.prediction_delay` ↑
3. 若使用 MPC：
   - 检查 `N*dt` 是否过短（看得不够远）

### 4.4 风险提示
- 盲目增大预测延时会从“慢半拍”直接变成“快半拍”。

---

## 5. 快半拍（超前、冲过头）

### 5.1 典型现象
- 准星总在目标前方，像“抢跑”
- 高速目标上来回超调

### 5.2 实现链路定位
- 同“慢半拍”，但方向相反：
  - `controller.delay.*`
  - `controller.solver.prediction_delay`
  - `controller.solver.controller_delay`（兼容链路）
  - `controller.mpc.enable_delay_compensation/prediction_delay_s`

### 5.3 优先调参顺序
1. 先减统一延时预测量：
   - `controller.delay.prediction_extra_s` ↓
2. 减 solver 预测延时：
   - `controller.solver.prediction_delay` ↓
3. MPC 场景检查：
   - `controller.mpc.enable_delay_compensation`
   - `controller.mpc.prediction_delay_s` 是否重复补偿

### 5.4 风险提示
- 新旧兼容延时键混用（`controller.delay.*` + `solver.*delay`）最容易引入双重补偿。

---

## 6. 误开火（不该开火时触发）

### 6.1 典型现象
- 背身/侧身板也触发开火
- 命中概率低仍频繁发 fire=true

### 6.2 实现链路定位
- 开火策略：`controller.fire.decision_policy`
- 朝向可见性：`controller.fire.target_visibility_policy`
- 朝向开角：`controller.fire.facing_filter_opening_angle_deg`
- 概率模型：`controller.fire.probability.*`

### 6.3 优先调参顺序
1. 确保可见性策略正确：
   - `controller.fire.target_visibility_policy: "facing_only"`
2. 收紧朝向窗口：
   - `controller.fire.facing_filter_opening_angle_deg` ↓
3. 概率门控收紧：
   - `controller.fire.probability.gate.fire_on_th` ↑
   - `controller.fire.probability.gate.fire_off_th` ↑（保持回差）
4. 减少概率“虚高”：
   - `controller.fire.probability.softmax_beta` ↑
   - 或降低窗口融合激进度

### 6.4 风险提示
- 过度收紧会从“误开火”变成“不开火”。
- 建议先固定 `probability.enable=false` 验证基础几何触发，再启用概率模块。

---

## 7. 推荐排障流程（实战）

1. 固定场景（同距离、同速度、同旋转方式）录制 10~20s
2. 先看“关联是否正确”（错板/丢锁）
3. 再看“时延是否匹配”（慢半拍/快半拍）
4. 最后看“输出品质”（抖动/误开火）
5. 每轮只改一组参数，保留前后对比数据

---

## 8. 与当前配置文件的配合使用

- 参数主入口：`src/rm_auto_aim/gimbal_pipeline/config/gimbal_pipeline.yaml`
- 当前 yaml 已按模块加注释，本文用于“问题驱动”的定位路径。
- 建议将本文件与 yaml 同时打开，按“症状 -> 模块 -> 参数”联动调试。

