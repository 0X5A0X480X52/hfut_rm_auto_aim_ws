# Fire Advice 概率误差方案（实现与可视化一体化设计）

## 1. 背景与目标

当前 `fire_advice` 以几何阈值判定为主，容易出现阈值附近抖动，导致下位机连续触发窗口不稳定。为此将方案升级为：

1. 基于概率的命中评估（`P_hit`）。
2. 未来时间窗融合（`P_window`）。
3. 低通/积分 + 滞回输出稳定 `fire_advice`。
4. 首版即实现 sigma-point 扩展。
5. `state_covariance` 可选启用，异常时可降级。
6. 新增 3D 可视化（弹道散布 + 装甲板平面概率椭圆）。

---

## 2. 现有架构对接原则

### 2.1 保持职责清晰

- `FireAdvisor`：保留“几何判定内核 + 兼容回退”。
- `FireAdviceEngine`：保留“时序/候选装甲板/弹道解算 orchestration”。
- 新增 `fire_advice` 概率模块：负责概率、融合、状态门控、调试快照。

### 2.2 集成位置

主要接入点：

- `src/rm_auto_aim/gimbal_pipeline/include/gimbal_controller/fire_advice_engine.hpp`
- `src/rm_auto_aim/gimbal_pipeline/src/gimbal_controller/fire_advice_engine.cpp`
- `src/rm_auto_aim/gimbal_pipeline/src/gimbal_pipeline_node.cpp`

---

## 3. 目录与模块拆分

建议新增：

- `src/rm_auto_aim/gimbal_pipeline/include/gimbal_controller/fire_advice/`
- `src/rm_auto_aim/gimbal_pipeline/src/gimbal_controller/fire_advice/`

建议文件：

1. `types.hpp`
2. `probability_estimator.hpp/.cpp`
3. `window_fuser.hpp/.cpp`
4. `fire_gate.hpp/.cpp`
5. `sigma_points.hpp/.cpp`
6. `covariance_adapter.hpp/.cpp`
7. `debug_snapshot.hpp`

### 3.1 types.hpp（核心结构）

- `FireProbabilityConfig`
- `FireGateConfig`
- `SigmaPointConfig`
- `TrackerCovarianceConfig`
- `CandidateTauEvaluation`
- `CandidateProbabilityResult`
- `FireProbabilityEngineResult`

### 3.2 probability_estimator

输入：

- 候选装甲板姿态（center/right/up/normal）。
- 子弹均值/速度、飞行时间、目标预测时序。
- tracker 协方差（或 fallback）。
- sigma-point 参数。

输出：

- `e_u/e_v`。
- `sigma_u/sigma_v`。
- `P_hit(tau)`。
- 供可视化的椭圆与点云辅助数据。

### 3.3 window_fuser

- `max` 融合。
- `softmax(beta)` 融合。

输出：`P_window` 与 `best_tau`。

### 3.4 fire_gate

支持两种模式：

- `lowpass`: `score = alpha * score + (1-alpha) * P_window`
- `integrator`: 参考 demo 的 rise/fall 基线积分

最终通过滞回阈值输出：

- `fire_on_th`
- `fire_off_th`

并维护 `fire_state`。

### 3.5 sigma_points

首版实现：

- `unscented`（默认）
- `fixed_five_points`

扰动维度：

- `delta_v0`
- `delta_trigger_delay`

输出附加投影协方差 `sigma_extra_cov_uv`。

### 3.6 covariance_adapter

输入：`TrackedRobot.state_covariance + covariance_dim`

输出：用于装甲板平面投影的 3D 协方差。

策略：

1. 可用且维度合法：使用 tracker 协方差。
2. 不可用：按配置回退 `diag(fallback_sigma_x/y/z)`。
3. 若 `strict_covariance=true` 且不可用：标记 invalid。

---

## 4. 核心流程（Engine 内）

在 `FireAdviceEngine::evaluate(...)` 中：

1. 先走现有 `timeline` 与候选 impact 解算。
2. 对每个候选装甲板，遍历 `tau in [0, window]`（步长 `future_step_ms`）：
   - 预测 impact 时刻目标装甲板状态。
   - 计算弹着均值误差 `e_uv`。
   - 计算总协方差 `Sigma_total_uv`：
     - `Sigma_bullet_uv`
     - `Sigma_tracker_uv`
     - `Sigma_sigma_point_uv`（启用时）
   - 计算 `P_hit(tau)`。
3. 候选内做窗口融合得 `P_window(candidate)`。
4. 在候选之间选择最优候选。
5. 将最优 `P_window` 输入 `fire_gate`，得 `fire_score` 与稳定 `fire_advice`。
6. 输出兼容字段（`fire_advice` 等）+ 新增调试字段（概率/椭圆/散布）。

---

## 5. 参数设计（建议）

新增参数前缀：`controller.fire.probability.*`

### 5.1 总开关与时窗

- `enable` (bool, default: false)
- `future_window_ms` (double, default: 50.0)
- `future_step_ms` (double, default: 10.0)
- `window_fusion` (string: `max|softmax`, default: `max`)
- `softmax_beta` (double, default: 20.0)

### 5.2 tracker 协方差

- `use_tracker_covariance` (bool, default: true)
- `strict_covariance` (bool, default: false)
- `fallback_sigma_x` (double, default: 0.02)
- `fallback_sigma_y` (double, default: 0.02)
- `fallback_sigma_z` (double, default: 0.03)

### 5.3 法向速度加权（可选）

- `normal_velocity_weight.enable` (bool)
- `normal_velocity_weight.v_ref` (double)
- `normal_velocity_weight.w_min` (double)

### 5.4 sigma-point

- `sigma_point.enable` (bool, default: true)
- `sigma_point.method` (`unscented|fixed`)
- `sigma_point.sigma_v0` (double)
- `sigma_point.sigma_delay` (double)
- `sigma_point.rho` (double)
- `sigma_point.alpha` (double)
- `sigma_point.beta` (double)
- `sigma_point.kappa` (double)
- `sigma_point.min_bullet_speed` (double)

### 5.5 gate

- `gate.mode` (`lowpass|integrator`)
- `gate.alpha`
- `gate.fire_on_th`
- `gate.fire_off_th`
- `gate.integrator_base_probability`
- `gate.integrator_rise`
- `gate.integrator_fall`

---

## 6. 状态管理与复位策略

`fire_gate` 建议按 `target_id` 维护状态（LRU 小缓存），避免切目标串扰。

复位条件建议：

1. `target_id` 变化。
2. `track_state != TRACKING` 持续 N 帧。
3. 超时未更新目标（如 >300ms）。

---

## 7. 可视化 Marker 方案

基于现有 `debug_gimbal_marker_pub_`，新增命名空间 `fire_prob/*`。

### 7.1 3D 弹道与散布

1. `fire_prob/trajectory_mean` (`LINE_STRIP`)
   - 最优候选+最优 tau 的弹道均值轨迹。
2. `fire_prob/trajectory_sigma_tube` (`SPHERE_LIST`)
   - 沿轨迹显示 1σ 半径，体现随飞行时间增长的扩散。
3. `fire_prob/impact_cloud` (`POINTS`)
   - 命中时刻散点（Monte Carlo 或 sigma-point 重建）。

### 7.2 装甲板平面概率投影

4. `fire_prob/armor_plane` (`LINE_LIST`)
   - 最优装甲板矩形边框。
5. `fire_prob/error_ellipse_1sigma` (`LINE_STRIP`)
6. `fire_prob/error_ellipse_2sigma` (`LINE_STRIP`)
   - 将 `(u,v)` 协方差椭圆映射回 3D 装甲板平面。
7. `fire_prob/mean_error_point` (`SPHERE`)
   - 均值误差点。

### 7.3 时间窗候选

8. `fire_prob/tau_candidates` (`SPHERE_LIST`)
   - 每个 `tau` 候选的命中点，按 `P_hit(tau)` 着色。
9. `fire_prob/text` (`TEXT_VIEW_FACING`)
   - 文本：`P_hit/P_window/score/tau*/fire`。

### 7.4 可视化参数

新增 `controller.fire.visualization.*`：

- `enable` (bool)
- `publish_rate_hz` (double, default: 30)
- `traj_samples` (int, default: 30)
- `ellipse_samples` (int, default: 64)
- `max_impact_points` (int, default: 120)
- `show_sigma_tube` (bool)
- `show_impact_cloud` (bool)
- `show_tau_candidates` (bool)
- `only_when_tracking` (bool)
- `only_best_candidate` (bool)

---

## 8. 调试消息扩展建议

扩展 `FireAdviceDebugSnapshot` 与 `rm_interfaces/msg/FireAdviceDebug.msg`：

- `float64 p_hit_window`
- `float64 fire_score`
- `float64 best_tau_ms`
- `float64 sigma_u`
- `float64 sigma_v`
- `float64 e_u`
- `float64 e_v`
- `string covariance_source` (`tracker|fallback|invalid`)

用途：

- 图表快速验证抖动是否改善。
- 现场定位“协方差不可用导致回退”的状态。

---

## 9. 性能预算与降级

目标：在 120 FPS 控制预算下稳定运行。

建议：

1. 默认 `tau` 采样 6 点以内（0~50ms, 10ms step）。
2. `flight_time_iters` 默认 2。
3. marker 发布降频到 30Hz。
4. 点云上限硬限制。
5. 分级可视化：
   - L1: 轨迹均值+文本
   - L2: +概率椭圆
   - L3: +impact cloud

---

## 10. 落地阶段计划

### 阶段 A：旁路计算（不接管 fire）

- 概率模块全链路计算。
- 仅发布 debug msg + marker。
- 与现有几何 fire_advice 对比。

### 阶段 B：开关接管

- `controller.fire.probability.enable=true` 时由概率 gate 输出 fire。
- 保留几何回退路径（异常兜底）。

### 阶段 C：固化参数

- 打开 sigma-point。
- 打开 tracker covariance（`strict=false`）。
- 现场回放调参，收敛默认参数。

---

## 11. 风险与对策

1. 协方差维度语义不一致
   - 对策：`covariance_adapter` 做维度检查 + 显式 source 标注。
2. 计算量超预算
   - 对策：减少 `tau` 点数，关闭点云，降 marker 频率。
3. 切目标状态残留
   - 对策：target 级 gate 状态管理 + 复位规则。
4. 可视化残影
   - 对策：invalid 帧发 `DELETE` 同 id marker。

---

## 12. 验收标准

1. 功能：
   - 能输出 `P_window + fire_score + 滞回 fire_advice`。
   - sigma-point 开启/关闭行为可控。
   - state_covariance 开关与回退行为正确。

2. 可视化：
   - 轨迹、散布、概率椭圆、tau 候选正常显示。
   - 与 debug 数值一致。

3. 性能：
   - 控制周期无明显超时。
   - 可视化开启后仍满足实时性要求。

4. 稳定性：
   - 比原几何阈值方案，`fire_advice` 抖动显著下降。

