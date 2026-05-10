# Norm4 与 Backend UKF V2 重构详细方案（Legacy）

> 旧版本，仅供参考。最新方案请看 [norm4_backend_ukf_v2_refactor_design.md](norm4_backend_ukf_v2_refactor_design.md)。

## 1. 背景与目标

当前 `Norm4ArmorTracker` 已经把 4 装甲板目标拆成了 observation frontend、binder、mode FSM、ambiguous backend、structured backend、output adapter 等模块。这个方向是有价值的：它把离散相位、单板歧义、四板结构化跟踪、发布语义分离开了。

但实际跟踪中，普通 4 装甲板车的 NIS 经常达到几百甚至几千，说明滤波器统计一致性明显异常。问题不只是参数阈值，而是链路中存在几个正反馈：

1. UKF 前端用复杂关联提前硬判定 `panel_id / layer / r_type`。
2. `DualRadiusSpinUKF` 单观测冻结 `R1 / R2 / DZA`，结构参数主要依赖稀有双观测修正。
3. 双观测几何更新把射线交点和半径估计当作高精度量测，噪声过小且没有传播 yaw/PnP 不确定性。
4. Norm4 structured backend 在双观测时会先单板更新，再双板更新，存在重复消费同一观测的风险。
5. NIS/马氏距离更多被当作调试指标，而不是前端关联、后端更新、状态机健康度的共同依据。

本方案目标是设计一个面向 Norm4 的新后端 `DualRadiusSpinUKFV2`，以及配套的 Norm4 链路重构。核心原则：

1. 前端关联不再做最终裁决，而是生成多假设与时序先验。
2. UKF 后端负责用一致的观测模型计算 innovation、S、NIS、likelihood。
3. Binder/ModeFSM 融合 UKF likelihood、相位连续性、z jump、2D continuity，再决定更新强度。
4. 结构参数不再只靠稀有双观测，单观测提供受限的弱结构约束，双观测提供联合原始观测约束。
5. 所有新行为必须可开关、可回放对比、可逐步迁移。

## 2. 当前实现的主要问题

### 2.1 UKF 前关联过早硬绑定

当前 `ObservationFrontend` 和 `PanelAssociator` 会把单帧观测压缩为一个最优 `candidate_panel_id`。Norm4 双观测 assignment 也会直接选出一组相邻 panel pair。随后 `apply_forced_assignment()` 会显著提高候选置信度。

风险：

1. 一旦前端选错，错误会被高置信送入 binder 和 backend。
2. 前端 cost 是启发式的 `yaw_err + xy_err + z_state_err`，和 UKF 后端真正使用的 innovation covariance 不一致。
3. 0/2、1/3 同高度相位混淆时，前端只保留 best panel，会丢失“高度对但相位不确定”的信息。
4. 前端使用的 `ctx.center_pos / ctx.center_yaw / r1 / r2 / dza` 来自上一轮 backend。如果 backend 已被错误观测带偏，前端会继续强化错误。

结论：复杂关联仍有意义，但它应该变成候选生成器和时序先验层，而不是硬判定层。

### 2.2 单观测冻结结构参数导致系统性偏差

旧 `DualRadiusSpinUKF::update_single()` 在计算 Kalman gain 后直接清零：

```cpp
K.row(idx.R1()).setZero();
K.row(idx.R2()).setZero();
K.row(idx.DZA()).setZero();
```

这避免了单板观测把结构参数带飞，但代价是：

1. `r1/r2/dza` 初值错误时，单观测无法纠正。
2. 结构误差会被中心位置、yaw、速度吸收。
3. 吸收后的中心状态又参与下一帧关联，形成“结构错 -> 中心偏 -> panel 关联错 -> innovation 更大”的正反馈。
4. 当前默认 `default_dza = 0.0`，而 `is_dza_converged()` 又要求 `abs(dza) > 0.005`。双观测少时，`dza` 很容易长期不收敛。

### 2.3 双观测几何更新过度自信

旧双观测路径先用两块装甲板 yaw 反向射线求交，得到：

```text
z_geometry = [xc, yc, zc, r1, r2, dza]
```

然后用 `dual_obs_noise_pos` 构造 6D 对角 R。

风险：

1. yaw 一点点误差会被射线交点放大，尤其两条射线夹角小的时候。
2. PnP 深度误差、角度误差、相机坐标系噪声没有传播到 `R_geo`。
3. `dual_obs_noise_yaw` 和 `dual_obs_geometry_noise_scale` 在当前主更新中基本没有发挥应有作用。
4. 双观测本来稀少，一旦错误且高权重，会把结构参数快速拉偏。

### 2.4 Norm4 structured backend 重复消费观测

当前 Norm4 structured 模式中，双观测有效时大致是：

1. `structured_backend_.update(*selected, hint)` 做一次单观测更新。
2. `structured_backend_.update_dual(obs[0], obs[1], ...)` 再做一次双观测更新。

这意味着 selected 观测可能被同一帧重复使用。Adaptive tracker 的双观测路径也有类似问题：先 `update_single(obs1)`，再 `update_dual(obs1, obs2)`。

V2 应改为帧级 batch update：同一帧内决定 single 或 dual batch，只更新一次。

### 2.5 NIS 统计没有反哺链路

当前代码虽然有 `last_nis()`、`check_innovation_gate()`、maneuver detector，但 NIS 主要作为调试和机动指标。innovation gating 默认关闭，而且 tracker 状态机在“有 detector 观测但 UKF 拒绝更新”时不一定视为观测缺失。

合理行为应是：

1. NIS/chi2 参与 hypothesis scoring。
2. gate reject streak 参与 tracker health。
3. NIS 分量用于区分“观测噪声 R 不合理”“结构参数偏”“panel/layer 错配”。

## 3. 其他实现可参考的部分

### 3.1 FYT2024 armor_solver

可参考点：

1. 遇到 armor jump 时显式 swap 半径/高度相关状态。
2. 当预测装甲板与当前观测位置差太大时，直接重置中心位置和速度。
3. 对半径/高度结构项给出较强的过程噪声或手动修正路径，不让错误状态长期卡死。

不建议照搬：

1. 单一 EKF 状态较粗糙。
2. panel 身份和四板相位表达弱。
3. 对复杂单板歧义的处理能力不足。

### 3.2 JLU2026 armor_tracker

可参考点：

1. 把 `radius_a / radius_b / dz` 作为带先验的结构变量，而不是纯随机游走。
2. 每帧多个 armor factor 可以共同约束结构和中心。
3. 冷启动阶段使用 batch 信息，避免前几帧约束不足时直接优化爆炸。

不建议照搬：

1. GTSAM/iSAM2 引入复杂依赖和计算开销。
2. 当前工程已有 UKF + binder 体系，完全切因子图迁移成本高。

建议吸收为 UKF 里的轻量设计：

1. 结构先验 pseudo-measurement。
2. 多观测 batch update。
3. 冷启动阶段更保守的更新强度。

### 3.3 SP2025 auto_aim

可参考点：

1. 使用 YPD 观测域，噪声随距离和角度变化。
2. EKF 的 NIS failure window 用于坏收敛检测。
3. 单观测 Jacobian 中结构参数并非完全冻结，可以提供弱约束。
4. 遍历多块同目标装甲板更新，说明“多观测输入”在实战中有价值。

不建议照搬：

1. 顺序更新多块 armor 会引入更新顺序和线性化点偏移问题。
2. NIS 阈值实现存在具体数值疑问，应按卡方自由度和实际 NIS 统计重新标定。

### 3.4 ShanghaiTechImp 技术文档

可直接纳入 V2：

1. YPD 球坐标观测噪声建模。
2. 马氏距离门控和 chi2 分量统计。
3. gate reject streak 与状态机联动。
4. IUKF/iterated UKF 作为初始化或大机动时的可选增强。

需要修正或扩展：

1. 文档 P0 主要讨论单观测 R，Norm4 V2 还需要双观测 raw batch R。
2. 旧文档认为双观测几何更新已经足够严密，但当前审计表明射线几何 R 没有传播角度不确定性，仍需重构。

## 4. 重构后的总体架构

### 4.1 分层职责

```text
Norm4ObservationFrontend
  -> 生成 single / dual hypotheses
  -> 保留 top-k，不做最终硬绑定

Norm4HypothesisEvaluator
  -> 调用 Backend UKF V2 的 predict_measurement()
  -> 计算 innovation / S / chi2 / likelihood

Norm4BinderBridge / PhaseMemory
  -> 融合 UKF likelihood、z jump、相位连续性、2D continuity、历史 bound id

Norm4ModeFSM
  -> 决定 AMBIGUOUS / STRUCTURED

Norm4StructuredBackendV2
  -> 单帧 batch update
  -> single / dual raw observation update
  -> optional geometry pseudo update
  -> structural prior update

Norm4OutputAdapter
  -> 维持现有发布语义
```

### 4.2 前端关联的新定位

前端仍然保留，但输出从“唯一结论”改成“候选集合”：

```cpp
struct Norm4PanelHypothesis {
  int panel_id;
  binder::HeightLabel layer;
  double frontend_cost;
  double temporal_prior_logp;
  double z_jump_score;
  double phase_score;
};

struct Norm4DualHypothesis {
  int panel_id_1;
  int panel_id_2;
  binder::HeightLabel layer_1;
  binder::HeightLabel layer_2;
  double frontend_cost;
  double temporal_prior_logp;
  bool adjacent;
};
```

single 观测生成：

```text
4 panels x possible layers
```

默认层规则仍可作为先验，而不是硬规则：

```text
even panel -> lower prior high
odd panel  -> upper prior high
```

dual 观测生成：

```text
ordered adjacent pairs only:
(0,1), (1,2), (2,3), (3,0) and reverse direction candidates
```

当双板 z 差强时，高低顺序作为强先验，但仍保留次优候选用于 UKF likelihood 对比。

### 4.3 后端参与关联

新增后端评估接口：

```cpp
struct MeasurementLikelihood {
  bool valid = false;
  double nis = 0.0;
  double chi2_pos = 0.0;
  double chi2_yaw = 0.0;
  double log_likelihood = 0.0;
  Eigen::VectorXd innovation;
  Eigen::MatrixXd S;
};

MeasurementLikelihood evaluate_single(
    const ObservationData &obs,
    int panel_id,
    binder::HeightLabel layer) const;

MeasurementLikelihood evaluate_dual(
    const ObservationData &obs1,
    const ObservationData &obs2,
    const Norm4DualHypothesis &hyp) const;
```

Norm4 最终候选分数：

```text
score = w_ukf * (-0.5 * nis)
      + w_frontend * (-frontend_cost)
      + w_phase * phase_score
      + w_jump * z_jump_score
      + w_continuity * track2d_score
      + switch_penalty
```

这样 UKF 的统计一致性成为关联的一部分，而不是更新后才发现 NIS 爆炸。

## 5. DualRadiusSpinUKFV2 设计

### 5.1 状态定义

沿用旧状态布局，减少迁移风险：

```text
X = [x, vx, y, vy, z, vz, delta, delta_rate, r1, r2, dza]
```

语义：

```text
center_yaw = compose_yaw(k, delta)
panel_angle(i) = i * pi / 2
r(i) = i even ? r1 : r2
z_offset(layer) = upper ? +dza : -dza
```

建议配置层明确：

```yaml
default_dza_half: 0.03   # 如果实际高低差 6cm，则 half offset 为 3cm
```

如果暂时不改参数名，也必须在文档和 YAML 注释里说明当前 `default_dza` 是 half offset。

### 5.2 单观测模型

观测仍为：

```text
z_single = [x_a, y_a, z_a, yaw_a]
```

预测：

```text
h_single(x, panel, layer) = [
  x_c + r(panel) * cos(center_yaw + panel_angle),
  y_c + r(panel) * sin(center_yaw + panel_angle),
  z_c + sign(layer) * dza,
  center_yaw + panel_angle
]
```

旧实现的 single yaw 观测是 `center_yaw_obs = obs.yaw - panel_angle`。V2 建议观测模型直接使用 armor yaw，即第四维是 `yaw_a`，这样 single 和 dual raw batch 的观测语义统一。

### 5.3 单观测结构阻尼更新

V2 不再完全清零结构行，而是根据置信度阻尼：

```cpp
double gr = config.ukf_v2.structural_gain_r;
double gz = config.ukf_v2.structural_gain_dza;

if (!hypothesis_high_confidence) {
  gr *= 0.2;
  gz *= 0.0;
}

K.row(idx.R1()) *= gr_for_r1;
K.row(idx.R2()) *= gr_for_r2;
K.row(idx.DZA()) *= gz;
```

建议默认：

```yaml
ukf_v2.structural_gain_r_single: 0.05
ukf_v2.structural_gain_dza_single: 0.02
ukf_v2.structural_gain_dza_when_layer_uncertain: 0.0
```

约束：

1. 只更新当前 panel 对应的半径，另一半径弱更新或不更新。
2. `DZA` 只在 layer 置信度高时弱更新。
3. NIS 高但未 reject 的观测，不更新结构参数，只更新中心/yaw 或直接跳过。
4. 初始化前若 `dza` 接近 0，可用结构先验或双观测先验唤醒，而不是依赖 `is_dza_converged()`。

### 5.4 双观测 raw batch 更新

V2 主双观测不再以射线交点为主观测，而是联合原始观测：

```text
z_dual_raw = [
  x1, y1, z1, yaw1,
  x2, y2, z2, yaw2
]
```

预测：

```text
h_dual_raw = [
  h_single(obs1, panel1, layer1),
  h_single(obs2, panel2, layer2)
]
```

R：

```text
R_dual_raw = blockdiag(R_obs1, R_obs2)
```

其中每块的 `R_obs` 由 YPD/YPR 噪声模型生成。若两块来自同一帧同一相机，也可以在第二阶段加入跨观测相关项，第一阶段先 block diagonal。

优势：

1. 保留联合更新，不走 SP 式顺序更新。
2. 不把射线求交误差伪装成高精度中心观测。
3. yaw 误差通过每块 raw observation 的 R 进入滤波器。
4. 双板同时约束中心、yaw、r1/r2/dza，结构参数可观测性更自然。

### 5.5 双观测 geometry pseudo update

旧射线交点逻辑可保留为辅助，不作为主更新。

使用条件：

```text
dual assignment margin 足够高
两条射线夹角足够大
两块观测各自 single chi2 通过
height confidence 足够高
```

pseudo 观测建议只更新结构或低权重中心：

```text
z_geom = [r1_est, r2_est, dza_est]
```

R 根据几何条件放大：

```text
angle = abs(sin(yaw1_to_center - yaw2_to_center))
condition_scale = 1.0 / max(angle, 0.15)
sigma_r_geom = base_sigma_r * condition_scale
sigma_dza_geom = base_sigma_dza / max(height_confidence, 0.2)
```

若继续使用 `[xc, yc, zc, r1, r2, dza]`，必须显著放大 `xc/yc` 噪声，并记录 ray condition，否则仍会出现 NIS 虚高和结构拉偏。

### 5.6 结构先验 pseudo update

借鉴 JLU 的结构先验，给 `r1/r2/dza` 一个弱 prior：

```text
z_prior = [r1_nominal, r2_nominal, dza_nominal]
h_prior = [r1, r2, dza]
R_prior = diag(sigma_r_prior^2, sigma_r_prior^2, sigma_dza_prior^2)
```

触发策略：

1. 初始化后前 N 帧启用。
2. 长时间单观测且结构未收敛时低频启用。
3. panel mismatch correction 后短时间启用。

推荐默认：

```yaml
ukf_v2.enable_structural_prior: true
ukf_v2.structural_prior_period: 5
ukf_v2.sigma_r_prior: 0.05
ukf_v2.sigma_dza_prior: 0.03
```

注意：prior 是弱约束，不应压制真实不同车型结构。更好的长期方案是从 `robot_description` 按车型提供 nominal geometry。

### 5.7 YPD/YPR 噪声建模

采用 ShanghaiTechImp P0 的思路，但建议抽象成独立 builder：

```cpp
class ObservationNoiseBuilder {
 public:
  Eigen::Matrix4d build_xyz_yaw_R(
      const Eigen::Vector3d &predicted_obs_in_target,
      const Eigen::Matrix3d &camera_to_target_R,
      double yaw_sigma,
      double confidence_scale) const;
};
```

关键点：

1. PnP 噪声天然在 camera YPD/YPR 域更接近对角。
2. 若 observation 已经变换到 `odom`，应把 camera frame 下的 XYZ covariance 旋转到 target/world frame。
3. 第一阶段可用当前预测观测近似 camera ray，避免离群观测直接放大或缩小 R。
4. 保留旧 `obs_noise_pos` 作为 fallback。

R 结构：

```text
R_ypd = diag(
  sigma_azi^2,
  sigma_ele^2,
  (sigma_dist_coeff * range)^2,
  sigma_yaw^2
)
R_xyz = J_xyz_ypd * R_ypd * J_xyz_ypd^T
R = [R_xyz, yaw_cov]
```

### 5.8 马氏距离与 gate

V2 gate 不只返回 true/false，还返回诊断：

```cpp
struct GateResult {
  bool pass = true;
  bool yaw_pass = true;
  bool pos_pass = true;
  double chi2_yaw = 0.0;
  double chi2_pos = 0.0;
  double nis = 0.0;
};
```

使用策略：

1. hypothesis scoring 使用连续 likelihood。
2. update gate 使用较宽阈值，避免误拒绝。
3. 连续 reject 触发 pseudo lost。

推荐参数：

```yaml
ukf_v2.enable_gate: true
ukf_v2.gate_chi2_yaw: 9.0
ukf_v2.gate_chi2_pos: 16.0
ukf_v2.gate_reject_streak_to_temp_lost: 3
```

不要再用单一 threshold 同时乘 3 来近似不同自由度，建议显式区分 1 DoF 与 3 DoF。

### 5.9 IUKF 可选增强

IUKF 不作为第一阶段核心。建议只在以下场景启用：

1. 初始化前 5 帧。
2. NIS 高但 gate 未拒绝。
3. maneuver detector 触发。
4. mode 从 ambiguous 切 structured 的前几帧。

默认：

```yaml
ukf_v2.iukf_enable: false
ukf_v2.iukf_max_iterations: 3
ukf_v2.iukf_convergence_eps: 1e-3
```

## 6. Norm4 重构方案

### 6.1 新增 StructuredBackendV2

新增：

```text
include/max_entropy_tracker/filters/dual_radius_spin_ukf_v2.hpp
src/max_entropy_tracker/filters/dual_radius_spin_ukf_v2.cpp
include/max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend_v2.hpp
src/max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend_v2.cpp
```

`Norm4StructuredBackendV2` 对外提供：

```cpp
class Norm4StructuredBackendV2 : public INorm4Backend {
 public:
  void reset(...);
  void predict(double dt);

  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;

  bool update_frame(const Norm4FrameUpdate &frame);

  BackendStateSnapshot snapshot() const override;
  DualRadiusSpinUKFV2 &ukf();
};
```

为了兼容旧接口，`update()` 可以内部包装成 single frame。但 Norm4 主链路应逐步改用 `update_frame()`。

### 6.2 帧级更新输入

```cpp
struct Norm4FrameUpdate {
  std::vector<ObservationData> observations;
  std::vector<Norm4PanelHypothesis> single_hypotheses;
  std::vector<Norm4DualHypothesis> dual_hypotheses;
  int selected_single_hyp = -1;
  int selected_dual_hyp = -1;
  bool use_dual = false;
  double binding_confidence = 0.0;
  double height_confidence = 0.0;
  bool structural_update_allowed = false;
};
```

这样 backend 能知道本帧是 single 还是 dual，避免重复消费。

### 6.3 ObservationFrontend 改造

保留：

1. 主观测选择。
2. panel 角度拓扑。
3. 双观测相邻 pair 枚举。
4. z jump、phase memory、2D evidence 输入。

修改：

1. `build_binding_candidate()` 不再只输出一个 best panel，应输出 top-k distribution。
2. `assign_dual_observations()` 不再直接强制 candidate high confidence，应输出候选列表和 cost margin。
3. `apply_forced_assignment()` 逐步废弃或改名为 `apply_dual_prior()`，只写 prior，不硬改 probability 到 0.85。

### 6.4 BinderBridge 改造

Binder 输入新增：

```text
ukf_log_likelihood
ukf_nis
ukf_chi2_pos
ukf_chi2_yaw
top2_likelihood_margin
```

Binder 输出新增：

```text
structural_update_allowed
update_strength
selected_hypothesis_index
gate_rejected
```

规则建议：

1. bound panel 与 UKF likelihood 都支持时，允许 full update。
2. binder 支持但 UKF NIS 高时，center/yaw damped update，结构不更新。
3. binder 与 UKF 分歧大时，保持 ambiguous 或 pending switch。
4. 连续分歧进入 rebind 或 temp lost，而不是继续硬更新。

### 6.5 ModeFSM 改造

当前 ModeFSM 可以保留，但 evidence 应加入：

1. dual hypothesis likelihood margin。
2. structured backend health。
3. gate reject ratio。
4. structure convergence quality。

进入 STRUCTURED 的条件不应只看双观测、entropy、binder margin，还应要求：

```text
UKF likelihood top1 明显优于 top2
结构参数没有明显发散
NIS rolling median 在合理范围
```

## 7. 与当前实现的差异

| 模块 | 当前实现 | 重构后 |
|---|---|---|
| 前端关联 | 输出单个 best panel/pair | 输出 top-k hypotheses |
| panel/layer | 早期硬绑定 | 作为先验，最终由 UKF likelihood + binder 融合 |
| 单观测结构参数 | 完全冻结 `R1/R2/DZA` | 按置信度阻尼弱更新 |
| 双观测 | 射线几何 6D 主更新 | raw 8D batch 主更新，geometry pseudo 辅助 |
| R 矩阵 | XYZ 固定对角或局部 YPD | 统一 YPD/YPR builder，支持距离相关和坐标旋转 |
| NIS | 更新后调试/机动指标 | hypothesis scoring、gate、状态机健康度 |
| Norm4 structured update | single update 后再 dual update | frame-level single/dual batch，只更新一次 |
| 结构先验 | 主要靠初值和随机游走 | 弱 structural prior，可按车型注入 |
| 错配处理 | mismatch detector 默认 log only | gate streak、binder conflict、patch/reinit 形成闭环 |

## 8. 哪些可以保留

建议保留：

1. `Norm4ArmorTracker` 的模块化方向。
2. `AmbiguousBackend` 与 `StructuredBackend` 双后端思想。
3. `ModeFSM` 的 hysteresis 与 dwell 机制。
4. `PhaseSequenceMemory` 的 anti-pingpong 思路。
5. Binder 的 transition confirm、cooldown、z jump history。
6. `DualRadiusSpinUKF` 的状态布局和 yaw decomposition。
7. UKF 而不是 EKF，保留非线性观测的二阶近似优势。
8. OutputAdapter 的 ambiguous single semantics。
9. Panel mismatch correction 的 patch/reinit 设计，但应默认纳入更完整的健康度闭环后再启用 correction。

建议弱化或替换：

1. `apply_forced_assignment()` 的强置信覆盖。
2. `update_dual()` 中先 single 后 dual 的调用方式。
3. 射线交点 6D 几何主观测。
4. 单观测结构参数完全冻结。
5. 默认 `default_dza = 0.0` 且没有车型结构先验。
6. 单一 heuristic cost 决定 panel。

## 9. 配置建议

新增配置组：

```yaml
norm4_v2:
  structured_backend: "dual_radius_v2"  # legacy | dual_radius_v2
  enable_hypothesis_evaluator: true
  max_single_hypotheses: 4
  max_dual_hypotheses: 8

ukf_v2:
  enable_ypd_noise: true
  ypd_sigma_azi: 0.010
  ypd_sigma_ele: 0.010
  ypd_sigma_dist_coeff: 0.08

  enable_gate: true
  gate_chi2_yaw: 9.0
  gate_chi2_pos: 16.0
  gate_reject_streak_to_temp_lost: 3

  enable_structural_damped_update: true
  structural_gain_r_single: 0.05
  structural_gain_dza_single: 0.02
  structural_gain_dza_when_layer_uncertain: 0.0

  enable_dual_raw_batch: true
  enable_dual_geometry_pseudo: true
  dual_geometry_min_sin_angle: 0.15
  dual_geometry_sigma_r: 0.04
  dual_geometry_sigma_dza: 0.03

  enable_structural_prior: true
  structural_prior_period: 5
  sigma_r_prior: 0.05
  sigma_dza_prior: 0.03

  iukf_enable: false
  iukf_max_iterations: 3
  iukf_convergence_eps: 1e-3
```

旧配置兼容：

1. `ukf.obs_noise_pos` 作为 fallback。
2. `ukf.obs_noise_yaw` 继续作为 yaw std。
3. `ukf.dual_obs_noise_pos` 作为 geometry pseudo fallback。
4. 旧 `tracker.implementation: norm4` 不变，只通过 `norm4_v2.structured_backend` 切后端。

## 10. 实施路线

### Phase 0: 诊断增强，不改行为

目标：确认问题分布。

任务：

1. 给旧 `DualRadiusSpinUKF` 增加 NIS 分量日志：
   - `chi2_pos`
   - `chi2_yaw`
   - `Pzz.diag`
   - `innov`
   - `r1/r2/dza/P(dza)`
   - update type
2. 给 Norm4 debug snapshot 增加：
   - selected hypothesis source
   - dual assignment cost margin
   - gate pass/reject
3. 离线 replay 统计 single/dual NIS 分布。

验收：

1. 能区分 NIS 大来自 position、yaw 还是 z/layer。
2. 能知道 dual update 是否显著拉动结构参数。

### Phase 1: 引入 V2 类但不接主链路

任务：

1. 新建 `DualRadiusSpinUKFV2`。
2. 复用旧 process model、state layout、sigma point helper。
3. 实现 `evaluate_single()`，不做 update。
4. 写最小单元测试或 replay 对比旧预测观测和 V2 预测观测。

验收：

1. 同状态同 hypothesis 下，V2 预测观测符合几何预期。
2. likelihood/NIS 可稳定输出。

### Phase 2: YPD R 与 single update

任务：

1. 实现 `ObservationNoiseBuilder`。
2. 实现 V2 single update。
3. 加入结构阻尼 gain。
4. 默认只在 shadow backend 运行，不影响发布。

验收：

1. shadow NIS 分布比旧 backend 更接近自由度。
2. 结构参数不会快速发散。

### Phase 3: Norm4 hypothesis evaluator

任务：

1. ObservationFrontend 输出 top-k single/dual hypotheses。
2. V2 evaluator 计算每个 hypothesis 的 likelihood。
3. Binder 输入加入 UKF likelihood。
4. 暂不改 selected 行为，只记录“旧选择 vs UKF top1”差异。

验收：

1. 离线能看到错配帧中 UKF top1/top2 margin。
2. 识别 0101 ping-pong 的 likelihood 异常。

### Phase 4: dual raw batch update

任务：

1. 实现 `update_dual_raw_batch()`。
2. Norm4StructuredBackendV2 支持 `update_frame()`。
3. 移除 V2 路径中的 single+dual 重复消费。
4. geometry pseudo 默认关闭或低权重 shadow。

验收：

1. 双观测帧只进行一次 batch update。
2. 双观测 NIS 不再因几何 R 过小大量爆炸。
3. `r1/r2/dza` 能在双观测帧合理收敛。

### Phase 5: 切主链路灰度

任务：

1. 配置 `norm4_v2.structured_backend=dual_radius_v2`。
2. 先只对指定 robot id 或 replay 开启。
3. gate reject streak 接状态机。
4. 根据 NIS rolling median 调整 R/Q。

验收：

1. 线上不出现长期错误观测保活。
2. tracking/temp_lost 状态与 gate reject 有一致行为。
3. 命中率、抖动、NIS、结构收敛均优于旧 backend。

## 11. 风险与回退

### 11.1 结构弱更新可能引入慢漂

控制：

1. 默认 gain 小。
2. 只有高置信 hypothesis 放开。
3. 加 structural prior。
4. 用 rolling NIS 监控结构发散。

### 11.2 YPD R 初始标定不准

控制：

1. 保留 fallback。
2. 先 shadow 统计不同距离的 NIS。
3. 对近距/远距分别看 NIS median，而不是只看总体均值。

### 11.3 Hypothesis 增多导致逻辑复杂

控制：

1. single 最多 4 个。
2. dual 最多 8 个。
3. 每个 hypothesis 只做观测预测与 S 计算，不立即更新状态。
4. 先记录差异，后切行为。

### 11.4 Gate 误拒绝导致掉跟踪

控制：

1. 先 P1-A 统计，不影响状态机。
2. 再启用 reject streak。
3. 单帧拒绝不立刻 lost。

## 12. 推荐最终链路

```text
输入观测
  |
  v
ObservationFrontend 生成 top-k hypotheses
  |
  v
DualRadiusSpinUKFV2 evaluate hypotheses
  |
  v
Binder/PhaseMemory 融合时序与 UKF likelihood
  |
  v
ModeFSM 决定 ambiguous / structured
  |
  v
StructuredBackendV2 frame-level batch update
  |
  v
OutputAdapter 发布 center 或 single semantics
```

这条链路保留了当前实现最有价值的部分：Norm4 模块化、max-entropy/binder 思路、UKF 状态表达、ambiguous/structured 双模式。同时修正当前最危险的部分：前端硬绑定、结构参数不可观测、双观测几何过度自信、NIS 不参与决策、重复消费观测。

## 13. 最小可落地版本

如果只做一个最小闭环，建议范围如下：

1. 新增 `DualRadiusSpinUKFV2`，复用旧状态和 process model。
2. 实现 YPD R 的 single update。
3. 单观测结构 gain 默认 `r=0.05, dza=0.0`。
4. 实现 dual raw batch update。
5. Norm4StructuredBackendV2 使用 `update_frame()`，双观测不再先 single 后 dual。
6. ObservationFrontend 暂时仍给一个 selected panel，但额外记录 top-k NIS，用于后续替换。

这个版本已经可以直接验证两个核心假设：

1. NIS 是否主要由 R 建模和重复更新导致。
2. 结构参数是否需要从“完全冻结”改为“弱可观测”。

如果最小版本有效，再推进完整 hypothesis evaluator 和 binder 融合。
