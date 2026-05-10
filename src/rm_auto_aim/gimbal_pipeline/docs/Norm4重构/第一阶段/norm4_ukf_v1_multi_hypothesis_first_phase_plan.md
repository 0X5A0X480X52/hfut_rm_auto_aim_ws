# Norm4 第一阶段 UKF V1 多假设后端实施计划

## 0. 第一阶段结论

第一阶段只做一个可落地、可回放、可灰度的闭环：

```text
Norm4ArmorTrackerV2
  -> HypothesisGenerator
  -> UKF Backend V1 evaluate all hypotheses
  -> Top1 commit
  -> TopK debug log
  -> dual raw batch update
```

第一阶段不做：

1. InEKF 主链路。
2. StructureSlowUpdater 主链路。
3. 长期多分支滤波。
4. 证据层强剪枝。
5. 旧 `Norm4ArmorTracker` 内部的大改造。

推荐新增一个独立 tracker：`Norm4ArmorTrackerV2`。旧 `Norm4ArmorTracker` 保留为 legacy / A-B 对照。这样可以避免在旧链路中同时维护“前端硬绑定”和“后端多假设裁决”两套相反的逻辑。

## 1. 为什么第一阶段应独立接入

当前 Norm4 链路的主流程仍是：

```text
ObservationFrontend 选择 selected obs
  -> assign_dual_observations 选择一个 pair
  -> BinderBridge 选择 selected panel
  -> BackendPlanner 生成 update plan
  -> StructuredBackend.update(selected)
  -> StructuredBackend.update_dual(obs0, obs1)
```

这和新方案的主流程冲突：

```text
不先选 panel
  -> 枚举所有 panel / pair hypothesis
  -> 后端在同一个 prior 上 evaluate
  -> Top1 通过 gate 后 commit
```

现有接口也不够：

1. `INorm4Backend` 只有 `reset / predict / update / snapshot`，没有 `evaluate / tryUpdate / commit`。
2. `Norm4StructuredBackend::update()` 会立即修改 UKF 状态，不能用于无副作用评估。
3. `BackendPlanner` 当前会在双观测时先 single update 再 dual update，和“一帧只 commit 一次”冲突。
4. 当前 `DualRadiusSpinUKF::update_single()` 会修改 `k_ / last_k_`，并冻结 `r1/r2/dza`。
5. 当前 `DualRadiusSpinUKF::update_dual()` 使用 ray-intersection geometry pseudo measurement，不是 raw batch。

因此第一阶段最佳路线是：

```text
新增 Norm4ArmorTrackerV2
旧 Norm4ArmorTracker 不动或只加 shadow log
通过 tracker.implementation = "norm4_v2" 接入 TrackerManager
默认配置仍保持 adaptive / norm4，不自动切换到 V2
```

## 2. 第一阶段目标

### 2.1 功能目标

必须完成：

1. 单观测枚举 4 个 hypothesis：

```text
obs0 -> panel 0
obs0 -> panel 1
obs0 -> panel 2
obs0 -> panel 3
```

2. 双观测枚举 8 个有序相邻 hypothesis：

```text
obs0 -> 0, obs1 -> 1
obs0 -> 1, obs1 -> 0
obs0 -> 1, obs1 -> 2
obs0 -> 2, obs1 -> 1
obs0 -> 2, obs1 -> 3
obs0 -> 3, obs1 -> 2
obs0 -> 3, obs1 -> 0
obs0 -> 0, obs1 -> 3
```

这等价于对 `01 / 12 / 23 / 30` 做顺序无关评估，但显式保存有序 hypothesis 更方便 debug。

3. 固定 panel 语义：

```text
panel 0 / 2 = lower, 使用 r1
panel 1 / 3 = upper, 使用 r2
```

4. UKF backend V1 支持：

```text
evaluateSingle()
evaluateDual()
tryUpdateSingle()
tryUpdateDual()
commit()
snapshot()
spin_filter()
```

5. 所有 hypothesis 从同一个 predict 后 prior 评估。

6. Top1 commit，TopK 只记录 debug，不做多分支融合。

7. 双观测使用 raw batch 8D 更新：

```text
z_dual = [
  obs0.x, obs0.y, obs0.z, obs0.yaw,
  obs1.x, obs1.y, obs1.z, obs1.yaw
]
```

8. 同一帧只 commit 一次：

```text
单观测帧 -> single update
双观测帧 -> dual raw batch update
```

不得再出现同一帧先 single 再 dual 的主状态更新。

### 2.2 诊断目标

必须记录：

1. 每个 hypothesis 的 panel assignment。
2. `NIS / Mahalanobis / log_likelihood`。
3. `chi2_pos / chi2_yaw`。
4. gate pass / reject reason。
5. Top1 / Top2 margin。
6. Top1 confidence。
7. commit / reject / fallback 决策。
8. commit 后 reconstruction error。
9. posterior sanity 结果。
10. `r1/r2/dza` 和对应协方差摘要。

这些日志要能回答：

```text
旧链路选中的 panel 是否经常不是 UKF Top1？
NIS 爆炸主要来自 position、yaw 还是结构参数？
双观测 raw batch 是否比 geometry pseudo update 稳？
Top1 和 Top2 是否长期接近，说明相位本身不确定？
```

### 2.3 非目标

第一阶段不要求：

1. 完整替换旧 `norm4_v2::SerialTrackerPipeline`。
2. 让 evidence/binder 参与主决策。
3. 处理所有多观测组合。
4. 让结构参数在单观测中完全自由更新。
5. 实现 InEKF 或 GTSAM。

## 3. 推荐新增文件

建议新增以下文件。命名可以微调，但职责边界应保持。

```text
include/max_entropy_tracker/trackers/norm4_v3/
  norm4_tracker_v2.hpp
  norm4_hypothesis_types.hpp
  norm4_hypothesis_generator.hpp
  norm4_hypothesis_selector.hpp
  norm4_structured_hypothesis_backend.hpp
  norm4_ukf_backend_v1.hpp
  norm4_hypothesis_debug.hpp

src/max_entropy_tracker/trackers/norm4_v3/
  norm4_tracker_v2.cpp
  norm4_hypothesis_generator.cpp
  norm4_hypothesis_selector.cpp
  norm4_ukf_backend_v1.cpp
```

如果希望把滤波数学和 tracker 后端分开，可再新增：

```text
include/max_entropy_tracker/filters/dual_radius_spin_ukf_v2.hpp
src/max_entropy_tracker/filters/dual_radius_spin_ukf_v2.cpp
```

建议第一阶段先使用 `Norm4UkfBackendV1` 包含 UKF 数学，等行为稳定后再抽成独立 filter core。这样初版改动更集中。

CMake 当前使用 `file(GLOB_RECURSE ALL_SRCS CONFIGURE_DEPENDS "src/*.cpp" "src/**/*.cpp")` 收集源文件，因此新增普通 `.cpp` 会自动进入库编译。但新增 gtest 仍需要显式加到 `CMakeLists.txt`。

## 4. 数据结构设计

### 4.1 Hypothesis 类型

```cpp
enum class Norm4HypothesisKind {
  Single,
  Dual
};

struct Norm4PanelAssignment {
  int obs_index = -1;
  int panel_id = -1;
};

struct Norm4Hypothesis {
  Norm4HypothesisKind kind = Norm4HypothesisKind::Single;
  std::array<Norm4PanelAssignment, 2> assignments{};
  int assignment_count = 0;

  double prior_log_weight = 0.0;
  std::string debug_name;
};
```

第一阶段 `prior_log_weight` 默认 0。后续 evidence/binder 接入时再赋值。

### 4.2 Evaluation 类型

```cpp
struct Norm4MeasurementEval {
  bool valid = false;
  bool gate_pass = false;

  double nis = 0.0;
  double mahalanobis = 0.0;
  double log_likelihood = 0.0;
  double score = 0.0;

  double chi2_pos = 0.0;
  double chi2_yaw = 0.0;

  Eigen::VectorXd innovation;
  Eigen::MatrixXd S;
  Eigen::VectorXd z_pred;
  Eigen::VectorXd z_obs;

  std::string reject_reason;
};
```

`score` 推荐：

```text
score = log_likelihood + prior_log_weight
```

第一阶段 prior 为 0，所以本质由 UKF likelihood 决定。

### 4.3 Trial 类型

第一阶段可以先用 UKF 专用 trial，避免过早抽象。

```cpp
struct Norm4UkfTrial {
  bool success = false;

  Norm4Hypothesis hypothesis;
  Norm4MeasurementEval eval;

  Eigen::VectorXd x_post;
  Eigen::MatrixXd P_post;
  int k_post = 0;
  std::optional<int> last_k_post;

  double reconstruction_pos_error = 0.0;
  double reconstruction_yaw_error = 0.0;
  bool posterior_sanity_pass = false;
  std::string reject_reason;
};
```

后续接 InEKF 时再改成 `shared_ptr<IBackendTrialState>`。第一阶段不要为未来抽象牺牲可读性。

### 4.4 TopK debug

```cpp
struct Norm4TopKHypothesisDebug {
  Norm4Hypothesis hypothesis;
  Norm4MeasurementEval eval;
  double normalized_weight = 0.0;
};

struct Norm4HypothesisDebugFrame {
  bool valid = false;
  int obs_count = 0;
  bool committed = false;
  bool degraded = false;

  std::vector<Norm4TopKHypothesisDebug> topk;
  double top1_confidence = 0.0;
  double top1_top2_margin = 0.0;
  std::string decision_reason;
};
```

## 5. HypothesisGenerator

### 5.1 输入策略

第一阶段只处理：

```text
obs_count == 1 -> single hypotheses
obs_count >= 2 -> dual hypotheses
```

若 `obs.size() > 2`，第一阶段建议选择前两个高质量观测：

```text
优先 confidence 高
其次 2D bbox 面积大
再其次原始输入顺序
```

不要第一阶段就枚举所有观测二元组合。多观测组合可以作为 Phase 1.5。

### 5.2 单观测枚举

```cpp
std::vector<Norm4Hypothesis> generate_single(int obs_index) {
  for panel_id in [0, 1, 2, 3]:
    hyp.kind = Single
    hyp.assignments = [{obs_index, panel_id}]
}
```

### 5.3 双观测枚举

```cpp
std::vector<Norm4Hypothesis> generate_dual(int obs0, int obs1) {
  pairs = [(0,1), (1,0), (1,2), (2,1),
           (2,3), (3,2), (3,0), (0,3)]
}
```

### 5.4 后续 evidence 入口

第一阶段只保留接口：

```cpp
void attach_prior(
    std::vector<Norm4Hypothesis>* hypotheses,
    const Norm4RuntimeContext& ctx,
    const evidence::ArmorEvidenceFrame* evidence);
```

默认不修改 `prior_log_weight`。后续可让 2D continuity、z layer 统计、phase memory 进入这里。

## 6. UKF Backend V1

### 6.1 状态维度

第一阶段推荐不要把“15 维”作为阻塞条件。更稳的策略是：

```text
Backend V1 使用现有 CompositeProcessModel 的动态 layout。
默认平动沿用 config.motion.translation_model。
旋转第一版可以沿用 CV，也可以通过新配置启用 CA。
```

现有默认平动是 CA，旋转若使用 CV，则状态通常为：

```text
[x, vx, ax, y, vy, ay, z, vz, az, delta, delta_rate, r1, r2, dza]
```

如果旋转启用 CA，则为：

```text
[x, vx, ax, y, vy, ay, z, vz, az,
 delta, delta_rate, delta_acc,
 r1, r2, dza]
```

也就是文档里的 15 维。建议实施顺序：

```text
Phase 1A: 先支持动态维度，默认保持旧 rotation CV。
Phase 1B: 加 norm4_v2.ukf_v1.rotation_model = "CA"，再验证 15 维。
```

这样第一阶段的风险集中在多假设和 raw batch，而不是同时调运动模型。

### 6.2 PredictContext

后端需要一个无副作用评估上下文：

```cpp
struct Norm4UkfPredictContext {
  Eigen::VectorXd x_prior;
  Eigen::MatrixXd P_prior;
  int k_prior = 0;
  std::optional<int> last_k_prior;
  DynamicStateIndex state_idx;
  double timestamp = 0.0;
};
```

所有 hypothesis 都必须使用同一个 `x_prior / P_prior / k_prior`。

### 6.3 Panel profile

集中定义，不要散落在各模块：

```cpp
struct Norm4PanelProfile {
  int panel_id = 0;
  bool upper = false;
  bool use_r2 = false;
  double phase_offset = 0.0;
  double z_sign = -1.0;
};
```

映射：

```text
panel 0: lower, r1, phase 0
panel 1: upper, r2, phase pi/2
panel 2: lower, r1, phase pi
panel 3: upper, r2, phase 3pi/2
```

### 6.4 单观测模型

单观测使用 4D：

```text
z_single = [armor_x, armor_y, armor_z, armor_yaw]
```

预测：

```text
theta_i = center_yaw + phase_offset(i)

x_pred   = x_c + r(i) cos(theta_i)
y_pred   = y_c + r(i) sin(theta_i)
z_pred   = z_c + z_sign(i) * dza
yaw_pred = theta_i
```

残差：

```text
innovation = z_obs - z_pred
innovation_yaw = normalize_angle(obs.yaw - yaw_pred)
```

注意不要使用 `mod pi`。否则 0/2 或 1/3 互跳会被错误消掉。

### 6.5 k / delta 的处理

旧 `DualRadiusSpinUKF::update_single()` 会在 update 开头修改 `k_`。V1 必须改为：

```text
evaluate 时只计算临时 k_candidate，不写 backend 成员。
tryUpdate 成功后把 k_post 存进 trial。
commit(trial) 时才写 k_ / last_k_ / x_ / P_。
```

否则第一个 hypothesis 的评估会污染后续 hypothesis。

### 6.6 单观测结构参数更新

第一阶段建议：

```yaml
norm4_v2:
  ukf_v1:
    single_update:
      structural_gain_r: 0.0
      structural_gain_dza: 0.0
```

也就是 commit 时默认仍冻结结构参数，但 evaluate 必须记录结构相关残差和 NIS。这样能先验证错绑和 raw batch 是否是 NIS 爆炸主因。

当 Top1 稳定后再尝试：

```text
structural_gain_r = 0.02
structural_gain_dza = 0.0
```

单观测不建议第一版更新 dza。

### 6.7 双观测 raw batch

双观测使用 8D raw batch：

```text
z_dual = [
  obs0.x, obs0.y, obs0.z, obs0.yaw,
  obs1.x, obs1.y, obs1.z, obs1.yaw
]
```

给定 hypothesis：

```text
obs0 -> panel i
obs1 -> panel j
```

预测：

```text
h_dual(x, i, j) = [
  h_single(x, i),
  h_single(x, j)
]
```

残差 yaw 分量分别 wrap：

```text
innovation[3] = normalize_angle(obs0.yaw - yaw_pred_i)
innovation[7] = normalize_angle(obs1.yaw - yaw_pred_j)
```

### 6.8 R 设计

第一阶段先使用对角 R，保守可调：

```yaml
norm4_v2:
  ukf_v1:
    sigma_pos_xy: 0.06
    sigma_pos_z: 0.08
    sigma_yaw: 0.12
    dual_raw_R_scale: 1.5
```

构造：

```text
R_single = diag(xy², xy², z², yaw²)
R_dual = dual_raw_R_scale * blockdiag(R_single(obs0), R_single(obs1))
```

如果当前 `config.ukf.enable_ypd_observation_noise` 已验证稳定，可在 Phase 1B 接入 YPD R。第一版不建议同时改变 R 建模和关联主链路。

### 6.9 NIS / likelihood

使用 LLT 或 LDLT，不要显式求逆：

```cpp
auto llt = S.llt();
auto solved = llt.solve(innovation);
double nis = innovation.dot(solved);
double logdet = 2.0 * sum(log(llt.matrixL().diagonal()));
double log_likelihood = -0.5 * (nis + logdet + m * log(2*pi));
```

如果分解失败：

```text
valid = false
reject_reason = "S_not_spd"
```

### 6.10 Gate

第一阶段 gate 分三层：

```text
hard gate:
  失败则 hypothesis 不可 commit

soft ranking:
  即使 hard gate 失败，也记录 score 和 debug

posterior sanity:
  Top1 tentative update 后再检查
```

建议配置：

```yaml
norm4_v2:
  ukf_v1:
    gate:
      single_total_nis: 25.0
      single_pos_chi2: 16.0
      single_yaw_chi2: 9.0
      dual_total_nis: 45.0
      dual_each_pos_chi2: 16.0
      dual_each_yaw_chi2: 9.0
```

这些数值偏宽，适合第一阶段减少误拒绝。正式阈值应通过 rosbag 回放标定。

### 6.11 tryUpdate 与 commit

`evaluate` 只算统计量，不算 posterior 也可以。`tryUpdate` 对 Top1 重新计算或复用中间量，生成 trial：

```text
x_trial = x_prior + K * innovation
P_trial = P_prior - K S K^T
```

然后做：

```text
symmetrize(P_trial)
ensure_positive_definite(P_trial)
apply_constraints(r1, r2, dza)
posterior_sanity_check()
reconstruction_error_check()
```

只有 `commit(trial)` 能写入 backend 主状态。

## 7. HypothesisSelector

### 7.1 主流程

```cpp
bool Norm4HypothesisSelector::process(
    const std::vector<ObservationData>& observations,
    Norm4StructuredHypothesisBackend& backend,
    Norm4SelectorOutput* output)
{
  backend.predict(dt);
  auto ctx = backend.buildPredictContext();

  auto hyps = generator.generate(observations);
  attach_prior_if_enabled(&hyps);

  std::vector<EvalWithHypothesis> evals;
  for (const auto& h : hyps) {
    auto eval = backend.evaluate(ctx, observations, h);
    evals.push_back({h, eval});
  }

  auto ranked = rank(evals);
  auto topk = select_topk(ranked);
  auto decision = decide(topk);

  if (!decision.try_commit) {
    output->degraded = true;
    output->debug = make_debug(topk, decision);
    return false;
  }

  auto trial = backend.tryUpdate(ctx, observations, topk.front().hypothesis);
  if (!trial.success) {
    output->degraded = true;
    output->debug = make_debug(topk, trial.reject_reason);
    return false;
  }

  backend.commit(trial);
  output->committed = true;
  output->debug = make_debug(topk, decision);
  return true;
}
```

### 7.2 排序

排序规则：

```text
先按 valid
再按 gate_pass
再按 score = log_likelihood + prior_log_weight
```

如果所有 hypothesis 都 gate fail：

```text
不 commit
进入 predict_only 或 fallback
```

### 7.3 Top1 confidence

对 TopK 使用 log-sum-exp：

```text
w_i = exp(score_i - logsumexp(score_topk))
confidence_top1 = w_0
margin_12 = score_0 - score_1
```

第一阶段 commit 条件：

```yaml
norm4_v2:
  hypothesis_selector:
    topk: 4
    commit_top1_only: true
    min_top1_confidence: 0.55
    min_top1_top2_margin: 0.0
    ambiguous_margin: 1.0
```

`min_top1_top2_margin` 第一阶段可设为 0，只记录 ambiguity，不阻止 commit。等 replay 看清 Top1/Top2 分布后再收紧。

### 7.4 Ambiguous 决策

第一阶段可以简化：

```text
Top1 gate pass 且 sanity pass:
  commit Top1

Top1 gate pass 但 Top1/Top2 margin 小:
  commit Top1，但输出 confidence 降低，debug 标记 ambiguous

无 gate pass:
  不更新主 UKF，fallback
```

不要第一版就做长期 TopK 分支。

## 8. Norm4ArmorTrackerV2

### 8.1 类职责

`Norm4ArmorTrackerV2` 继承 `BaseTracker`，只负责生命周期和接入：

```text
initialize()
predict()
update()
get_center_position()
get_yaw()
get_radii()
spin_filter()
debug_snapshot()
last_evidence_frame()
```

核心算法委托给：

```text
HypothesisGenerator
HypothesisSelector
UkfBackendV1
Norm4OutputAdapter
EvidenceBuilder
PhaseSequenceMemory
SinglePlate3DBackend (2D-tracker -> 3D-tracker bridge)
```

### 8.2 初始化策略

初始化是第一阶段的一个现实问题：第一帧没有 prior，就无法真正比较 4 个 panel hypothesis。

推荐策略：

```text
如果 obs.panel_id 有值:
  用 obs.panel_id 初始化。

否则如果双观测存在:
  启动 0/1 双初始化 warmup（内部两个假设并行）。

否则:
  使用 panel 0 和 panel 1 双初始化 warmup，
  同时放大 yaw / radius / dza 协方差。
```

初始化来源必须记录到 debug：

```text
init_panel_source = detector | warmup_seed_01 | warmup_seed_fallback
```

第一阶段不建议一开始维护 4 个独立主分支。初始化阶段只维护两个内部 warmup 分支：

```text
H0: panel seed = 0
H1: panel seed = 1
```

`H0/H1` 不直接对外暴露，对外统一走 `ambiguous single` 语义。

建议 warmup 参数：

```yaml
norm4_v2:
  warmup:
    enable_dual_seed_01: true
    warmup_frames: 8
    min_settle_frames: 3
    min_margin_to_commit: 1.5
    min_confidence_to_commit: 0.70
```

### 8.3 update 流程

```cpp
bool Norm4ArmorTrackerV2::update(const std::vector<ObservationData>& obs) {
  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(...);
    return false;
  }

  sync_time_and_predict(obs);
  handle_observation_received(...);

  if (config_.norm4_v2.enable_2d_tracker ||
      config_.norm4_v2.enable_proxy_manager) {
    ctx_.evidence_frame = evidence_builder_.build(obs, timestamp);
  }

  if (warmup_state_.active) {
    // 对外: ambiguous single。对内: H0/H1 shallow update。
    run_warmup_shallow_update(obs, ctx_);
    if (warmup_converged_by_evidence_and_margin()) {
      promote_winner_to_structured();
      warmup_state_.active = false;
    } else {
      publish_ambiguous_single();
      update_time(timestamp);
      increment_frame();
      return true;  // 有观测处理，不视作失败
    }
  }

  Norm4SelectorOutput sel;
  const bool committed = selector_.process(obs, backend_, &sel);

  last_hypothesis_debug_ = sel.debug;

  if (!committed) {
    fallback_update_or_predict_only(obs, sel);
  }

  sync_runtime_from_backend(backend_.snapshot());
  output_adapter_.update_publish_state(...);

  update_time(timestamp);
  increment_frame();
  return true;  // 第一阶段约定: 有观测且链路处理完成即返回 true
}
```

`shallow update` 的定义建议固定：

```text
1) 仅允许单观测更新或保守 dual 更新
2) 单观测结构增益保持 0
3) R 放大，避免 warmup 早期过拟合
4) 不触发结构重置，不做硬切板
```

`warmup -> structured` 的收敛证据建议：

```text
Top1/Top2 margin 连续超过阈值
top1_confidence 连续超过阈值
NIS 连续处于可接受区间
2D continuity 无明显 ID switch
```

若 warmup 超时仍不收敛：

```text
保持 ambiguous single 模式
维持 shallow 更新
延迟 structured 切换
```

模式路由在第一阶段必须固定为：

```text
Mode = AMBIGUOUS:
  对外输出: 2D-tracker -> 3D-tracker 单板链路
  UKF structured 后端: shallow update 或 predict-only

Mode = STRUCTURED:
  对外输出: UKF structured 后端
  2D-tracker -> 3D-tracker 单板链路: shallow update 保温
```

其中单板链路建议复用 `Norm4AmbiguousBackend` 语义（参考
`include/max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp`）：

```text
2D tracker 维护“同一装甲板”语义
  -> 生成单板 3D observation
  -> 单板 3D backend 更新 armor pose/vel/yaw
```

### 8.4 predict 流程

```cpp
void Norm4ArmorTrackerV2::predict(std::optional<double> target_time) {
  double dt = compute_dt(target_time);
  backend_.predict(dt);
  fallback_backend_.predict(dt);
  sync_runtime_from_backend(backend_.snapshot());
  if (target_time) update_time(*target_time);
}
```

### 8.5 spin_filter 兼容

`BaseTracker` 和 `TrackerManager` 仍依赖 `spin_filter()`。第一阶段后端必须暴露一个实现 `SpinFilterInterface` 的 UKF 对象：

```cpp
SpinFilterInterface& Norm4ArmorTrackerV2::spin_filter() {
  return backend_.spin_filter();
}
```

这也是第一阶段不直接上 InEKF 的重要原因。InEKF 后续要么提供兼容 shim，要么先改上层接口。

## 9. 接入现有链路

### 9.1 配置接入

当前 `tracker.implementation` 只允许：

```text
adaptive | norm4
```

第一阶段要扩展为：

```text
adaptive | norm4 | norm4_v2
```

需要改：

```text
src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/core/config.hpp
src/rm_auto_aim/gimbal_pipeline/src/gimbal_pipeline_node.cpp
src/rm_auto_aim/gimbal_pipeline/config/gimbal_pipeline.yaml
```

`gimbal_pipeline_node.cpp` 中需要：

1. 参数声明注释增加 `norm4_v2`。
2. 参数合法性检查允许 `norm4_v2`。
3. warning 文案同步更新。

### 9.2 TrackerManager 接入

`TrackerManager::get_or_create()` 中增加分支：

```cpp
if (config_.tracker.implementation == "norm4_v2") {
  t = std::make_unique<Norm4ArmorTrackerV2>(config_, dt_, enable_osc_);
} else if (config_.tracker.implementation == "norm4") {
  t = std::make_unique<Norm4ArmorTracker>(config_, dt_, enable_osc_);
} else {
  t = std::make_unique<AdaptiveArmorTracker>(config_, dt_, enable_osc_);
}
```

并 include：

```cpp
#include "max_entropy_tracker/trackers/norm4_tracker_v2.hpp"
```

### 9.3 Debug 接入

当前 node 和 replay 里有对 `Norm4ArmorTracker` 的 `dynamic_cast`。第一阶段有两种方案。

推荐方案：

```text
新增 INorm4DebugProvider 接口。
旧 Norm4ArmorTracker 和新 Norm4ArmorTrackerV2 都实现它。
node / replay 只 dynamic_cast INorm4DebugProvider。
```

接口示例：

```cpp
class INorm4DebugProvider {
 public:
  virtual ~INorm4DebugProvider() = default;
  virtual const Norm4DebugSnapshot& debug_snapshot() const = 0;
  virtual const evidence::ArmorEvidenceFrame& last_evidence_frame() const = 0;
};
```

如果想减少第一版改动，也可以临时在 node / replay 中同时 cast：

```cpp
if (const auto* norm4 = dynamic_cast<const Norm4ArmorTracker*>(tracker)) { ... }
else if (const auto* norm4v2 = dynamic_cast<const Norm4ArmorTrackerV2*>(tracker)) { ... }
```

但长期不推荐重复 cast。

### 9.4 Output 接入

第一阶段应复用 `Norm4OutputAdapter`，保持发布语义一致：

```text
center_pos
center_yaw
center_vel
r1/r2/dza
selected_panel_id
ambiguous_single_mode
confidence_scale
armors_offset
```

V2 新增的 confidence 可以映射到：

```text
ctx_.binding_confidence = top1_confidence
ctx_.entropy_norm = 1.0 - top1_confidence
ctx_.max_prob = top1_confidence
```

这样上层不必第一阶段改消息结构。

### 9.5 Fallback 接入

第一阶段 fallback 不是“可选项”，而是必须包含 `2D-tracker -> 3D-tracker` 的 ambiguous 单板降级模式。

```text
Mode = AMBIGUOUS:
  输出链路: 2D-tracker -> 3D-tracker 单板输出
  UKF structured: shallow / predict-only（不作为主输出）

Mode = STRUCTURED:
  输出链路: UKF structured 主输出
  2D-tracker -> 3D-tracker: shallow update 保温（不作为主输出）

若 structured 下所有 hypothesis 失败:
  回退到 ambiguous single 输出链路
  UKF structured 转为 shallow / predict-only
  待证据恢复后再切回 structured
```

为了避免重新引入前端硬绑定，fallback 的 selected obs 只能影响输出保底，不应反向更新 structured 主 UKF。

补充第一阶段审计修复要求：

```text
Tracker::update() 在“有观测但未commit”时返回 true，
避免 TrackerManager 把 reject 帧当成缺观测导致过早 stale/lost。
```

### 9.6 旧 SerialPipeline 的处理

`SerialTrackerPipeline` 第一阶段不接入 V2 主链路。

原因：

1. 它仍然先 selected / binder / intent，再 backend execute。
2. 它依赖 `BackendPlanner`，而 planner 当前会 single + dual 顺序更新。
3. 强行复用会把新 selector 夹在旧 selected 逻辑之后，失去多假设意义。

可以后续重构为：

```text
ObservationFrontendLite
  -> HypothesisSelector
  -> BackendCommitExecutor
```

但这不是第一阶段范围。

## 10. 推荐配置草案

```yaml
tracker:
  implementation: "norm4_v2"  # adaptive | norm4 | norm4_v2

norm4_v2:
  enable_2d_tracker: true
  enable_proxy_manager: true

  mode_routing:
    ambiguous_output: "single_plate_3d"   # single_plate_3d | structured_ukf
    structured_output: "structured_ukf"   # structured_ukf | single_plate_3d
    ambiguous_structured_backend_mode: "shallow_or_predict"
    structured_single_plate_mode: "shallow"

  single_plate_bridge:
    enable: true
    source_semantic: "track2d_id"
    backend_type: "norm4_ambiguous_backend"
    require_semantic_stable_frames: 2

  warmup:
    enable_dual_seed_01: true
    warmup_frames: 8
    min_settle_frames: 3
    min_margin_to_commit: 1.5
    min_confidence_to_commit: 0.70

  hypothesis_selector:
    topk: 4
    commit_top1_only: true
    min_top1_confidence: 0.55
    ambiguous_margin: 1.0
    include_rejected_in_debug: true
    evidence_prior_enable: false

  ukf_v1:
    enabled: true
    force_rotation_ca: false
    dual_raw_batch: true

    sigma_pos_xy: 0.06
    sigma_pos_z: 0.08
    sigma_yaw: 0.12
    dual_raw_R_scale: 1.5

    gate:
      single_total_nis: 25.0
      single_pos_chi2: 16.0
      single_yaw_chi2: 9.0
      dual_total_nis: 45.0
      dual_each_pos_chi2: 16.0
      dual_each_yaw_chi2: 9.0

    single_update:
      structural_gain_r: 0.0
      structural_gain_dza: 0.0

    dual_update:
      structural_gain_r: 0.05
      structural_gain_dza: 0.02

    posterior_sanity:
      max_center_jump: 0.25
      max_yaw_jump: 0.80
      min_r: 0.05
      max_r: 0.50
      max_r_jump: 0.05
      min_dza: 0.0
      max_dza: 0.15
      max_dza_jump: 0.03

  fallback:
    predict_only_on_reject: true
    enable_ambiguous_single_fallback: true
```

第一阶段可以先只把这些配置加到 `UnifiedConfig::Norm4V2Config`，不必一次性全部调通。未使用配置也应打印到 debug 或保留注释，避免后续误以为已经生效。

## 11. 实施拆分

### PR 0：审计修复与运行语义修正

改动：

1. 修正 `update()` 返回语义：有观测且流程执行完即返回 true。
2. 将 single/dual gate、dual_raw_R_scale、commit 阈值配置化。
3. 将 `top1_confidence / top1_top2_margin / reconstruction_error` 纳入 commit policy。
4. 接入 V2 debug 输出：node/replay 支持 V2 或统一 debug provider。
5. 初始化默认策略改为支持 `warmup_seed_01`。

验收：

```text
reject 帧不会导致 TrackerManager 误判缺观测
参数可通过 yaml 调整并生效
TopK/NIS/decision 在 node/replay 可见
```

### PR 1：类型、配置、接入骨架

改动：

1. 新增 hypothesis/debug 类型头文件。
2. 新增 `Norm4ArmorTrackerV2` 空骨架，能编译。
3. 新增 `tracker.implementation = "norm4_v2"` 合法值。
4. `TrackerManager` 能创建 V2。
5. V2 初始化后可以 predict only，不参与复杂更新。
6. node/replay debug 能识别 V2 或至少不崩。

验收：

```text
colcon build --packages-select gimbal_pipeline
tracker.implementation=adaptive 行为不变
tracker.implementation=norm4 行为不变
tracker.implementation=norm4_v2 能启动，不崩溃
```

### PR 2：HypothesisGenerator + TopK debug + Warmup 双种子

改动：

1. 实现单观测 4 hypothesis。
2. 实现双观测 8 ordered hypothesis。
3. 实现 TopK debug 容器。
4. 暂时用 dummy score 或旧 selected panel 生成 debug。
5. 实现 warmup `H0/H1` 双种子管理状态（仅内部可见）。
6. 对外输出 ambiguous single，不暴露内部双分支。
7. 接入 `2D-tracker -> 3D-tracker` 单板桥接骨架（语义源为 `track2d_id`）。
8. 增加模式路由状态机：`ambiguous_output` 与 `structured_output` 可切换。

测试：

```text
test_norm4_hypothesis_generator
  single_count == 4
  dual_count == 8
  dual_pairs == 01/10/12/21/23/32/30/03
  panel 0/2 lower, 1/3 upper
```

验收：

```text
每帧 debug 能看到枚举出的 hypothesis。
warmup 期间对外模式保持 ambiguous single。
ambiguous 下对外输出来自 single-plate 3D backend。
```

### PR 3：UKF Backend V1 evaluateSingle

改动：

1. 实现 `buildPredictContext()`。
2. 实现 `evaluateSingle()`。
3. 支持 yaw wrap。
4. 支持 LLT NIS / likelihood。
5. 不修改 `x/P/k`。

测试：

```text
同一个 prior 下，evaluate 4 个 panel 后 backend 状态不变。
正确 panel 的 NIS 小于明显错误 panel。
yaw residual 不使用 mod pi。
```

验收：

```text
V2 可以在单观测 rosbag 中输出 TopK NIS log。
不 commit 时状态只 predict。
```

### PR 4：tryUpdateSingle + Top1 commit

改动：

1. 实现 `tryUpdateSingle()`。
2. 实现 `commit()`。
3. 实现 posterior sanity。
4. V2 单观测主链路启用 Top1 commit。
5. `spin_filter()` 返回 backend UKF。

测试：

```text
Top1 commit 后 x/P 更新。
gate fail 不更新 x/P。
P 保持对称正定。
结构参数默认不被单观测更新。
commit policy 使用 confidence/margin/reconstruction 三重门控。
```

验收：

```text
单观测普通 rosbag 可以稳定输出 center/yaw。
NIS 日志可用。
```

### PR 5：evaluateDual + dual raw batch

改动：

1. 实现 `evaluateDual()` 8D raw batch。
2. 实现 `tryUpdateDual()`。
3. 双观测帧只执行 dual commit，不再 single + dual。
4. dual R 使用 `dual_raw_R_scale`。
5. dual TopK debug 记录 8 个 ordered hypothesis。
6. structured 下切换主输出为 structured UKF，single-plate backend 进入 shallow 保温。

测试：

```text
dual_count == 8
交换 obs 顺序时，最佳有序 hypothesis 语义一致。
dual update 不调用 single update。
raw batch NIS 可计算。
```

验收：

```text
双观测帧 last_update_type = dual_raw
没有 ray-intersection geometry pseudo update
双观测 NIS 极端值下降或至少可解释
warmup 阶段可基于证据从 ambiguous 收敛到 structured
structured/ambiguous 两种输出路由行为与配置一致
```

### PR 6：replay 对比与阈值标定

改动：

1. offline replay 输出 V2 TopK CSV。
2. 记录旧 Norm4 selected panel 与 V2 Top1 的差异。
3. 统计 single / dual NIS 分布。
4. 统计 reject / fallback 次数。

验收指标建议：

```text
普通 4 装甲车：
  Top1 NIS rolling median 明显低于旧链路。
  NIS 几百/几千的极端帧能被 gate reject 或标记为 wrong hypothesis。
  双观测帧没有重复消费。
  Top1/Top2 接近的帧能被 debug 标记 ambiguous。
```

## 12. 测试计划

### 12.1 单元测试

新增：

```text
test_norm4_hypothesis_generator.cpp
test_norm4_ukf_backend_v1_eval.cpp
test_norm4_hypothesis_selector.cpp
```

测试点：

1. hypothesis 数量与顺序。
2. panel profile。
3. yaw wrap。
4. `evaluate` 无副作用。
5. LLT 分解失败时 reject。
6. gate fail 不 commit。
7. TopK confidence 归一化。
8. dual raw batch 维度和 yaw wrap。

### 12.2 编译测试

```bash
colcon build --packages-select gimbal_pipeline
```

### 12.3 Replay 测试

至少准备三类 bag：

```text
普通 4 装甲车单观测为主
普通 4 装甲车有少量双观测
快速自旋或 yaw/PnP 抖动明显
```

每类输出：

```text
old selected panel
V2 Top1 panel
V2 TopK score
single/dual NIS
reject reason
center/yaw output
r1/r2/dza
```

### 12.4 实车灰度

灰度顺序：

```text
adaptive baseline
norm4 legacy
norm4_v2 predict/debug only
norm4_v2 single commit
norm4_v2 single+dual commit
```

每一步都保留一键回退：

```yaml
tracker:
  implementation: "adaptive"
```

## 13. 第一阶段风险与控制

### 13.1 初始化 panel 错误

风险：

```text
第一帧没有 prior，初始化 panel 可能错。
```

控制：

1. 优先使用 detector `panel_id`。
2. 无 detector id 时放大 yaw / radius / dza 协方差。
3. 第二帧开始用 hypothesis selector 接管。
4. 初始化来源写 debug。

### 13.2 evaluate 污染状态

风险：

```text
如果 evaluate 修改 k_ 或 x/P，TopK 排名失真。
```

控制：

1. `buildPredictContext()` 拷贝 `x/P/k`。
2. evaluate 函数全部 const。
3. 单元测试检查 evaluate 前后状态完全一致。

### 13.3 yaw 语义错误

风险：

```text
把 armor yaw、center yaw、delta 混用，会导致所有 panel 的 yaw residual 失真。
```

控制：

1. hypothesis 内明确 panel phase。
2. 单观测模型使用 armor yaw residual。
3. 所有 yaw residual 只做 `normalize_angle`，不做 `mod pi`。
4. debug 同时输出 `obs_yaw / pred_yaw / yaw_residual`。

### 13.4 dual raw batch R 太小

风险：

```text
双观测 raw batch 虽然比 geometry 稳，但 R 太小仍会强行拉状态。
```

控制：

1. `dual_raw_R_scale` 初始大于 1。
2. dual gate 比 single 更宽，但 posterior sanity 更严格。
3. 双观测结构 gain 初始很小。

### 13.5 BaseTracker 上层强依赖 spin_filter

风险：

```text
TrackerManager / smoother 会直接读取 spin_filter().x() 和 state_idx。
```

控制：

1. 第一阶段 backend 必须是 UKF 并实现 `SpinFilterInterface`。
2. InEKF 不进入第一阶段主链路。
3. 后续 InEKF 必须提供 compatibility shim 或先重构上层接口。

### 13.6 Debug dynamic_cast 断裂

风险：

```text
node / replay 只识别 Norm4ArmorTracker，V2 debug 不显示。
```

控制：

1. 推荐新增 `INorm4DebugProvider`。
2. 或临时同时 cast `Norm4ArmorTrackerV2`。
3. replay CSV 必须覆盖 V2 TopK。

### 13.7 Warmup 双分支长期不收敛

风险：

```text
H0/H1 在噪声较大或遮挡场景下长期接近，
若强制切 structured 容易错误收敛。
```

控制：

1. 设置 `warmup_frames` 和 `min_settle_frames`。
2. 必须同时满足 margin + confidence + NIS 稳定才允许切 structured。
3. 超时不强切，继续 ambiguous single + shallow 更新。
4. 记录 `warmup_reason` 与每帧分支分数，便于回放分析。

### 13.8 2D 语义与单板 3D 语义漂移

风险：

```text
2D tracker 发生 ID switch 或短时重绑时，
single-plate 3D backend 可能跟错板，导致 ambiguous 输出抖动。
```

控制：

1. single-plate bridge 以 `track2d_id` 为主键并加入稳定帧门槛。
2. 对 2D 语义突变设置 cooldown，cooldown 内只 predict 或 shallow。
3. 输出层对 ambiguous 模式使用较低 confidence_scale。
4. 记录 `track2d_id -> panel_id` 映射变更日志，支持回放追因。

## 14. 第一阶段完成定义

第一阶段完成需要满足：

1. `tracker.implementation = "norm4_v2"` 可以独立运行。
2. 单观测帧枚举 4 个 hypothesis。
3. 双观测帧枚举 8 个 ordered hypothesis。
4. 所有 hypothesis 在同一个 prior 上 evaluate。
5. TopK debug 可从 replay 导出。
6. Top1 commit 只发生一次。
7. 双观测使用 raw batch 8D，不使用 geometry pseudo measurement。
8. gate fail 时不污染主 UKF。
9. 有观测但未commit帧不会被上层误判为缺观测失败。
10. 2D-tracker -> 3D-tracker ambiguous 单板降级模式可用。
11. warmup 双种子仅内部维护，对外保持 ambiguous single 语义。
12. 收敛证据满足时可从 ambiguous 切换到 structured。
13. ambiguous 与 structured 的输出路由行为和配置严格一致。
14. 旧 `adaptive` 和旧 `norm4` 行为不受影响。
15. `colcon build --packages-select gimbal_pipeline` 通过。

一句话验收：

```text
第一阶段不是追求最终最优跟踪，而是先建立可信的统计裁判：
同一帧、同一 prior、所有离散 panel 假设都由 UKF 用同一套 NIS / likelihood 评估，
然后只提交一个可解释的 Top1。
```
