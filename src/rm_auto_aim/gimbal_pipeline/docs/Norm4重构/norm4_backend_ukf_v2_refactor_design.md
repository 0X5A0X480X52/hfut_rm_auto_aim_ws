# Norm4 与 Backend UKF V2 重构方案

> 本文档是重整版方案。旧稿保留在
> `norm4_backend_ukf_v2_refactor_design.legacy.md`，用于追溯更完整的审计过程。

## 0. 核心结论

Norm4 的第一阶段重构不建议继续把主要复杂度放在 UKF 之前的数据关联上，也不建议一开始就实现完整的证据融合系统。更稳的最小闭环是：

```text
对离散 panel 假设做完整枚举
  -> UKF 在同一个 prior 上非破坏性 shadow 评估每个假设
  -> 用 NIS / 马氏距离 / likelihood 排序
  -> 保留 TopK shadow
  -> 满足 commit 条件时只提交 Top1 posterior
  -> 不满足时降级为 ambiguous / 2D continuity / predict only
```

核心变化是：前端不再过早裁决 `panel_id / layer / pair`，而是只生成候选、先验和降级参考；真正决定某帧观测是否可信、应该绑定到哪块装甲板，交给后端 UKF 在统一的观测模型和协方差下评估。

这能直接处理当前最可疑的 NIS 爆炸链路：

1. 前端关联一旦错绑，会以高置信把错误观测送进 UKF。
2. 单观测冻结 `r1 / r2 / dza`，结构误差只能被中心和 yaw 吸收。
3. 双观测稀少且几何更新过度自信，容易把结构参数一次拉偏。
4. 双观测路径存在同帧观测重复消费风险。
5. NIS 现在更像日志指标，没有成为关联、更新、降级的主控信号。

V2 的第一性原则是：**先让 UKF 解释观测，再让关联层相信 UKF；不要先让关联层替 UKF 作最终决定。**

## 1. 目标与边界

### 1.1 第一阶段目标

第一阶段要做的是一个最小、可回放、可对照的闭环：

1. 新增 `DualRadiusSpinUKFV2`，保留旧状态表达和主要过程模型。
2. 单观测时枚举 `panel_id = 0, 1, 2, 3`。
3. 双观测时枚举相邻 pair：`01, 12, 23, 30`。
4. 双观测在 UKF 内部做顺序无关评估，即同一个 pair 同时评估两种观测到 panel 的分配。
5. 使用 UKF 预测得到的 innovation、S、NIS、马氏距离、likelihood 对所有假设排序。
6. 只提交 Top1 posterior，但保留 TopK shadow 作为不确定性、输出置信度和下一帧先验。
7. 证据层、2D tracker、旧关联逻辑保留，但第一阶段只作为 prior、参考和降级方案。

### 1.2 暂不做的事情

这些事情有价值，但不应成为第一阶段的阻塞项：

1. 不在 UKF 前做复杂硬裁决。
2. 不一开始就把 TopK posterior 做高斯混合融合进主状态。
3. 不让证据层提前剪掉正确假设，除非只是非常宽松的安全剪枝。
4. 不依赖稀有双观测才能修正全部结构偏差。
5. 不把双观测几何射线交点当作高置信结构量测。

### 1.3 后期目标

当第一阶段的 shadow 评估和 Top1 commit 稳定后，再逐步加入：

1. 证据层提前剪枝，用于减少 hypothesis 数量，而不是替代 UKF 判断。
2. 基于 TopK shadow 的多帧相位记忆。
3. 结构参数弱观测或结构先验的自适应放开。
4. IUKF / iterated update，用于 yaw 非线性和近距离大角速度场景。
5. 更完整的 2D tracker 到 3D tracker 降级与恢复机制。

## 2. 当前链路的主要问题

### 2.1 UKF 前关联过早硬绑定

当前 `Norm4ArmorTracker` 的前端会把检测结果压缩为一个较强的 `candidate_panel_id` 或 selected observation。随后 binder、mode FSM 和 backend 都会沿用这个结果。

风险是：

1. 前端 cost 和 UKF 后端的 innovation covariance 不一致。
2. 前端使用上一帧 backend 给出的 center、yaw、radius、dza；如果 backend 已偏，前端会继续强化偏差。
3. 0/2、1/3 这类同层相位歧义会被提前丢掉。
4. 双观测 pair 一旦选错，结构更新影响比单观测更大。

结论：当前相对复杂的数据关联仍有意义，但它不应再作为最终裁决层。它应该降级为：

1. hypothesis proposer：生成候选。
2. temporal prior：给候选加先验 log weight。
3. fallback：UKF 指标爆炸时，提供 2D continuity 或 ambiguous 模式的保底。
4. later pruning：后期用于提前剪掉极不可能的候选。

### 2.2 单观测冻结结构参数的隐患

旧 `DualRadiusSpinUKF::update_single()` 会对结构参数行清零：

```cpp
K.row(idx.R1()).setZero();
K.row(idx.R2()).setZero();
K.row(idx.DZA()).setZero();
```

这么做能防止单板观测把半径和高度差带飞，但代价很高：

1. `r1 / r2 / dza` 初值错误时，单观测完全不能纠正。
2. 结构误差会被中心位置、yaw、速度吸收。
3. 中心和 yaw 被吸收偏后，又会影响下一帧 panel 关联。
4. 双观测占比少时，结构参数长时间不收敛。
5. 默认 `dza = 0.0` 时，如果收敛判据要求非零高度差，系统可能长期处于“结构未收敛但又无法单观测修正”的状态。

V2 第一阶段仍可以默认弱冻结结构参数，但必须把结构误差暴露出来，至少通过 hypothesis NIS、reject streak 和 shadow 排名反映，而不是让错误静默进入主状态。

### 2.3 双观测几何更新过度自信

旧双观测路径先用两块装甲板 yaw 反向射线求交，再构造：

```text
z_geometry = [xc, yc, zc, r1, r2, dza]
```

风险是：

1. 两条 yaw 射线夹角小时，交点对 yaw 噪声极敏感。
2. PnP 位置误差和 yaw 误差没有完整传播到几何量测协方差。
3. 双观测少，一次高权重错误更新就可能显著拉偏结构。
4. 部分配置项如 yaw 噪声、geometry noise scale 没有真正发挥主控作用。

V2 推荐双观测直接使用 raw batch observation：

```text
z = [x_a, y_a, z_a, yaw_a, x_b, y_b, z_b, yaw_b]
```

由 UKF 对每个 pair hypothesis 预测两块装甲板的原始观测，再用完整 innovation 计算 NIS。几何量可以作为派生诊断或弱先验，不作为第一阶段的主更新。

### 2.4 双观测重复消费

当前 structured 模式可能先对 selected 单观测更新一次，再对同一帧两块装甲板做 dual update。Adaptive tracker 中也有类似“先 single，再 dual”的路径。

V2 应改成 frame-level update：

```text
同一帧单观测 -> single hypothesis update
同一帧双观测 -> dual hypothesis update
同一帧多观测 -> 选择 single 或 dual batch，不重复消费同一块观测
```

同一帧的观测只能进入一次主 posterior。shadow eval 可以评估很多次，但 commit 只能提交一个 posterior。

## 3. 可参考实现与可吸收点

### 3.1 FYT2024 armor_solver

可吸收点：

1. 链路更简单，关联和滤波之间的正反馈较少。
2. 对装甲板跳变有工程化修正，例如 radius / height swap、center reset。
3. 当离散结构明显不一致时，敢于做 patch 或 reset。

对 Norm4 的启发：

1. 保留结构异常时的 reset / partial reset。
2. 不要让错误结构长期靠协方差慢慢恢复。
3. 复杂模型必须有简单保底路径。

### 3.2 JLU2026 armor_tracker

可吸收点：

1. 用结构变量和多装甲板约束表达目标几何。
2. radius、dz 等结构参数有先验，不完全依赖单帧观测。
3. 多观测更像联合约束，而不是逐个贪心更新。

对 Norm4 的启发：

1. 给 `r1 / r2 / dza` 加结构先验和合理范围。
2. 双观测使用联合 batch 模型。
3. TopK shadow 后期可以升级为小窗口结构优化或因子图式校正。

### 3.3 SP2025 auto_aim

可吸收点：

1. YPD / YPR 观测域更符合相机测量误差形态。
2. NIS failure window 可以作为 tracking health 的核心信号。
3. 对结构弱可观测问题更保守。

对 Norm4 的启发：

1. 单观测 R 不应只用 xyz 常量对角阵。
2. 马氏距离和 NIS 应进入 gate、commit、降级，而不是只打印。
3. 需要连续失败窗口，避免单帧误拒绝导致频繁 lost。

### 3.4 ShanghaiTechImp 文档

可吸收点：

1. YPR / YPD 噪声建模。
2. 马氏距离 gating。
3. gate reject streak。
4. 可选 IUKF。

对 Norm4 的启发：

1. 用测量域噪声替代过于理想的世界坐标 xyz 噪声。
2. 单观测和双观测都输出统一质量指标。
3. 后期在非线性强的场景中再启用 IUKF。

## 4. V2 最小闭环设计

### 4.1 输入抽象

每个 3D 装甲板观测至少需要：

```cpp
struct ArmorObs3D {
  Eigen::Vector3d pos_world;
  double yaw_world;
  double stamp;
  int detector_id;        // 原始检测 id，可选
  double detector_score;  // 2D / PnP 置信，可选
};
```

第一阶段不要求前端给出确定 panel id，只要求给出本帧可用观测。

### 4.2 单观测完整枚举

单观测时，枚举所有 4 个 panel：

```text
H_single = [
  obs -> panel 0, layer lower, radius r1
  obs -> panel 1, layer upper, radius r2
  obs -> panel 2, layer lower, radius r1
  obs -> panel 3, layer upper, radius r2
]
```

机器人语义固定为：

```text
panel 0 / panel 2 -> lower layer
panel 1 / panel 3 -> upper layer
```

也就是“02 为 lower，13 为 upper”。这条语义应直接进入 UKF panel profile，而不是由前端动态猜测。

### 4.3 双观测完整枚举

双观测时，枚举相邻 pair：

```text
H_dual_pair = [
  pair 01,
  pair 12,
  pair 23,
  pair 30
]
```

对每个 pair，UKF 内部评估两种观测顺序：

```text
obsA -> i, obsB -> j
obsA -> j, obsB -> i
```

因此双观测最多 8 个 assignment hypothesis。最终只保留其中质量最好的若干个。

注意：`02` 和 `13` 在这里不是双观测 pair，而是 layer 语义。双观测 pair 仍只枚举相邻装甲板。

### 4.4 非破坏性 shadow 评估

所有 hypothesis 都必须从同一个 prior 评估：

```cpp
auto prior = ukf.snapshot();

for (auto& h : hypotheses) {
  auto eval = ukf.evaluate(prior, h);  // 只预测 z, innovation, S, NIS, likelihood
  topk.insert(eval);
}

if (should_commit(topk)) {
  ukf.restore(prior);
  ukf.apply(topk[0].posterior_delta);
} else {
  ukf.restore(prior);
  ukf.predict_only_or_degrade();
}
```

不要在评估第一个 hypothesis 后改变主状态，再用改变后的状态评估下一个 hypothesis。否则排序本身会被前一个假设污染。

建议结构：

```cpp
enum class HypothesisType {
  Single,
  Dual
};

struct PanelAssignment {
  int obs_index;
  int panel_id;
};

struct Hypothesis {
  HypothesisType type;
  std::vector<PanelAssignment> assignments;
  double prior_log_weight = 0.0;  // 来自证据层/2D continuity/phase memory
};

struct HypothesisEval {
  Hypothesis hypothesis;
  Eigen::VectorXd innovation;
  Eigen::MatrixXd innovation_cov;
  double nis = 0.0;
  double mahalanobis = 0.0;
  double log_likelihood = 0.0;
  double posterior_log_weight = 0.0;
  double normalized_weight = 0.0;
  bool gate_pass = false;
};
```

### 4.5 评分指标

对每个 hypothesis：

```text
nu = z - h(x_prior)
S  = HPH' + R
NIS = nu' S^-1 nu
log_likelihood = -0.5 * (NIS + log(det(S)) + m * log(2pi))
posterior_log_weight = log_likelihood + prior_log_weight
```

其中：

1. `nis` 用于统计一致性和 chi-square gate。
2. `mahalanobis = sqrt(nis)` 用于直观阈值和日志。
3. `log_likelihood` 用于 hypothesis 排序。
4. `prior_log_weight` 来自前端证据，但第一阶段权重应小，避免压过 UKF。

### 4.6 Null hypothesis

建议加入一个 null hypothesis：

```text
H_null = 本帧不更新，只 predict
```

它的作用是避免“所有真实 hypothesis 都很差，但系统仍被迫提交 Top1”。当 Top1 的 NIS 已明显爆炸，或者 Top1 与 Top2 没有足够区分度时，允许本帧不更新。

### 4.7 TopK 保留

每帧保留 TopK：

```text
TopK = sort_by(posterior_log_weight).take(K)
```

推荐第一阶段：

```text
single K = 2 或 3
dual   K = 2 或 3
```

TopK 的用途：

1. 计算当前帧关联置信度。
2. 给下一帧 phase memory 提供候选先验。
3. 输出 ambiguity 状态。
4. 作为调试数据，观察错绑时 Top2 是否其实更合理。
5. 后期可升级为多假设短窗口。

### 4.8 置信度计算

用 log-sum-exp 归一化：

```text
w_i = exp(logw_i - logsumexp(logw_topk))
confidence_top1 = w_1
margin_12 = logw_1 - logw_2
```

推荐 commit 条件同时看：

```text
gate_pass(top1)
confidence_top1 > conf_min
margin_12 > margin_min
top1.nis < nis_commit_max
reject_streak 未超限
```

不要只看 Top1 是否最小。若 Top1 和 Top2 很接近，说明相位仍不确定，应该降低更新强度或进入 ambiguous。

### 4.9 Top1 commit + TopK shadow

第一阶段推荐策略：

1. 主 UKF 只提交 Top1 posterior。
2. TopK posterior 不直接融合成主状态。
3. TopK 作为 shadow 保存到 Norm4 tracker 的 phase memory。
4. 输出时同时给出主状态和 ambiguity/confidence。

原因：

1. 四板相位是离散变量，简单高斯融合 TopK 容易把相位平均成不存在的中间状态。
2. Top1 commit 最容易和旧链路对比，也最容易定位 NIS 改善来源。
3. TopK shadow 已足够支持后续剪枝、降级和恢复。

## 5. DualRadiusSpinUKFV2 后端设计

### 5.1 状态定义

第一阶段保留旧状态，降低迁移成本：

```text
x = [
  xc, vxc,
  yc, vyc,
  zc, vzc,
  yaw, yaw_rate,
  r1, r2, dza
]
```

含义：

1. `r1`：lower layer 半径，对应 panel 0 / 2。
2. `r2`：upper layer 半径，对应 panel 1 / 3。
3. `dza`：upper/lower 的 z 偏置差，建议定义为对称偏置或明确的 upper-minus-lower。
4. `yaw`：机器人中心朝向。
5. `panel_id`：离散相位，只在 hypothesis 中出现，不进入连续状态。

### 5.2 Panel profile

固定映射：

```cpp
struct PanelProfile {
  int panel_id;
  bool is_upper;
  int radius_index;    // r1 or r2
  double phase_offset; // 0, pi/2, pi, 3pi/2
  double z_sign;       // lower / upper
};
```

推荐：

```text
panel 0: lower, r1, yaw + 0
panel 1: upper, r2, yaw + pi/2
panel 2: lower, r1, yaw + pi
panel 3: upper, r2, yaw + 3pi/2
```

如果现有代码的相位零点不同，应在 V2 adapter 中兼容旧定义，不要在多个模块里重复写偏移。

### 5.3 单观测模型

对一个 hypothesis `(obs -> panel k)`：

```text
z = [armor_x, armor_y, armor_z, armor_yaw]
h_k(x) = panel_pose(x, k)
```

推荐单观测维度第一阶段使用 4D：

```text
[x, y, z, yaw]
```

其中 yaw 必须参与 gating。只用 xyz 会放大 0/2 或 1/3 的相位歧义。

如果 yaw 噪声暂时不可靠，可以配置成较大，但不建议完全移除。

### 5.4 单观测结构参数策略

第一阶段提供三档：

```text
structure_gain_mode = frozen | weak | adaptive
```

推荐默认：

```text
frozen for commit update
weak for evaluation diagnostics
```

也就是：

1. commit 时仍可保守冻结 `r1/r2/dza`，避免引入新不稳定源。
2. evaluate 时记录如果放开弱结构增益，NIS 是否显著下降。
3. 当连续多帧同一个 panel/layer 稳定、NIS 低、相位置信度高时，再放开弱更新。

后续可改成：

```text
K(row r1/r2) *= 0.02 ~ 0.10
K(row dza)   *= 0.00 ~ 0.05
```

但要配合结构先验，否则单观测仍可能把结构慢慢带偏。

### 5.5 双观测模型

双观测使用 raw batch：

```text
z = [
  obsA.x, obsA.y, obsA.z, obsA.yaw,
  obsB.x, obsB.y, obsB.z, obsB.yaw
]

h_ij(x) = [
  panel_pose(x, i),
  panel_pose(x, j)
]
```

对 pair `ij` 同时评估：

```text
obsA -> i, obsB -> j
obsA -> j, obsB -> i
```

这样 dual update 天然顺序无关。

### 5.6 双观测结构参数策略

双观测比单观测更能约束结构，但也更危险。建议：

1. 不再用射线交点作为主量测。
2. 对 raw batch 使用较保守的 R。
3. 当两块观测 yaw 夹角导致几何条件数很差时，提高 R 或拒绝结构更新。
4. 双观测 commit 后允许更新 `r1/r2/dza`，但加结构先验和范围 clamp。
5. 如果 dual hypothesis 的 NIS 很大，不因为“双观测看起来更强”就强行更新。

### 5.7 观测噪声

推荐第一阶段支持两种 R：

```text
R mode = cartesian | ypd_yaw
```

`cartesian` 用于兼容旧实现；`ypd_yaw` 用于新路径。YPD/YPR 的收益是距离方向和横向角度噪声更接近实际相机误差。

简单实现可以先在观测侧构造局部坐标 R，再旋回世界系：

```text
sigma_yaw: 来自 armor yaw / PnP yaw
sigma_y:   随 distance 增长
sigma_p:   横向角误差转位置误差
sigma_d:   深度误差随 distance 增长
```

如果第一阶段时间有限，至少做到：

1. yaw 进入观测。
2. R 随距离放大。
3. 双观测 R 比单观测更保守，而不是更激进。

### 5.8 结构先验

给结构参数加入软先验：

```text
r1 ~ nominal_r1
r2 ~ nominal_r2
dza ~ nominal_dza
```

推荐用途：

1. 初始化。
2. predict 后的弱约束。
3. NIS 爆炸后的结构恢复。
4. 防止单观测弱更新长期漂移。

先验不是每帧强行 reset，而是像 pseudo measurement 或 covariance regularization 一样温和拉回。

### 5.9 Gate 与健康度

每个 update 输出：

```cpp
struct UkfUpdateReport {
  double nis;
  double mahalanobis;
  double log_likelihood;
  double confidence_top1;
  double margin_12;
  int reject_streak;
  bool committed;
  bool degraded;
};
```

健康度推荐分层：

```text
good:       NIS 合理，Top1 区分明显
ambiguous:  NIS 合理，但 Top1/Top2 接近
suspect:    Top1 NIS 偏大或连续边缘通过
reject:     所有 hypothesis 都不通过 gate
lost:       reject 连续超过窗口
```

## 6. Norm4 链路重构

### 6.1 新模块职责

推荐拆分：

```text
ObservationFrontend
  只负责把 detector / PnP 输出整理成 ArmorObs3D

HypothesisGenerator
  单观测枚举 0..3
  双观测枚举 01/12/23/30 和两种顺序

EvidencePrior
  输出 prior_log_weight
  第一阶段权重小，仅辅助排序

DualRadiusSpinUKFV2
  evaluate hypotheses
  apply selected posterior

Norm4StructuredBackendV2
  frame-level update
  Top1 commit + TopK shadow

ModeFSM
  根据 UKF report 选择 structured / ambiguous / fallback / lost

OutputAdapter
  发布 center、target armor、confidence、ambiguity
```

### 6.2 帧级更新流程

推荐伪代码：

```cpp
Norm4FrameResult Norm4TrackerV2::update_frame(const std::vector<ArmorObs3D>& obs) {
  ukf_.predict(dt);

  auto hypotheses = hypothesis_generator_.enumerate(obs);
  evidence_prior_.attach_prior(hypotheses, phase_memory_, tracker2d_);

  auto evals = ukf_.evaluate_all(hypotheses);
  auto topk = select_topk(evals, config_.topk);
  auto decision = commit_policy_.decide(topk, health_);

  if (decision.commit) {
    ukf_.commit(topk.front());
    phase_memory_.update(topk);
  } else {
    ukf_.restore_prior_after_predict();
    fallback_.update(obs, topk);
  }

  health_.update(topk, decision);
  return output_adapter_.make_result(ukf_, topk, health_);
}
```

### 6.3 证据层第一阶段定位

证据层第一阶段不要负责硬筛选，只提供小权重先验：

```text
prior_log_weight =
  w_phase * phase_continuity_score
+ w_z     * z_layer_consistency_score
+ w_2d    * tracker2d_continuity_score
+ w_det   * detector_score
```

建议：

```text
abs(prior_log_weight contribution) <= 1.0 ~ 2.0
```

这样证据层可以打破轻微平局，但不能压过明显的 UKF innovation。

### 6.4 证据层后期剪枝

当日志证明 UKF 排序稳定后，证据层可以提前剪枝：

1. 剪掉与 2D tracker 明显不连续的观测。
2. 剪掉与 phase memory 明显矛盾的 panel。
3. 剪掉高度语义明显不可能的候选。
4. 在双观测很多时限制 pair 数量。

但必须满足：

1. 保留至少一个 null hypothesis。
2. 保留至少两个相位候选，除非 evidence 极强。
3. 被剪枝的 hypothesis 要记录日志，方便回放分析。

## 7. 输出策略

### 7.1 主输出

主输出来自 Top1 commit 后的 UKF 状态：

```text
center pose
center velocity
yaw / yaw_rate
r1 / r2 / dza
selected target armor
```

### 7.2 置信度输出

同时输出：

```text
association_confidence = confidence_top1
association_margin = margin_12
nis = top1.nis
mode = structured / ambiguous / fallback / lost
topk_panel_ids
```

这能让上层发弹策略区分：

1. 中心跟踪稳定但当前击打装甲板不确定。
2. 当前观测 NIS 爆炸，应保守发弹或暂停。
3. 结构参数未收敛，使用单板语义输出更可靠。

### 7.3 TopK shadow 的使用

TopK shadow 不直接改变主状态，但可以影响：

1. 下一帧 prior。
2. ambiguity mode 的目标选择。
3. debug overlay。
4. reject streak 判定。
5. 后期短窗口平滑。

## 8. 降级方案

### 8.1 触发条件

建议进入降级的条件：

1. Top1 NIS 超过 hard gate。
2. 所有 hypothesis 都 gate fail。
3. Top1 和 Top2 margin 长期过小。
4. `r1/r2/dza` 超出物理范围或协方差异常。
5. 同一类错绑在短时间内反复出现。

### 8.2 降级路径

推荐分层：

```text
structured_good
  -> ambiguous_3d
  -> 2d_continuity_fallback
  -> predict_only
  -> lost
```

含义：

1. `ambiguous_3d`：中心状态仍可信，但 panel id 不可信。
2. `2d_continuity_fallback`：3D NIS 爆炸，用 2D tracker 维持短期目标连续性。
3. `predict_only`：本帧不吃观测，只保留短时间预测。
4. `lost`：连续失败，等待重新初始化。

2D tracker -> 3D tracker 链路建议保留，尤其用于快速旋转、PnP 跳变、遮挡后的短期恢复。

## 9. 与当前实现的区别

| 方面 | 当前实现 | V2 第一阶段 |
| --- | --- | --- |
| panel 关联 | 前端选 best，后端接受 | 前端枚举，UKF 统一评估 |
| 单观测 | 只更新 selected panel | 枚举 0..3，按 NIS/likelihood 排序 |
| 双观测 | 前端选 pair，可能先 single 再 dual | 枚举 01/12/23/30，顺序无关 batch，一帧只 commit 一次 |
| 结构参数 | 单观测冻结，双观测几何更新 | 单观测保守，双观测 raw batch，结构先验约束 |
| NIS | 主要用于日志 | gate、排序、commit、降级核心指标 |
| 证据层 | 容易变成硬绑定 | 第一阶段仅 prior/fallback，后期再剪枝 |
| TopK | 多数情况下丢失 | 保留 shadow，输出置信度 |
| 降级 | 分散在 tracker 逻辑中 | 按 health report 统一驱动 |

## 10. 当前实现中应保留的优势

这些部分值得保留：

1. Norm4 已经把 frontend、binder、mode FSM、backend、output adapter 拆开，模块边界比单体 tracker 更好。
2. `DualRadiusSpinUKF` 的状态表达能描述双半径、上下层高度差和自旋。
3. ambiguous / structured 双模式思路正确，只是切换依据需要更多依赖 UKF 一致性。
4. max-entropy / binder 的思路适合做多假设先验。
5. 当前配置项较丰富，适合把 V2 做成可开关、可回放对比。
6. 2D tracker 到 3D tracker 的链路可以作为 NIS 爆炸时的保底，不应删除。

## 11. 推荐配置

第一阶段新增或重命名配置：

```yaml
norm4_v2:
  enabled: true

  hypothesis:
    enumerate_single_all_panels: true
    enumerate_dual_adjacent_pairs: true
    topk: 3
    include_null_hypothesis: true
    evidence_prior_weight: 0.5

  commit:
    confidence_min: 0.65
    margin_min: 1.0
    nis_soft_gate_scale: 1.0
    nis_hard_gate_scale: 2.0
    reject_streak_to_fallback: 3
    reject_streak_to_lost: 8

  ukf:
    measurement_model: ypd_yaw
    dual_update_model: raw_batch
    dual_order_invariant: true
    single_structure_gain_mode: frozen
    dual_structure_gain_mode: conservative
    enable_structure_prior: true

  structure_prior:
    r1_nominal: 0.26
    r2_nominal: 0.26
    dza_nominal: 0.10
    r_sigma: 0.06
    dza_sigma: 0.06

  fallback:
    enable_2d_continuity: true
    enable_predict_only: true
```

阈值必须通过 rosbag 回放标定。上面只是结构示例，不是最终参数。

## 12. 实施步骤

### P0：只加 shadow 评估，不改变行为

1. 保留旧 Norm4 输出。
2. 新增 hypothesis enumeration。
3. 新增 `DualRadiusSpinUKFV2::evaluate()`。
4. 每帧记录 TopK NIS、likelihood、panel id、pair id。
5. 对比旧 selected panel 是否经常不是 UKF Top1。

验收：

1. 不影响现有行为。
2. 日志能回答 NIS 爆炸时是哪个 hypothesis 爆了。
3. 能看到 Top2/Top3 是否解释了当前错绑。

### P1：启用 Top1 commit，证据层仅 prior

1. structured backend 改为 frame-level update。
2. 单观测枚举 0..3。
3. 双观测枚举 01/12/23/30 和两种顺序。
4. Top1 满足 gate 才 commit。
5. 不满足则 predict only 或 ambiguous。

验收：

1. 删除同帧 single+dual 重复消费。
2. NIS 中位数和极端值明显下降。
3. 错绑时更常进入 ambiguous，而不是强行拉偏主状态。

### P2：引入 YPD/YPR 噪声与结构先验

1. 单观测使用 `[x,y,z,yaw]` 或 YPD+Yaw。
2. R 随距离和 yaw 质量变化。
3. 加结构先验，限制 `r1/r2/dza` 漂移。
4. 双观测 raw batch 允许保守更新结构。

验收：

1. 远距离 NIS 不再系统性偏大。
2. 双观测不会一次性把结构带飞。
3. 结构参数收敛速度和稳定性可解释。

### P3：证据层剪枝和短窗口多假设

1. 使用 phase memory、2D continuity 提前剪掉明显不可能候选。
2. 保留 TopK shadow 的短窗口。
3. 对持续接近的 Top1/Top2 做 ambiguity 输出。
4. 可选 IUKF。

验收：

1. 计算量下降。
2. 快速旋转时相位跳变减少。
3. 不牺牲 P1/P2 已获得的 NIS 一致性。

## 13. 最小可落地版本

如果只做一个最小 PR，建议范围如下：

1. 新增 `DualRadiusSpinUKFV2`，复用旧 state 和 predict。
2. 新增 `evaluate_single(panel_id)`，单观测枚举 0..3。
3. 新增 `evaluate_dual(pair_id, order)`，双观测枚举 01/12/23/30 和两种顺序。
4. 新增 `HypothesisEval`，输出 NIS、马氏距离、log likelihood。
5. 新增 `TopKSelector` 和 `CommitPolicy`。
6. 修改 Norm4 structured backend 为一帧只 commit 一次。
7. 证据层只给 `prior_log_weight`，默认权重很小。
8. NIS hard gate 失败时不更新 UKF，转 ambiguous 或 2D fallback。

这个版本已经能验证最关键的问题：

1. NIS 爆炸是否主要来自错绑和重复消费。
2. 旧前端 selected panel 是否经常不是 UKF 最优解释。
3. 双观测 raw batch 是否比几何射线交点更稳定。
4. 单观测冻结结构是否造成系统性偏差。

## 14. 关键风险与控制

### 14.1 Top1 错误 commit

控制：

1. 加 null hypothesis。
2. 同时看 NIS、confidence、margin。
3. 单帧不确定不 commit。
4. 连续低置信进入 ambiguous。

### 14.2 TopK 复杂度上升

控制：

1. single 最多 4 个。
2. dual 最多 8 个。
3. 第一阶段只做 shadow，不做多 posterior 融合。
4. 后期再由证据层剪枝。

### 14.3 结构弱更新导致漂移

控制：

1. 第一阶段 commit 默认仍可冻结单观测结构。
2. 双观测结构更新保守。
3. 加结构先验和物理范围 clamp。
4. 结构参数变化写入日志。

### 14.4 Gate 过严导致掉跟踪

控制：

1. 使用 soft gate / hard gate 两级。
2. reject streak 而不是单帧 lost。
3. 保留 2D continuity fallback。
4. rosbag 回放标定 chi-square 阈值。

## 15. 推荐最终链路

```text
Detector / PnP
  |
  v
ObservationFrontend
  只输出 ArmorObs3D，不硬判 panel
  |
  v
HypothesisGenerator
  单观测: 0/1/2/3
  双观测: 01/12/23/30 x two orders
  |
  v
EvidencePrior
  phase / z / 2D continuity 仅提供弱先验
  |
  v
DualRadiusSpinUKFV2
  shadow evaluate all hypotheses
  NIS / Mahalanobis / likelihood
  |
  v
TopKSelector + CommitPolicy
  Top1 commit + TopK shadow
  |
  v
ModeFSM
  structured / ambiguous / fallback / lost
  |
  v
OutputAdapter
  center + selected armor + confidence + diagnostics
```

这条链路保留当前 Norm4 的模块化和结构化 UKF 优势，同时把最危险的“前端硬绑定”和“结构参数不可观测正反馈”从主路径中移出去。第一阶段的目标不是把所有证据都融合得很漂亮，而是先建立一个可信的裁判：同一帧、同一 prior、所有离散假设都由 UKF 用同一套统计指标比较。
