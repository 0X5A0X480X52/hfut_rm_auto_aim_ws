# Outpost V3 假设-检验 + InEKF 方案可行性调研

## 结论

可行，且比当前 outpost_v2 更适合形成一个可解释、可调试的估计闭环。

> 保留 outpost_v2 的全部实现，完全重新实现一个 outpost_v3，先在 UKF 上验证假设-检验框架，再替换为 InEKF 核心。

推荐架构是：

```text
多装甲板观测
  -> Outpost 假设生成
  -> 基于同一个 prior 的假设评价
  -> TopK / 置信度 / margin / NIS gate
  -> trial update
  -> posterior sanity / 重构误差检验
  -> commit best trial
```

其中严格滤波核心建议使用结构参数已知的 InEKF：

```text
X = {yaw, p, v, a, yaw_rate, yaw_acc}
```

结构参数，包括三块装甲板的半径、相位和 z offset，全部作为已知常量进入观测模型，不进入状态，不做慢结构参数估计。

## 现有代码基础

当前 `outpost_tracker_v2.cpp` 已经具备可复用的外层组件：

- `ObservationFrontend`：从多观测中选择主观测，并构造 3 个 panel ID 候选的代价和概率。
- `OutpostBinderBridge`：融合历史绑定、z jump、周期性 2dz 证据和绑定置信度。
- `ModeFSM`：在 ambiguous / structured 模式之间切换。
- `OutpostStructuredBackend`：目前封装 `OutpostSpinUKF`，执行带 panel_id 的单假设更新。
- `OutpostOutputAdapter`：把 backend snapshot 转成发布状态。

当前主要问题是：ID 判定和滤波更新耦合较强。`update()` 先通过 binder 得到一个 `selected_panel`，然后直接更新 backend；失败时没有像 norm4_v3 那样从同一 prior 上试验多个候选并拒绝坏 posterior。

`norm4_v3` 的关键模式值得迁移：

- `buildPredictContext()` 固定本帧 prior；
- `generate()` 枚举单/双观测假设；
- `evaluateSingle/evaluateDual()` 只评价，不改状态；
- `tryUpdateSingle/tryUpdateDual()` 在 trial 副本上更新；
- `commit()` 只提交通过检验的 trial；
- 用 confidence、top1-top2 margin、NIS、posterior sanity、重构误差共同决定是否更新。

## Outpost 假设空间设计

Outpost 只有 3 块装甲板，因此假设空间比 norm4 更小。

### 单观测

对 1 个观测枚举 3 个假设：

```text
single_obs0_panel0
single_obs0_panel1
single_obs0_panel2
```

每个假设使用已知结构计算：

```text
z_pred_i = p + Rz(yaw) * r_i
yaw_pred_i = yaw + panel_angle_i
```

其中：

```text
r_i = [radius * cos(panel_angle_i),
       radius * sin(panel_angle_i),
       z_offset_i]
```

### 双观测

对 2 个观测枚举 ordered distinct pairs：

```text
(0,1), (1,0), (1,2), (2,1), (2,0), (0,2)
```

是否只允许相邻 pair 取决于检测器一帧是否可能同时看到非相邻装甲板。前哨站三面 120 度分布，工程上可以先允许所有 `i != j` 的 ordered pairs，然后用几何 gate 和 likelihood 自然淘汰不合理组合。

### 多观测

初版建议只取质量最高的前 2 个观测，与 norm4_v3 当前策略一致。后续可扩展为从 N 个观测里选 1 或 2 个组合：

```text
C(N,1) * 3 + P(N,2) * 6
```

为了控制实时性，实际可限制为前 3 个观测。

> 考虑到可识别性，仅实现单观测即可，基本不可能出现多观测

## 假设评分

每个假设的总分建议为：

```text
score = log_likelihood + prior_log_weight + binder_log_weight
```

`log_likelihood` 来自滤波器创新：

```text
nis = innovation^T S^-1 innovation
log_likelihood = -0.5 * (nis + log(det(S)) + m * log(2*pi))
```

`prior_log_weight` 可来自：

- 当前绑定 ID 的保守偏置；
- `ObservationFrontend` 的候选概率；
- `OutpostBinderBridge` 的周期相位、2dz signature、z audit health；
- panel 切换惩罚。

注意这些权重只参与假设排序，不应该直接改写 InEKF 状态。

## 检验与提交策略

建议沿用 norm4_v3 的三层检验：

1. 评价 gate：
   - `valid`
   - `nis < gate_threshold`
   - 位置创新、yaw 创新分别不过大

2. commit gate：
   - `top1_confidence >= min_top1_confidence`
   - `top1_top2_margin >= min_top1_top2_margin`
   - structured 模式才允许强 commit；ambiguous 模式可 predict-only 或使用弱更新

3. trial gate：
   - trial update 成功；
   - `P` 正定且对角线在合理范围；
   - yaw rate / yaw acc 不超过物理上限；
   - 用 posterior 反算该 panel 观测，`reconstruction_pos_error` 低于阈值。

若本帧所有假设被拒绝，backend 只保持预测态，并把该帧记为 degraded observation，而不是把错误 ID 强行灌入滤波器。

## 早期绑定策略

Outpost 单目标观测下，前期不建议强绑定 Top1 panel ID。更稳妥的策略是：

```text
未稳定前：保留 3 个 panel 假设概率，不强 commit ID
稳定后：只保留 Top1，进入 structured 3D tracker
```

这样做的主要原因是：如果初始化阶段 panel 相位绑定错误，后续的中心 z、center yaw、yaw rate 都可能被错误结构解释拉偏。由于 outpost 的高度跳变序列是 `dz, dz, 2dz` 的周期结构，错误相位在一段时间内也可能形成自洽解释，导致 binder 和 backend 越来越相信错误 ID。

### Ambiguous 阶段

前期建议走 `2D-tracker -> 3D-tracker` 的 ambiguous backend，而不是一开始就把观测强解释为某个中心结构状态。

Ambiguous 阶段职责：

- 跟踪“当前看到的是同一个运动目标”；
- 维护 `P(id=0), P(id=1), P(id=2)`；
- 用 NIS、几何 residual、z jump、周期相位证据更新 belief；
- 不把 Top1 作为真值写入 structured backend；
- 可维护弱 3D shadow state，但 z / yaw 更新必须保守。

推荐流程：

```text
Early ambiguous:
  2D / 弱 3D 跟踪单个观测点
  维护 3 panel belief
  不强 commit 结构 ID

Stabilizing:
  每帧枚举 3 个 panel hypothesis
  用 NIS / likelihood / z jump phase / top1 margin 更新 belief
  允许 TopK 存活

Structured:
  top1 margin、NIS、周期相位证据连续稳定后
  初始化或切换到 3D structured backend
```

### 进入 structured 的条件

不要只看 `top1_prob`，建议使用组合条件：

```text
top1_prob > P_enter
top1_margin > M_enter
best_nis < NIS_gate
second_best_nis - best_nis > delta_nis_gate
period_phase_confidence > C_phase
连续满足 stable_frames
```

其中 `period_phase_confidence` 很关键。单帧高度只能说明观测像哪个高度层，窗口内 `dz, dz, 2dz` 序列才能判断循环相位是否一致。

进入 structured 后，也不建议完全丢掉 TopK。可以由 structured backend commit Top1，同时 debug / audit 逻辑继续评估 Top2 / Top3。一旦出现以下情况，应触发 rebind 或退回 ambiguous：

- 当前 Top1 NIS 连续变差；
- Top2 / Top3 长窗口得分反超；
- `2dz` phase 与当前绑定冲突；
- posterior 重构误差长期偏大。

### 设计原则

前期先解决：

```text
我看到的是不是同一个运动目标
```

不要急着回答：

```text
它是哪一块板
```

只有几何残差、NIS 和 `dz, dz, 2dz` 周期相位同时稳定后，才把 Top1 写入 3D structured backend。

## 观测更新方向

滤波更新时更推荐从中心状态正向预测观测，而不是先把观测反推成中心状态。

推荐形式：

```text
z_pred_i = p + Rz(yaw) * r_i
innovation_i = z_obs - z_pred_i
```

不推荐在正式滤波更新中直接使用：

```text
p_obs_i = z_obs - Rz(yaw_obs) * r_i
```

后者可以用于初始化或粗略打分，但不适合作为主要更新路径。

原因是观测反推中心必须先假定 `panel_id=i`。如果 ID 错误，它会直接构造一个错误但看似合理的中心观测，例如：

```text
center_z = obs_z - z_offset_wrong
```

一旦这个中心观测被写入 backend，后续 candidate 评估会围绕错误中心状态形成偏置。正向观测模型则把 `panel_id` 保持为一个假设，只在该假设通过 NIS、likelihood、posterior sanity 和重构误差检验后才 commit。

因此正式更新建议采用：

```text
prior center state
  -> for each panel hypothesis i:
       z_pred_i = h_i(x_prior)
       residual_i = z_obs - z_pred_i
       NIS_i / likelihood_i
       try update i
       posterior sanity check
  -> choose best valid trial
  -> commit
```

这样错误 ID 只会产生一个候选残差，不会在检验前污染中心状态。

## InEKF 核心模型

### 状态

```text
x = [p, v, a, yaw, yaw_rate, yaw_acc]
```

维度为 12：

```text
p, v, a: 3 + 3 + 3
yaw, yaw_rate, yaw_acc: 1 + 1 + 1
```

### 预测

```text
p_k+1   = p_k + v_k * dt + 0.5 * a_k * dt^2
v_k+1   = v_k + a_k * dt
a_k+1   = a_k
yaw_k+1 = yaw_k + yaw_rate_k * dt + 0.5 * yaw_acc_k * dt^2
rate_k+1 = rate_k + yaw_acc_k * dt
acc_k+1  = acc_k
```

预测误差传播矩阵 `F` 与当前状态无关，因此在结构参数固定、yaw-only 的条件下满足文档中分析的 group-affine / log-linear 误差传播要求。

### 观测

对 panel `i`：

```text
z_i = p + Rz(yaw) * r_i + n
```

可选观测维度：

- 初版：3D 位置观测 `[x, y, z]`，最干净，最符合 InEKF 观测假设；
- 工程增强：加入装甲板 yaw，形成 `[x, y, z, armor_yaw]`，但 yaw 观测需要单独处理 wrap；
- 不建议初版直接用像素重投影做滤波更新，重投影可作为外层重构误差检验。

### 严格性边界

可保持 InEKF 核心干净的条件：

- 不把 radius、z offset、panel phase 放入状态；
- 不在滤波器内部做经验 ID 修正；
- 不在一个滤波器内部混合 IMM；
- 不在预测模型中加入依赖当前残差的经验机动项。

外层的假设、binder、模式 FSM 不破坏 InEKF 核心的 group-affine 性质，因为它们只选择观测关联和是否 commit。

## 建议新增接口

可以新增一个 outpost_v3 backend，接口对齐 norm4_v3：

```cpp
struct OutpostPredictContext {
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  double timestamp;
};

struct OutpostMeasurementEval {
  bool valid;
  bool gate_pass;
  double nis;
  double log_likelihood;
  double score;
  Eigen::VectorXd innovation;
};

struct OutpostTrial {
  bool success;
  bool posterior_sanity_pass;
  double reconstruction_pos_error;
  std::string reject_reason;
  OutpostMeasurementEval eval;
  Eigen::VectorXd x_post;
  Eigen::MatrixXd P_post;
};
```

backend 方法：

```cpp
buildPredictContext()
evaluateSingle(ctx, obs, panel_id)
evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1)
tryUpdateSingle(ctx, obs, panel_id)
tryUpdateDual(ctx, obs0, obs1, panel_id_0, panel_id_1)
commit(trial)
```

这样 `OutpostTrackerV2::update()` 可以改成和 norm4_v3 类似的事务式更新。

## 推荐迁移步骤

### Phase 1：在现有 UKF 上补齐假设-检验框架

先不替换滤波核心，只给 `OutpostStructuredBackend` 增加 evaluate / try / commit 能力。这样可以快速验证：

- outpost 3 panel 假设枚举是否稳定；
- binder prior 是否能提升 ID 稳定性；
- TopK / margin / NIS gate 是否减少错误 commit。

这一阶段风险最低，但还不是严格 InEKF。

### Phase 2：实现 `OutpostInEKFBackend`

新增 InEKF backend，保持与 Phase 1 相同接口。`OutpostTrackerV2` 外层逻辑不需要大改，只替换 structured backend。

重点测试：

- 单观测 ID 错误时是否拒绝；
- 双观测时是否快速收敛 yaw；
- 旋转加速突变时 yaw_acc 是否稳定；
- 丢帧后重捕获是否比当前 UKF 更少跳 ID。

### Phase 3：弱化 ambiguous backend

如果假设-检验表现稳定，可以将 ambiguous 模式简化为：

- 初始化/重捕获阶段的宽 gate；
- commit 不足时的 predict-only；
- 不再长期维护独立 ambiguous backend。

这样 outpost 的状态源会更单一，debug 也更清晰。

## 风险点

- 仅单装甲板观测时，panel ID 与 center yaw 仍然存在离散歧义，必须依赖历史 prior、z offset 和 yaw 连续性。
- yaw 观测来自 PnP 时可能有朝向翻转或 wrap，需要独立 robust gate。
- 如果检测器一帧中多观测排序不稳定，双观测假设不能假设输入顺序可靠，必须枚举 ordered assignments。
- InEKF 的理论优势主要体现在预测误差传播；观测更新仍需要针对 3D PnP 噪声做好 R 建模。
- 当前配置里已有 translation/rotation model 多选项；严格 InEKF 版本应固定 CA + yaw CA，避免把 IMM/Singer 混进核心模型。

## 总体判断

方案值得推进。最稳妥路径是先把 norm4_v3 的假设-检验事务框架迁移到 outpost structured backend，再把 backend 内核从 `OutpostSpinUKF` 替换为结构参数固定的 `OutpostInEKFBackend`。

这样可以把风险拆开：

- 先验证数据关联和 commit 机制；
- 再验证 InEKF 的估计收益；
- 最后再收敛 ambiguous / structured 双 backend 的复杂度。
