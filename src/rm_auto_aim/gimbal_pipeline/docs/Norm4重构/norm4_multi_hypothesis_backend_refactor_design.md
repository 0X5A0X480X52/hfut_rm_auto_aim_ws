# Norm4 多假设跟踪后端重构设计文档

## 0. 文档目的

本文档整理 Norm4 结构化跟踪后端的重构设计，目标是在不一次性推翻现有链路的前提下，逐步实现：

1. 基于多假设枚举的数据关联；
2. 第一版 15 维 UKF 后端，用于验证 NIS / likelihood / dual raw batch update；
3. 第二版 InEKF / IEKF-inspired 位姿估计后端；
4. 结构参数 `r1 / r2 / dza` 从主滤波状态中剥离，改为慢速异步更新；
5. 两种后端对上层 `HypothesisSelector` 透明，方便 A/B 测试与逐步迁移。

核心原则：

```text
HypothesisSelector 只负责“哪个 hypothesis 最可信”；
TrackingBackend 只负责“给定 hypothesis 后如何评估和更新状态”。
```

---

# 1. 总体架构设计

## 1.1 总体分层

推荐整体架构如下：

```text
Detector / PnP
  ↓
ObservationFrontend
  - 生成 ObservationData
  - 不做最终 panel_id 硬绑定
  ↓
HypothesisSelector
  - 单观测枚举 panel 0/1/2/3
  - 双观测枚举 01/10/12/21/23/32/30/03
  - 调用后端 evaluate
  - 根据 NIS / likelihood / gate 选择 Top1
  ↓
IStructuredBackend
  ├── V1: DualRadiusSpinUKFBackend
  └── V2: InvariantPoseBackend + StructureSlowUpdater
  ↓
OutputAdapter
  - 正常结构化输出
  - UKF / InEKF 不可信时降级输出单板 3D tracker
```

## 1.2 关键设计思想

### 1.2.1 前端只生成候选，不做硬裁决

旧设计中，前端如果过早确定 `panel_id / layer / pair`，一旦选错，后端会被高置信错误观测带偏。

新设计中，前端只负责生成候选：

```text
单观测：
  obs -> panel 0
  obs -> panel 1
  obs -> panel 2
  obs -> panel 3

双观测：
  obs0 -> 0, obs1 -> 1
  obs0 -> 1, obs1 -> 0
  obs0 -> 1, obs1 -> 2
  obs0 -> 2, obs1 -> 1
  obs0 -> 2, obs1 -> 3
  obs0 -> 3, obs1 -> 2
  obs0 -> 3, obs1 -> 0
  obs0 -> 0, obs1 -> 3
```

其中机器人装甲板语义固定为：

```text
panel 0 / 2 = lower
panel 1 / 3 = upper
```

第一阶段不枚举 layer，直接由 panel id 决定高度层。

### 1.2.2 后端负责给定假设下的统计评估

后端统一提供：

```text
evaluateSingle()
evaluateDual()
tryUpdateSingle()
tryUpdateDual()
commit()
snapshot()
```

外层不关心后端是 UKF 还是 InEKF，只关心：

```text
NIS
Mahalanobis
log-likelihood
chi2_pos
chi2_yaw
gate_pass
reconstruction_error
posterior_sanity
```

### 1.2.3 第一阶段只 Top1 commit

第一版不维护长期 TopK 分支。

流程为：

```text
枚举所有 hypothesis
  ↓
后端 evaluate
  ↓
按 likelihood / NIS 排序
  ↓
选 Top1
  ↓
Top1 tentative update
  ↓
重构误差检查
  ↓
后验合理性检查
  ↓
通过则 commit
```

但建议记录 TopK debug 信息，包括：

```text
Top1 / Top2 margin
Top1 confidence
每个 hypothesis 的 NIS / chi2 / likelihood
reject reason
```

### 1.2.4 证据层第一阶段不参与主决策

已有的：

```text
2D IOU tracker
2D -> 3D tracker
z 高低统计
bbox 左右关系
```

第一阶段只作为：

```text
debug evidence
降级输出
后续剪枝依据
```

不直接参与 Top1 选择。后续稳定后再逐步引入证据层作为：

```text
提前剪枝
prior score
update strength 控制
```

---

# 2. 统一后端接口设计

## 2.1 接口命名建议

不建议命名为 `IUKFBackend`，因为第二版会接入 InEKF。

推荐：

```cpp
class IStructuredBackend;
class IHypothesisBackend;
class ITrackingBackend;
```

下文使用 `IStructuredBackend`。

## 2.2 核心接口

```cpp
class IStructuredBackend {
public:
  virtual ~IStructuredBackend() = default;

  virtual void reset(const BackendInit& init) = 0;

  virtual void predict(double dt) = 0;

  virtual BackendPredictContext buildPredictContext() const = 0;

  virtual MeasurementEvalResult evaluateSingle(
      const BackendPredictContext& ctx,
      const ObservationData& obs,
      const SingleHypothesis& hyp) const = 0;

  virtual MeasurementEvalResult evaluateDual(
      const BackendPredictContext& ctx,
      const ObservationData& obs0,
      const ObservationData& obs1,
      const DualHypothesis& hyp) const = 0;

  virtual UpdateTrialResult tryUpdateSingle(
      const BackendPredictContext& ctx,
      const ObservationData& obs,
      const SingleHypothesis& hyp,
      const UpdateOptions& options) const = 0;

  virtual UpdateTrialResult tryUpdateDual(
      const BackendPredictContext& ctx,
      const ObservationData& obs0,
      const ObservationData& obs1,
      const DualHypothesis& hyp,
      const UpdateOptions& options) const = 0;

  virtual void commit(const UpdateTrialResult& trial) = 0;

  virtual BackendSnapshot snapshot() const = 0;

  virtual BackendHealth health() const = 0;
};
```

## 2.3 MeasurementEvalResult

```cpp
struct MeasurementEvalResult {
  bool valid = false;
  bool gate_pass = false;

  double nis = 0.0;
  double mahalanobis = 0.0;
  double log_likelihood = 0.0;

  double chi2_pos = 0.0;
  double chi2_yaw = 0.0;

  Eigen::VectorXd innovation;
  Eigen::MatrixXd S;

  Eigen::VectorXd z_pred;
  Eigen::VectorXd z_obs;

  std::string reject_reason;
};
```

## 2.4 UpdateTrialResult

不同后端的 trial state 类型不同：

```text
V1 UKF:
  x_post, P_post

V2 InEKF:
  nominal pose/motion state, error covariance
```

因此建议使用后端私有 trial 句柄：

```cpp
struct UpdateTrialResult {
  bool success = false;

  MeasurementEvalResult eval;
  ReconstructionError recon_error;
  PosteriorSanityResult sanity;

  double score = 0.0;
  std::string reject_reason;

  std::shared_ptr<IBackendTrialState> backend_trial;
};
```

`commit()` 由具体后端解析 `backend_trial`。

## 2.5 BackendSnapshot

两个后端需要对外提供统一 snapshot：

```cpp
struct BackendSnapshot {
  Pose4D robot_pose;

  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;

  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_acc = 0.0;

  double r1 = 0.0;
  double r2 = 0.0;
  double dza = 0.0;

  bool structure_valid = false;
  bool tracking_valid = false;

  // 可选：协方差摘要、NIS rolling、健康度等
};
```

V1 的 `r1/r2/dza` 来自 UKF 状态；V2 的 `r1/r2/dza` 来自结构慢更新器或结构参数提供器。

---

# 3. HypothesisSelector 设计

## 3.1 职责

`HypothesisSelector` 负责：

```text
1. 枚举单观测 / 双观测 hypothesis
2. 调用后端 evaluate
3. 按 log-likelihood / NIS 排序
4. 对 Top1 做 tentative update
5. 检查 reconstruction error
6. 检查 posterior sanity
7. 通过则 commit，否则 reject / degrade
```

不负责：

```text
1. 具体 UKF / InEKF 数学更新
2. 结构参数如何估计
3. P 矩阵如何传播
4. 观测残差具体如何定义
```

## 3.2 单观测 hypothesis

```cpp
struct SingleHypothesis {
  int obs_index;
  int panel_id;  // 0,1,2,3
};
```

第一版枚举：

```text
obs -> panel 0
obs -> panel 1
obs -> panel 2
obs -> panel 3
```

## 3.3 双观测 hypothesis

```cpp
struct DualHypothesis {
  int obs_index_0;
  int obs_index_1;

  int panel_id_0;
  int panel_id_1;
};
```

第一版枚举有序相邻 pair：

```text
01, 10, 12, 21, 23, 32, 30, 03
```

注意：即使想实现“顺序无关”，第一版也建议显式枚举有序假设，方便 debug。

## 3.4 选择流程伪代码

```cpp
void HypothesisSelector::processFrame(
    IStructuredBackend& backend,
    const std::vector<ObservationData>& observations,
    double dt)
{
  backend.predict(dt);

  auto ctx = backend.buildPredictContext();

  auto hypotheses = generateHypotheses(observations);

  std::vector<HypEval> evals;

  for (const auto& hyp : hypotheses) {
    auto eval = evaluateHypothesis(backend, ctx, observations, hyp);

    if (eval.gate_pass) {
      evals.push_back(eval);
    }

    debug_record(eval);
  }

  if (evals.empty()) {
    rejectUpdate();
    maybeDegradeTo2D3DTracker();
    return;
  }

  sortByLikelihood(evals);

  auto best = evals[0];

  UpdateOptions options;
  options.confidence = computeSoftmaxConfidence(evals);
  options.top1_top2_margin = computeMargin(evals);

  if (options.top1_top2_margin < config.ambiguous_margin) {
    options.allow_structure_update = false;
    options.damped_update = true;
  }

  auto trial = tryUpdateBest(backend, ctx, observations, best, options);

  if (!trial.success ||
      !trial.recon_error.pass ||
      !trial.sanity.pass) {
    rejectUpdate();
    maybeDegradeTo2D3DTracker();
    return;
  }

  backend.commit(trial);
}
```

---

# 4. V1：15 维 UKF 后端设计

## 4.1 设计目标

V1 目标是快速落地和验证：

```text
1. 多假设 dry-run 是否可行
2. NIS / likelihood 是否能拒绝错误 panel 假设
3. 0/2、1/3 错误互跳是否能被 yaw / position gate 拒绝
4. 双观测 raw batch update 是否比旧 geometry update 稳定
5. Top1 commit + debug TopK 是否足够
```

V1 不追求最终理论最优，而是作为 baseline。

## 4.2 状态定义

状态维度：

```text
n = 15
```

状态向量：

```text
X = [
  x, vx, ax,
  y, vy, ay,
  z, vz, az,
  delta, delta_dot, delta_ddot,
  r1, r2, dza
]
```

其中：

```text
x,y,z:
  机器人中心位置

vx,vy,vz:
  机器人中心速度

ax,ay,az:
  机器人中心加速度

delta:
  机器人 yaw / 相位角

delta_dot:
  yaw rate

delta_ddot:
  yaw acceleration

r1:
  panel 0 / 2 半径

r2:
  panel 1 / 3 半径

dza:
  高低装甲板高度半差
```

## 4.3 运动模型

三阶匀加速度模型：

```text
p' = p + v dt + 0.5 a dt²
v' = v + a dt
a' = a
```

作用于 `x/y/z`。

yaw 三阶模型：

```text
delta' = delta + delta_dot dt + 0.5 delta_ddot dt²
delta_dot' = delta_dot + delta_ddot dt
delta_ddot' = delta_ddot
```

结构参数随机游走：

```text
r1' = r1
r2' = r2
dza' = dza
```

## 4.4 单观测模型

观测：

```text
z_single = [x_a, y_a, z_a, yaw_a]
```

给定 `panel_id = i`：

```text
panel_angle(0) = 0
panel_angle(1) = pi/2
panel_angle(2) = pi
panel_angle(3) = 3pi/2
```

半径：

```text
r(i) = r1, if i = 0 or 2
r(i) = r2, if i = 1 or 3
```

高度：

```text
z_offset(i) = -dza, if i = 0 or 2
z_offset(i) = +dza, if i = 1 or 3
```

预测：

```text
theta_i = delta + panel_angle(i)

x_a_pred   = x_c + r(i) cos(theta_i)
y_a_pred   = y_c + r(i) sin(theta_i)
z_a_pred   = z_c + z_offset(i)
yaw_a_pred = theta_i
```

即：

```cpp
Vector4d h_single(const StateVec& x, int panel_id);
```

残差：

```text
innovation = z_obs - z_pred
```

其中 yaw 必须 `2π wrap`：

```cpp
innovation[3] = normalize_angle(yaw_obs - yaw_pred);
```

不能用 `mod π`，否则 0/2、1/3 互跳会被错误消掉。

## 4.5 双观测 raw batch 模型

双观测不再使用射线几何求交作为主更新，而是将两个原始观测拼成 8 维 raw batch：

```text
z_dual = [
  x0, y0, z0, yaw0,
  x1, y1, z1, yaw1
]
```

给定有序 hypothesis：

```text
obs0 -> panel i
obs1 -> panel j
```

预测：

```text
h_dual(X, i, j) = [
  h_single(X, i),
  h_single(X, j)
]
```

即：

```cpp
Vector8d h_dual_raw(
    const StateVec& x,
    int panel_id_0,
    int panel_id_1);
```

双观测更新时：

```text
同一帧两个观测只消费一次；
不再先 single update 再 dual update。
```

## 4.6 z_dual 的使用

UKF 中对 sigma points 逐个传播：

```cpp
for each sigma point X_k:
    z_sigma[k] = h_dual_raw(X_k, panel_id_0, panel_id_1);
```

计算观测均值：

```text
z_pred = weighted_mean(z_sigma)
```

计算残差：

```text
innovation = z_dual - z_pred
```

其中：

```cpp
innovation[3] = normalize_angle(z_dual[3] - z_pred[3]);
innovation[7] = normalize_angle(z_dual[7] - z_pred[7]);
```

计算：

```text
S   = Σ wc_k * dz_k * dz_kᵀ + R
Pxz = Σ wc_k * dx_k * dz_kᵀ
K   = Pxz * S⁻¹
```

更新：

```text
x_post = x_pred + K * innovation
P_post = P_pred - K S Kᵀ
```

这里：

```text
S:   8 x 8
Pxz: 15 x 8
K:   15 x 8
```

## 4.7 噪声 R 设计

第一版使用对角 R：

```text
R_single = diag(
  sigma_x²,
  sigma_y²,
  sigma_z²,
  sigma_yaw²
)
```

双观测：

```text
R_dual = blockdiag(R_obs0, R_obs1)
```

由于双观测来自同一帧同一相机，误差存在相关性，第一版建议保守放大：

```text
R_dual = dual_raw_R_scale * blockdiag(R_obs0, R_obs1)
```

推荐初始配置：

```yaml
ukf_v1:
  sigma_pos_xy: 0.05
  sigma_pos_z: 0.06
  sigma_yaw: 0.10
  dual_raw_R_scale: 1.5
```

## 4.8 NIS / likelihood

```text
NIS = innovationᵀ S⁻¹ innovation
```

使用分解求解，避免显式求逆：

```cpp
auto solved = llt.solve(innovation);
double nis = innovation.dot(solved);
```

log-likelihood：

```text
log_likelihood =
-0.5 * NIS
-0.5 * logdet(S)
-0.5 * m * log(2π)
```

排序时可省略常数项：

```text
score = -0.5 * NIS - 0.5 * logdet(S)
```

## 4.9 Gate 设计

不只看 total NIS，应拆分：

```text
position chi2
yaw chi2
total NIS
```

单观测：

```yaml
single_chi2_pos: 16.0
single_chi2_yaw: 9.0
single_nis_total: 25.0
```

双观测：

```yaml
dual_each_pos_chi2: 16.0
dual_each_yaw_chi2: 9.0
dual_total_nis: 45.0
```

双观测还应分别检查 obs0、obs1，避免一个观测很好、另一个观测很差被总 NIS 掩盖。

## 4.10 结构参数更新策略

V1 可以保守更新结构参数。

单观测：

```yaml
single_update:
  structural_gain_r: 0.02
  structural_gain_dza: 0.0
```

双观测：

```yaml
dual_update:
  structural_gain_r: 0.10
  structural_gain_dza: 0.05
```

当 Top1/Top2 margin 小，或者 confidence 不高时：

```text
禁止结构参数更新
只更新 center / delta 相关状态
```

实现上可对 Kalman gain 的结构行缩放：

```cpp
K.row(idx.R1()) *= structural_gain_r;
K.row(idx.R2()) *= structural_gain_r;
K.row(idx.DZA()) *= structural_gain_dza;
```

## 4.11 Reconstruction Error

更新后用 `x_post` 重建装甲板观测：

```text
z_rebuild = h_single(x_post, panel_id)
```

单观测检查：

```text
pos_error = ||p_obs - p_rebuild||
yaw_error = abs(wrap(yaw_obs - yaw_rebuild))
```

双观测分别检查 obs0 / obs1：

```text
max_pos_error
max_yaw_error
```

建议第一版作为 gate：

```yaml
reconstruction_gate:
  single_max_pos_error: 0.12
  single_max_yaw_error: 0.25
  dual_each_max_pos_error: 0.15
  dual_each_max_yaw_error: 0.30
```

## 4.12 Posterior Sanity

中文称为：

```text
后验合理性检查
```

检查 UKF 为了解释当前观测，是否把状态拉飞。

主要检查：

```text
center jump
delta jump
delta_dot / delta_ddot 是否异常
r1/r2/dza 是否超范围
r1/r2/dza 单帧变化是否过大
P 是否正定 / 半正定
P 是否异常塌缩
```

示例配置：

```yaml
posterior_sanity:
  max_center_jump: 0.20
  max_delta_jump: 0.60
  max_delta_dot: 20.0
  max_delta_ddot: 200.0

  min_r: 0.05
  max_r: 0.50
  max_r_jump: 0.05

  min_dza: 0.0
  max_dza: 0.12
  max_dza_jump: 0.03
```

---

# 5. V2：InEKF 位姿后端 + 慢结构参数设计

## 5.1 设计目标

V2 的目标是将结构参数从主滤波器中剥离：

```text
主 InEKF / IEKF-inspired 后端：
  高频估计机器人 pose/motion

StructureSlowUpdater：
  低频估计 r1/r2/dza
```

这样主滤波器中，给定结构参数后：

```text
T_WA_i = T_WR · T_RA_i(s_hat)
```

其中 `T_RA_i(s_hat)` 在短时间内可视为已知常量，因此观测残差更接近不变观测形式。

## 5.2 主状态设计

主滤波器状态：

```text
X_robot = {
  T_WR,
  v,
  a,
  omega,
  alpha
}
```

含义：

```text
T_WR:
  机器人中心 4DoF pose
  包含 x,y,z,yaw
  pitch/roll 先验为 0

v:
  [vx, vy, vz]

a:
  [ax, ay, az]

omega:
  yaw_rate

alpha:
  yaw_acc
```

结构参数不进入主状态：

```text
s = [r1, r2, dza]
```

由 `StructureProvider` 提供。

## 5.3 位姿群设计

可使用 yaw-only 位姿群：

```text
SE_yaw(3) ≈ SO(2)_yaw ⋉ R³
```

也可理解为：

```text
SE(2) + z
```

误差状态可设计为：

```text
δx = [
  δp_x, δp_y, δp_z,
  δyaw,
  δv_x, δv_y, δv_z,
  δa_x, δa_y, δa_z,
  δω,
  δalpha
]
```

误差维度：

```text
12
```

工程上可先做 error-state IEKF-inspired 实现，不必第一版就证明严格 group-affine。

## 5.4 运动模型

仍使用三阶运动模型：

```text
p' = p + v dt + 0.5 a dt²
v' = v + a dt
a' = a

yaw'   = yaw + omega dt + 0.5 alpha dt²
omega' = omega + alpha dt
alpha' = alpha
```

误差协方差传播：

```text
P' = F P Fᵀ + Q
```

平动误差传播：

```text
[δp']   [I  dtI  0.5dt²I] [δp]
[δv'] = [0   I     dtI  ] [δv]
[δa']   [0   0      I   ] [δa]
```

yaw 误差传播：

```text
[δyaw']   [1  dt  0.5dt²] [δyaw]
[δomega']=[0  1   dt    ] [δomega]
[δalpha'] [0  0   1     ] [δalpha]
```

## 5.5 观测模型

装甲板观测：

```text
T_WA_obs
```

给定 hypothesis：

```text
panel_id = i
```

由结构参数得到机器人系下装甲板变换：

```text
T_RA_i = T_RA_i(s_hat)
```

预测关系：

```text
T_WA_pred = T_WR_hat · T_RA_i
```

使用不变残差：

```text
e_i = Log( T_RA_i^-1 · T_WR_hat^-1 · T_WA_obs )
```

只取 yaw-only 4D：

```text
e_i = [
  ρ_x,
  ρ_y,
  ρ_z,
  φ_yaw
]
```

双观测时：

```text
e_dual = [
  e_i,
  e_j
]
```

即 8 维残差。

## 5.6 与 V1 的接口一致

对 Selector 来说：

```text
evaluateSingle()
evaluateDual()
tryUpdateSingle()
tryUpdateDual()
commit()
```

仍然相同。

区别只在内部：

```text
V1:
  sigma point UKF + xyz/yaw residual

V2:
  error-state InEKF / IEKF-inspired update + invariant residual
```

输出仍然是：

```text
NIS
log_likelihood
chi2_pos
chi2_yaw
gate_pass
reconstruction_error
posterior_sanity
```

## 5.7 结构参数提供器

为方便从 UKF snapshot 平滑迁移到独立结构估计器，设计接口：

```cpp
class IStructureProvider {
public:
  virtual StructureParams current() const = 0;
  virtual StructureConfidence confidence() const = 0;
};
```

第一阶段实现：

```cpp
class UkfSnapshotStructureProvider : public IStructureProvider {
public:
  StructureParams current() const override {
    auto snap = ukf_backend.snapshot();
    return {snap.r1, snap.r2, snap.dza};
  }
};
```

后续替换：

```cpp
class RmStructureProvider : public IStructureProvider;
class GtsamStructureProvider : public IStructureProvider;
```

InEKF 后端永远只依赖 `IStructureProvider`，不关心结构参数来自哪里。

---

# 6. StructureSlowUpdater 设计

## 6.1 设计目标

结构参数：

```text
s = [r1, r2, dza]
```

是慢变量，不应被单帧观测快速拉动。

慢更新器目标：

```text
1. 使用累计高置信窗口样本
2. 低频、小步长更新结构参数
3. 避免错误 hypothesis 污染结构
4. 给主滤波器提供结构参数和置信度
```

## 6.2 样本定义

每个样本记录：

```cpp
struct StructureSample {
  Pose4D T_WR_hat;
  Pose4D T_WA_obs;

  int panel_id;

  double confidence;
  double nis;
  double reconstruction_error;

  bool is_dual_frame;
  double timestamp;
};
```

样本进入窗口条件：

```text
Top1 hypothesis 通过 gate
Top1 / Top2 margin 足够大
reconstruction error 小
posterior sanity 通过
当前不是 degraded mode
2D tracker 无明显 ID switch
```

双观测样本权重大，单观测样本降权。

## 6.3 RM 慢更新

结构残差：

```text
e_k(s) = Log( T_RA_i(s)^-1 · T_WR_hat_k^-1 · T_WA_obs_k )
```

取 yaw-only 4D：

```text
e_k = [dx, dy, dz, dyaw]
```

目标：

```text
min_s Σ ρ( || e_k(s) ||²_R )
    + ||s - s_prior||²
```

RM / Gauss-Newton 小步长更新：

```text
H = Σ J_kᵀ R_k⁻¹ J_k + H_prior
g = Σ J_kᵀ R_k⁻¹ e_k + g_prior

Δs = - (H + λI)⁻¹ g

s_new = Proj( s_old + α Δs )
```

其中：

```text
α:
  Robbins-Monro 小步长

Proj:
  物理范围投影
```

示例配置：

```yaml
structure_rm:
  enable: true

  window_size: 30
  min_samples: 10
  min_dual_samples: 3

  update_period_frames: 10

  alpha_init: 0.2
  alpha_min: 0.02
  alpha_decay: 0.995

  lambda_damping: 1.0e-3

  max_r_step: 0.005
  max_dza_step: 0.003

  min_r: 0.05
  max_r: 0.50
  min_dza: 0.0
  max_dza: 0.12
```

## 6.4 结构更新健康度

更新前后检查：

```text
H condition number
窗口样本数量
dual 样本数量
panel 覆盖度
更新前后 loss 是否下降
更新量是否超限
结构参数是否接近边界
```

若不健康：

```text
不更新结构参数
继续积累样本
```

## 6.5 GTSAM 可选替代

后续也可将 RM 替换为 GTSAM：

```text
min_s Σ || Log( T_RA_i(s)^-1 · T_WR_hat_k^-1 · T_WA_obs_k ) ||²
      + ||s - s_prior||²
```

第一版建议 RM，因为更轻量、更容易在线运行。

---

# 7. Phase 3 Shadow InEKF 设计

## 7.1 目的

在不影响主链路的情况下验证 InEKF 后端可行性。

主链路仍为：

```text
V1 15维 UKF
```

并行 shadow：

```text
V2 InEKF 位姿后端
```

InEKF 的结构参数暂时来自 UKF snapshot：

```text
s_hat = [r1, r2, dza] from V1 UKF
```

InEKF 不输出、不控制、不影响 Selector，只记录日志。

## 7.2 Phase 3A：跟随 UKF Top1

流程：

```text
UKF 主链路完成 Top1 commit
  ↓
取 UKF snapshot 中的 r1/r2/dza
  ↓
InEKF shadow 使用同一个 Top1 hypothesis
  ↓
执行 evaluate / tryUpdate
  ↓
只记录 debug
```

目标：

```text
验证 InEKF 残差和 update 是否数值稳定
```

## 7.3 Phase 3B：InEKF 独立 evaluate 所有 hypothesis

流程：

```text
UKF evaluate all hypotheses -> UKF Top1
InEKF evaluate all hypotheses -> InEKF Top1
```

但主链路仍使用 UKF Top1。

记录：

```text
UKF Top1 vs InEKF Top1
UKF NIS vs InEKF NIS
UKF margin vs InEKF margin
pose difference
```

用于判断 InEKF 是否具备替代主后端的条件。

## 7.4 Phase 3C：接入 StructureProvider

InEKF 不直接从 UKF 类取结构参数，而是依赖：

```text
IStructureProvider
```

初始 provider 仍由 UKF snapshot 实现，后续可替换成 RM / GTSAM。

---

# 8. 演进计划

## Phase 0：诊断增强

目标：

```text
不改行为，只增强日志
```

任务：

```text
1. 记录 NIS / chi2_pos / chi2_yaw
2. 记录 innovation / S.diag
3. 记录 TopK hypothesis 评估结果
4. 记录 r1/r2/dza 与其协方差
5. 记录 dual update 是否重复消费观测
```

验收：

```text
能区分 NIS 爆炸来自 position、yaw 还是结构参数。
```

## Phase 1：V1 UKF 后端 + Top1 Selector

任务：

```text
1. 实现 IStructuredBackend
2. 实现 15维 DualRadiusSpinUKFBackend
3. 实现 HypothesisSelector
4. 单观测枚举 4 个 panel
5. 双观测枚举 8 个有序相邻 pair
6. evaluate all hypotheses
7. Top1 commit
8. TopK debug log
```

验收：

```text
1. 正确 hypothesis 下 NIS 基本合理
2. 0/2、1/3 错误互跳能被 gate 或 posterior sanity 拒绝
3. Top1 选择不会频繁异常跳变
```

## Phase 2：dual raw batch + sanity gate

任务：

```text
1. 实现 dual raw 8D batch update
2. 禁止同帧 single + dual 重复消费
3. 加 reconstruction error gate
4. 加 posterior sanity gate
5. 保守控制结构参数更新
```

验收：

```text
1. 双观测帧只更新一次
2. dual NIS 不再因 geometry pseudo 过度自信而爆炸
3. r1/r2/dza 不被单帧错误观测拉飞
```

## Phase 3：InEKF Shadow

任务：

```text
1. 实现 InvariantPoseBackend
2. 结构参数来自 UKF snapshot
3. InEKF 不进入主链路
4. Phase 3A 跟随 UKF Top1
5. Phase 3B 独立 evaluate all hypotheses
6. Phase 3C 使用 IStructureProvider
```

验收：

```text
1. InEKF residual 数值稳定
2. InEKF NIS 与 UKF 可对比
3. InEKF Top1 与 UKF Top1 差异可解释
4. pose_diff 不异常发散
```

## Phase 4：结构参数慢更新器

任务：

```text
1. 实现 RM StructureSlowUpdater
2. 仅使用高置信样本窗口
3. 低频小步长更新 r1/r2/dza
4. 输出结构参数置信度
5. 与 UKF snapshot provider 并行对比
```

验收：

```text
1. 结构参数更新平滑
2. 窗口 residual 更新后下降
3. 结构参数不越界、不快速漂移
4. 使用 RM 参数的 InEKF shadow 更稳定
```

## Phase 5：后端切换灰度

任务：

```text
1. backend_type = ukf15 | invariant_pose
2. 先 replay，后实车灰度
3. InEKF + RM 进入主链路
4. 保留 UKF15 作为回退后端
```

验收：

```text
1. tracking 稳定性不低于 V1
2. NIS rolling median 更合理
3. 结构参数更稳定
4. 降级次数不增加
5. 命中效果或瞄准稳定性提升
```

## Phase 6：证据层参与剪枝

任务：

```text
1. 2D tracker continuity 参与 prior score
2. 3D tracker z stats 参与 height consistency
3. 左右关系参与 dual pair 排序
4. 证据层只在高置信时 hard reject
```

验收：

```text
1. hypothesis 数量减少
2. Top1/Top2 margin 增大
3. 错误切换减少
4. 不引入新的硬规则误杀
```

---

# 9. 推荐配置草案

```yaml
structured_backend:
  type: "ukf15"     # ukf15 | invariant_pose
  enable_shadow_inekf: false

hypothesis_selector:
  max_single_hypotheses: 4
  max_dual_hypotheses: 8
  commit_top1_only: true
  log_topk: 4
  ambiguous_margin: 2.0
  confidence_temperature: 2.0

ukf_v1:
  sigma_pos_xy: 0.05
  sigma_pos_z: 0.06
  sigma_yaw: 0.10
  dual_raw_R_scale: 1.5

  gate:
    single_chi2_pos: 16.0
    single_chi2_yaw: 9.0
    single_nis_total: 25.0

    dual_each_pos_chi2: 16.0
    dual_each_yaw_chi2: 9.0
    dual_total_nis: 45.0

  single_update:
    structural_gain_r: 0.02
    structural_gain_dza: 0.0

  dual_update:
    structural_gain_r: 0.10
    structural_gain_dza: 0.05

reconstruction_gate:
  single_max_pos_error: 0.12
  single_max_yaw_error: 0.25
  dual_each_max_pos_error: 0.15
  dual_each_max_yaw_error: 0.30

posterior_sanity:
  max_center_jump: 0.20
  max_delta_jump: 0.60
  max_delta_dot: 20.0
  max_delta_ddot: 200.0

  min_r: 0.05
  max_r: 0.50
  max_r_jump: 0.05

  min_dza: 0.0
  max_dza: 0.12
  max_dza_jump: 0.03

inekf_v2:
  residual_type: "lie_yaw_only"
  use_structure_provider: true
  structure_provider: "ukf_snapshot"  # ukf_snapshot | rm | gtsam

structure_rm:
  enable: false
  window_size: 30
  min_samples: 10
  min_dual_samples: 3
  update_period_frames: 10

  alpha_init: 0.2
  alpha_min: 0.02
  alpha_decay: 0.995

  lambda_damping: 1.0e-3

  max_r_step: 0.005
  max_dza_step: 0.003

  min_r: 0.05
  max_r: 0.50
  min_dza: 0.0
  max_dza: 0.12
```

---

# 10. 总结

本设计采用演进式重构路线：

```text
V1:
  15维 UKF 后端
  快速验证多假设 dry-run、NIS、likelihood、dual raw batch update

V2:
  InEKF / IEKF-inspired 位姿后端
  结构参数 r1/r2/dza 由慢更新器维护

中间阶段:
  InEKF shadow 使用 UKF snapshot 中的结构参数，
  不进入主链路，只验证可行性

最终目标:
  HypothesisSelector 对后端透明；
  后端可在 ukf15 和 invariant_pose 之间切换；
  结构参数从主滤波状态中剥离，使用高置信窗口慢更新。
```

一句话概括：

```text
先用 15维 UKF 建立可工作的多假设统计关联基线；
再用 InEKF 位姿后端 + 慢结构参数估计提升几何一致性和长期稳定性。
```
