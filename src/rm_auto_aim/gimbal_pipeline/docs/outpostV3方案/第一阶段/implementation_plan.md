# OutpostV3 第一阶段实施方案

## 0. 第一阶段目标

第一阶段完成一个可落地、可灰度、可回放的闭环：

```text
OutpostTrackerV3
  -> OutpostHypothesisGenerator (3-panel 枚举)
  -> OutpostInEKFBackend evaluate all hypotheses
  -> TopK selection + confidence/margin gate
  -> tryUpdate + posterior sanity + reconstruction check
  -> commit best trial
```

第一阶段不做：

1. 保留 outpost_v2 链路作为 A-B 对照，不做替换。
2. 不把 binder bridge / evidence fuser / legacy binding policy 引入新链路。
3. 不实现 dual-observation path（初版仅单观测）。
4. 不实现 warmup / dual-seed 机制（outpost 3 面板天然需要全枚举）。
5. 不把结构参数（radius, z_offset, panel_phase）放入状态估计。

## 1. 架构概览

### 1.1 文件清单

所有新增文件位于 `outpost_v3/` 子目录下，与 `norm4_v3/` 和 `outpost_v2/` 平级：

**头文件** (`include/max_entropy_tracker/trackers/outpost_v3/`):

| 文件 | 职责 |
|------|------|
| `outpost_hypothesis_types.hpp` | Outpost 专用类型：`OutpostHypothesis`、`OutpostPanelGeometry`、gate 配置 |
| `outpost_hypothesis_generator.hpp` | `OutpostHypothesisGenerator` — 从多观测枚举 3-panel 假设 |
| `outpost_inekf_backend.hpp` | `OutpostInEKFBackend` — 标准 12D InEKF，实现 `IStructuredBackend` |
| `outpost_tracker_v3.hpp` | `OutpostTrackerV3` — 主跟踪器，事务式假设-检验编排 |

**源文件** (`src/max_entropy_tracker/trackers/outpost_v3/`):

| 文件 | 职责 |
|------|------|
| `outpost_hypothesis_generator.cpp` | 假设生成实现 |
| `outpost_inekf_backend.cpp` | InEKF 预测/评估/tryUpdate/commit 实现 |
| `outpost_tracker_v3.cpp` | 主跟踪器 update() 编排 |

CMakeLists.txt 使用 `GLOB_RECURSE "src/**/*.cpp"` 自动拾取，无需修改构建配置。

### 1.2 与现有代码的关系

```text
norm4_v3::IStructuredBackend     ← OutpostInEKFBackend 实现此接口
norm4_v3::PredictContext         ← 复用
norm4_v3::MeasurementEval        ← 复用
norm4_v3::UkfTrial               ← 复用
norm4_v3::TopKEntry              ← 复用
norm4_v3::HypothesisDebugFrame   ← 复用
SpinFilterInterface              ← OutpostInEKFBackend 实现此接口
BaseTracker                      ← OutpostTrackerV3 继承
```

OutpostV3 直接复用 norm4_v3 的接口和类型，因为假设-检验事务框架的类型（`PredictContext`、`MeasurementEval`、`UkfTrial`）是通用的，不限于 norm4 的四面板场景。

## 2. 假设空间设计

### 2.1 OutpostHypothesis

```cpp
struct OutpostHypothesis {
  int obs_index = 0;        // 观测在输入数组中的索引
  int panel_id = -1;        // 假设的面板 ID (0, 1, 2)
  double prior_log_weight = 0.0;  // 先验对数权重
  std::string debug_name;   // 如 "O0_P0", "O0_P1", "O0_P2"
};
```

### 2.2 枚举策略

初版仅实现**单观测 3 假设**（见可行性文档结论：仅实现单观测即可）：

```text
obs[0] -> panel 0 (角度偏移 0)
obs[0] -> panel 1 (角度偏移 +2π/3)
obs[0] -> panel 2 (角度偏移 -2π/3)
```

预留双观测接口但不实现评估逻辑：
```text
(obs[0], obs[1]) × ordered distinct panel pairs = 6 hypotheses
```

> 考虑到可识别性（可行性文档结论），单观测已能通过 NIS/likelihood 区分面板 ID。

### 2.3 先验权重来源（Phase 1 简化）

Phase 1 不接入 binder bridge / evidence fuser。先验权重来源：

- 上一帧 committed panel_id 提供保守偏置：`prior_log_weight = panel_switch_penalty * (panel_id == last_committed_panel ? 0 : -1)`
- `panel_switch_penalty` 默认 0.5（可配置），表示切换面板的轻微惩罚

后续 Phase 可在不改变 InEKF 核心的前提下接入：
- `ObservationFrontend` 的 candidate probability
- `OutpostBinderBridge` 的周期相位/2dz signature
- z audit health

## 3. InEKF 后端设计

### 3.1 状态向量（12D）

```
x = [X, Y, Z, VX, VY, VZ, AX, AY, AZ, YAW, YAW_RATE, YAW_ACC]ᵀ

Index 0:  X        — 目标中心 x 位置 (world frame)
Index 1:  Y        — 目标中心 y 位置
Index 2:  Z        — 目标中心 z 位置
Index 3:  VX       — 目标中心 x 速度
Index 4:  VY       — 目标中心 y 速度
Index 5:  VZ       — 目标中心 z 速度
Index 6:  AX       — 目标中心 x 加速度
Index 7:  AY       — 目标中心 y 加速度
Index 8:  AZ       — 目标中心 z 加速度
Index 9:  YAW      — yaw 角度 (SO(2))
Index 10: YAW_RATE — yaw 角速度
Index 11: YAW_ACC  — yaw 角加速度
```

### 3.2 结构参数（已知常量，不进入状态）

```cpp
static constexpr double kRadius = 0.26;
static constexpr std::array<double, 3> kZOffsets = {0.06, 0.0, -0.06};
static constexpr std::array<double, 3> kPanelAngles = {0.0, 2.0*M_PI/3.0, -2.0*M_PI/3.0};
```

### 3.3 预测模型（CA + yaw CA，group-affine）

```
p_{k+1}   = p_k + v_k·dt + 0.5·a_k·dt²
v_{k+1}   = v_k + a_k·dt
a_{k+1}   = a_k + w_a              (过程噪声)
ψ_{k+1}   = ψ_k + ω_k·dt + 0.5·α_k·dt²
ω_{k+1}   = ω_k + α_k·dt
α_{k+1}   = α_k + w_α              (过程噪声)
```

误差状态转移矩阵 F（12×12）不依赖当前状态估计：

```
F = I +
  [0, 0, 0, dt, 0,  0,  dt²/2, 0,     0,     0, 0,       0     ]  // X 行
  [0, 0, 0, 0,  dt, 0,  0,     dt²/2, 0,     0, 0,       0     ]  // Y 行
  [0, 0, 0, 0,  0,  dt, 0,     0,     dt²/2, 0, 0,       0     ]  // Z 行
  [0, 0, 0, 0,  0,  0,  dt,    0,     0,     0, 0,       0     ]  // VX 行
  [0, 0, 0, 0,  0,  0,  0,     dt,    0,     0, 0,       0     ]  // VY 行
  [0, 0, 0, 0,  0,  0,  0,     0,     dt,    0, 0,       0     ]  // VZ 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     0, 0,       0     ]  // AX 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     0, 0,       0     ]  // AY 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     0, 0,       0     ]  // AZ 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     dt,0,      dt²/2  ]  // YAW 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     0, 0,       dt    ]  // RATE 行
  [0, 0, 0, 0,  0,  0,  0,     0,     0,     0, 0,       0     ]  // ACC 行
```

**满足 group-affine 条件**：F 不依赖当前估计状态，误差传播与估计值解耦。

### 3.4 观测模型（3D 位置）

对面板 i：

```
z_pred_i = p + Rz(yaw) · r_i

r_i = [radius · cos(panel_angle_i),
       radius · sin(panel_angle_i),
       z_offset_i]
```

展开：
```
armor_yaw_i = yaw + panel_angle_i
z_pred_i = [p_x + radius · cos(armor_yaw_i),
            p_y + radius · sin(armor_yaw_i),
            p_z + z_offset_i]
```

### 3.5 Body-Frame 不变创新

```
ν = Rz(-yaw) · (z_obs - z_pred)

ν_x =  cos(yaw)·dx + sin(yaw)·dy
ν_y = -sin(yaw)·dx + cos(yaw)·dy
ν_z = dz
```

这是左不变残差，将世界系位置误差旋转到目标自身坐标系，使误差与全局 yaw 估计解耦。

### 3.6 Body-Frame 雅可比

世界系 H（3×12）：
```
H_world = [
  [1, 0, 0, 0, 0, 0, 0, 0, 0, -r·sin(armor_yaw_i), 0, 0],   // ∂x_obs/∂state
  [0, 1, 0, 0, 0, 0, 0, 0, 0,  r·cos(armor_yaw_i), 0, 0],   // ∂y_obs/∂state
  [0, 0, 1, 0, 0, 0, 0, 0, 0,  0,                   0, 0],   // ∂z_obs/∂state
]
```

本体系 H_body（关键简化——不再依赖 center_yaw，仅依赖面板相位 φᵢ）：
```
H_body(0, IDX_YAW) = -r · sin(φᵢ)
H_body(1, IDX_YAW) =  r · cos(φᵢ)
```

### 3.7 状态 Retraction（左不变）

```
x⁺ = Exp(δξ̂) ∘ x̂
```

SO(2) 部分：`yaw⁺ = normalize_angle(yaw + δψ)`
ℝⁿ 部分：`x⁺ = x + δx`（所有位置、速度、加速度分量）

### 3.8 过程噪声 Q

采用离散白噪声 jerk 模型（Bar-Shalom 形式）：

对 CA 链 [p, v, a]：
```
Q_CA(dt, q) = q² · [
  dt⁵/20,  dt⁴/8,  dt³/6
  dt⁴/8,   dt³/3,  dt²/2
  dt³/6,   dt²/2,  dt
]
```

三条平动链 (X, Y, Z) 和一条转动链 (YAW) 各自独立施加。

### 3.9 观测噪声 R

初版使用固定对角矩阵：
```
R = diag(σ_pos_xy², σ_pos_xy², σ_pos_z²)
```

默认值：`σ_pos_xy = 0.02m`, `σ_pos_z = 0.03m`。

## 4. 假设评分

```
score = log_likelihood + prior_log_weight
```

`log_likelihood` 来自高斯似然：
```
nis = νᵀ · S⁻¹ · ν
log_likelihood = -0.5 · (nis + log(det(S)) + m · log(2π))
```
其中 m = 3（3D 观测维度）。

## 5. 检验与提交策略

### 5.1 评价 Gate

- `valid`: 数值计算成功（S 矩阵 SPD）
- `nis < gate_threshold`（默认 11.34，对应 χ²(3) 的 0.99 分位数）
- 位置 innovation 各分量通过 chi2 检验

### 5.2 Commit Gate

- `top1_confidence >= min_top1_confidence`（默认 0.5）
- `top1_top2_margin >= min_top1_top2_margin`（默认 1.0）
- 仅在 STRUCTURED 模式允许 commit

### 5.3 Trial Gate

- trial update 成功
- posterior sanity pass:
  - center 跳变 < max_center_jump（默认 0.5m）
  - yaw 跳变 < max_yaw_jump（默认 0.5 rad）
  - yaw rate < max_yaw_rate（默认 15 rad/s）
  - yaw acc < max_yaw_acc（默认 30 rad/s²）
  - P 正定且所有元素有限
- reconstruction error: `||z_obs_pos - z_rebuild_pos|| < max_reconstruction_pos_error`（默认 0.3m）

### 5.4 拒绝处理

本帧所有假设被拒绝时：
- backend 保持 predict-only 状态
- 帧标记为 degraded observation
- 不把错误 ID 灌入滤波器

## 6. Mode 管理

### 6.1 简化 ModeFSM

Phase 1 使用简化的 mode 切换逻辑，不依赖完整的 binder bridge / evidence fuser：

```text
初始化 -> AMBIGUOUS

AMBIGUOUS:
  - 每帧 3-panel hypothesis 评估
  - 不 commit（predict-only）
  - 维护 top1_confidence 和 top1_margin 的滑动窗口
  - 进入条件:
    top1_confidence > P_enter_structured (默认 0.7)
    top1_margin > M_enter_structured (默认 1.5)
    连续 stable_frames 帧满足 (默认 5)

STRUCTURED:
  - 允许 commit
  - 退化条件（退回 AMBIGUOUS）:
    top1_confidence < P_exit_structured (默认 0.4)
    或 top1_margin < M_exit_structured (默认 0.5)
    连续 degraded_frames 帧 (默认 10)
```

## 7. OutpostTrackerV3::update() 流程

```text
1. predict to observation time
2. buildPredictContext() — 固定本帧 prior
3. generate hypotheses:
   ObsHypothesisGenerator::generate(obs) -> 3 或 6 个假设
4. evaluate loop:
   for each hyp:
     eval = backend_->evaluateSingle(ctx, obs[hyp.obs_index], hyp.panel_id)
     eval.score = eval.log_likelihood + hyp.prior_log_weight
5. select_topk + softmax confidence:
   按 (gate_pass desc, score desc) 排序
   log-sum-exp softmax 归一化
   输出: topk_entries, top1_confidence, top1_top2_margin
6. Mode routing:
   AMBIGUOUS: 评估进入条件，不 commit
   STRUCTURED:
     a. 找第一个 gate_pass 的 hypothesis
     b. commit gate: confidence >= min, margin >= min
     c. tryUpdateSingle(ctx, obs, panel_id)
     d. trial gate: success + sanity + reconstruction
     e. commit(trial)
7. 更新状态机
8. 填充 debug snapshot
```

## 8. 配置参数

新增配置节 `outpost_v3`（可配置，含合理默认值）：

```yaml
outpost_v3:
  # Gate thresholds
  gate:
    single_total_nis: 11.34       # χ²(3) 0.99 quantile
    single_pos_chi2: 9.0          # χ²(1) per-dimension

  # Hypothesis selection
  hypothesis_selector:
    topk: 3
    min_top1_confidence: 0.5
    min_top1_top2_margin: 1.0
    max_reconstruction_pos_error: 0.3

  # Posterior sanity
  posterior_sanity:
    max_center_jump: 0.5           # m
    max_yaw_jump: 0.5              # rad
    max_yaw_rate: 15.0             # rad/s
    max_yaw_acc: 30.0              # rad/s²

  # Mode routing
  mode_routing:
    P_enter_structured: 0.7
    M_enter_structured: 1.5
    stable_frames: 5
    P_exit_structured: 0.4
    M_exit_structured: 0.5
    degraded_frames: 10

  # Prior weights
  prior:
    panel_switch_penalty: 0.5

  # Initial covariance
  initial_P:
    pos: 0.01      # m²
    vel: 1.0       # (m/s)²
    acc: 10.0      # (m/s²)²
    yaw: 0.1       # rad²
    yaw_rate: 1.0  # (rad/s)²
    yaw_acc: 5.0   # (rad/s²)²

  # Process noise
  process_noise:
    acc: 2.0         # m/s² (CA jerk std)
    yaw_acc: 3.0     # rad/s² (yaw CA jerk std)

  # Observation noise
  observation_noise:
    sigma_pos_xy: 0.02  # m
    sigma_pos_z: 0.03   # m
```

## 9. 接口对齐

`OutpostInEKFBackend` 实现 `norm4_v3::IStructuredBackend`：

```cpp
class OutpostInEKFBackend : public norm4_v3::IStructuredBackend,
                            public SpinFilterInterface {
 public:
  // IStructuredBackend
  void reset(const ObservationData &obs, int panel_id,
             double r1, double r2, double dza) override;
  void predict(double dt) override;
  bool initialized() const override;
  PredictContext buildPredictContext() const override;
  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const override;
  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const override;
  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const override;
  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const override;
  void commit(const UkfTrial &trial) override;
  BackendSnapshot snapshot() const override;
  SpinFilterInterface &spin_filter() override;
  const SpinFilterInterface &spin_filter() const override;

  // SpinFilterInterface (12 accessors)
  ...
};
```

## 10. 测试与验证

### 10.1 单元测试场景

1. **单观测 3 假设评估**：固定 prior，对 3 个 panel hypothesis 计算 NIS，验证正确 panel 的 NIS 最小
2. **预测一致性**：连续多帧 predict-only，验证误差传播矩阵 F 不依赖状态
3. **ID 错误拒绝**：故意用错误 panel ID 更新，验证 reconstruction error 触发拒绝
4. **旋转估计**：模拟旋转目标，验证 yaw rate 和 yaw 收敛
5. **丢帧重捕获**：模拟丢帧后恢复，验证不跳 ID

### 10.2 离线回放验证

使用 `gimbal_pipeline_kf_opt` pybind 模块对 outpost 场景 bag 进行离线回放，对比：
- outpost_v2 的 ID 切换频率
- outpost_v3 的 ID 稳定性
- NIS 分布（是否接近理论 χ²(3) 分布）

### 10.3 灰度上线

- 通过 `tracker.implementation = "outpost_v3"` 配置切换
- 默认保持 `outpost_v2`
- v2 和 v3 可同时运行，通过 debug snapshot 对比

## 11. 风险与边界

### 11.1 当前不做的事项

- 不替换 outpost_v2 tracker
- 不接入 binder bridge / evidence fuser
- 不实现 dual-observation evaluate/update
- 不把结构参数放入状态
- 不引入 IMM / Singer 机动模型（保持严格 group-affine）

### 11.2 已知局限

- 仅单装甲板观测时，panel ID 与 center yaw 仍有离散歧义，依赖 NIS 和 likelihood 区分
- 固定结构参数假设检测器提供的 3D 位置足够准确
- yaw 不直接观测，仅通过 3D 位置间接约束旋转估计
- 若检测器一帧中多观测排序不稳定，双观测 extension 需要枚举 ordered assignments

### 11.3 后续演进

- Phase 2: 接入 dual-observation path
- Phase 3: 接入 binder prior（周期相位/2dz signature）
- Phase 4: 弱化 outpost_v2 ambiguous backend，统一状态源
