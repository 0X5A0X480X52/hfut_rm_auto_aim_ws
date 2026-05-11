# armor_pnp_refiner 第一阶段演进计划

日期：2026-05-11

## 1. 目标与边界

本计划用于指导 `armor_pnp_refiner` 从“设计文档”演进为可接入 `armor_detector` 与 `armor_detector_nn` 的公共 PnP 微调链路。第一阶段按 Phase0、Phase1、Phase2 三步推进：

1. Phase0：先接入链路并移植现有 `armor_detector` 的 yaw-only `BaSolver`，保证行为与旧链路一致、可配置关闭、可回退。
2. Phase1：实现单帧 `xyz-yaw` 优化，输出后端需要的协方差、置信度和诊断值，并打通 `R_floor + R_fixed + R_YPD/R_BA` 的后端接口链路。
3. Phase2：实现短期数据关联模块，基于 2D tracker 结果接入完整滑窗 `xyz-yaw` 优化。

模块定位不变：

```text
Detector PnP result in
  -> armor_pnp_refiner
  -> PnP-like result out
```

本包不承担系统级 tracker、机器人中心估计、目标选择或弹道解算。内部短期关联只服务于 PnP 微调窗口，输出的 `refine_track_id` 不具有系统级语义。

## 2. 总体接入链路

### 2.1 detector 侧统一链路

`armor_detector` 与 `armor_detector_nn` 都应在 PnP 成功后进入同一个可配置的 refiner 链路：

```text
detection / keypoints
  -> solvePnP / IPPE PnP
  -> if pnp_refiner.enable:
       ArmorPnpRefiner::refine(input)
     else:
       raw PnP
  -> detector 原有发布链路
```

硬性要求：

1. PnP 失败时不调用 refiner。
2. refiner 失败时返回 raw PnP。
3. 配置关闭时行为等价于旧链路。
4. Phase0 默认只验证行为一致性，不改变下游消息语义。
5. 所有输出必须保留 `valid/refined/status/mode/reason`，方便回放和定位问题。

### 2.2 推荐配置开关

公共配置建议采用以下语义：

```yaml
pnp_refiner:
  enable: false
  mode: "single_yaw"  # none | single_yaw | single_xyz_yaw | sliding_window
  fallback:
    use_raw_pnp_on_failure: true
    max_pose_delta_m: 0.20
    max_yaw_delta_rad: 0.50
  diagnostics:
    publish_debug: false
    log_statistics: true
```

`armor_detector` 可先使用：

```yaml
pose:
  use_pnp_refiner: false
  pnp_refiner_backend: "single_yaw"  # legacy_ba | single_yaw
```

`armor_detector_nn` 可保留现有：

```yaml
pose:
  refiner:
    mode: "single_yaw"  # none | single_yaw | sliding_window
```

但内部逐步切换为公共 `armor_pnp_refiner` adapter，避免两个 detector 分叉维护同类 BA 逻辑。

## 3. 模块目录组织

后续实现不要采用扁平化文件组织。建议按语义分层：

```text
src/rm_auto_aim/armor_pnp_refiner/
├── include/armor_pnp_refiner/
│   ├── core/
│   │   ├── armor_pnp_refiner.hpp
│   │   ├── pnp_refiner_config.hpp
│   │   ├── pnp_refiner_types.hpp
│   │   └── pnp_refiner_diagnostics.hpp
│   ├── adapters/
│   │   ├── armor_detector_adapter.hpp
│   │   └── armor_detector_nn_adapter.hpp
│   ├── geometry/
│   │   ├── armor_geometry.hpp
│   │   ├── camera_model.hpp
│   │   ├── pose_parameterization.hpp
│   │   └── structural_priors.hpp
│   ├── graph/
│   │   ├── common/
│   │   │   ├── reprojection_edge.hpp
│   │   │   ├── pnp_prior_edge.hpp
│   │   │   └── graph_diagnostics.hpp
│   │   ├── single_yaw/
│   │   │   ├── yaw_vertex.hpp
│   │   │   └── single_yaw_optimizer.hpp
│   │   ├── single_xyz_yaw/
│   │   │   ├── xyz_yaw_vertex.hpp
│   │   │   └── single_xyz_yaw_optimizer.hpp
│   │   └── sliding_window/
│   │       ├── window_state.hpp
│   │       ├── temporal_prior_edges.hpp
│   │       └── sliding_window_optimizer.hpp
│   ├── association/
│   │   ├── short_term_associator.hpp
│   │   ├── track2d_bridge.hpp
│   │   └── association_gate.hpp
│   ├── covariance/
│   │   ├── covariance_estimator.hpp
│   │   ├── covariance_clamp.hpp
│   │   └── covariance_transform.hpp
│   └── quality/
│       ├── quality_gate.hpp
│       ├── confidence_estimator.hpp
│       └── fallback_manager.hpp
├── src/
│   ├── core/
│   ├── adapters/
│   ├── geometry/
│   ├── graph/
│   ├── association/
│   ├── covariance/
│   └── quality/
└── test/
    ├── graph/
    ├── association/
    ├── covariance/
    └── integration/
```

组织原则：

1. `core/` 只编排流程，不放具体 g2o 边和数学细节。
2. `graph/` 只负责优化问题定义和求解。
3. `geometry/` 负责装甲板尺寸、相机模型、坐标和结构先验。
4. `association/` 负责短期关联，不参与滤波器语义。
5. `covariance/` 负责 Hessian/marginal covariance、clamp 和坐标变换。
6. `quality/` 负责门控、置信度和 fallback 策略。
7. `adapters/` 隔离不同 detector 的数据结构差异。

## 4. 结构先验约定

优化过程中需要包含装甲板在 odom 系下的结构姿态先验：

```text
roll_odom_armor = 0
pitch_odom_armor =
  +15 deg for outpost
  -15 deg for others
```

第一版实现可以写死，但必须在代码附近加注释：

```cpp
// Phase1/Phase2 temporary structural prior:
// armor roll in odom is assumed 0 deg.
// armor pitch in odom is assumed +15 deg for outpost and -15 deg for others.
// TODO: move these values to armor_pnp_refiner config after the first version is validated.
```

后续应迁移为配置：

```yaml
structural_prior:
  roll_odom_deg: 0.0
  pitch_odom_deg:
    outpost: 15.0
    default: -15.0
  source_frame: "odom"
```

注意：如果优化变量在 camera frame，结构先验来自 odom frame，则需要通过 `T_odom_camera` 或等价 TF 显式转换，不允许混用 camera/odom 下的 yaw、pitch、roll。

## 5. Phase0：可回退链路与 yaw-only BA 移植

### 5.1 目标

Phase0 的目标不是提升精度，而是建立公共 refiner 链路，并证明：

```text
开启公共 single_yaw refiner 后，行为与旧 armor_detector::BaSolver 链路一致；
关闭 refiner 后，行为与原始 PnP 链路一致；
优化失败时，链路可退回 raw PnP。
```

### 5.2 范围

1. 新建 `armor_pnp_refiner` 包源码骨架。
2. 定义 `PnpRefineInput`、`PnpRefineOutput`、`PnpRefinerConfig`。
3. 将 `armor_detector` 中的 yaw-only `BaSolver` 移植为 `graph/single_yaw/SingleYawOptimizer`。
4. 将 `VertexYaw` 和 `EdgeProjection` 迁移到公共包，并整理到 `graph/single_yaw/` 与 `graph/common/`。
5. 在 `armor_detector` 中增加可配置 refiner 调用点。
6. 在 `armor_detector_nn` 中通过 adapter 接入公共 refiner，保留现有 `IPoseRefiner` 风格。
7. 增加 fallback 和质量门控最小实现。

### 5.3 输入输出

Phase0 输入：

```cpp
struct PnpRefineInput {
  Eigen::Vector3d t_camera_armor;
  Eigen::Quaterniond q_camera_armor;
  double yaw_rad;
  double pitch_rad;
  double roll_rad;
  std::vector<Eigen::Vector2d> image_points;
  std::vector<Eigen::Vector3d> object_points;
  Eigen::Matrix3d K;
  std::vector<double> dist_coeffs;
  std::optional<int> external_track_id;
  std::string armor_number;
  ArmorSizeType armor_type;
};
```

Phase0 输出：

```cpp
struct PnpRefineOutput {
  bool valid;
  bool refined;
  RefineMode mode;        // PNP_FALLBACK | G2O_SINGLE_YAW
  RefineStatus status;    // GOOD | DEGRADED | REJECTED
  Eigen::Vector3d t_camera_armor;
  Eigen::Quaterniond q_camera_armor;
  double yaw_rad;
  Eigen::Matrix4d covariance_xyz_yaw;
  bool covariance_valid;
  double confidence;
  double reproj_error_raw_px;
  double reproj_error_refined_px;
  double yaw_delta_rad;
  std::string reason;
};
```

Phase0 中 `covariance_valid` 默认应为 `false` 或 `DEGRADED`，因为 yaw-only BA 不能生成完整 `xyz-yaw` 协方差。`covariance_xyz_yaw` 可以填保守对角矩阵，仅用于占位和后续接口验证。

### 5.4 行为一致性验证

必须完成以下验证：

1. `pnp_refiner.enable=false`：输出与旧 PnP 链路一致。
2. `pnp_refiner.enable=true, mode=single_yaw`：`armor_detector` 输出与旧 `BaSolver` 结果在数值容差内一致。
3. 强制优化失败：输出回退 raw PnP，`valid=true, refined=false`。
4. 输入角点顺序错误或点数不足：输出回退 raw PnP，并记录 reason。
5. NaN、负深度、yaw delta 超限：输出回退 raw PnP。
6. detector 与 detector_nn 都能通过配置独立关闭公共 refiner。

### 5.5 Phase0 验收

Phase0 完成标志：

1. `armor_detector` 不再直接依赖本地 `BaSolver` 作为唯一 BA 实现。
2. 公共包 `single_yaw` 输出与旧 BA 链路一致。
3. detector_nn 可通过 adapter 调用公共包。
4. 所有失败路径可回退。
5. 默认配置可以保持旧系统行为。

## 6. Phase1：单帧 xyz-yaw 与 R_BA 链路打通

### 6.1 目标

Phase1 实现单帧 `xyz-yaw` 优化，并输出后续后端需要的协方差等值：

```text
pose_xyz_yaw
cov_xyz_yaw
cov_valid
confidence
reproj_rms
condition_number
chi2_per_dof
num_observations
num_inliers
```

同时打通后端动态 R 的接口链路：

【后端链路并未实现，预留接口即可】

```text
R_final = R_floor + (1 - lambda) R_fixed + lambda * [(1 - w) R_YPD + w * s_BA * R_BA]
```

### 6.2 单帧 xyz-yaw 优化问题

状态：

```text
x = [tx, ty, tz, yaw]
```

残差：

```text
r_i = observed_uv_i - project(K, D, R(yaw, structural_pitch, structural_roll) * P_i + t)
```

先验：

```text
r_pnp_t = t - t_pnp
r_pnp_yaw = wrap(yaw - yaw_pnp)
```

第一版结构先验：

```text
roll_odom_armor = 0
pitch_odom_armor = +15 deg for outpost, -15 deg for others
```

若当前实现暂时只能稳定使用 camera frame 参数化，应先显式写清楚先验转换方式，避免把 odom pitch 直接当 camera pitch 使用。

### 6.3 协方差输出

优化收敛后，在最终线性化点上计算：

```text
H = J^T W J
R_BA_raw = sigma_px^2 * H^-1
scale = max(1, chi2 / dof)
R_BA = R_floor_local + scale * R_BA_raw
```

必须执行：

1. finite check。
2. symmetric check。
3. positive definite check。
4. eigenvalue clamp。
5. condition number check。
6. reprojection RMS check。
7. pose/yaw delta check。

推荐输出：

```cpp
struct PnpCovarianceDiagnostics {
  bool covariance_valid = false;
  Eigen::Matrix4d covariance_xyz_yaw = Eigen::Matrix4d::Identity();
  double condition_number = 0.0;
  double chi2_per_dof = 0.0;
  double reproj_rms = 0.0;
  double residual_scale = 1.0;
  int num_observations = 0;
  int num_inliers = 0;
};
```

### 6.4 后端 R 链路

Phase1 需要在 `gimbal_pipeline` 或 `max_entropy_tracker` 的观测层追加 BA/PnP covariance 元数据：

```cpp
struct ObservationCovarianceMeta {
  bool valid = false;
  bool cov_valid = false;
  Eigen::Matrix4d cov_xyz_yaw = Eigen::Matrix4d::Identity();
  double confidence = 0.0;
  double reproj_rms = 0.0;
  double condition_number = 0.0;
  int num_observations = 0;
  int num_inliers = 0;
};
```

建议以 append-only 方式加入 `ObservationData`：

```cpp
std::optional<ObservationCovarianceMeta> ba_pnp;
```

然后抽出观测噪声构造器：

```text
ObservationNoiseBuilder
  -> compute R_floor
  -> compute R_fixed
  -> compute R_YPD
  -> validate/clamp R_BA
  -> blend R_dynamic
  -> blend R_final
```

### 6.5 Phase1 启用策略

Phase1 初始不应直接改变滤波器行为：

```yaml
dynamic_observation_noise:
  enable: true
  lambda: 0.0
  ba_cov:
    enable: true
    scale: 5.0
    confidence_weight_mode: "square"
```

先只记录：

1. `R_fixed`
2. `R_YPD`
3. `R_BA`
4. `R_final`
5. confidence
6. condition number
7. NIS
8. innovation

稳定后再离线验证：

```text
lambda = 0.3 -> 0.5 -> 0.7
```

### 6.6 Phase1 验收

1. 单帧 `xyz-yaw` 优化失败时可回退 `single_yaw` 或 raw PnP。
2. `cov_valid=true` 只在完整检查通过后出现。
3. 静态目标下 covariance 正定且量级合理。
4. 远距离或小装甲板场景下 z 方差和 condition number 合理增大。
5. 后端 `lambda=0.0` 时行为等价旧固定 R。
6. `lambda>0` 离线回放时 NIS 和 innovation 可解释，无明显发散。

## 7. Phase2：短期关联与滑窗 xyz-yaw

### 7.1 目标

Phase2 实现短期数据关联模块，并基于 2D tracker 输出实现完整滑窗 `xyz-yaw` 优化：

```text
2D detection / 2D track evidence
  -> ShortTermAssociator
  -> per refine_track_id window
  -> SlidingWindowXyzYawOptimizer
  -> current-frame refined PnP + covariance
```

### 7.2 短期关联模块

短期关联优先使用外部 `track2d_id`：

1. `armor_detector_nn` 已有 2D tracker，可传入 detector 级 `track2d_id`。
2. `armor_detector` 没有稳定 detector 内部 track_id 时，由 `ShortTermAssociator` 使用 IoU、中心距离、类别、PnP 位置距离做轻量关联。
3. 内部只生成 `refine_track_id`，不输出系统级 target id。

推荐结构：

```text
association/
  track2d_bridge
    - 将 detector_nn 的 2D tracker 结果转为 refiner 输入
  short_term_associator
    - 无外部 track id 时生成 refine_track_id
  association_gate
    - IoU / center distance / class / pose distance / time gap gates
```

关联质量应进入 confidence：

```text
c_association = external_track_id ? 1.0 : association_score_normalized
```

窗口刚建立、关联断裂或重置后，输出应为 `DEGRADED` 或回退单帧。

### 7.3 滑窗优化问题

窗口状态：

```text
X = [x_0, x_1, ..., x_k]
x_i = [tx_i, ty_i, tz_i, yaw_i]
```

观测边：

```text
r_reproj(i, j) = observed_uv_ij - project(K, D, R(yaw_i, structural_pitch_i, structural_roll_i) * P_j + t_i)
```

先验边：

```text
r_pnp_t(i) = t_i - t_pnp_i
r_pnp_yaw(i) = wrap(yaw_i - yaw_pnp_i)
```

时间平滑边：

```text
second_order_translation = t_i - 2 t_{i-1} + t_{i-2}
second_order_yaw = wrap(yaw_i - 2 yaw_{i-1} + yaw_{i-2})
```

结构先验继续使用：

```text
roll_odom_armor = 0
pitch_odom_armor = +15 deg for outpost, -15 deg for others
```

第一版可以写死，但必须保留 TODO 注释并预留配置字段。

### 7.4 滑窗协方差

滑窗输出仍只给当前帧观测：

```text
z_current = [x_k, y_k, z_k, yaw_k]
```

当前帧 marginal covariance：

```text
H = [ H_oo  H_oc
      H_co  H_cc ]

Omega_c = H_cc - H_co * H_oo^-1 * H_oc
Sigma_c = Omega_c^-1
```

必须注意：

1. 滑窗先验会让 covariance 偏乐观，需要残差尺度修正和额外 inflate。
2. 关联置信度低时应加 `R_assoc` 或直接降权。
3. 窗口过短时不输出 `cov_valid=true`，可回退单帧 covariance。
4. 窗口过长时会引入延迟和运动模型偏置，第一版建议 `3~5` 帧、`max_time_span_ms <= 60 ms`。

### 7.5 Phase2 验收

1. `track2d_id` 存在时优先使用外部 track。
2. 无外部 track 时内部关联可生成稳定短期 `refine_track_id`。
3. 关联断裂、类别变化、时间间隔过大时窗口重置。
4. 滑窗优化失败时回退 `single_xyz_yaw -> single_yaw -> raw PnP`。
5. 滑窗输出当前帧 refined PnP，不输出系统级 tracker 状态。
6. 滑窗 covariance 经过 marginal、scale、clamp、confidence 处理后才允许进入 `R_BA`。

## 8. 回退链路

推荐统一回退链：

```text
sliding_window_xyz_yaw
  -> single_xyz_yaw
  -> single_yaw
  -> raw PnP
```

对 Phase0：

```text
single_yaw -> raw PnP
```

对 Phase1：

```text
single_xyz_yaw -> single_yaw -> raw PnP
```

对 Phase2：

```text
sliding_window_xyz_yaw -> single_xyz_yaw -> single_yaw -> raw PnP
```

任意阶段必须满足：

```text
PnP 成功，则最终有可发布 pose；
refiner 失败，不阻塞 detector 发布。
```

## 9. 里程碑与产物

| Phase | 主要产物 | 默认启用建议 | 验收重点 |
|---|---|---|---|
| Phase0 | 公共 refiner 骨架、single_yaw 移植、detector/detector_nn adapter | 默认关闭或与旧 BA 等价开启 | 行为一致、可回退 |
| Phase1 | single_xyz_yaw、协方差诊断、Observation covariance meta、R builder | refiner 可灰度开启，`lambda=0.0` | 协方差有效性、后端链路打通 |
| Phase2 | ShortTermAssociator、2D tracker bridge、sliding_window_xyz_yaw | 实验开启 | 关联稳定、窗口可控、covariance 不过度自信 |

## 10. 推荐实施顺序

1. 新建公共包源码骨架和 CMake/package 依赖。
2. 定义 core types/config/diagnostics。
3. 移植 yaw-only BA 到 `graph/single_yaw/`。
4. 在 `armor_detector` PnP 后接入公共 refiner，默认关闭。
5. 在 `armor_detector_nn` 通过 adapter 接入公共 refiner，保持现有配置兼容。
6. 做 Phase0 行为一致性回放。
7. 新增 `graph/single_xyz_yaw/` 优化器。
8. 新增 `covariance/` 估计、clamp 和诊断输出。
9. 扩展 `ObservationData` 或等价消息链路，打通 `R_BA` 元数据。
10. 抽出后端 `ObservationNoiseBuilder`，先以 `lambda=0.0` 记录日志。
11. 新增 `association/` 和 `graph/sliding_window/`。
12. 离线 bag 验证滑窗 pose、covariance、NIS，再决定线上启用比例。

## 11. 第一阶段最终形态

第一阶段结束时，系统应形成以下能力：

```text
armor_detector / armor_detector_nn
  -> raw PnP
  -> configurable armor_pnp_refiner
       - none
       - single_yaw
       - single_xyz_yaw
       - sliding_window_xyz_yaw
  -> PnP-like output with covariance diagnostics
  -> gimbal_pipeline ObservationData
  -> R_floor + R_fixed + R_YPD/R_BA
  -> UKF/InEKF update
```

其中 Phase0 保证“能接入且不改变旧行为”，Phase1 保证“有可信的单帧 `R_BA` 接口”，Phase2 保证“具备短期关联和滑窗增强能力”。整个过程必须始终保留配置开关、质量门控和 raw PnP fallback。
