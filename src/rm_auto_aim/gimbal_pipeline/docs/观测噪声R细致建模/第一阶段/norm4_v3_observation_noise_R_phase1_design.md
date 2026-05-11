# Norm4 v3 观测噪声 R 细致建模第一阶段设计方案

日期：2026-05-11

## 1. 目标与边界

第一阶段目标是在 `norm4_v3` 试点完整打通动态观测噪声链路：

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
R_final   = R_floor + (1 - lambda) R_fixed + lambda * R_dynamic
```

并支持两个后端：

1. `UkfBackendV2`
2. `InvariantPoseBackend`，即 InEKF / invariant error-state EKF 后端

第一阶段不是“直接相信 BA Hessian covariance”的阶段。基于 `armor_pnp_refiner` 重新审计结论，当前 `single_xyz_yaw` / `sliding_window` 已能输出 `covariance_xyz_yaw` 字段，但实现仍是 conservative placeholder，且 `covariance_valid=false`。因此第一阶段应做到：

1. 完整实现公式和数据链路。
2. 支持 `R_BA` 输入、检查、clamp、缩放、融合。
3. 当 `cov_valid=false` 或质量不达标时强制 `w=0`，退回 `R_YPD`。
4. 默认以 shadow / low-weight 方式试点，不改变 `ukf_v1` 基线。
5. 为后续真实 Hessian / marginal covariance 接入留好接口。

一句话边界：

```text
第一阶段实现“可用、可回退、可观测”的 R_dynamic 框架；R_BA 默认安全降权，不裸用。
```

## 2. 现状依据

### 2.1 已有设计依据

本方案基于同目录下三份分析：

1. `observation_noise_R_design.md`
2. `norm4_v3_phase1_feasibility_analysis.md`
3. `r_ba_phase1_reaudit_with_armor_pnp_refiner.md`

关键共识：

1. `R_floor` 用于防止过度自信。
2. `R_fixed` 用于保持旧固定噪声锚点。
3. `R_YPD` 是第一阶段最可靠的动态项。
4. `R_BA` 可以进入链路，但必须由有效性、正定性、条件数、残差和置信度控制。
5. `lambda` 是迁移旋钮，不能省略。

### 2.2 当前代码基础

当前 `norm4_v3` 已有：

```cpp
IMeasurementNoiseModel
FixedCartesianNoiseModel
UkfBackendV2
InvariantPoseBackend
Norm4BackendFactory
```

当前不足：

1. `IMeasurementNoiseModel::build_R(UpdateKind)` 不接收 `ObservationData`。
2. `FixedCartesianNoiseModel` 只能输出固定 R。
3. `UkfBackendV2` 和 `InvariantPoseBackend` 在 single / dual evaluate/update 中只按 `UpdateKind` 取 R。
4. `ObservationData` 没有 BA/PnP covariance 元数据。
5. `Armor.msg` 没有 refiner quality / covariance 字段。
6. `noise_profile` 当前只读取固定噪声字段：`sigma_pos_xy / sigma_pos_z / sigma_yaw / dual_raw_R_scale`。

当前可复用：

1. `ObservationData::image` 已有 bbox/corners/type/confidence，可用于 `R_YPD`。
2. `armor_pnp_refiner::PnpRefineOutput` 已有 covariance/quality 字段。
3. `armor_detector` 已接入 `armor_pnp_refiner`，可成为第一批 R_BA 元数据来源。
4. `ukf_v2` 和 `inekf` 都通过同一个 `IMeasurementNoiseModel` 取 R，改造收益集中。

## 3. 第一阶段架构

### 3.1 数据流

```text
armor_detector / armor_detector_nn
  -> rm_interfaces::msg::Armor
       pose
       image geometry
       refiner quality
       optional cov_xyz_yaw
  -> gimbal_pipeline::TFHandler
       pose transform
       covariance transform / invalidate
  -> ObservationData
       x, y, z, yaw
       image metadata
       optional ba_pnp metadata
  -> BaAwareYpdNoiseModel
       R_fixed
       R_floor
       R_YPD
       R_BA_used
       R_dynamic
       R_final
  -> UkfBackendV2 / InvariantPoseBackend
       evaluateSingle / evaluateDual
       tryUpdateSingle / tryUpdateDual
```

### 3.2 模块关系

```text
IMeasurementNoiseModel
  ├── FixedCartesianNoiseModel        // 兼容旧行为
  └── BaAwareYpdNoiseModel            // 第一阶段新增
        ├── Fixed baseline builder
        ├── YPD covariance builder
        ├── BA covariance validator
        ├── Covariance clamp/inflate
        └── Debug snapshot builder
```

`BaAwareYpdNoiseModel` 是第一阶段核心。它不属于 UKF 或 InEKF 专属逻辑，而是一个观测噪声构造器，两个后端共用。

## 4. 数据结构设计

### 4.1 ObservationData 扩展

在 `include/max_entropy_tracker/core/observation.hpp` 中 append-only 增加：

```cpp
struct ObservationCovarianceMeta {
  bool valid = false;
  bool cov_valid = false;

  // Cov([x, y, z, yaw]) in the same frame and yaw convention as ObservationData.
  Eigen::Matrix4d cov_xyz_yaw = Eigen::Matrix4d::Identity();

  double confidence = 0.0;
  double reproj_rms = 0.0;
  double condition_number = 0.0;
  int num_observations = 0;
  int num_inliers = 0;
  int pose_estimate_mode = 0;

  // Diagnostics only. True when frame/yaw convention has been checked.
  bool frame_aligned = false;
};
```

并在 `ObservationData` 中增加：

```cpp
std::optional<ObservationCovarianceMeta> ba_pnp;
```

字段语义：

1. `valid=true` 表示上游提供了 refiner 元数据。
2. `cov_valid=true` 表示 `cov_xyz_yaw` 可作为候选 R_BA。
3. `frame_aligned=true` 表示 covariance 已转换到 `ObservationData` 的观测空间。
4. `cov_valid=false` 时仍可使用 `confidence/reproj_rms/mode` 做日志和质量缩放，但 `w_BA=0`。

### 4.2 Armor.msg 扩展建议

在 `rm_interfaces/msg/Armor.msg` append-only 增加：

```text
# Pose/refiner quality metadata
uint8 pose_estimate_mode
float32 pose_quality_score
float32 reproj_error_raw
float32 reproj_error_refined
float32 pose_condition_number
uint16 pose_num_points
uint16 pose_num_inliers

# Covariance of [x, y, z, yaw] in the same frame as Armor.pose.
bool pose_covariance_valid
float64[16] pose_covariance_xyz_yaw
```

第一阶段约束：

1. 如果 covariance 与 `Armor.pose` 不在同一 frame，发布端必须置 `pose_covariance_valid=false`。
2. `single_yaw` 模式不得发布完整有效 `xyz+yaw` covariance。
3. `single_xyz_yaw` / `sliding_window` 当前实现仍应发布 `pose_covariance_valid=false`，直到 Hessian covariance 真正完成。

### 4.3 PoseEstimate 扩展建议

在 `armor_detector_nn::PoseEstimate` 中 append-only 增加：

```cpp
bool covariance_valid{false};
Eigen::Matrix4d covariance_xyz_yaw{Eigen::Matrix4d::Identity()};
double condition_number{0.0};
int num_points{0};
int num_inliers{0};
```

目的：

1. 让 NN detector 与传统 detector 发布同构质量字段。
2. 后续接入 `armor_pnp_refiner` 时可直接映射 `PnpRefineOutput`。
3. 第一阶段即使不启用 NN 侧 covariance，也能把 `quality_score/reproj_error/mode` 下传。

## 5. 噪声接口设计

### 5.1 当前接口问题

当前接口：

```cpp
virtual Eigen::MatrixXd build_R(UpdateKind kind) const = 0;
```

无法根据当前观测的：

1. 距离
2. bbox size
3. image corners
4. detection confidence
5. BA covariance
6. reprojection RMS

动态生成 R。

### 5.2 第一阶段接口

建议改为：

```cpp
class IMeasurementNoiseModel {
 public:
  virtual ~IMeasurementNoiseModel() = default;

  virtual Eigen::Matrix4d build_single_R(
      const ObservationData& obs) const = 0;

  virtual Eigen::Matrix<double, 8, 8> build_dual_R(
      const ObservationData& obs0,
      const ObservationData& obs1) const = 0;

  virtual std::string name() const = 0;

  // Compatibility/debug accessors.
  virtual double sigma_pos_xy() const = 0;
  virtual double sigma_pos_z() const = 0;
  virtual double sigma_yaw() const = 0;
  virtual double dual_scale() const = 0;
};
```

`FixedCartesianNoiseModel` 兼容策略：

```cpp
Eigen::Matrix4d build_single_R(const ObservationData&) const override;
Eigen::Matrix<double, 8, 8> build_dual_R(
    const ObservationData&, const ObservationData&) const override;
```

其行为完全复现当前固定 R。

### 5.3 后端调用点

`UkfBackendV2`：

```cpp
Eigen::Matrix4d R = noise_->build_single_R(obs);
Eigen::Matrix<double, 8, 8> R = noise_->build_dual_R(obs0, obs1);
```

`InvariantPoseBackend` 同样替换。

必须覆盖：

1. `evaluateSingle()`
2. `evaluateDual()`
3. `tryUpdateSingle()` 中 Joseph form 使用的 R
4. `tryUpdateDual()` 中 Joseph form 使用的 R

注意：evaluate 和 tryUpdate 必须使用同一套 R，否则 NIS/gate 与真正更新不一致。

## 6. R_fixed 与 R_floor

### 6.1 R_fixed

`R_fixed` 是旧固定观测噪声锚点：

```text
R_fixed = diag(sigma_x_fixed^2,
               sigma_y_fixed^2,
               sigma_z_fixed^2,
               sigma_yaw_fixed^2)
```

默认建议：

```yaml
r_fixed:
  sigma_x: 0.06
  sigma_y: 0.06
  sigma_z: 0.08
  sigma_yaw: 0.12
```

与当前 `default.yaml` 保持一致。

### 6.2 R_floor

`R_floor` 是最小观测噪声下限：

```text
R_floor = diag(sigma_x_floor^2,
               sigma_y_floor^2,
               sigma_z_floor^2,
               sigma_yaw_floor^2)
```

默认建议：

```yaml
r_floor:
  sigma_x: 0.005
  sigma_y: 0.005
  sigma_z: 0.010
  sigma_yaw: 0.010
```

注意：`R_floor` 不替代 `R_fixed`。它只防止最终 R 过小。

## 7. R_YPD 构造

### 7.1 输入

`R_YPD` 需要：

1. `obs.x/y/z/yaw`
2. `obs.image.bbox_w/h`
3. `obs.image.image_center_x/y`
4. `obs.image.type`
5. camera intrinsics: `fx/fy/cx/cy`
6. armor geometry: small/large width/height

若缺少 `obs.image` 或相机内参，则回退：

```text
R_YPD = R_fixed
```

### 7.2 像素域噪声

配置：

```yaml
ypd_prior:
  sigma_center_px: 2.0
  sigma_size_px: 2.0
  sigma_corner_px: 1.5
  sigma_azi_min: 0.0005
  sigma_azi_max: 0.020
  sigma_ele_min: 0.0005
  sigma_ele_max: 0.020
  sigma_dist_min: 0.02
  sigma_dist_max: 1.00
  sigma_yaw_min: 0.03
  sigma_yaw_max: 0.50
  sigma_yaw_scale: 10.0
  global_scale: 1.0
```

角噪声：

```text
sigma_azi ~= fx / (fx^2 + du^2) * sigma_center_px
sigma_ele ~= fy / (fy^2 + dv^2) * sigma_center_px
```

距离噪声：

```text
dist_w ~= fx * armor_width / bbox_w
sigma_dist_w ~= dist_w / bbox_w * sigma_size_px

dist_h ~= fy * armor_height / bbox_h
sigma_dist_h ~= dist_h / bbox_h * sigma_size_px

1 / sigma_dist^2 = 1 / sigma_dist_w^2 + 1 / sigma_dist_h^2
```

yaw 噪声：

```text
sigma_yaw ~= sigma_yaw_scale * sigma_corner_px / max(bbox_w, bbox_h)
```

全部需要 clamp。

### 7.3 YPD 到 xyz+yaw

局部：

```text
R_YPD_local = diag(sigma_azi^2,
                   sigma_ele^2,
                   sigma_dist^2,
                   sigma_yaw^2)
```

转换：

```text
R_YPD = J * R_YPD_local * J^T
```

`J = d(x,y,z,yaw) / d(azi,ele,dist,yaw)`，沿用 `observation_noise_R_design.md` 的雅可比。

### 7.4 质量缩放

第一阶段建议引入温和缩放：

```text
R_YPD_used = ypd_global_scale * quality_scale * R_YPD
```

`quality_scale` 可由 detection confidence 控制：

```text
quality_scale = clamp(1 / max(confidence, confidence_floor), min_scale, max_scale)
```

默认建议先关闭或限制在较小范围：

```yaml
quality_scale:
  enable: true
  confidence_floor: 0.30
  min_scale: 1.0
  max_scale: 3.0
```

## 8. R_BA 构造与安全门

### 8.1 输入

来自：

```cpp
obs.ba_pnp->cov_xyz_yaw
obs.ba_pnp->cov_valid
obs.ba_pnp->confidence
obs.ba_pnp->reproj_rms
obs.ba_pnp->condition_number
obs.ba_pnp->num_observations
obs.ba_pnp->num_inliers
obs.ba_pnp->frame_aligned
```

### 8.2 必要条件

只有同时满足：

```text
valid == true
cov_valid == true
frame_aligned == true
cov finite
cov symmetric positive definite
condition_number <= max_condition_number
reproj_rms <= max_reproj_rms
confidence >= min_confidence
num_observations >= min_observations
```

才允许 `R_BA` 参与融合。

否则：

```text
w = 0
R_dynamic = R_YPD
```

### 8.3 covariance clamp

对输入 `R_BA` 做：

1. symmetry: `(R + R^T) / 2`
2. eigenvalue clamp
3. diagonal min/max clamp
4. condition number check
5. scale inflation

建议配置：

```yaml
ba_covariance:
  enable: true
  require_cov_valid: true
  require_frame_aligned: true
  min_confidence: 0.60
  max_reproj_rms_px: 3.0
  max_condition_number: 10000000.0
  min_observations: 4
  min_inlier_ratio: 0.75
  scale: 4.0
  weight_power: 2.0
  max_weight: 0.05
  eigen_clamp:
    min: 1.0e-6
    max: 4.0
  diag_clamp:
    x_min: 1.0e-5
    y_min: 1.0e-5
    z_min: 4.0e-5
    yaw_min: 1.0e-5
    x_max: 1.0
    y_max: 1.0
    z_max: 4.0
    yaw_max: 1.0
```

### 8.4 权重 w

建议：

```text
w = max_weight * confidence^weight_power
```

第一阶段默认：

```text
max_weight = 0.0   // shadow 模板
或
max_weight = 0.05  // low-weight 灰度模板
```

如果 `cov_valid=false`，即使 `max_weight > 0`，也强制 `w=0`。

### 8.5 BA scale

最终用于融合的是：

```text
R_BA_used = s_BA * clamp(R_BA)
```

`s_BA` 第一阶段建议不小于 `4.0`。

## 9. R_dynamic 与 R_final

### 9.1 单观测

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
R_final   = R_floor + (1 - lambda) R_fixed + lambda * R_dynamic
```

处理顺序建议：

1. build `R_fixed`
2. build `R_floor`
3. build `R_YPD`，失败则 `R_YPD = R_fixed`
4. validate/build `R_BA_used`，失败则 `w=0`
5. blend `R_dynamic`
6. blend `R_final`
7. symmetry + SPD check + eigen clamp
8. debug snapshot

### 9.2 双观测

```text
R0 = build_single_R(obs0)
R1 = build_single_R(obs1)
R_dual = blockdiag(R0, R1) * dual_raw_R_scale
```

第一阶段不建模两个观测之间的相关性。

### 9.3 与 NIS/gate 的一致性

`evaluate*()` 和 `tryUpdate*()` 必须调用同样的 `build_single_R/build_dual_R`。如果担心重复计算，可在 `MeasurementEval` 或 `UpdateTrial` 中缓存 R，但第一阶段可以先重复计算，保证逻辑简单。

## 10. UKF v2 试点实现

### 10.1 改造点

文件：

1. `include/max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_measurement_noise.hpp`
2. `src/max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v2.cpp`
3. `include/max_entropy_tracker/trackers/norm4_v3/backends/norm4_backend_factory.hpp`
4. `include/max_entropy_tracker/core/observation.hpp`

`UkfBackendV2` 需要替换：

```cpp
noise_->build_R(UpdateKind::Single)
noise_->build_R(UpdateKind::Dual)
```

为：

```cpp
noise_->build_single_R(obs)
noise_->build_dual_R(obs0, obs1)
```

### 10.2 验收

1. `noise_profile=fixed` 下行为与当前一致。
2. `noise_profile=ypd_ba_shadow.yaml` 下可运行，`R_BA` 不影响主滤波。
3. single/dual gate 与 NIS 可记录。
4. 缺失 `obs.image` 或 `obs.ba_pnp` 不崩溃。

## 11. InEKF 试点实现

### 11.1 改造点

文件：

1. `src/max_entropy_tracker/trackers/norm4_v3/backends/norm4_inekf_backend.cpp`
2. 共享同一个 `IMeasurementNoiseModel`

替换 single/dual evaluate 和 tryUpdate 中 R 的构造。

### 11.2 特别注意

InEKF 的 Joseph form 中直接使用 R：

```cpp
P_post = I_KH * P * I_KH.transpose() + K * R * K.transpose();
```

所以动态 R 过小会更直接地影响后验协方差。第一阶段建议：

1. InEKF 默认使用 shadow 或 `lambda <= 0.1`。
2. 不允许 `R_BA` 在 InEKF 中高于 UKF v2 权重。
3. InEKF 先以回放 NIS / gate pass / posterior sanity 为验收，不以命中率为首要指标。

## 12. 配置结构设计

### 12.1 当前兼容字段

现有 profile：

```yaml
noise:
  sigma_pos_xy: 0.06
  sigma_pos_z: 0.08
  sigma_yaw: 0.12
  dual_raw_R_scale: 1.5
```

第一阶段继续保留这些字段，作为 fixed baseline。

### 12.2 新增字段结构

建议仍以 `noise:` 为根节点：

```yaml
noise:
  type: ypd_ba

  sigma_pos_xy: 0.06
  sigma_pos_z: 0.08
  sigma_yaw: 0.12
  dual_raw_R_scale: 1.5

  dynamic_blend:
    lambda: 0.30

  r_floor:
    sigma_x: 0.005
    sigma_y: 0.005
    sigma_z: 0.010
    sigma_yaw: 0.010

  r_fixed:
    sigma_x: 0.060
    sigma_y: 0.060
    sigma_z: 0.080
    sigma_yaw: 0.120

  camera:
    source: config
    fx: 1556.34704
    fy: 1557.43488
    cx: 610.59754
    cy: 503.80001
    image_width: 1280
    image_height: 1024

  armor_geometry:
    small:
      width: 0.135
      height: 0.055
    large:
      width: 0.230
      height: 0.055
    outpost:
      width: 0.230
      height: 0.055

  ypd_prior:
    ...

  ba_covariance:
    ...

  debug:
    enable_snapshot: true
```

### 12.3 Profile 加载策略

当前 `load_noise_profile_or_default()` 只加载固定字段。第一阶段需要改为：

1. 继续加载固定字段到 `Norm4V3UkfConfig`，保证兼容。
2. 新增 `Norm4V3ObservationNoiseConfig` 或 `MeasurementNoiseConfig` 承载扩展字段。
3. `backend_factory` 根据 `noise.type` 创建：

```cpp
if (noise_type == "fixed") {
  return FixedCartesianNoiseModel(...);
}
if (noise_type == "ypd_ba") {
  return BaAwareYpdNoiseModel(...);
}
```

兼容规则：

1. `noise.type` 缺失时视为 `fixed`。
2. 旧 `default/high_precision/high_robust` profile 不需要改。
3. 新模板放在 `config/norm4_v3/profiles/noise/ypd_ba_shadow.yaml` 和 `ypd_ba_low_weight.yaml`。

## 13. 示例模板

本方案配套新增：

1. `config/norm4_v3/profiles/noise/ypd_ba_shadow.yaml`
2. `config/norm4_v3/profiles/noise/ypd_ba_low_weight.yaml`

### 13.1 shadow 模板

用途：第一阶段默认试点。

行为：

1. 完整构造 `R_YPD`。
2. 读取并校验 `R_BA`。
3. 记录 shadow 调试量。
4. `ba_covariance.max_weight=0.0`，不让 R_BA 影响主滤波。

### 13.2 low-weight 模板

用途：完成 `cov_valid=true` 与 TF 转换验证后的灰度。

行为：

1. `lambda=0.30`
2. `ba_covariance.max_weight=0.05`
3. `ba_covariance.scale=4.0`
4. 所有 gate 未通过时自动 `w=0`

## 14. Debug 与日志

建议为每次构造 R 输出可选 debug snapshot：

```cpp
struct ObservationNoiseDebugSnapshot {
  bool image_valid;
  bool ba_valid;
  bool ba_cov_valid;
  bool ba_used;
  double lambda;
  double ba_weight;
  double ba_confidence;
  double ba_reproj_rms;
  double ba_condition_number;
  Eigen::Vector4d diag_fixed;
  Eigen::Vector4d diag_floor;
  Eigen::Vector4d diag_ypd;
  Eigen::Vector4d diag_ba;
  Eigen::Vector4d diag_final;
};
```

第一阶段可以先不发布 ROS topic，只进入 throttle log 或 prediction logger CSV。

关键指标：

1. NIS single/dual P50/P90/P99
2. gate pass ratio
3. posterior sanity fail ratio
4. `ba_cov_valid` ratio
5. `ba_used` ratio
6. `R_BA / R_YPD` 对角比值
7. 缺失 image metadata 的比例
8. TF covariance invalidation 的比例

## 15. 安全策略

### 15.1 默认安全

默认 profile 必须满足：

```text
R_BA 不影响主滤波
缺字段自动回退
cov_valid=false 自动 w=0
lambda 可一键置 0
noise.type 缺失回到 fixed
```

### 15.2 回退路径

回退层级：

```text
R_BA invalid -> R_dynamic = R_YPD
R_YPD invalid -> R_dynamic = R_fixed
lambda = 0 -> R_final = R_floor + R_fixed
noise.type fixed -> 旧固定 R
backend_type ukf_v1 -> 完全旧链路
```

### 15.3 SPD 保障

所有输出 R 必须：

1. finite
2. symmetric
3. positive definite
4. eigenvalue clamped
5. diagonal lower bounded

否则回退 fixed。

## 16. 实施步骤

### Step 1：数据结构和消息链路

1. 扩展 `Armor.msg`。
2. 扩展 `ObservationData`。
3. 扩展 `PoseEstimate`。
4. `armor_detector` 映射 `PnpRefineOutput` 到 `Armor.msg`。
5. `armor_detector_nn` 映射 `PoseEstimate` 到 `Armor.msg`。
6. `msg_converter` 映射 `Armor.msg` 到 `ObservationData::ba_pnp`。
7. `TFHandler` 处理 covariance frame 对齐。

### Step 2：噪声接口改造

1. 修改 `IMeasurementNoiseModel`。
2. 适配 `FixedCartesianNoiseModel`。
3. 修改 `UkfBackendV2`。
4. 修改 `InvariantPoseBackend`。
5. 保证 fixed profile 行为一致。

### Step 3：实现 `BaAwareYpdNoiseModel`

1. build fixed/floor。
2. build YPD。
3. validate/clamp BA。
4. blend dynamic/final。
5. build dual block diagonal。
6. debug snapshot。

### Step 4：配置加载

1. 新增 `Norm4V3ObservationNoiseConfig`。
2. 扩展 noise profile loader。
3. 支持 `noise.type=fixed|ypd_ba`。
4. 支持相对路径 profile。

### Step 5：shadow 回放

1. 使用 `ypd_ba_shadow.yaml`。
2. 记录所有 debug 指标。
3. 不改变主滤波或只启用 `R_YPD` 的小 `lambda`。
4. 对比 fixed baseline。

### Step 6：低权重灰度

1. 仅在 `cov_valid=true` 后使用 `ypd_ba_low_weight.yaml`。
2. `w <= 0.05 * confidence^2`。
3. 逐步评估 NIS、gate、track lost。

## 17. 验收标准

### 17.1 编译与兼容

1. 旧 fixed profile 编译运行通过。
2. `ukf_v1` 不受影响。
3. `ukf_v2` fixed 行为与改造前一致。
4. `inekf` fixed 行为与改造前一致。

### 17.2 功能

1. `ukf_v2` 能使用 `ypd_ba_shadow.yaml` 构造 single/dual R。
2. `inekf` 能使用同一 noise model。
3. 缺少 image/covariance 时自动回退。
4. `R_final` 始终 SPD。
5. dual R 正确 block diagonal。

### 17.3 统计

1. shadow 模式下 `ba_used=false`，主链路 NIS 不劣化。
2. `R_YPD` 随距离和 bbox 尺寸变化符合直觉。
3. `cov_valid=false` 不进入 BA 融合。
4. 低权重启用后 gate pass ratio 不明显下降。

## 18. 后续衔接

第一阶段完成后，第二阶段重点：

1. 在 `armor_pnp_refiner::single_xyz_yaw` 中计算真实 Hessian / marginal covariance。
2. 输出有效 `condition_number`。
3. 完成 covariance 的 camera -> tracker frame 转换单元测试。
4. 用 bag 回放标定 `s_BA / max_weight / lambda`。
5. 评估滑窗 covariance 是否只做 pose refine，不直接提供 R_BA。

## 19. 最终建议

建议第一阶段正式推进，并采用以下默认策略：

```text
ukf_v2: ypd_ba_shadow 起步，确认后 lambda=0.1/0.3
inekf:  ypd_ba_shadow 起步，低于 ukf_v2 的 lambda 和 w
R_BA:   完整链路实现，但 cov_valid=false 时 w=0
```

第一阶段的成功标志不是“R_BA 立即提升命中率”，而是：

```text
动态 R 框架完整、回退可靠、debug 可解释、ukf_v2/InEKF 共用、后续真实 R_BA 可无痛接入。
```
