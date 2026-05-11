# 基于 armor_pnp_refiner 的第一阶段 R_BA 可行性重新审计

日期：2026-05-11

## 1. 审计结论

在引入 `src/rm_auto_aim/armor_pnp_refiner` 的现有设计与实现后，第一阶段引入 `R_BA` 的可行性需要从“暂不适合”修正为：

```text
第一阶段可以引入 R_BA 数据链路与保守协方差诊断；
但不建议第一阶段把它作为高权重、真实 Hessian marginal covariance 参与主滤波。
```

更具体地说：

1. `armor_pnp_refiner` 已经实现 `PnpRefineOutput`，其中包含 `covariance_xyz_yaw`、`covariance_valid`、`confidence`、`reproj_error_*`、`chi2_per_dof`、`condition_number` 等 R_BA 所需字段。
2. `armor_detector` 已经接入 `armor_pnp_refiner`，支持 `pnp_refiner.mode = single_xyz_yaw | sliding_window`，并能用 refined pose 替换 PnP pose。
3. 当前 `single_xyz_yaw` 和 `sliding_window` 的协方差仍是保守对角 placeholder，代码明确设置 `covariance_valid = false`，尚未从 g2o Hessian / marginal covariance 计算真实 `R_BA`。
4. 当前 `rm_interfaces/msg/Armor.msg`、`gimbal_pipeline::ObservationData`、`msg_converter` 和 `TFHandler` 还没有承载和转换 `R_BA` 元数据。
5. `armor_detector_nn` 侧 `PoseEstimate` 尚未集成 `armor_pnp_refiner` 输出；`armor_pnp_refiner` 的 NN adapter 目前也是占位返回 fallback。

因此，第一阶段的合理目标应调整为：

```text
P1-RBA-link = detector/refiner 输出质量与保守 covariance
           -> Armor.msg
           -> ObservationData
           -> dynamic R builder shadow / low-weight fusion
```

而不是：

```text
P1-RBA-full = 直接用 g2o Hessian-derived R_BA 替代或主导观测噪声
```

## 2. 新证据摘要

### 2.1 `armor_pnp_refiner` 已有完整输出类型

`PnpRefineOutput` 当前包含：

```cpp
Eigen::Matrix4d covariance_xyz_yaw;
bool covariance_valid;
double confidence;
double reproj_error_raw_px;
double reproj_error_refined_px;
double yaw_delta_rad;
double pose_delta_m;
double chi2_per_dof;
double condition_number;
double cost_before;
double cost_after;
int num_points;
int num_inliers;
```

并且 `pnp_refiner_types.hpp` 已定义：

```cpp
struct ObservationCovarianceMeta {
  bool valid{false};
  bool cov_valid{false};
  Eigen::Matrix4d cov_xyz_yaw{Eigen::Matrix4d::Identity()};
  double confidence{0.0};
  double reproj_rms{0.0};
  double condition_number{0.0};
  int num_observations{0};
  int num_inliers{0};
};
```

这说明 R_BA 所需的数据结构已经在 `armor_pnp_refiner` 内部成型。

### 2.2 `armor_detector` 已经接入 refiner

`armor_detector/src/armor_pose_estimator.cpp` 中已经有：

```cpp
void ArmorPoseEstimator::configurePnpRefiner(bool enable, const std::string &mode)
```

并且在 PnP 后构造 `PnpRefineInput`，调用：

```cpp
const auto refined = pnp_refiner_->refine(input);
if (refined.valid) {
  R = refined.q_camera_armor.toRotationMatrix();
  t = refined.t_camera_armor;
}
```

这意味着传统 detector 侧已经不是“完全没有 BA/refiner 链路”，而是“refiner 结果只用于 pose，尚未发布质量与协方差元数据”。

### 2.3 单帧 xyz-yaw 优化已实现，但 covariance 仍不是 Hessian marginal

`single_xyz_yaw_optimizer.cpp` 已实现状态：

```text
[tx, ty, tz, yaw]
```

并添加：

1. 角点重投影边。
2. translation prior。
3. yaw prior。
4. robust kernel。
5. reprojection error / chi2 / inlier / pose delta 诊断。

但当前 covariance 生成是：

```cpp
// Conservative diagonal covariance (placeholder, not Hessian marginal).
output.covariance_xyz_yaw(0, 0) = scale * config_.min_var_x;
output.covariance_xyz_yaw(1, 1) = scale * config_.min_var_y;
output.covariance_xyz_yaw(2, 2) = scale * config_.min_var_z;
output.covariance_xyz_yaw(3, 3) = scale * config_.min_var_yaw;
output.covariance_valid = false;
```

因此，当前实现已经能提供“保守 R_BA 形态字段”，但不能声称提供“真实 BA 协方差”。

### 2.4 滑窗优化已实现，但 covariance 仍不应第一阶段高权重使用

`sliding_window_optimizer.cpp` 已实现多帧 `xyz-yaw` 优化和二阶/三阶平滑边，但当前 covariance 同样是 placeholder：

```cpp
// Conservative covariance placeholder (not Schur-marginal covariance yet).
output.covariance_valid = false;
```

滑窗 pose refine 对稳定输出有价值，但滑窗 covariance 更容易受关联污染和平滑先验影响，第一阶段最多用于日志或 shadow，不建议直接作为主滤波 R_BA。

### 2.5 `CovarianceEstimator` 存在，但还未真正接入 Hessian marginal

`covariance_estimator.cpp` 中接口已经规划：

```cpp
computeSingleFrame(...)
computeWindowMarginal(...)
buildFinalCovariance(...)
```

但当前实现说明：

```cpp
// For simplicity in Phase1, use conservative diagonal.
```

这再次说明：第一阶段可以接入数据链路，但如果要让 `cov_valid=true`，还需要补齐从 g2o 线性化信息到 4D covariance 的真实计算。

## 3. 当前链路缺口重新评估

### 3.1 已经解决的缺口

相较于早先评估，以下缺口已经明显改善：

1. **refiner 统一输出类型**：已具备 `PnpRefineOutput`。
2. **单帧 xyz-yaw 状态**：已具备 `G2O_SINGLE_XYZ_YAW`。
3. **滑窗 xyz-yaw 状态**：已具备 `G2O_WINDOW_XYZ_YAW`。
4. **质量诊断字段**：已有 reprojection、delta、cost、inlier、confidence 等。
5. **保守协方差字段**：已有 `covariance_xyz_yaw` 字段和 floor/clamp 相关工具。
6. **传统 detector 接入点**：`armor_detector` 已可配置启用 refiner。

这意味着第一阶段可以不再停留于“只做 R_YPD”，而是可以同步推进 R_BA 链路打通。

### 3.2 仍未解决的缺口

但以下缺口仍然阻止“第一阶段高权重使用真实 R_BA”：

1. `covariance_valid=false`：当前优化器没有输出可信 Hessian marginal。
2. `condition_number` 基本未计算：`ConfidenceEstimator` 虽使用该字段，但优化器没有填入有效值。
3. `Armor.msg` 无字段：无法把 refiner covariance/quality 发布给 gimbal pipeline。
4. `ObservationData` 无字段：无法在 tracker 内部承载 BA/PnP covariance。
5. `TFHandler` 未转换 covariance：pose 已从 camera frame 转到 target frame，但 covariance 没有同步旋转。
6. `armor_detector_nn` 未集成：NN 侧 `PoseEstimate` 还没有承接 `PnpRefineOutput`。
7. `IMeasurementNoiseModel` 仍是 `build_R(UpdateKind)`：无法根据单个观测读取 `R_BA`。

所以当前不是算法方向不可行，而是“接口链路、坐标变换、真实 covariance 计算”还没有闭环。

## 4. 第一阶段引入 R_BA 的推荐边界

### 4.1 建议允许进入第一阶段的内容

第一阶段建议纳入：

1. `Armor.msg` append-only 扩展 R_BA/quality 字段。
2. `ObservationData` append-only 扩展 `ObservationCovarianceMeta`。
3. `armor_detector` 发布 `PnpRefineOutput` 中的 quality/covariance 元数据。
4. `armor_detector_nn::PoseEstimate` 扩展同构字段，先从现有 refiner quality 填充，后续再接 `armor_pnp_refiner`。
5. `msg_converter` 和 `TFHandler` 复制、转换 covariance 元数据。
6. `IMeasurementNoiseModel` 改为 per-observation R 接口。
7. `BaAwareNoiseModel` 支持读取 `obs.ba_pnp`，但默认 `use_ba_covariance=false` 或 `w=0`。
8. 日志记录 `R_YPD / R_BA / R_final / confidence / cov_valid / NIS`。

这部分收益明确，而且风险可控。

### 4.2 不建议第一阶段直接做的内容

第一阶段不建议：

1. 不建议将 `covariance_valid=false` 的 placeholder 当作真实 `R_BA` 高权重使用。
2. 不建议滑窗 covariance 直接影响主滤波。
3. 不建议让 `R_BA` 绕过 `R_floor / R_fixed / R_YPD / lambda` 直接裸用。
4. 不建议在未完成坐标系转换前让 camera-frame covariance 进入 odom-frame tracker。
5. 不建议把 `single_yaw` 输出伪装成完整 `xyz+yaw` covariance。

## 5. 第一阶段融合策略修订

基于当前实现，推荐第一阶段分两档：

### 5.1 P1a：链路打通与 shadow

配置：

```text
use_ba_covariance = false
w_ba = 0
lambda = 0 或仅对 R_YPD 生效
```

行为：

1. detector/refiner 发布 covariance/quality 字段。
2. gimbal pipeline 接收并保存到 `ObservationData`。
3. 噪声模型计算 shadow `R_BA_used`，但不影响滤波更新。
4. 记录动态 R 与 NIS 统计。

验收：

1. 默认主链路行为与当前一致。
2. 所有消息字段可回放、可记录。
3. `cov_valid=false` 的观测不会进入 BA 融合。
4. TF 转换后的 covariance 坐标系定义清楚。

### 5.2 P1b：保守低权重启用

只有满足以下条件才允许：

```text
obs.ba_pnp.valid = true
obs.ba_pnp.cov_valid = true
condition_number < threshold
reproj_rms < threshold
confidence > threshold
covariance positive definite
coordinate_frame_aligned = true
```

融合公式：

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
R_final = R_floor + (1 - lambda) R_fixed + lambda * R_dynamic
```

保守起步：

```text
w <= 0.05 * confidence^2
s_BA >= 4.0
lambda <= 0.3
```

在当前实现 `covariance_valid=false` 的情况下，P1b 实际不会启用 BA 协方差；它只是把安全门写好，等 Hessian covariance 完成后自然生效。

## 6. 对 PoseEstimate 与 detector 链路的审计建议

### 6.1 `armor_detector` 链路

当前状态：

1. 已接入 `ArmorPnpRefiner`。
2. refined pose 已用于发布。
3. `PnpRefineOutput` 的 covariance/quality 尚未写入 `Armor.msg`。

建议第一阶段修改：

1. 在 `extractArmorPoses()` 内保留 `PnpRefineOutput refined_meta`。
2. 若 refiner 执行过，将 `refined_meta` 映射到 `rm_interfaces::msg::Armor` 新字段。
3. 对 `single_yaw` 模式，强制 `pose_covariance_valid=false`，但允许发布 `pose_quality_score / reproj_error_refined / pose_estimate_mode`。
4. 对 `single_xyz_yaw` / `sliding_window`，只有当实现真正设置 `covariance_valid=true` 时才发布 valid covariance。

### 6.2 `armor_detector_nn` / `PoseEstimate` 链路

当前状态：

1. `PoseEstimate` 有 `quality_score`、`reproj_error_raw`、`reproj_error_refined`、`track_id`。
2. `PoseEstimate` 没有 covariance 字段。
3. `armor_pnp_refiner` 的 NN adapter 是占位 fallback，未真正转换 NN 类型。

建议第一阶段修改：

```cpp
struct PoseEstimate {
  ...
  bool covariance_valid{false};
  Eigen::Matrix4d covariance_xyz_yaw{Eigen::Matrix4d::Identity()};
  double condition_number{0.0};
  int num_inliers{0};
  int num_points{0};
};
```

并在发布 `Armor.msg` 时同步：

1. `pose_quality_score`
2. `reproj_error_raw`
3. `reproj_error_refined`
4. `pose_estimate_mode`
5. `pose_covariance_valid`
6. `pose_covariance_xyz_yaw`
7. `pose_condition_number`
8. `pose_num_inliers / pose_num_points`

短期可以不强制 NN 侧使用 `armor_pnp_refiner`，先把已有 `PoseEstimate` 质量字段传给下游；但如果目标是统一 `R_BA`，后续应让 NN 的 refiner 输出也转换为同一套元数据。

## 7. 消息与 ObservationData 建议扩展

### 7.1 `Armor.msg`

建议 append-only 增加：

```text
# Pose/refiner quality metadata
uint8 pose_estimate_mode
float32 pose_quality_score
float32 reproj_error_raw
float32 reproj_error_refined
float32 pose_condition_number
uint16 pose_num_points
uint16 pose_num_inliers

# Covariance of [x, y, z, yaw] in the message pose frame.
bool pose_covariance_valid
float64[16] pose_covariance_xyz_yaw
```

关键约定：

```text
pose_covariance_xyz_yaw 必须与 Armor.pose 所在 frame 一致。
```

如果 detector 发布 camera frame，而 gimbal pipeline 再 TF 到 odom，则有两种选择：

1. detector 只发布 camera-frame covariance，gimbal pipeline 在 TFHandler 中转换。
2. detector 若已发布 target-frame pose，则同步发布 target-frame covariance。

不允许 pose 与 covariance 处于不同 frame。

### 7.2 `ObservationData`

建议增加：

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
  int pose_estimate_mode = 0;
};

std::optional<ObservationCovarianceMeta> ba_pnp;
```

该字段应为 append-only，旧 tracker 忽略即可。

## 8. 坐标系与 yaw 语义风险

当前 `msg_converter.hpp` 有一个重要语义转换：

```cpp
double yaw_normal = quaternion_to_yaw(armor.pose.orientation);
double yaw_radial = yaw_normal + M_PI;
obs.yaw = yaw_radial;
```

这意味着 `R_BA` 中 yaw covariance 若来自 detector 的 armor normal yaw，需要与 tracker 的 radial yaw 对齐。加 `pi` 本身不改变 yaw 方差，但如果 covariance 中存在 `xyz-yaw` 相关项，yaw 定义和 frame 旋转就必须严格一致。

第一阶段建议：

1. 只在 `cov_valid=true` 且明确 frame 对齐时使用 `R_BA`。
2. 如果无法可靠转换 `xyz-yaw` 相关项，先只使用 diagonal 或 block-diagonal conservative covariance。
3. 对 camera -> odom 的 covariance 转换，至少实现 `R_xyz_odom = R_oc * R_xyz_cam * R_oc^T`。
4. yaw 交叉项第一阶段可选择清零，以降低语义风险。

## 9. 重新评估后的收益/风险

### 9.1 收益

引入 `armor_pnp_refiner` 后，第一阶段收益上升：

1. 可以统一 detector pose refine 输出和 tracker 观测噪声建模。
2. 可以把 reprojection、quality、inlier、mode 等诊断真正传到下游。
3. 可以在不改变主滤波的情况下收集 `R_BA` 统计，为后续标定提供数据。
4. 对 `single_xyz_yaw`，后续补齐 Hessian covariance 后，观测空间与 Norm4 的 `xyz+yaw` 对齐度较高。

### 9.2 风险

风险仍然不低，但可通过边界控制：

1. 当前 covariance 是 placeholder，不能高权重使用。
2. 消息接口扩展会影响编译和 bag 兼容。
3. 坐标转换和 yaw radial/normal 语义容易出错。
4. 滑窗 covariance 可能受平滑先验和关联污染影响。
5. 未计算 condition number 时，confidence 会偏乐观或缺少退化判据。

### 9.3 最终风险评级

| 子任务 | 第一阶段可行性 | 风险 | 备注 |
|---|---:|---:|---|
| 发布 quality/reproj/mode | 高 | 低 | 已有字段来源 |
| 发布 conservative covariance | 高 | 中 | 必须 `cov_valid=false` 或低权重 |
| `Armor.msg`/`ObservationData` 扩展 | 高 | 中 | 跨包接口变化 |
| `R_BA` shadow 记录 | 高 | 低-中 | 不影响主状态 |
| `R_BA` 低权重启用 | 中 | 中 | 需 cov_valid 与 TF 转换闭环 |
| Hessian marginal covariance | 中 | 中-高 | 需要补实现与验证 |
| 滑窗 covariance 主链路使用 | 低-中 | 高 | 不建议第一阶段 |

## 10. 修订后的第一阶段任务清单

建议第一阶段拆成以下工作包：

### A. 数据契约

1. 扩展 `rm_interfaces/msg/Armor.msg`。
2. 扩展 `ObservationData`。
3. 扩展 `armor_detector_nn::PoseEstimate`。
4. 明确 covariance frame：与 `Armor.pose` frame 一致。

### B. detector 发布链路

1. `armor_detector` 从 `PnpRefineOutput` 填充 `Armor.msg` 新字段。
2. `armor_detector_nn` 从 `PoseEstimate` 填充同一套字段。
3. 对 `covariance_valid=false` 的输出，仍发布质量字段，但不发布有效 covariance。

### C. pipeline 接收链路

1. `msg_converter` 读取新字段到 `ObservationData::ba_pnp`。
2. `TFHandler` 同步转换 covariance 或清晰标记未转换不可用。
3. 日志增加 refiner quality/covariance 字段。

### D. 噪声模型

1. `IMeasurementNoiseModel` 改为 per-observation R。
2. 实现 `BaAwareYpdNoiseModel`。
3. 默认 `use_ba_covariance=false`。
4. shadow 记录 `R_BA_used`、`R_YPD`、`R_final`、NIS。

### E. covariance 实算补齐

1. 在 `single_xyz_yaw` 中接入 `CovarianceEstimator`。
2. 从最终线性化点提取或重建 `J^T W J`。
3. 输出 `condition_number`。
4. 通过 `checkCovariance / clampEigenvalues` 后才允许 `covariance_valid=true`。

E 可以作为第一阶段后半段或第一阶段验收后的增强，不应阻塞 A-D。

## 11. 修订后的推荐结论

基于 `armor_pnp_refiner` 已实现的内容，第一阶段引入 `R_BA` 链路是可行的，而且比先前评估更值得做。但第一阶段的“R_BA”应被定义为：

```text
R_BA metadata / conservative covariance / shadow fusion infrastructure
```

而不是：

```text
trusted Hessian-derived BA covariance in main filter
```

推荐落地策略：

1. 第一阶段立即打通 `PnpRefineOutput -> Armor.msg -> ObservationData -> NoiseModel`。
2. 主滤波默认仍以 `R_fixed + R_YPD` 为主。
3. `R_BA` 默认 shadow；若 `cov_valid=false`，强制 `w=0`。
4. 只有当 `single_xyz_yaw` 真正输出 `cov_valid=true`、完成坐标转换和 NIS 回放验证后，才以 `w <= 0.05 * confidence^2` 小权重启用。

一句话版本：

```text
可以在第一阶段引入 R_BA 链路，但第一阶段不要信任当前 placeholder covariance；先链路化、日志化、shadow 化，再低权重灰度。
```
