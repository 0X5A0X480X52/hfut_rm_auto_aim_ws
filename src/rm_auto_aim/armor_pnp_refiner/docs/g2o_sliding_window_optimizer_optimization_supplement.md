# g2o 滑窗重投影优化模块补充设计：优化细化与风险修正

> 本文档作为 `g2o_sliding_window_reprojection_optimizer_design.md` 的补充文档，不替代原设计文档。  
> 重点细化 `SlidingWindowOptimizer` 的优化建模、残差权重、协方差估计、置信度输出、门控回退策略，并对原设计中的部分风险项和默认参数提出修正建议。

---

## 1. 补充目标

原设计文档已经完成了模块边界、目录结构、接入路径、g2o 顶点/边草案、配置和测试计划等总体设计。本文档进一步补充以下内容：

1. 明确滑窗优化的工程定位：**只生成 detector 级高质量观测，不替代 tracker / robot_pose_estimator 的整车状态估计**。
2. 细化 `XYZ + yaw` 模式下的优化变量、残差项、权重设计和求解流程。
3. 增加当前帧 `xyz + yaw` 的协方差输出设计。
4. 增加结果置信度 `confidence` 与状态 `GOOD / DEGRADED / REJECT` 的计算方式。
5. 修正原设计中的若干风险点，尤其是：
   - 窗口大小和时间跨度偏大；
   - 一阶平滑边容易造成零速度偏置；
   - optimizer 内部维护 tracker-manager 的职责边界偏重；
   - 直接使用 Hessian 协方差可能过度自信；
   - 固定 pitch / roll 的系统偏差风险；
   - 滑窗 BA 与后端 UKF/InEKF 之间可能重复平滑。

---

## 2. 设计定位修正

### 2.1 原定位

原设计中滑窗优化模块的总体定位是：

```text
Detector keypoints + class/type + CameraInfo + PnP result
        |
        v
ArmorPoseObservation
        |
        v
armor_pose_graph_optimizer
        |
        +--> SingleFrameOptimizer
        +--> SlidingWindowOptimizer
        |
        v
ArmorPoseOptimizationResult
```

该定位基本正确，但需要进一步强调：

> `SlidingWindowOptimizer` 输出的是当前帧的高质量观测，而不是最终目标状态。

也就是说，滑窗 BA 不应该直接替代后端 `armor_tracker`、`gimbal_pipeline`、`robot_pose_estimator` 中的 UKF/InEKF/IMM 等状态估计器。

### 2.2 修正后的工程定位

建议将模块定位为：

```text
Detector / NN detector
  ↓
Single-frame PnP
  ↓
G2O Single / SlidingWindow PnP Refiner
  ↓
refined observation:
  z_refined = [x, y, z, yaw]
  R_refined = 4x4 observation covariance
  confidence ∈ [0, 1]
  status = GOOD / DEGRADED / REJECT
  ↓
UKF / InEKF / IMM / hypothesis backend
```

因此，滑窗优化模块的输出应从：

```cpp
translation + rotation + reproj_error + quality_score
```

扩展为：

```cpp
z_current = [x, y, z, yaw]
R_current = 4x4 covariance
confidence = scalar quality score
status = GOOD / DEGRADED / REJECT
diagnostics = reproj_rmse / chi2_per_dof / condition / inlier_ratio / cost_delta
```

---

## 3. 对原输出结构的补充建议

原文中的 `ArmorPoseOptimizationResult`：

```cpp
struct ArmorPoseOptimizationResult {
  bool valid{false};
  Mode mode{Mode::PNP_FALLBACK};

  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  double reproj_error_raw_px{0.0};
  double reproj_error_refined_px{0.0};
  double pose_delta_m{0.0};
  double yaw_delta_rad{0.0};
  double quality_score{0.0};

  std::string reason;
};
```

建议补充为：

```cpp
struct ArmorPoseOptimizationResult {
  enum class Mode {
    PNP_FALLBACK,
    G2O_SINGLE_POSE,
    G2O_SINGLE_YAW,
    G2O_WINDOW_POSE,
    G2O_WINDOW_XYZ_YAW
  } mode{Mode::PNP_FALLBACK};

  enum class Status {
    GOOD,
    DEGRADED,
    REJECT
  } status{Status::REJECT};

  bool valid{false};

  // Current-frame refined pose observation.
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  // Recommended compact observation for downstream filter.
  // z_current = [x, y, z, yaw]
  Eigen::Matrix<double, 4, 1> z_current{Eigen::Matrix<double, 4, 1>::Zero()};

  // Covariance from BA marginalization before engineering inflation.
  Eigen::Matrix4d R_ba{Eigen::Matrix4d::Identity()};

  // Final covariance for UKF/InEKF measurement update.
  Eigen::Matrix4d R_final{Eigen::Matrix4d::Identity()};

  // Quality and diagnostics.
  double confidence{0.0};
  double reproj_error_raw_px{0.0};
  double reproj_error_refined_px{0.0};
  double chi2_per_dof{0.0};
  double condition_number{0.0};
  double inlier_ratio{1.0};
  double cost_before{0.0};
  double cost_after{0.0};
  double pose_delta_m{0.0};
  double yaw_delta_rad{0.0};
  double solver_time_ms{0.0};
  int window_size_used{0};
  int num_observations{0};
  int num_inliers{0};

  std::string reason;
};
```

其中：

```text
R_ba     = 由 Hessian / marginal covariance 得到的理论协方差
R_final  = 加入下限、残差缩放、时间误差、关联误差后的最终观测噪声
confidence = 用于回退、降级、调试统计的综合置信度
```

---

## 4. 滑窗优化变量设计

### 4.1 首版推荐变量

首版 `G2O_WINDOW_XYZ_YAW` 模式建议每帧只优化：

```text
x_k = [t_x, t_y, t_z, yaw]^T
```

窗口内总变量：

```text
X = {x_0, x_1, ..., x_{N-1}}
```

输出只取当前帧：

```text
z_current = x_{N-1}
```

不建议首版显式加入：

```text
velocity, acceleration, jerk, yaw_rate, yaw_acc, yaw_jerk
```

原因：

1. 3~5 帧窗口过短，高阶变量容易吸收关键点噪声。
2. detector 级滑窗 BA 主要目的是降低单帧 PnP 随机抖动，而不是承担长期运动估计。
3. 后端 UKF/InEKF 已经负责速度、角速度和模型切换，前端 BA 不应重复估计完整动态状态。

### 4.2 pitch / roll 处理

原设计默认在 `XYZ + yaw` 中固定 pitch / roll，这个工程折中是可行的，但需要补充风险说明。

建议支持三种策略：

```cpp
enum class PitchRollMode {
  FIXED_FROM_CONFIG,     // pitch/roll 由配置给定
  FROM_PNP,              // pitch/roll 使用单帧 PnP 初值
  FROM_EXTERNAL_ATTITUDE // 由 IMU / robot frame / old BaSolver 约定给定
};
```

推荐默认：

```text
armor_detector_nn: FROM_PNP 或 FIXED_FROM_CONFIG
armor_detector: 与 legacy BaSolver 保持一致
```

风险：如果 pitch/roll 固定值与真实装甲板姿态存在系统偏差，则 `xyz/yaw` 优化会把该偏差转移到平移和 yaw 中。因此，应在输出的 `R_final` 中加入系统误差项：

```text
R_final = R_floor + R_ba_scaled + R_system
```

其中 `R_system` 用于覆盖固定 pitch/roll、外参、内参等不可由滑窗残差直接消除的系统误差。

---

## 5. 残差项设计

整体优化目标：

```text
min_X
  Σ ρ(||r_proj||^2)
+ Σ ||r_pnp||^2
+ Σ ||r_motion||^2
+ 可选 Σ ||r_tracker_prior||^2
```

建议第一版保留四类边：

1. 重投影边 `EdgeXyzYawReprojection`
2. PnP 弱先验边 `EdgeXyzYawPrior`
3. 二阶有限差分运动平滑边 `EdgeSecondOrderSmooth`
4. 可选当前帧 tracker prediction prior `EdgeCurrentPredictionPrior`

---

## 6. 重投影残差细化

### 6.1 残差定义

对窗口内每一帧 `k`、每个关键点 `j`：

```text
r_proj(k, j) = u_observed(k, j) - π(K, D, T_camera_armor(x_k) · P_object(j))
```

其中：

```text
x_k = [t_x, t_y, t_z, yaw]^T
T_camera_armor(x_k) = [R_camera_armor(yaw, pitch, roll), t]
```

如果使用 `R_imu_camera` 或 `R_camera_imu` 来保持 yaw 定义一致，需要在边内部明确约定：

```text
R_camera_armor = R_camera_imu · R_imu_armor(yaw, fixed_pitch, fixed_roll)
```

或者：

```text
R_camera_armor = R_camera_imu · R_z(yaw) · R_y(pitch) · R_x(roll)
```

注意：必须和 legacy `BaSolver`、`SingleYawRefiner` 的 yaw 语义一致，否则优化方向会出现看似收敛但实际 yaw 反向的问题。

### 6.2 像素噪声权重

原设计中已有：

```text
Information = diag(1 / sigma_u^2, 1 / sigma_v^2)
```

建议细化为：

```text
sigma_px_j = sigma_px_base * f_conf(conf_j) * f_area(area) * f_distance(distance)
```

第一版可以简化为：

```text
sigma_px_j = clamp(
  sigma_px_min + (1 - keypoint_conf_j) * sigma_px_scale,
  sigma_px_min,
  sigma_px_max
)
```

建议初值：

```yaml
pixel_noise:
  sigma_px_base: 1.5
  sigma_px_min: 0.8
  sigma_px_max: 6.0
  sigma_px_scale: 4.0
```

对应信息矩阵：

```text
Ω_proj_j = diag(1 / sigma_px_j^2, 1 / sigma_px_j^2)
```

### 6.3 鲁棒核

建议对每个重投影边使用 Huber kernel：

```yaml
robust_kernel:
  type: huber
  delta_px: 3.0
```

含义：

```text
小残差：近似二范数
大残差：降低 outlier 对优化的拉动
```

对于误检、遮挡、关键点错序，Huber 只能降低影响，不能完全解决。若角点顺序错误或 armor_id 错误，应由外部 hypothesis / association 层处理。

---

## 7. PnP 弱先验边细化

### 7.1 残差定义

对每帧：

```text
r_prior_k = [
  t_x - t_x_pnp,
  t_y - t_y_pnp,
  t_z - t_z_pnp,
  wrap(yaw - yaw_pnp)
]^T
```

### 7.2 信息矩阵

```text
Ω_prior = diag(
  1 / σ_prior_x^2,
  1 / σ_prior_y^2,
  1 / σ_prior_z^2,
  1 / σ_prior_yaw^2
)
```

推荐：

```yaml
pnp_prior:
  sigma_x: 0.08
  sigma_y: 0.08
  sigma_z: 0.15
  sigma_yaw_rad: 0.12
```

原设计中 `prior_sigma_yaw_rad = 0.35` 对 yaw 约束偏弱，可以作为远距离/低置信度下限，但首版若 yaw-only 或 xyz+yaw 需要稳定，可先从 `0.08 ~ 0.15 rad` 测试。

### 7.3 距离相关放大

PnP 深度误差通常随距离放大，建议二期加入：

```text
σ_prior_z(d) = σ_z0 + k_z · d^2
σ_prior_xy(d) = σ_xy0 + k_xy · d
σ_prior_yaw(d) = σ_yaw0 + k_yaw · d
```

其中 `d = ||t_pnp||`。

---

## 8. 帧间运动平滑边修正

### 8.1 原设计问题

原文中的平滑边为：

```text
EdgeTranslationSmooth: error = t_k - t_{k-1}
EdgeYawSmooth:         error = wrap(yaw_k - yaw_{k-1})
```

这相当于一阶平滑，会偏向：

```text
目标在窗口内静止
```

对于横向移动、前后移动、装甲板绕车体旋转，这会引入零速度偏置，表现为：

1. 平移被拉回上一帧，产生滞后；
2. yaw 被过度平滑，快速旋转时跟随不足；
3. 若权重过大，滑窗 BA 会比单帧 PnP 更“稳”但更偏。

### 8.2 推荐改为二阶有限差分

建议首版使用二阶平滑边：

```text
r_acc_k = x_k - 2 x_{k-1} + x_{k-2}
```

对平移：

```text
r_acc_t = t_k - 2t_{k-1} + t_{k-2}
```

对 yaw：

```text
r_acc_yaw = wrap(yaw_k - 2yaw_{k-1} + yaw_{k-2})
```

直观含义：

```text
允许匀速运动，不强行压制速度；
主要抑制短窗口内不合理的高频加速度抖动。
```

### 8.3 信息矩阵

```text
Ω_acc = diag(
  1 / σ_acc_x^2,
  1 / σ_acc_y^2,
  1 / σ_acc_z^2,
  1 / σ_acc_yaw^2
)
```

推荐初值：

```yaml
motion_smooth:
  type: second_order
  acc_sigma_x: 0.12
  acc_sigma_y: 0.12
  acc_sigma_z: 0.20
  acc_sigma_yaw_rad: 0.12
```

注意：这里的 `sigma` 不是物理加速度单位，而是窗口内二阶差分残差的允许尺度。后续若要严格按时间归一化，可以写成：

```text
r_acc_t = (t_k - 2t_{k-1} + t_{k-2}) / Δt^2
```

此时 `sigma` 才对应近似加速度噪声。首版建议不要时间归一化，避免 90 fps 下 `Δt^2` 很小导致数值过敏。

### 8.4 可选三阶有限差分

对于 90 fps、3~5 帧窗口，三阶弱正则是可控的，但不建议首版默认开启。

三阶残差：

```text
r_jerk_k = x_k - 3x_{k-1} + 3x_{k-2} - x_{k-3}
```

yaw：

```text
r_jerk_yaw = wrap(yaw_k - 3yaw_{k-1} + 3yaw_{k-2} - yaw_{k-3})
```

建议：

```yaml
motion_smooth:
  enable_jerk: false
  jerk_sigma_x: 0.25
  jerk_sigma_y: 0.25
  jerk_sigma_z: 0.40
  jerk_sigma_yaw_rad: 0.25
```

使用原则：

```text
二阶平滑是默认项；
三阶平滑只作为高速变速旋转场景下的弱正则；
不要显式估计 jerk 状态。
```

---

## 9. 窗口大小和时间跨度修正

### 9.1 原设计问题

原配置：

```cpp
int window_size{8};
int min_window_size{4};
double max_time_span_ms{300.0};
```

对于 90 fps：

```text
单帧间隔 ≈ 11.1 ms
8 帧覆盖 ≈ 77.8 ms
300 ms 对应 ≈ 27 帧
```

问题：

1. `max_time_span_ms = 300 ms` 对 detector 级实时精修过大；
2. 若后端还有 UKF/InEKF，会造成重复平滑和延迟；
3. 目标急转或装甲板切换时，窗口污染风险增加；
4. 8 帧窗口在 CPU 紧张时可能影响实时性。

### 9.2 推荐修正

对于 90 fps 左右的自瞄链路，建议：

```yaml
g2o_window:
  window_size: 5
  min_window_size: 3
  max_time_span_ms: 60.0
  max_iterations: 5
  max_solver_time_ms: 2.0
```

如果帧率下降到 30 fps：

```yaml
g2o_window:
  window_size: 3
  min_window_size: 2
  max_time_span_ms: 100.0
```

推荐原则：

```text
高帧率：3~5 帧，时间跨度 30~60 ms
低帧率：2~3 帧，时间跨度 60~100 ms
不建议 detector 级 BA 覆盖 150 ms 以上历史
```

---

## 10. 协方差输出设计

### 10.1 为什么需要协方差

滑窗 BA 的输出不应只有 refined pose，还应输出该观测的可信程度。后端 UKF/InEKF 需要：

```text
z = [x, y, z, yaw]
R = observation covariance
```

否则会出现两个问题：

1. BA 结果被后端过度相信，错误观测会直接拉飞 tracker；
2. BA 结果质量变化无法反映到滤波器更新中。

### 10.2 Hessian 反演得到 BA 协方差

优化收敛后，在最终线性化点处有：

```text
H = J^T W J
```

若窗口变量为：

```text
X = [x_0, x_1, ..., x_{N-1}]
```

只关心当前帧：

```text
x_c = x_{N-1}
```

将 Hessian 分块：

```text
H = [ H_oo  H_oc
      H_co  H_cc ]
```

当前帧边缘信息矩阵：

```text
Ω_c = H_cc - H_co H_oo^{-1} H_oc
```

当前帧边缘协方差：

```text
Σ_c = Ω_c^{-1}
```

即：

```text
R_ba = Σ_c
```

如果实现复杂，第一版可以先使用近似：

```text
R_ba ≈ inverse(H_cc)
```

但需要标记为近似，并在 `R_final` 中增加保守放大系数。

### 10.3 残差尺度缩放

Hessian 反演得到的协方差通常偏乐观，尤其在像素噪声假设不准时。建议使用：

```text
s² = max(1.0, chi2 / dof)
R_ba_scaled = s² · R_ba
```

其中：

```text
chi2 = Σ r_i^T Ω_i r_i
dof = residual_dim - effective_state_dim
```

### 10.4 协方差下限

必须设置下限：

```yaml
covariance_floor:
  sigma_x_min: 0.01
  sigma_y_min: 0.01
  sigma_z_min: 0.02
  sigma_yaw_min_rad: 0.01
```

即：

```text
R_floor = diag(0.01², 0.01², 0.02², 0.01²)
```

### 10.5 最终 R 设计

推荐：

```text
R_final = R_floor + α · R_ba_scaled + R_time + R_assoc + R_system
```

其中：

```text
α: 工程保守系数，初值 1.5 ~ 3.0
R_time: 时间戳/同步误差导致的不确定性
R_assoc: track_id / hypothesis 关联不确定性
R_system: 内参、外参、固定 pitch/roll、装甲板尺寸误差等系统项
```

第一版可简化为：

```text
R_final = R_floor + 2.0 · R_ba_scaled
```

若 `confidence < 0.75`：

```text
R_final = R_final / max(confidence, 0.2)
```

或：

```text
R_final *= degrade_factor
```

---

## 11. 置信度设计

### 11.1 综合置信度

建议：

```text
confidence =
  c_reproj^0.30
* c_condition^0.20
* c_inlier^0.15
* c_improvement^0.10
* c_motion^0.15
* c_assoc^0.10
```

范围：

```text
confidence ∈ [0, 1]
```

### 11.2 重投影项

```text
c_reproj = exp(-0.5 · max(0, chi2_per_dof - 1))
```

或者：

```text
c_reproj = exp(-reproj_rmse² / sigma_reproj_ref²)
```

建议：

```yaml
confidence:
  sigma_reproj_ref_px: 3.0
```

### 11.3 条件数项

```text
c_condition = clamp(
  log(kappa_bad / condition_number) / log(kappa_bad / kappa_good),
  0,
  1
)
```

建议：

```yaml
confidence:
  condition_good: 1.0e3
  condition_bad: 1.0e7
```

### 11.4 inlier 项

```text
c_inlier = num_inliers / num_points
```

其中 inlier 可按 robust kernel 后的残差判断：

```text
||r_proj|| < huber_delta_px
```

### 11.5 优化收益项

```text
improve_ratio = (cost_before - cost_after) / max(cost_before, eps)
c_improvement = clamp(improve_ratio / expected_improve_ratio, 0, 1)
```

注意：若初值很好，收益小不代表结果差，因此该项权重不宜太大。

### 11.6 运动一致性项

如果有 tracker 预测：

```text
innovation = z_refined - z_pred
S = P_pred + R_final
NIS = innovation^T S^{-1} innovation
```

根据 4 维观测的卡方门限：

```text
NIS < 9.49   约 95% 置信区间
NIS < 13.28  约 99% 置信区间
```

可设计：

```text
c_motion = exp(-0.5 · max(0, NIS - 4) / 4)
```

若没有 tracker 预测，则：

```text
c_motion = 1.0
```

### 11.7 关联置信度项

如果外部已经提供 hypothesis margin：

```text
c_assoc = sigmoid((score_top1 - score_top2 - margin0) / scale)
```

如果没有 hypothesis 信息，则按 track 稳定性给：

```text
confirmed track: c_assoc = 1.0
new track:       c_assoc = 0.5 ~ 0.7
unstable track:  c_assoc = 0.3
```

---

## 12. 状态判定与回退链

### 12.1 GOOD

条件示例：

```text
confidence >= 0.75
reproj_rmse <= 3.0 px
chi2_per_dof <= 3.0
condition_number <= 1e6
finite == true
positive_depth == true
pose_delta_m <= max_pose_delta_m
abs(yaw_delta) <= max_yaw_delta_rad
```

处理：

```text
使用 refined z_current
使用正常 R_final
```

### 12.2 DEGRADED

条件示例：

```text
0.40 <= confidence < 0.75
或窗口帧数不足但优化可用
或重投影误差略高
或 condition_number 偏大
```

处理：

```text
可以输出 refined z_current
但放大 R_final
或只用于调试，不覆盖 PnP
```

建议：

```text
R_final *= 2 ~ 5
```

### 12.3 REJECT

条件示例：

```text
confidence < 0.40
Hessian 不可逆
结果非有限
投影深度为负
reproj_rmse 明显异常
pose_delta / yaw_delta 超限
track_id / armor_id / hypothesis 跳变
```

处理：

```text
g2o_window -> g2o_single -> existing_single_yaw -> pnp
```

---

## 13. Track 管理职责修正

### 13.1 原设计风险

原文提出模块内部维护 `Tracker-manager`，用于给滑窗重投影优化提供稳定 `track_id` 和窗口生命周期管理。

该方向可以落地，但需要警惕职责过重：

1. 优化器变成了 detector + tracker + BA 的混合模块；
2. 与 `armor_detector_nn` 现有 tracker 重复；
3. 与后端 hypothesis / tracker manager 可能发生状态不一致；
4. 优化失败时可能影响 track 生命周期，导致调试困难。

### 13.2 修正建议

建议拆分为两层：

```text
外部：负责 track_id / hypothesis / armor_id
内部：只负责按 track_id 缓存窗口
```

也就是说，`armor_pose_graph_optimizer` 首版不要主动做全局关联，只要求调用方传入稳定 `track_id`。

推荐接口：

```cpp
class TrackWindowManager {
public:
  void pushObservation(const ArmorPoseObservation& obs);
  std::vector<ArmorPoseObservation> getWindow(int track_id) const;
  void resetTrack(int track_id);
  void resetAll();
  void pruneExpired(const rclcpp::Time& now);
};
```

外部如果没有 track_id：

```text
只允许 single-frame 模式
或使用临时 track_id 但不启用滑窗优化
```

二期可以添加可选 `ITrackAssociator`，但不建议作为优化器核心依赖。

---

## 14. 配置修正版

建议将原配置修正为：

```cpp
struct OptimizerConfig {
  std::string mode{"xyz_yaw_window"};

  // Window.
  int window_size{5};
  int min_window_size{3};
  double max_time_span_ms{60.0};

  // Solver.
  double max_solver_time_ms{2.0};
  int max_iterations{5};

  // Reprojection noise.
  double pixel_sigma_base{1.5};
  double pixel_sigma_min{0.8};
  double pixel_sigma_max{6.0};
  double huber_delta{3.0};

  // PnP prior.
  double prior_sigma_x{0.08};
  double prior_sigma_y{0.08};
  double prior_sigma_z{0.15};
  double prior_sigma_yaw_rad{0.12};

  // Motion smooth, second-order by default.
  bool use_second_order_smooth{true};
  double acc_sigma_x{0.12};
  double acc_sigma_y{0.12};
  double acc_sigma_z{0.20};
  double acc_sigma_yaw_rad{0.12};

  // Optional third-order smooth.
  bool use_third_order_smooth{false};
  double jerk_sigma_x{0.25};
  double jerk_sigma_y{0.25};
  double jerk_sigma_z{0.40};
  double jerk_sigma_yaw_rad{0.25};

  // Covariance.
  bool compute_covariance{true};
  double covariance_scale{2.0};
  double min_sigma_x{0.01};
  double min_sigma_y{0.01};
  double min_sigma_z{0.02};
  double min_sigma_yaw_rad{0.01};

  // Gate.
  double max_reproj_error_px{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_rad{20.0 * M_PI / 180.0};
  double good_confidence{0.75};
  double reject_confidence{0.40};

  bool use_robust_kernel{true};
  std::string robust_kernel{"huber"};
  bool require_positive_depth{true};
  bool require_finite{true};
};
```

对应 YAML：

```yaml
pose:
  g2o:
    mode: "xyz_yaw_window"

    window:
      window_size: 5
      min_window_size: 3
      max_time_span_ms: 60.0

    solver:
      max_solver_time_ms: 2.0
      max_iterations: 5

    reprojection:
      pixel_sigma_base: 1.5
      pixel_sigma_min: 0.8
      pixel_sigma_max: 6.0
      huber_delta: 3.0
      use_robust_kernel: true
      robust_kernel: "huber"

    pnp_prior:
      sigma_x: 0.08
      sigma_y: 0.08
      sigma_z: 0.15
      sigma_yaw: 0.12

    motion_smooth:
      use_second_order: true
      acc_sigma_x: 0.12
      acc_sigma_y: 0.12
      acc_sigma_z: 0.20
      acc_sigma_yaw: 0.12
      use_third_order: false
      jerk_sigma_x: 0.25
      jerk_sigma_y: 0.25
      jerk_sigma_z: 0.40
      jerk_sigma_yaw: 0.25

    covariance:
      compute: true
      scale: 2.0
      min_sigma_x: 0.01
      min_sigma_y: 0.01
      min_sigma_z: 0.02
      min_sigma_yaw: 0.01

    gate:
      max_reproj_error_px: 3.0
      max_pose_delta_m: 0.20
      max_yaw_delta_deg: 20.0
      good_confidence: 0.75
      reject_confidence: 0.40
      require_positive_depth: true
      require_finite: true
```

---

## 15. g2o 求解流程细化

推荐流程：

```cpp
ArmorPoseOptimizationResult SlidingWindowOptimizer::optimize(
    const std::vector<ArmorPoseObservation>& window,
    const std::optional<TrackerPrediction>& pred)
{
  ArmorPoseOptimizationResult result;

  // 1. Basic check.
  if (window.size() < config_.min_window_size) {
    return reject("not enough frames");
  }
  if (!isWindowStable(window)) {
    return reject("unstable track/hypothesis");
  }

  // 2. Build optimizer.
  g2o::SparseOptimizer optimizer;
  setupSolver(optimizer);

  // 3. Add VertexXyzYaw for each frame.
  for (int i = 0; i < window.size(); ++i) {
    auto* v = new VertexXyzYaw();
    v->setId(i);
    v->setEstimate(makeInitialXyzYaw(window[i]));
    optimizer.addVertex(v);
  }

  // 4. Add reprojection edges.
  for (int i = 0; i < window.size(); ++i) {
    for (int j = 0; j < window[i].image_points.size(); ++j) {
      auto* e = new EdgeXyzYawReprojection(...);
      e->setVertex(0, optimizer.vertex(i));
      e->setMeasurement(window[i].image_points[j]);
      e->setInformation(makePixelInformation(window[i], j));
      attachRobustKernelIfNeeded(e);
      optimizer.addEdge(e);
    }
  }

  // 5. Add PnP priors.
  for (int i = 0; i < window.size(); ++i) {
    auto* e = new EdgeXyzYawPrior(...);
    e->setVertex(0, optimizer.vertex(i));
    e->setMeasurement(makePnpXyzYaw(window[i]));
    e->setInformation(makePnpPriorInformation(window[i]));
    optimizer.addEdge(e);
  }

  // 6. Add second-order smooth edges.
  if (config_.use_second_order_smooth && window.size() >= 3) {
    for (int i = 2; i < window.size(); ++i) {
      auto* e = new EdgeSecondOrderXyzYawSmooth();
      e->setVertex(0, optimizer.vertex(i - 2));
      e->setVertex(1, optimizer.vertex(i - 1));
      e->setVertex(2, optimizer.vertex(i));
      e->setInformation(makeSecondOrderSmoothInformation());
      optimizer.addEdge(e);
    }
  }

  // 7. Optional third-order smooth.
  if (config_.use_third_order_smooth && window.size() >= 4) {
    for (int i = 3; i < window.size(); ++i) {
      auto* e = new EdgeThirdOrderXyzYawSmooth();
      e->setVertex(0, optimizer.vertex(i - 3));
      e->setVertex(1, optimizer.vertex(i - 2));
      e->setVertex(2, optimizer.vertex(i - 1));
      e->setVertex(3, optimizer.vertex(i));
      e->setInformation(makeThirdOrderSmoothInformation());
      optimizer.addEdge(e);
    }
  }

  // 8. Optional current-frame prediction prior.
  if (pred.has_value()) {
    auto* e = new EdgeXyzYawPrior(...);
    e->setVertex(0, optimizer.vertex(window.size() - 1));
    e->setMeasurement(pred->z_pred);
    e->setInformation(pred->P_pred.inverse());
    optimizer.addEdge(e);
  }

  // 9. Compute cost before.
  optimizer.initializeOptimization();
  double cost_before = computeTotalChi2(optimizer);

  // 10. Solve.
  auto t0 = now();
  optimizer.optimize(config_.max_iterations);
  double solver_time_ms = elapsedMs(t0);

  // 11. Extract current frame.
  auto* v_current = dynamic_cast<VertexXyzYaw*>(optimizer.vertex(window.size() - 1));
  Eigen::Vector4d z_current = v_current->estimate();

  // 12. Evaluate quality.
  double cost_after = computeTotalChi2(optimizer);
  QualityMetrics q = evaluateQuality(optimizer, window, z_current, pred);

  // 13. Covariance.
  Eigen::Matrix4d R_ba = computeCurrentMarginalCovariance(optimizer, window.size() - 1);
  Eigen::Matrix4d R_final = buildFinalCovariance(R_ba, q);

  // 14. Confidence and status.
  double confidence = computeConfidence(q, R_final, pred);
  auto status = decideStatus(q, confidence);

  // 15. Fill result.
  return makeResult(z_current, R_ba, R_final, confidence, status, q);
}
```

---

## 16. 对原风险章节的修正版

原文第 17 节风险与对策建议扩展为以下版本。

### 风险 1：滑窗优化引入延迟或超时

**修正说明：** 仅设置 `max_solver_time_ms` 不一定能硬中断 g2o 求解。工程上更可靠的是限制窗口大小和迭代次数，并统计实际耗时。

对策：

1. 默认窗口改为 `3~5` 帧；
2. `max_iterations` 默认 `5`；
3. 每帧统计 `solver_time_ms`；
4. 若连续超时，自动降级为 `g2o_single` 或 `single_yaw`；
5. 不建议 detector 级 BA 覆盖超过 `60~100 ms` 的历史窗口。

### 风险 2：平面目标深度漂移

**修正说明：** 仅靠 PnP translation prior 不够，还需要协方差下限和退化检测。

对策：

1. `z` 方向 prior 可强于 `xy`；
2. 输出 Hessian condition number；
3. 对 `z` 设置更高协方差下限；
4. 远距离、小面积目标自动放大 `R_final.z`；
5. 全 Pose3 模式首版默认关闭。

### 风险 3：track_id 切换导致窗口污染

**修正说明：** 不能只依赖 track_id 变化。track_id 未变但 hypothesis / armor_id / pose jump 变化时同样可能污染窗口。

对策：

1. 窗口 key 建议使用 `(track_id, armor_number, armor_type, hypothesis_id)`；
2. 若 armor_number / type / hypothesis 跳变，清空对应窗口；
3. 若相邻帧 `pose_delta` 或 `yaw_delta` 超限，清空窗口；
4. 若 Top1/Top2 hypothesis margin 太小，禁止启用滑窗，只用单帧 refine。

### 风险 4：4 点 / 6 点顺序不一致

**原对策基本正确。** 补充：

1. 单元测试必须覆盖 4 点和 6 点；
2. 每种点序必须用合成数据测试零残差；
3. 调试输出中增加每个点的 residual，便于排查点序错误。

### 风险 5：g2o 手写边数值错误

对策补充：

1. 首版可使用数值雅可比；
2. 解析雅可比上线前必须与数值雅可比对比；
3. 每条边提供 `checkJacobian()` 单元测试；
4. 对 OpenCV `projectPoints` 的输出做对照测试。

### 风险 6：优化结果比 PnP 更差

对策补充：

1. 不只比较重投影误差，还要比较 `pose_delta`、`yaw_delta`、`condition_number`；
2. 输出 `GOOD / DEGRADED / REJECT`，而不是简单 `valid`；
3. `DEGRADED` 结果可以输出，但必须放大 `R_final`；
4. `REJECT` 结果必须走回退链。

### 风险 7：一阶平滑导致运动滞后

这是原设计中隐含但未充分展开的风险。

对策：

1. 默认不用 `t_k - t_{k-1}` 作为主平滑项；
2. 改用二阶有限差分；
3. 三阶有限差分只作为可选弱正则；
4. 平滑权重不应大于重投影边和 PnP prior 对结果的主导权。

### 风险 8：Hessian 协方差过度自信

对策：

1. 使用 `chi2/dof` 做尺度修正；
2. 设置 `R_floor`；
3. 加工程保守系数 `covariance_scale`；
4. 对退化场景根据 condition number 放大 `R_final`；
5. 不将 `R_ba` 原样传给 UKF/InEKF。

### 风险 9：固定 pitch / roll 产生系统偏差

对策：

1. 明确 pitch/roll 来源；
2. 支持 `FIXED_FROM_CONFIG / FROM_PNP / FROM_EXTERNAL_ATTITUDE`；
3. 在 `R_system` 中补偿固定姿态不确定性；
4. 离线评估不同 pitch/roll 策略对 `xyz/yaw` 的偏差。

### 风险 10：滑窗 BA 与后端滤波器重复平滑

对策：

1. BA 只输出当前帧观测和协方差；
2. 不在 BA 内维护长期速度状态；
3. BA 窗口不宜太长；
4. 后端按 `R_final` 决定吸收强度；
5. 若后端 NIS 异常，应触发 hypothesis re-check，而不是强行 update。

---

## 17. 建议的阶段推进顺序修正

### Phase 1：g2o single yaw

保持原计划。

补充验收：

1. yaw 方向与 legacy `BaSolver` 完全一致；
2. 对不同 `R_imu_camera` 输入，投影残差方向正确；
3. 输出 `confidence` 的简单版本。

### Phase 2：g2o window xyz+yaw，不输出协方差

先实现：

1. 重投影边；
2. PnP prior；
3. 二阶平滑边；
4. 基础门控；
5. 输出 refined `xyz/yaw`。

验收：

1. 静态目标抖动下降；
2. 匀速横移不明显滞后；
3. 旋转目标 yaw 不被过度平滑；
4. 窗口 reset 正常。

### Phase 3：协方差和置信度

加入：

1. marginal covariance；
2. `chi2/dof` 缩放；
3. `R_floor`；
4. `confidence`；
5. `GOOD / DEGRADED / REJECT`。

验收：

1. 远距离、小面积目标 `R_final` 变大；
2. 残差异常时 `confidence` 降低；
3. 后端 UKF/InEKF 接入后 NIS 更稳定。

### Phase 4：三阶正则与系统项

加入：

1. 三阶有限差分弱正则；
2. `R_time`；
3. `R_assoc`；
4. `R_system`；
5. hypothesis margin 接入。

---

## 18. 最终推荐默认策略

### 18.1 `armor_detector_nn`

首版默认：

```text
pose.refiner.mode = g2o_single 或 existing single_yaw
```

实验开启：

```text
pose.refiner.mode = g2o_window
```

稳定后：

```text
g2o_window -> g2o_single -> existing_single_yaw -> pnp
```

### 18.2 传统 `armor_detector`

首版：

```text
graph_single -> legacy BaSolver -> pnp
```

有稳定 track_id 后再考虑：

```text
graph_window -> graph_single -> legacy BaSolver -> pnp
```

### 18.3 滑窗默认参数

```text
90 fps 左右：
  window_size = 5
  min_window_size = 3
  max_time_span_ms = 60
  smooth = second_order
  third_order = false

30 fps 左右：
  window_size = 3
  min_window_size = 2
  max_time_span_ms = 100
```

---

## 19. 总结

补充后的滑窗优化模块应遵循以下原则：

1. **只做 detector 级观测精修，不做最终目标状态估计。**
2. **首版变量只优化每帧 `xyz + yaw`，不显式估计速度、加速度、jerk。**
3. **重投影残差是主约束，PnP prior 是防漂移约束，二阶有限差分是弱平滑约束。**
4. **默认窗口应小，90 fps 下建议 3~5 帧，时间跨度控制在 60 ms 左右。**
5. **一阶平滑会带来零速度偏置，建议改为二阶有限差分。**
6. **必须输出协方差和置信度，且不能直接信任 Hessian 反演结果。**
7. **优化结果必须有 GOOD / DEGRADED / REJECT 状态，并始终保留 PnP fallback。**
8. **滑窗 BA 的输出应作为 `z, R` 输入 UKF/InEKF，而不是直接覆盖后端状态。**

采用该设计后，`armor_pose_graph_optimizer` 可以在不明显增加系统耦合和实时风险的前提下，为后端 tracker 提供更稳定、更可解释、可带协方差的当前帧装甲板观测。
