# Norm4 v3 观测噪声 R 细致建模第一阶段可行性分析

## 1. 结论概览

基于当前代码与既有设计文档调研，`观测噪声 R 细致建模` 在 `norm4_v3` 上推进到第一阶段是可行的，但第一阶段不宜把目标定义为“完整启用 BA/PnP 协方差 R_BA”。更稳妥的边界是：

```text
第一阶段 = 打通动态 R 框架 + 启用 R_YPD 物理先验 + 预留 R_BA 输入槽位 + 保持固定 R 回退
```

建议第一阶段最终落地公式为：

```text
R_final = R_floor + (1 - lambda) R_fixed + lambda R_YPD
```

其中 `R_BA` 暂时只做接口预留与数据链路规划：

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
```

在 `armor_detector` / `armor_detector_nn` 真实输出 `cov_xyz_yaw` 之前，建议强制：

```text
w = 0
```

这样可以先验证距离、视角、像素尺寸、关键点质量对 UKF / InEKF 观测更新的影响，同时不引入尚未闭环验证的 BA 协方差风险。

## 2. 调研范围

本次调研覆盖：

1. `gimbal_pipeline/docs/观测噪声R细致建模/observation_noise_R_design.md`
2. `armor_pose_graph_optimizer/docs/g2o_sliding_window_reprojection_optimizer_design.md`
3. `gimbal_pipeline/docs/Norm4重构/第二阶段/norm4_second_phase_plan.md`
4. `gimbal_pipeline/src/max_entropy_tracker/trackers/norm4_v3`
5. `gimbal_pipeline/include/max_entropy_tracker/trackers/norm4_v3`
6. `gimbal_pipeline/include/max_entropy_tracker/core/observation.hpp`
7. `armor_detector` 当前 `BaSolver`
8. `armor_detector_nn` 当前 `PoseEstimate` / refiner 链路

## 3. 当前现状

### 3.1 观测噪声总体方案已有完整设计

`observation_noise_R_design.md` 已经给出较完整的演进公式：

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
R_final = R_floor + (1 - lambda) R_fixed + lambda R_dynamic
```

该设计具备三个很适合灰度落地的特性：

1. `R_floor` 防止滤波器过度自信。
2. `R_fixed` 保留旧版固定 R 的稳定锚点。
3. `lambda` 允许从 `0.0 -> 1.0` 渐进启用动态模型。

这意味着第一阶段可以在不破坏现有行为的前提下接入动态 R。

### 3.2 `norm4_v3` 已具备后端抽象和噪声模型入口

第二阶段实现中，`norm4_v3` 已经引入后端抽象：

```cpp
IStructuredBackend
UkfBackendV2
InvariantPoseBackend
IMotionModelBundle
IMeasurementNoiseModel
```

当前 `UkfBackendV2` 和 `InvariantPoseBackend` 都通过 `IMeasurementNoiseModel` 获取观测噪声：

```cpp
Eigen::Matrix4d R = noise_->build_R(UpdateKind::Single);
Eigen::Matrix<double, 8, 8> R = noise_->build_R(UpdateKind::Dual);
```

当前默认实现是：

```cpp
FixedCartesianNoiseModel
```

它精确复现固定笛卡尔噪声语义：

```text
single: diag(sigma_pos_xy^2, sigma_pos_xy^2, sigma_pos_z^2, sigma_yaw^2)
dual:   two blocks of single R, then乘 dual_raw_R_scale
```

这说明第一阶段不需要改动 UKF / InEKF 主体数学结构，只需要扩展噪声模型接口和工厂创建逻辑。

### 3.3 `ObservationData` 已经携带部分 2D 图像证据

当前 `ObservationData` 已包含可选 `ImageObservation2D`：

```cpp
struct ImageObservation2D {
  bool valid = false;
  double bbox_x = 0.0;
  double bbox_y = 0.0;
  double bbox_w = 0.0;
  double bbox_h = 0.0;
  std::array<Eigen::Vector2d, 4> corners{};
  double image_center_x = 0.0;
  double image_center_y = 0.0;
  double detection_confidence = 0.0;
  std::string number;
  std::string type;
};
```

这对 `R_YPD` 第一阶段很关键，因为所需的 `u_center / v_center / width_px / height_px / armor_type / confidence` 大多已经有承载位置。

但它还缺两个关键输入来源：

1. 相机内参 `fx / fy / cx / cy` 不在 `ObservationData` 内。
2. `IMeasurementNoiseModel::build_R()` 当前不接收具体观测，因此无法根据每个观测动态生成 R。

### 3.4 `armor_detector` 当前不输出 R_BA

传统 `armor_detector` 的 `BaSolver::solveBa()` 当前只返回优化后的旋转矩阵：

```cpp
Eigen::Matrix3d solveBa(...)
```

其内部 g2o 单帧 yaw BA 只优化 yaw，并没有输出：

1. `cov_xyz_yaw`
2. Hessian / information matrix
3. 条件数
4. BA confidence
5. 优化后重投影 RMS 作为结构化结果字段

因此，当前不能从传统 `armor_detector` 直接获得可用于滤波器观测噪声的 `R_BA`。

### 3.5 `armor_detector_nn` 也尚未输出 R_BA

`armor_detector_nn::PoseEstimate` 当前已有：

```cpp
translation
rotation
yaw / pitch / roll
reprojection_error
reproj_error_raw
reproj_error_refined
quality_score
track_id
observation_stamp
R_imu_camera
```

这些字段足以支持 `R_YPD` 和启发式 confidence，但尚无：

```cpp
bool cov_valid;
Eigen::Matrix4d cov_xyz_yaw;
double condition_number;
```

所以 `armor_detector_nn` 也暂时不能提供严格意义上的 `R_BA`。

### 3.6 g2o 公共优化器文档已经规划了统一输出，但尚未覆盖协方差字段

`armor_pose_graph_optimizer` 设计文档已经规划了公共优化器：

```cpp
ArmorPoseObservation
ArmorPoseOptimizationResult
SingleFrameOptimizer
SlidingWindowOptimizer
TrackWindowManager
```

文档中的 `ArmorPoseOptimizationResult` 已包含：

```cpp
valid
mode
translation
rotation
yaw / pitch / roll
reproj_error_raw_px
reproj_error_refined_px
pose_delta_m
yaw_delta_rad
quality_score
reason
```

但当前规划片段里仍未显式加入：

```cpp
bool cov_valid;
Eigen::Matrix4d cov_xyz_yaw;
double condition_number;
```

因此如果后续要支持 `R_BA`，公共优化器的输出结构也需要补充协方差与健康度字段。

## 4. 第一阶段可行性判断

### 4.1 可行点

第一阶段可行，原因如下：

1. `norm4_v3` 已经有 `IMeasurementNoiseModel`，动态 R 可以作为新噪声模型接入。
2. `UkfBackendV2` 与 `InvariantPoseBackend` 都已经通过统一噪声接口获取 `R`，不需要分别重写滤波更新主体。
3. `ObservationData::image` 已经以 append-only 方式携带 2D 信息，适合继续扩展而不破坏旧链路。
4. `R_final` 公式天然支持 `lambda=0` 回退到固定 R，灰度风险可控。
5. 双观测更新当前已经使用 8D raw batch，适合后续把两个单观测动态 R 组合成 block-diagonal dual R。

### 4.2 主要缺口

第一阶段至少需要补齐以下缺口：

1. `IMeasurementNoiseModel` 当前没有观测输入。
2. 噪声模型当前无法读取或持有相机内参。
3. `ObservationData::image` 没有协方差字段，也没有直接的 `cov_xyz_yaw` 承载位。
4. `armor_detector` / `armor_detector_nn` 暂无 `R_BA` 输出。
5. `armor_pose_graph_optimizer` 虽有规划，但协方差输出字段仍需补充。

这些缺口都属于“接口与数据链路”问题，不是 UKF / InEKF 数学结构不可行问题。

### 4.3 第一阶段建议边界

建议第一阶段明确不做：

1. 不要求 `armor_detector` 立即输出 `R_BA`。
2. 不要求从 g2o Hessian 反推协方差并投入主链路。
3. 不要求 `lambda=1.0` 完全替代固定 R。
4. 不要求所有 detector 同时接入。
5. 不改 `ukf_v1` 稳定基线，只在 `ukf_v2` / `inekf` 试点。

建议第一阶段只做：

1. 接口支持 per-observation R。
2. 实现 `YpdYawNoiseModel`。
3. `R_BA` 字段预留，但默认 `cov_valid=false`、`w=0`。
4. 通过 `noise_profile` 选择 `fixed | ypd_yaw`。
5. 用 NIS、gate pass ratio、track lost ratio、prediction jump 做灰度评估。

## 5. 推荐第一阶段技术方案

### 5.1 噪声接口调整

当前接口：

```cpp
class IMeasurementNoiseModel {
 public:
  virtual Eigen::MatrixXd build_R(UpdateKind kind) const = 0;
};
```

建议调整为：

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
};
```

兼容策略：

1. `FixedCartesianNoiseModel::build_single_R(obs)` 忽略 `obs`，返回旧固定 R。
2. `FixedCartesianNoiseModel::build_dual_R(obs0, obs1)` 返回旧 dual R。
3. `YpdYawNoiseModel` 使用 `obs.image`、`obs.position()`、相机内参与配置生成动态 R。

### 5.2 第一阶段新增模型

建议新增：

```cpp
YpdYawNoiseModel
```

职责：

1. 根据 `obs.x/y/z/yaw` 计算距离与雅可比。
2. 根据 `obs.image.bbox_w/h`、`corners`、`type`、`detection_confidence` 估计像素域噪声。
3. 根据相机内参计算 `sigma_azi / sigma_ele / sigma_dist / sigma_yaw`。
4. 构造 `R_YPD = J * R_YPD_local * J.transpose()`。
5. 应用 clamp、floor、fixed blend、lambda。
6. 若输入不足，自动回退到 `R_floor + R_fixed`。

第一阶段公式：

```text
R_dynamic = R_YPD
R_final = R_floor + (1 - lambda) R_fixed + lambda R_dynamic
```

### 5.3 相机内参来源

可选路径有两种：

1. 噪声模型配置中静态读取 `fx / fy / cx / cy`。
2. 在上游构造 `ObservationData` 时，把相机内参扩展进 `ImageObservation2D` 或新增 `CameraObservationMeta`。

第一阶段建议选第 1 种：

```yaml
norm4_v3:
  backend_config:
    backend_type: ukf_v2
    noise_profile: ypd_yaw

  observation_noise:
    camera:
      fx: 1556.34704
      fy: 1557.43488
      cx: 610.59754
      cy: 503.80001
      image_width: 1280
      image_height: 1024
```

原因：

1. 改动小。
2. 不影响消息结构。
3. 便于 bag 回放固定对比。
4. 后续可以再切到 `/camera_info` 动态来源。

### 5.4 建议配置草案

```yaml
norm4_v3:
  backend_config:
    backend_type: ukf_v2
    motion_profile: default
    noise_profile: ypd_yaw

  observation_noise:
    dynamic_blend:
      lambda: 0.30

    r_floor:
      sigma_x: 0.005
      sigma_y: 0.005
      sigma_z: 0.010
      sigma_yaw: 0.010

    r_fixed:
      sigma_x: 0.030
      sigma_y: 0.030
      sigma_z: 0.050
      sigma_yaw: 0.050

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

    ba_covariance:
      enable: false
      weight: 0.0
      scale: 4.0
```

### 5.5 Dual update 处理

单观测：

```text
R_single = build_single_R(obs)
```

双观测：

```text
R_dual = blockdiag(R_single(obs0), R_single(obs1)) * dual_raw_R_scale
```

注意：

1. 第一阶段不建模两个观测之间的互相关。
2. 若其中一个观测缺少 image metadata，可只对该观测回退固定 R。
3. 双观测的 `dual_raw_R_scale` 继续保留，保持与现有调参语义兼容。

### 5.6 R_BA 预留字段

建议在 `ObservationData` 中 append-only 增加可选字段：

```cpp
struct ObservationCovarianceMeta {
  bool valid = false;
  bool cov_valid = false;
  Eigen::Matrix4d cov_xyz_yaw = Eigen::Matrix4d::Zero();
  double confidence = 0.0;
  double reproj_rms = 0.0;
  double condition_number = 0.0;
  int num_observations = 0;
  int num_inliers = 0;
};

std::optional<ObservationCovarianceMeta> covariance;
```

第一阶段策略：

```text
如果 covariance 不存在或 cov_valid=false，则 w=0，仅使用 R_YPD。
```

后续 `armor_pose_graph_optimizer` 或 detector 输出协方差后，再启用：

```text
w = confidence_BA^2
R_dynamic = (1 - w) R_YPD + w * s_BA * clamp(R_BA)
```

## 6. 与 UKF / InEKF 的适配性

### 6.1 UKF v2

`UkfBackendV2` 当前所有 single / dual evaluate 与 tryUpdate 都从 `noise_` 取 R。因此第一阶段只需要把调用替换为：

```cpp
noise_->build_single_R(obs)
noise_->build_dual_R(obs0, obs1)
```

可行性：高。

风险：低到中。

主要风险在于动态 R 过小导致 gate 过严或更新过猛，因此必须保留：

1. `R_floor`
2. eigenvalue clamp
3. `lambda` 灰度
4. fixed fallback

### 6.2 InEKF

`InvariantPoseBackend` 同样持有 `IMeasurementNoiseModel`，且 Joseph form 更新中使用 R：

```cpp
P_post = I_KH * P * I_KH.transpose() + K * R * K.transpose();
```

因此接口层面也可直接复用动态 R。

可行性：高。

风险：中。

原因：InEKF 使用解析 H，动态 R 会直接影响 Kalman gain 和 NIS，若 R 估计过乐观，可能更快触发 gate fail 或导致结构参数间接受扰。建议第一阶段先：

1. `ukf_v2` 主试点。
2. `inekf` shadow 观察。
3. 不把 `inekf` 作为第一阶段默认主链路。

## 7. 与 detector / optimizer 的依赖关系

### 7.1 第一阶段不阻塞于 detector 输出 R_BA

由于第一阶段可以令 `w=0`，所以当前 `armor_detector` 和 `armor_detector_nn` 不输出 `R_BA` 并不会阻塞试点。

第一阶段真正依赖的是：

1. 3D 观测 `x/y/z/yaw`。
2. 2D bbox 或 corners。
3. armor type。
4. detection confidence。
5. 相机内参。

这些输入当前已经基本具备或可由配置提供。

### 7.2 后续 R_BA 需要公共优化器补字段

若后续基于 `armor_pose_graph_optimizer` 输出 `R_BA`，建议把 `ArmorPoseOptimizationResult` 扩展为：

```cpp
bool cov_valid{false};
Eigen::Matrix4d cov_xyz_yaw{Eigen::Matrix4d::Zero()};
double condition_number{0.0};
int num_observations{0};
int num_inliers{0};
```

同时需要规定：

1. `cov_xyz_yaw` 的坐标系必须与 `ObservationData::as_4d()` 一致。
2. yaw covariance 的 yaw 定义必须与 Norm4 观测模型一致。
3. 协方差必须经过正定性检查和 eigenvalue clamp。
4. BA confidence 不得直接等于 detector confidence，应综合 reprojection RMS、condition number、inlier ratio。

## 8. 推荐实施步骤

### 8.1 Step 1：文档与配置落点

1. 明确 `noise_profile = fixed | ypd_yaw`。
2. 新增 `Norm4V3ObservationNoiseConfig` 配置结构。
3. 默认仍为 `fixed`，保证现有行为不变。

验收：默认配置下输出行为与当前一致。

### 8.2 Step 2：接口改造

1. 修改 `IMeasurementNoiseModel` 为 per-observation API。
2. 适配 `FixedCartesianNoiseModel`。
3. 修改 `UkfBackendV2` / `InvariantPoseBackend` 中 single / dual R 调用。

验收：`noise_profile=fixed` 下编译通过，bag 回放 NIS 与 gate 统计基本一致。

### 8.3 Step 3：实现 `YpdYawNoiseModel`

1. 从配置读取相机内参、armor size、像素噪声参数。
2. 从 `ObservationData::image` 获取 bbox / corners / type / confidence。
3. 生成 `R_YPD`。
4. 应用 floor / fixed blend / lambda。
5. 输入不足时回退 fixed。

验收：离线打印不同距离、不同 bbox 尺寸下的 `sigma_x/y/z/yaw` 符合直觉。

### 8.4 Step 4：灰度试点

建议试点顺序：

1. `backend_type=ukf_v2` + `noise_profile=fixed`，确认第二阶段链路稳定。
2. `backend_type=ukf_v2` + `noise_profile=ypd_yaw` + `lambda=0.1`。
3. `lambda=0.3`。
4. `inekf` shadow 模式观察同一批数据。

不要第一天就 `lambda=1.0`。这个旋钮很好用，别把它当摆设。

### 8.5 Step 5：指标对比

建议至少记录：

1. single / dual NIS 均值、P50、P90、P99。
2. gate pass ratio。
3. posterior sanity fail ratio。
4. top1 confidence 与 top1-top2 margin。
5. track lost / reacquire 次数。
6. 中心位置跳变量。
7. yaw 跳变量。
8. 发弹链路命中相关指标，如果有实车数据。

## 9. 风险与对策

### 9.1 动态 R 过小

现象：

1. NIS 激增。
2. gate fail 增多。
3. 更新后跳变。

对策：

1. 增大 `R_floor`。
2. 降低 `lambda`。
3. 提高 `sigma_center_px / sigma_size_px / sigma_corner_px`。
4. 对 `R_YPD` 做 eigenvalue clamp。

### 9.2 动态 R 过大

现象：

1. 滤波器明显滞后。
2. 观测更新弱。
3. 快速机动时跟踪漂移。

对策：

1. 降低像素噪声参数。
2. 提高 `lambda`。
3. 检查 bbox 尺寸是否来自 resize 后坐标但内参未同步缩放。

### 9.3 坐标系不一致

现象：

1. x/y/z covariance 方向不符合实际。
2. z 或横向噪声异常。

对策：

1. 明确 `ObservationData` 是否处于相机系、odom 系或 target/world 系。
2. 如果观测已经从 camera frame 转到 odom，需要同步旋转 `R_xyz`。
3. 第一阶段文档和代码注释中必须写清楚坐标系契约。

### 9.4 图像坐标与内参不匹配

现象：

1. resize / letterbox 后角噪声估计错误。
2. 距离噪声与实际距离不匹配。

对策：

1. 第一阶段只允许使用与 `camera_info` 同尺度的 bbox / corners。
2. 若 detector 使用 resize 图像，必须在进入 `ObservationData::image` 前映射回原图坐标，或同步缩放内参。

### 9.5 R_BA 过度自信

第一阶段默认不启用 `R_BA`，但后续启用时风险很高。

对策：

1. `w = confidence_BA^2` 或 `confidence_BA^3`。
2. `s_BA >= 4.0` 起步。
3. 强制 eigenvalue clamp。
4. condition number gate。
5. reprojection RMS gate。

## 10. 第一阶段验收标准

建议第一阶段验收定义为：

1. `noise_profile=fixed` 下行为与当前基线一致。
2. `noise_profile=ypd_yaw` 可在 `ukf_v2` 下运行，不影响 `ukf_v1`。
3. 缺少 `obs.image` 时自动回退 fixed，不崩溃、不输出非正定 R。
4. 单观测和双观测都能生成正定 R。
5. 动态 R debug 输出可解释：距离越远，横向角噪声传播越大；bbox 越小，距离/yaw 噪声越大。
6. `lambda=0.1/0.3` 回放下 gate pass ratio 与 track lost ratio 不劣于可接受阈值。
7. `R_BA` 字段预留完成，但默认不参与主链路。

## 11. 建议结论

推荐推进第一阶段，但建议把第一阶段命名为：

```text
Norm4 v3 YPD-aware Observation Noise Pilot
```

而不是：

```text
BA covariance based Observation Noise
```

原因是当前 detector 与 g2o 公共优化器还没有真实 `R_BA` 输出，直接把目标绑定到 `R_BA` 会让试点被上游阻塞。先落地 `R_YPD + fixed blend + floor + lambda`，既能验证动态观测噪声对 UKF / InEKF 的收益，也能为后续 `R_BA` 接入预留干净接口。

一句话版本：

```text
可行，但第一阶段应先做 R_YPD 动态噪声试点；R_BA 只预留，不启用。
```

---

## 12. R_BA 链路收益与风险专项评估

### 12.1 问题定义

这里的 `R_BA 链路` 指完整打通以下路径：

```text
PnP / BA / g2o refiner
  -> PoseEstimate 扩展 cov_xyz_yaw / quality / condition
  -> rm_interfaces::msg::Armor 扩展协方差与诊断字段
  -> gimbal_pipeline Armors callback
  -> ObservationData::covariance
  -> YpdYawNoiseModel / BaAwareNoiseModel
  -> UKF v2 / InEKF 观测更新
```

它不是只在 `PoseEstimate` 里加一个 `Eigen::Matrix4d` 字段。由于 detector 与 gimbal pipeline 之间通过 ROS message 隔离，完整链路还必须修改消息定义、发布端、订阅端、日志/回放兼容与噪声模型接口。

### 12.2 当前链路缺口

当前 `armor_detector_nn` 已经发布：

1. `pose`
2. `detection_confidence`
3. `bbox_xywh`
4. `image_corners`
5. `corners_ordering`

当前 `armor_detector` 也已发布类似 2D image geometry。

但当前 `rm_interfaces::msg::Armor` 没有：

1. pose covariance
2. yaw covariance
3. covariance valid flag
4. reprojection RMS
5. condition number
6. inlier count
7. pose refine mode / quality score

当前 `ObservationData` 也没有 `R_BA` 承载字段。

因此，如果要让 `R_BA` 真的进入滤波器，需要至少改动：

1. `PoseEstimate`
2. `rm_interfaces/msg/Armor.msg`
3. `armor_detector_nn_node.cpp` 发布字段
4. `armor_detector` 传统发布字段
5. `gimbal_pipeline` Armors 消息转换
6. `ObservationData`
7. `IMeasurementNoiseModel`
8. `UkfBackendV2` / `InvariantPoseBackend` 的 per-observation R 调用
9. 日志、bag 回放、pybind 离线工具的兼容字段

这说明它是跨包数据契约改造，不是一个低风险小补丁。

### 12.3 潜在收益

`R_BA` 的理论收益主要有三类。

#### 12.3.1 表达几何退化

平面装甲板 PnP / BA 在远距离、小像素尺寸、大倾角、角点共线趋势明显时，位姿不确定性会快速增大。固定 R 或简单距离模型无法完整表达这种退化。

如果 `R_BA` 可信，它可以让滤波器在退化帧中自动降低观测权重，减少错误强更新。

#### 12.3.2 表达 xyz/yaw 相关性

`R_YPD` 可以表达部分 `xyz` 相关性，但 BA Hessian 理论上能进一步表达：

1. 深度与 yaw 的耦合。
2. 横向位置与角点布局的耦合。
3. 不同关键点质量导致的方向性不确定性。

这对 NIS 校准、假设评分和 gate 一致性有潜在帮助。

#### 12.3.3 统一 detector 质量与 tracker 更新强度

如果 `R_BA` 与 `quality_score / reproj_error / condition_number` 统一输出，下游不再需要只依赖启发式 confidence。长期看，这能让 detector 的“这帧我有多确定”更真实地进入 tracker。

### 12.4 收益是否会显著

短期判断：**不确定，且大概率不如先做 R_YPD 显著稳定。**

原因：

1. 当前 `armor_detector` 的 BA 是 yaw-only，只优化旋转，不优化平移，因此无法自然给出完整 `xyz + yaw` 协方差。
2. 当前 `armor_detector_nn` 的 `SingleYawRefiner` / `SlidingWindowRefiner` 主要输出 refined pose 与 reprojection error，没有 Hessian covariance 输出。
3. 单帧平面目标的 Hessian covariance 往往偏乐观，尤其在使用固定 pitch/roll、强 prior、Huber loss 或窗口平滑时，数学协方差不等于真实误差分布。
4. 真实误差还包含 detector bias、装甲板尺寸误差、角点系统性偏移、曝光/运动模糊、TF 时间误差，这些不一定会被 BA Hessian 捕获。
5. Norm4 的主要难点不仅是单帧位姿精度，还包括多假设选择、panel 绑定、结构参数和机动模型。`R_BA` 只能改善观测可信度，不会单独解决绑定与拓扑错误。

因此，第一阶段如果目标是“显著收益且风险不过大”，更推荐：

```text
R_YPD + reprojection/quality scale + fixed blend
```

而不是：

```text
直接启用 Hessian-derived R_BA
```

### 12.5 风险是否过大

如果直接把 `R_BA` 作为主链路观测噪声使用，风险偏高。

主要风险如下：

1. **协方差过度自信**：Hessian inverse 在模型假设正确时才近似有效，实际 detector bias 会让它严重低估误差。
2. **坐标系风险**：BA 输出通常在 camera frame，而 `ObservationData` 进入 tracker 时可能已经被 TF 转到发布 frame；`R_xyz` 必须同步旋转。
3. **yaw 语义风险**：detector 的 armor yaw、Norm4 的 center yaw、panel yaw 之间有 panel angle 差异，错误使用会直接污染 yaw gate。
4. **消息契约风险**：修改 `Armor.msg` 会触发接口重编译，影响所有发布/订阅方和 bag 兼容。
5. **算法耦合风险**：detector 内部 BA 模式不同，`R_BA` 含义不同。PnP-only、single-yaw、sliding-window、future g2o window 的 covariance 不能混为一谈。
6. **实时性风险**：若为了 covariance 增加 Hessian 求逆、条件数计算、窗口边缘化信息提取，需要评估 detector 主线程耗时。
7. **调参风险**：一旦 `R_BA` 影响 NIS，现有 gate 阈值、lambda、dual scale 都可能要重标定。

### 12.6 推荐结论

对“实现 R_BA 链路是否能带来显著收益，同时不会带来过大风险”的判断是：

```text
不能作为第一阶段的确定性高收益、低风险项。
```

更细一点：

1. **长期值得做**：因为它能把 detector 几何质量传给 tracker，是正确架构方向。
2. **短期不宜直接启用**：因为当前 detector/refiner 没有可靠 covariance，消息链路也没有承载字段。
3. **短期收益不保证显著**：`R_YPD + quality scale` 已经能覆盖距离、像素尺寸、重投影质量的大部分一阶收益。
4. **直接启用风险偏高**：尤其是协方差过小导致 gate fail 或错误强更新。

建议把 R_BA 链路拆成三步，降低风险。

### 12.7 建议分阶段策略

#### A 阶段：只传质量诊断，不传协方差

先扩展 `PoseEstimate` 内部字段：

```cpp
struct PoseQualityInfo {
  bool valid{false};
  double reproj_rms{0.0};
  double reproj_raw{0.0};
  double reproj_refined{0.0};
  double quality_score{0.0};
  double condition_proxy{0.0};
};
```

下游先使用：

```text
R = R_YPD * quality_scale(reproj_rms, detection_confidence, bbox_size)
```

收益：中。

风险：低。

这是最值得先做的版本。

#### B 阶段：消息链路预留 covariance，但默认不参与滤波

扩展 `Armor.msg`：

```text
bool pose_covariance_valid
float64[16] pose_covariance_xyz_yaw
float32 pose_quality_score
float32 reproj_error_raw
float32 reproj_error_refined
float32 pose_condition_number
uint8 pose_estimate_mode
```

扩展 `ObservationData`：

```cpp
std::optional<ObservationCovarianceMeta> covariance;
```

但噪声模型中仍默认：

```text
use_ba_covariance = false
w = 0
```

收益：低到中，主要是架构铺路和日志分析。

风险：中，主要是消息接口变更。

#### C 阶段：shadow 使用 R_BA，主链路仍不用

在 `BaAwareNoiseModel` 中计算：

```text
R_shadow = (1 - w) R_YPD + w * s_BA * clamp(R_BA)
```

只记录：

1. shadow NIS
2. shadow gate pass
3. R_BA eigenvalues
4. R_YPD vs R_BA ratio
5. covariance valid ratio

不改变 tracker 主状态。

收益：中，能判断 R_BA 是否真的校准。

风险：低到中。

#### D 阶段：小权重灰度启用

只有满足以下条件才启用：

```text
cov_valid = true
condition_number < threshold
reproj_rms < threshold
all eigenvalues in clamp range
pose_estimate_mode in allowlist
```

启用时从非常保守开始：

```text
w = 0.05 * confidence_BA^2
s_BA >= 4.0
lambda <= 0.3
```

收益：有机会中到高。

风险：中。

### 12.8 推荐优先级

建议优先级如下：

1. `P0`: per-observation R 接口 + `R_YPD` 动态噪声。
2. `P1`: `PoseEstimate` / `Armor.msg` 传递 `quality_score`、`reproj_error_raw/refined`、`pose_estimate_mode`。
3. `P2`: 基于质量诊断对 `R_YPD` 做 scale。
4. `P3`: 预留 `cov_xyz_yaw` 字段并做日志，不参与滤波。
5. `P4`: g2o / BA 输出真正 covariance，并 shadow 评估。
6. `P5`: 小权重灰度启用 `R_BA`。

最终建议：

```text
现在不建议直接投入完整 R_BA 主链路；建议先做质量诊断链路和 R_YPD 动态 R。
R_BA 作为中长期增强项，以 shadow + 小权重方式逐步验证。
```
