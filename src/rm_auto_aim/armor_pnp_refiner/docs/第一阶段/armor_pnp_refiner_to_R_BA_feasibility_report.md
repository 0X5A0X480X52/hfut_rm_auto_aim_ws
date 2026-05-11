# armor_pnp_refiner 生成 R_BA 可行性调研报告

日期：2026-05-11

## 1. 调研结论

`armor_pnp_refiner` 作为 `armor_detector` 中 PnP/BA 链路的扩展，并将输出协方差用于 `gimbal_pipeline` 的 `R_BA` 观测噪声建模，整体方向可行，且与当前两份设计文档的职责边界基本一致。

建议采用分阶段落地：

1. 第一阶段先实现 `PnP in -> PnP out` 的透明 refiner 接口、质量诊断字段和保守协方差输出。
2. 不建议直接把当前 `armor_detector::BaSolver` 的结果作为完整 `R_BA` 使用，因为现有 `BaSolver` 只优化 yaw，平移 `xyz` 是固定 PnP 输入，无法从该图中得到真实的 `Cov([x, y, z, yaw])`。
3. 若要支持后端动态观测噪声，必须新增 `xyz-yaw` 或窗口 `xyz-yaw` 优化顶点，并在最终线性化点上计算当前帧 marginal covariance。
4. 后端 `R_BA` 只能作为动态噪声的一部分参与融合，不能裸用；必须经过正定检查、特征值 clamp、残差尺度修正、置信度权重和固定噪声锚点融合。

可行性评级：

| 项目 | 结论 | 说明 |
|---|---|---|
| 作为 PnP 后处理模块 | 可行 | 当前 detector 已有关键点、相机内参、PnP 初值和 g2o 依赖 |
| 单帧 yaw-only 迁移 | 可行 | 可直接复用现有 `BaSolver` 思路，但只能输出 yaw 方向诊断 |
| 单帧 `xyz-yaw` 优化 | 可行，建议作为 R_BA 首个有效来源 | 能形成 4 维状态协方差，工程复杂度可控 |
| 滑窗 `xyz-yaw` 优化 | 可行，但不建议第一阶段强依赖 | 需要短期关联、窗口污染控制和边缘协方差计算 |
| R_BA 接入 UKF/InEKF | 可行，但需要消息/结构扩展 | 当前 `ObservationData` 还没有观测协方差字段 |
| 裸 Hessian inverse 直接作为 R | 不可行 | 平面 PnP、固定 pitch/roll、像素噪声假设都会导致过度自信 |

## 2. 当前实现现状

### 2.1 `armor_detector` 的 BaSolver

当前 `src/rm_auto_aim/armor_detector` 中的 `BaSolver` 是 yaw-only BA：

- 顶点：`VertexYaw`，维度为 1。
- 边：`EdgeProjection`，每个装甲板角点一条 2D 重投影边。
- 固定项：PnP 平移 `t_camera_armor`、固定 pitch、装甲板 3D 点。
- 输出：优化后的旋转矩阵 `R_camera_armor`。

这说明现有 BA 的主要价值是修正 yaw，而不是估计完整位姿。它可以作为 `armor_pnp_refiner` 的 `G2O_SINGLE_YAW` 模式保留，但不应伪装成完整 `xyz+yaw` 协方差来源。

### 2.2 `gimbal_pipeline` 的观测噪声现状

当前 `DualRadiusSpinUKF` 已有固定对角 R 和可选 YPD 噪声：

```text
R = diag(obs_noise_pos^2, obs_noise_pos^2, obs_noise_pos^2, obs_noise_yaw^2)
```

开启 `enable_ypd_observation_noise` 后，会基于 `z_pred` 构造 YPD 噪声并用雅可比映射到 `xyz+yaw`。这与 `observation_noise_R_design.md` 中的 `R_YPD` 设计一致，说明后端已经具备动态 R 的一个雏形。

当前 `ObservationData` 包含：

- `x, y, z, yaw`
- `confidence`
- `image`
- `track2d_id`

但还没有：

- `cov_xyz_yaw`
- `cov_valid`
- `ba_confidence`
- `ba_status`
- `ba_reproj_rms`
- `ba_condition_number`

因此 `R_BA` 接入前需要先扩展观测数据结构或增加旁路元数据。

## 3. 参考实现对本设计的启发

`tmp/jlu_vision_26/src/auto_aim/armor_tracker/src/factors.cpp` 的参考实现有两个值得借鉴的点。

第一，重投影 factor 使用相机内参和畸变模型，把装甲板局部点经 `Pose3` 投到像素平面，并显式返回雅可比：

```text
H = H_calib * H_norm * H_transform
```

这比当前 `armor_detector` 的 yaw-only 边更接近完整 BA/PnP refiner 所需的形态。若本项目继续使用 g2o，也应保持同样的结构：状态扰动、投影模型、畸变模型和雅可比链路清晰分层。

第二，参考实现中还有 `TranslationFactor`、`YawFactor`、`VelocityFactor`、`VyawFactor` 等时间约束。这说明滑窗优化不是单纯把多帧重投影边堆在一起，还需要合理的运动/平滑先验。`armor_pnp_refiner_revised_design.md` 中选择二阶有限差分平滑是合理的，但第一阶段不应把它作为 R_BA 的唯一可信来源。

## 4. 协方差生成可行性分析

### 4.1 yaw-only BA 的协方差边界

当前 `BaSolver` 的 Hessian 只对应 1 维 yaw 状态。即使从 g2o 中取到 `H^{-1}`，也只能近似得到：

```text
Cov(yaw)
```

不能得到：

```text
Cov([x, y, z, yaw])
```

如果第一阶段为了打通接口复用 yaw-only BA，建议输出：

```text
cov_valid = false 或 DEGRADED
cov_xyz_yaw = conservative_diag
```

其中 yaw 方差可参考 yaw-only Hessian，但 `x/y/z` 必须使用保守先验或 YPD 模型，不能填入虚假的小方差。

### 4.2 单帧 `xyz-yaw` 是首个推荐有效 R_BA 来源

若状态定义为：

```text
x = [tx, ty, tz, yaw]
```

残差为 4 个角点的 2D 重投影误差：

```text
r_i = observed_uv_i - project(K, D, R(yaw, fixed_pitch, fixed_roll) * P_i + t)
```

则最终 Hessian：

```text
H = J^T W J
```

可形成 4 维近似协方差：

```text
R_BA = sigma_px^2 * H^{-1}
```

这一路径最适合作为第一阶段后半段或第二阶段的目标，原因是：

- 与后端观测空间 `xyz+yaw` 对齐。
- 不依赖跨帧关联。
- 比完整 Pose3 对平面目标更可控。
- 可以继续保留 PnP translation/yaw prior 抑制退化。

需要注意，单帧角点只有 8 维残差，状态 4 维，可观测性受装甲板面积、距离、角点分布和 pitch/roll 固定假设影响明显，必须输出 `condition_number` 和 `chi2_per_dof`。

### 4.3 滑窗协方差可行但实现成本更高

滑窗状态可写为：

```text
X = [X_old, x_current]
```

当前帧边缘信息矩阵：

```text
Omega_c = H_cc - H_co H_oo^{-1} H_oc
Sigma_c = Omega_c^{-1}
```

理论上可行，但工程上有四类额外风险：

1. 内部关联错误会污染窗口，使协方差看似更小但观测更偏。
2. 平滑先验会引入跨帧相关性，后端若继续强平滑，可能重复降低噪声。
3. 窗口过长会增加延迟和运动模型偏置。
4. 边缘化实现和数值稳定性比单帧更难验证。

因此滑窗优化可以先用于改善 pose，但其 covariance 在第一阶段应标记为实验或降权，等 NIS/NEES 和 bag 回放验证后再提高权重。

## 5. R_BA 接入后端的接口缺口

`observation_noise_R_design.md` 要求 `BaPnpResult` 至少包含：

```cpp
bool valid;
bool cov_valid;
Eigen::Vector4d pose_xyz_yaw;
Eigen::Matrix4d cov_xyz_yaw;
double confidence;
double reproj_rms;
double condition_number;
int num_observations;
int num_inliers;
```

当前后端 `ObservationData` 尚未承载这些字段。建议第一阶段以 append-only 方式扩展：

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

std::optional<ObservationCovarianceMeta> ba_pnp;
```

这样旧 tracker 可忽略该字段，新 R 构造器可在存在且可用时融合 `R_BA`。

## 6. 坐标系约束

设计文档中 `PnpRefineOutput::covariance_xyz_yaw` 默认是 camera frame，而 `gimbal_pipeline` 的跟踪观测通常经过 TF 转到 odom/world 类坐标系。

因此直接接入前必须明确：

```text
R_BA、R_YPD、z_obs、UKF observation_model 输出必须位于同一观测空间。
```

若 refiner 输出 camera frame 协方差，而 tracker 使用 odom frame，应在 `transform_armor_to_observation` 或等价转换层做：

```text
Cov_odom = J_tf * Cov_camera * J_tf^T
```

其中 yaw 也要考虑 camera/odom 旋转对装甲板 yaw 定义的影响。第一阶段可以先只传递协方差元数据但不启用融合，避免坐标系未闭合时影响滤波稳定性。

## 7. 动态 R 融合策略可行性

推荐保持 `observation_noise_R_design.md` 的总体公式：

```text
R_dynamic = (1 - w) R_YPD + w * s_BA * R_BA
R_final = R_floor + (1 - lambda) R_fixed + lambda * R_dynamic
```

该策略是可行的，原因是它同时提供了：

- `R_floor`：防止过度自信。
- `R_fixed`：迁移期稳定锚点。
- `R_YPD`：当 BA 协方差缺失或低置信时的物理先验。
- `R_BA`：表达当前重投影几何、残差和退化程度。
- `lambda`：允许从旧固定 R 平滑迁移到动态 R。

第一阶段建议：

```text
lambda = 0.0
```

先打通日志与字段，再用离线 bag 记录：

- `R_fixed`
- `R_YPD`
- `R_BA`
- `R_final`
- `confidence`
- `condition_number`
- `NIS`
- innovation

确认统计合理后再逐步启用：

```text
lambda = 0.3 -> 0.5 -> 0.7
```

## 8. 主要风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| yaw-only BA 被误用为完整 R_BA | 后端低估位置噪声 | yaw-only 只输出 DEGRADED/保守协方差 |
| 平面 PnP 深度退化 | z 方差过小、滤波发散 | PnP translation prior、z 方差下限、条件数门控 |
| Hessian covariance 过度自信 | R 过小，UKF 过度吸收观测 | 残差尺度修正、`s_BA` 放大、eigen clamp |
| 固定 pitch/roll 带来系统偏差 | 协方差无法表达 bias | 加 `R_system` 保守项，后续用标定表处理 bias |
| 滑窗关联污染 | 输出 pose 平滑但偏移 | 短窗口、关联置信度、窗口重置、DEGRADED 降权 |
| 坐标系不一致 | R 与观测方向不匹配 | 在 TF 转换层同步变换 covariance |
| 消息链路不支持协方差 | R_BA 无法到达后端 | append-only 扩展 `ObservationData` 或中间消息 |
| 动态 R 突然启用 | 跟踪参数重新耦合，稳定性下降 | `lambda` 渐进启用，保留固定 R 锚点 |

## 9. 第一阶段建议任务清单

### 9.1 refiner 侧

1. 建立 `armor_pnp_refiner` 基础数据结构：`PnpRefineInput`、`PnpRefineOutput`、`RefineStatus`。
2. 迁移当前 yaw-only BA 为 `G2O_SINGLE_YAW` 模式，保持 PnP fallback。
3. 输出基础诊断：raw/refined reprojection RMS、cost before/after、pose/yaw delta、solve time。
4. 输出保守 `covariance_xyz_yaw`，并明确 `cov_valid=false` 或 `status=DEGRADED`。
5. 实现质量门控，确保优化失败不影响 detector 发布。

### 9.2 covariance 侧

1. 新增 `xyz-yaw` 单帧优化原型。
2. 计算最终线性化点的 `J^T W J`。
3. 加入 `sigma_px^2`、`chi2/dof` 残差尺度修正。
4. 加入最小方差、最大方差、条件数和正定检查。
5. 输出 `cov_valid`、`confidence`、`condition_number`。

### 9.3 pipeline 侧

1. append-only 扩展 `ObservationData`，承载 BA/PnP covariance 元数据。
2. 在 TF 转换层明确 covariance 的坐标系转换责任。
3. 抽出 `ObservationNoiseBuilder` 或等价函数，统一生成 `R_final`。
4. 初始保持 `lambda=0.0`，只记录动态 R 调试量。
5. 用 bag 回放统计 NIS 和 innovation 分布后再启用融合。

## 10. 推荐验收标准

第一阶段不应以“动态 R 已经提升命中率”为验收目标，而应以接口和统计闭环为目标：

1. `armor_detector` 在 refiner 失败时仍发布原始 PnP。
2. yaw-only 模式不输出误导性的完整有效协方差。
3. `xyz-yaw` 原型在静态目标上协方差正定，且 z 方差随距离增大。
4. `condition_number` 在小目标、远距离、斜视角场景中明显升高。
5. `R_final` 在 `lambda=0.0` 时与旧固定 R 行为一致。
6. `lambda>0` 的离线回放中，NIS 均值和异常率可被日志解释。
7. 所有动态 R 都经过 symmetry、positive definite、eigen clamp。

## 11. 最终建议

当前设计可以继续推进，但应把“生成可被后端信任的 `R_BA`”拆成两步：

第一步，`armor_pnp_refiner` 先成为稳定、透明、可回退的 PnP 微调器，并产出诊断和保守协方差。

第二步，新增真正的 `xyz-yaw` 优化和 covariance estimator，再把 `cov_valid=true` 的结果以低权重接入 `R_dynamic`。

这样既能复用现有 `BaSolver` 和 g2o 基础，又不会让后端 UKF/InEKF 过早相信尚未标定的 Hessian 协方差。整体架构与后续 `R_BA` 机器人观测噪声建模兼容，工程风险可控。
