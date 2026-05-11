# Norm4 第二阶段实施方案（复用 CS/IMM 现有实现版）

> 适用前提：第一阶段已经完成实车验证，`norm4_v3` 多假设 + UKF Top1 提交链路可稳定运行。
>
> 参考文档：
> - `docs/Norm4重构/norm4_backend_ukf_v2_refactor_design.md`
> - `docs/Norm4重构/norm4_multi_hypothesis_backend_refactor_design.md`
> - `docs/Norm4重构/第一阶段/norm4_ukf_v1_first_audit_report.md`
> - `docs/Norm4重构/第二阶段/hypothesis_inekf_slow_structure_derivation.md`（InEKF 主要实现参考）

## 0. 第二阶段目标

第二阶段聚焦三件事：

1. 将后端抽象为统一滤波器接口，实现与接口分离，支持 UKF/InEKF 切换。
2. 将后端运动模型抽象为接口，并优先复用现有 CS/IMM 实现：
   - `xyz`：`CV`、`CA`、`Singer`、`CS`、`IMM`
   - `yaw`：`CV`、`CA`、`CS`
   - IMM 组合：`IMM-2D(CV,CA,CTRV,CS) + IMM-z(CV,CA,CS)`，以及 z 轴单模型变体（`CV`/`CA`/`CS`）。
3. 推进 `InEKF 位姿后端 + 慢结构参数(r1/r2/dza)` 链路，先 shadow 再灰度。

## 0.0 第二阶段硬约束

1. **尽可能不改动当前实现链路**，第一阶段已验证路径保持可直接运行。
2. **不修改** `include/max_entropy_tracker/trackers/norm4_v3/norm4_ukf_backend_v1.hpp`（及其现有行为语义）。
3. 在接口抽象下新增 `ukf_backend_v2`，通过配置切换，不替换/重写 `ukf_backend_v1`。
4. 前端观测噪声生成逻辑抽象为接口，后端通过接口取 `R`，便于后续扩展噪声模型。

## 0.1 已有实现资产（必须复用）

### gimbal_pipeline 内

1. `include/max_entropy_tracker/filters/single_armor_imm_tracker.hpp`
   - 已实现 `IMM(CV+CA+CS+CTRV)` 的 XY 融合。
   - 已实现 z 轴 1D KF、yaw 1D KF。
   - 已暴露 `model_probabilities()`，便于调试与健康度评估。
2. `include/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp`
   - 已有 “legacy KF / IMM” 双实现路由框架。
   - 开关已配置化：`outpost.ambiguous_backend_use_imm_adapter`。

### kalmanFilters 内

1. `src/kalmanFilters/filters/basic_models`
   - `CV_KF`、`CA_KF`、`CS_KF`、`Singer_KF`、`CTRV_EKF`。
2. `src/kalmanFilters/filters/combined_models`
   - `IMM_CV_CA_CS_3Dim`、`IMM_CV_CA_CS_Singer_3Dim`、`IMM_CV_CA_CTRV`、`IMM` 通用工厂。
3. `src/kalmanFilters/filters/models`
   - 已有 `ModelFactoryRegistry` + `ModelConfigLoader`（YAML 到模型实例）。

## 0.2 复用原则

1. 第二阶段不重新发明 CS/IMM 数学实现。
2. 先复用 `gimbal_pipeline` 现有 `SingleArmorIMMTracker` 完成后端抽象与主链路接入。
3. `kalmanFilters` 的 `basic_models/combined_models` 通过 adapter 渐进接入（用于补齐模型矩阵和做 A/B 对照）。
4. 新增代码优先做“接口对齐与状态映射”，而不是“重复实现滤波器”。
5. 当前 `ukf_backend_v1` 作为稳定基线保留；新增能力全部进入 `ukf_backend_v2`。

## 1. 现状约束与缺口

1. `Norm4ArmorTrackerV2` 当前直接持有 `Norm4UkfBackendV1`，后端仍具体类耦合。
2. `norm4_v3` 的 `trial/result` 数据结构仍偏 UKF 专属。
3. 过程模型虽已组件化，但 `norm4_v3` 后端未暴露“外部模型工厂接入点”。
4. `SingleArmorIMMTracker` 当前 z/yaw 是 1D CV（尚未提供 z:CA/CS、yaw:CA/CS）。
5. `gimbal_pipeline` 当前 `CMakeLists.txt` / `package.xml` 尚未声明 `models/basic_models/combined_models` 依赖。
6. `norm4_v2`/`norm4_v3` 参数重复，模型组合继续扩展会引发配置爆炸。

结论：第二阶段核心不是“新建更多滤波器”，而是“打通后端接口 + 模型适配层 + 配置组合层”。

## 2. 目标架构（第二阶段完成态）

```text
HypothesisSelector
  -> IStructuredBackend
      -> UkfBackendV1Adapter               (现有链路，保持不动)
      -> UkfBackendV2                      (新增，可配置运动模型)
      -> InvariantPoseBackend              (shadow/灰度)

UkfBackendV2
  -> IMotionModelBundle
      -> NativeProcessModelBundle           (现有 process_models)
      -> SingleArmorImmBundle               (复用 single_armor_imm_tracker)
      -> KalmanFiltersModelBundle           (复用 models/basic_models/combined_models)
  -> IMeasurementNoiseModel
      -> FixedCartesianNoiseModel           (兼容现有 sigma_pos/sigma_yaw)
      -> DistanceAwareNoiseModel            (随距离/置信度变化)
      -> YpdYawNoiseModel                   (后续扩展)

InvariantPoseBackend
  -> IStructureProvider
      -> UkfSnapshotStructureProvider
      -> SlowStructureErrorUpdaterProvider
```

## 3. 工作包 A：后端接口抽象（保持第一阶段行为）

### 3.1 接口目标

1. tracker / selector 不依赖具体后端实现。
2. UKF 与 InEKF 对外统一 `evaluate/tryUpdate/commit`。
3. 兼容 `SpinFilterInterface` 调试视图。
4. `v1` 与 `v2` 并存，默认仍走 `v1`。

### 3.2 关键拆分

1. 新增 `UkfBackendV1Adapter`：包装现有 `Norm4UkfBackendV1`，不改其头文件与行为。
2. 新增 `UkfBackendV2`：在同一接口下实现可配置运动模型与噪声模型。
3. `UkfTrial` 升级为通用 `UpdateTrialResult` + 后端私有句柄。
4. `Norm4ArmorTrackerV2` 改为持有 `std::unique_ptr<IStructuredBackend>`。
5. 新增 `BackendFactory`，通过配置选择 `ukf_v1 | ukf_v2 | inekf`。

### 3.3 验收

1. `backend_type=ukf_v1` 下 bag 回放与第一阶段一致（基线完全保留）。
2. tracker 侧不再 include 具体 UKF 后端头文件。
3. InEKF shadow 可接入且不影响主链路。
4. `ukf_v2` 可灰度接入，失败可一键回退 `ukf_v1`。

## 4. 工作包 B：运动模型抽象与 CS/IMM 复用接入

### 4.1 能力矩阵（按“现有可用/需补齐”）

1. `xyz: CV/CA/Singer`
   - 现有可用：`process_models` + `kalmanFilters/basic_models`。
2. `xyz: CS`
   - 现有可用：`kalmanFilters/basic_models/CS_KF`、`SingleArmorIMMTracker::CsModel6D`。
3. `xyz: IMM`
   - 现有可用：
     - `SingleArmorIMMTracker`（XY IMM + z CV + yaw CV）
     - `combined_models/IMM_*`（3D IMM 变体）
4. `yaw: CV`
   - 现有可用：`SingleArmorIMMTracker` 1D yaw KF。
5. `yaw: CA/CS`
   - 需补齐：在 `SingleArmorIMMTracker` 或 `KalmanFiltersModelBundle` 中新增 yaw 子模型。
6. `IMM-z: CV/CA/CS`
   - 当前默认可用是 `CV`；`CA/CS` 需补齐。

### 4.2 接入策略（先稳后广）

1. **B1（最小风险）**：
   - `UkfBackendV2` 增加 `SingleArmorImmBundle`，直接复用 `SingleArmorIMMTracker`。
   - 先交付 `IMM-2D + z:CV + yaw:CV`。
2. **B2（补齐矩阵）**：
   - 扩展 `SingleArmorIMMTracker`：
     - z 轴增加 `CA/CS` 可选。
     - yaw 增加 `CA/CS` 可选。
3. **B3（深度复用 kalmanFilters）**：
   - 新增 `KalmanFiltersModelBundle`，通过 `ModelFactoryRegistry` 创建 `CS_KF/IMM_*`。
   - 作为可选后端模型执行 A/B 对照。

### 4.3 适配层接口

```cpp
class IMotionModelBundle {
 public:
  virtual ~IMotionModelBundle() = default;
  virtual int state_dim() const = 0;
  virtual const StateLayout& layout() const = 0;
  virtual Eigen::VectorXd predict(const Eigen::VectorXd& x, double dt) const = 0;
  virtual Eigen::MatrixXd build_Q(double dt) const = 0;
  virtual ModelDebugInfo debug_info() const = 0;
};
```

### 4.3.1 观测噪声接口（新增）

```cpp
class IMeasurementNoiseModel {
 public:
  virtual ~IMeasurementNoiseModel() = default;
  virtual Eigen::MatrixXd build_single_R(const ObservationData& obs) const = 0;
  virtual Eigen::MatrixXd build_dual_R(
      const ObservationData& obs0,
      const ObservationData& obs1) const = 0;
  virtual std::string name() const = 0;
};
```

说明：

1. `ukf_backend_v2` 不再在后端内部硬编码 `sigma_pos/sigma_yaw` 组装 `R`。
2. 噪声模型由 `IMeasurementNoiseModel` 注入，默认实现先复用当前参数语义。
3. 允许后续按距离、角度、置信度、观测域（YPD/Yaw）扩展，而不改后端主体。

### 4.4 适配层职责边界

1. `SingleArmorImmBundle`：
   - 负责将 IMM 输出映射到 Norm4 主状态布局。
2. `KalmanFiltersModelBundle`：
   - 负责 `ModelConfig <-> UnifiedConfig` 映射。
   - 负责 `Models` 抽象的状态维度转换（如 CV/CA/CTRV/IMM 的 4D/6D/9D 差异）。
3. 任何 bundle 均不得直接修改结构参数估计链路（`r1/r2/dza`）。

### 4.5 验收

1. 模型切换不改业务代码，仅改配置。
2. `IMM-2D + z:CV + yaw:CV` 能稳定运行并输出模型概率。
3. 补齐后可配置 `z:CV/CA/CS`、`yaw:CV/CA/CS`。
4. NIS 与命中稳定性可回放对比。
5. 观测噪声模型可配置切换，且不需要修改 `ukf_backend_v2` 主流程。

## 5. 工作包 C：InEKF 位姿后端 + 慢结构参数链路

### 5.1 后端约束

1. InEKF 对 Selector 保持统一接口。
2. 结构参数不进入 InEKF 主状态，统一从 `IStructureProvider` 获取。

### 5.2 结构链路

1. 初期：`UkfSnapshotStructureProvider`。
2. 中期：`SlowStructureErrorUpdaterProvider`（按 InEKF 推导文档的慢结构误差路径实现）。
3. 当前阶段先不引入 RM 算法。

### 5.2.1 慢结构误差更新路径（采用推导文档方案）

以 `hypothesis_inekf_slow_structure_derivation.md` 为主，采用：

```text
Hybrid InEKF / invariant error-state EKF
快状态:   R, p, v, beta
慢参数:   theta = [r1, r2, dza]
```

慢结构更新形式：

```text
theta^+ = theta + Lambda_theta * delta_hat_theta
Lambda_theta = diag(alpha_r1, alpha_r2, alpha_dza), alpha << 1
```

并配套三条约束：

1. `Q_theta` 很小（慢时间尺度）。
2. 结构先验伪观测：`theta ~ N(theta0, Sigma_theta)`。
3. 物理范围 clamp（`r1/r2/dza` 有界）。

放开更新条件（建议同时满足）：

1. Top1 `NIS` 过门。
2. `confidence_top1` 与 `margin_12` 足够高。
3. 非 ambiguous，且非初始化早期帧。
4. 更新后仍在物理边界内。

### 5.3 分阶段接入

1. `C0`：InEKF 编译链路与接口打通。
2. `C1`：InEKF shadow 跟随 UKF Top1。
3. `C2`：InEKF shadow 独立 evaluate 全假设。
4. `C3`：慢结构误差更新链路上线（`Lambda_theta + Q_theta + 结构先验`，先供 shadow）。
5. `C4`：`backend_type=inekf` 灰度切换，保留 UKF 回退。

## 6. 配置组织方案（防止配置爆炸）

### 6.1 原则

1. 主配置只放“选择器键”，不放所有模型细节。
2. `ukf`、`motion`、`structure` 分 profile 管理。
3. 兼容 `kalmanFilters` 的 `model_type` 命名（`CS_KF`、`IMM_CV_CA_CTRV` 等）。
4. `backend` 与 `noise_model` 独立 profile，避免模型组合和噪声组合相互复制。

### 6.2 推荐目录

```text
config/
  gimbal_pipeline.yaml
  norm4_profiles/
    backend_profiles.yaml
    ukf_profiles.yaml
    motion_profiles.yaml
    structure_profiles.yaml
```

### 6.3 主配置引用键

```yaml
norm4_v3:
  backend:
    backend_type: "ukf_v1"              # ukf_v1 | ukf_v2 | inekf
    ukf_profile_id: "ukf_default"
    motion_profile_id: "imm2d_cv_zcv_yawcv"
    noise_profile_id: "cartesian_fixed_v1_compatible"
    structure_profile_id: "slow_error_state_default"
```

### 6.4 motion profile（兼容现有实现）

```yaml
norm4_profiles:
  motion:
    imm2d_cv_zcv_yawcv:
      provider: "single_armor_imm"      # single_armor_imm | kalmanfilters_factory | native
      xyz:
        type: "imm"
        imm_2d_models: ["cv", "ca", "ctrv", "cs"]
      z:
        type: "cv"                      # cv | ca | cs
      yaw:
        type: "cv"                      # cv | ca | cs

  noise:
    cartesian_fixed_v1_compatible:
      provider: "fixed_cartesian"
      sigma_pos_xy: 0.06
      sigma_pos_z: 0.08
      sigma_yaw: 0.12
      dual_raw_R_scale: 1.5

    distance_aware_a:
      provider: "distance_aware"
      sigma_pos_xy_base: 0.04
      sigma_pos_xy_k: 0.02
      sigma_pos_z_base: 0.05
      sigma_pos_z_k: 0.03
      sigma_yaw_base: 0.10
      sigma_yaw_k: 0.04

    imm3d_factory_a:
      provider: "kalmanfilters_factory"
      model_type: "IMM_CV_CA_CTRV"
      sub_models: ["CV_KF", "CA_KF", "CTRV_EKF"]
```

### 6.5 组合方式

1. 参数文件层组合：launch 加载 `base + profile`。
2. profile-id 层组合：节点内部解析 profile 并做校验。

### 6.6 structure profile（慢结构误差更新）

```yaml
norm4_profiles:
  structure:
    slow_error_state_default:
      provider: "slow_error_state"

      # 慢噪声 Q_theta
      q_theta_r1: 1.0e-6
      q_theta_r2: 1.0e-6
      q_theta_dza: 5.0e-7

      # 结构先验 N(theta0, Sigma_theta)
      prior_r1: 0.15
      prior_r2: 0.20
      prior_dza: 0.0
      prior_sigma_r: 0.06
      prior_sigma_dza: 0.06

      # Lambda_theta（单观测默认冻结）
      alpha_r1_single: 0.00
      alpha_r2_single: 0.00
      alpha_dza_single: 0.00
      alpha_r1_dual: 0.05
      alpha_r2_dual: 0.05
      alpha_dza_dual: 0.02

      # 放开条件
      min_confidence: 0.65
      min_margin: 1.0
      stable_frames_for_single: 5

      # 物理边界
      min_r: 0.05
      max_r: 0.50
      min_dza: 0.0
      max_dza: 0.12
```

## 7. 里程碑与交付

### M0：资产复用基线梳理（新增）

1. 固化 CS/IMM 复用清单。
2. 完成 `gimbal_pipeline` 与 `kalmanFilters` 依赖评估。
3. 输出状态映射设计（4D/6D/9D）。

### M1：后端接口化（不改行为）

1. `IStructuredBackend` 与 `BackendFactory`。
2. `UkfBackendV1Adapter` 接入，保证 `ukf_v1` 行为完全等价。

### M2：先接现有 IMM（最小风险）

1. 新增 `UkfBackendV2` 基础骨架（默认运动模型与噪声模型保持 v1 语义）。
2. 接入 `SingleArmorIMMTracker` 到 motion bundle。
3. 交付 `IMM-2D + z:CV + yaw:CV`。

### M2.5：观测噪声接口化（新增）

1. 新增 `IMeasurementNoiseModel` 与 `FixedCartesianNoiseModel`。
2. 验证 `noise_profile=cartesian_fixed_v1_compatible` 与 v1 指标一致。

### M3：补齐目标模型矩阵

1. 增加 `z:CA/CS`、`yaw:CA/CS`。
2. 支持 `CS`/`IMM` 的 profile 切换。

### M4：kalmanFilters 工厂接入（可选增强）

> 实现暂缓，优先保证 `SingleArmorIMMTracker` 复用稳定后再接入更丰富的 `kalmanFilters` 模型。

1. `KalmanFiltersModelBundle` + `ModelFactoryRegistry` 对接。
2. 完成与 `SingleArmorIMMTracker` 的 A/B 回放。

### M5：InEKF + 慢结构误差更新 + 灰度

1. InEKF shadow 全链路。
2. 慢结构误差更新链路上线（非 RM）。
3. 灰度切换与回退策略验收。

## 8. 风险与控制

1. **重复实现风险**：强制“先复用、后扩展”，禁止并行重写 CS/IMM。
2. **状态维度映射风险**：在 adapter 层统一做维度转换，增加单测覆盖（CV/CA/CTRV/IMM）。
3. **kalmanFilters IMM 工程化风险**：
   - 当前 `IMM` 基类存在较多 `std::cout` 调试输出。
   - `IMM_Factory` 子模型生命周期管理使用原始指针，需在接入层做严格托管。
4. **能力缺口风险**：`SingleArmorIMMTracker` 当前 z/yaw 为 CV，需按里程碑补齐 CA/CS。
5. **集成风险**：`gimbal_pipeline` 尚未声明 `models/basic_models/combined_models` 依赖，先完成编译链路打通再接主链。
6. **链路扰动风险**：任何涉及 `ukf_backend_v1` 语义变化的修改一律禁止；新增功能仅进入 `ukf_backend_v2`。
7. **慢结构漂移风险**：若 `Lambda_theta`/先验配置过松，`r1/r2/dza` 会漂移；必须联动 NIS、confidence、margin 控制放开。

## 9. 第二阶段完成定义（DoD）

满足以下条件视为第二阶段完成：

1. 后端接口与实现解耦，UKF 与 InEKF 可配置切换。
2. `ukf_backend_v1` 保持不改动，`ukf_backend_v2` 独立实现并可灰度切换。
3. 运动模型接口化完成，并已复用现有 CS/IMM 实现（非重复开发）。
4. 观测噪声生成完成接口化，可通过 profile 独立切换。
5. UKF 模型矩阵达到目标（`xyz: CV/CA/Singer/CS/IMM`，`yaw: CV/CA/CS`，IMM-z 变体可配）。
6. InEKF + 慢结构误差更新链路（非 RM）完成 shadow 与灰度验证。
7. 配置体系完成 profile 化改造，新增组合不再复制大块 yaml。
8. 输出回放与实车验收数据，指标不低于第一阶段基线。
