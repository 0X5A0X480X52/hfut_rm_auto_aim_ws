# Norm4 V3 第二阶段实现审计报告

## 1. 审计范围与基准
- 审计对象：`src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/trackers/norm4_v3` 及其对应 `src/.../norm4_v3/*.cpp` 实现。
- 对照基准：`norm4_second_phase_plan.md`（第二阶段计划）与同目录 InEKF 慢结构推导文档。
- 目标：确认实现是否与计划一致、识别风险、给出整改建议。

## 2. 总体结论
- 总体状态：**部分完成且主链路可运行**。
- 已完成核心抽象：后端接口、后端工厂、UKF v2 后端、运动模型 bundle 接口、观测噪声接口、InEKF 后端与结构 provider 接口。
- 主要偏差：
  - InEKF 默认仍挂载 `UkfSnapshotStructureProvider`，未默认走“慢结构误差更新”路径。
  - 配置组织尚未实现“UKF 与运动模型配置解耦 + 引用式组合”，目前仍大量依赖 `norm4_v3.ukf_v1.*`。
  - 运动模型可扩展接口已建好，但 UKF v2 的默认工厂仅走 `CompositeProcessModel + RotationModel::CV`，尚未完成计划中的模型族可配置落地（CV/CA/Singer/CS/IMM 等）。

## 3. 与计划一致性审计

### 3.1 已对齐项
1. 后端接口抽象完成（满足“接口与实现分离”）
- 证据：`IStructuredBackend` 已定义统一生命周期/评估/提交接口。
- 参考：`include/.../norm4_backend_interface.hpp:25`

2. Tracker 已通过接口持有后端，链路未强耦合具体实现
- 证据：`Norm4ArmorTrackerV2` 持有 `std::unique_ptr<IStructuredBackend>`。
- 参考：`include/.../norm4_tracker_v2.hpp:71`

3. 后端工厂完成多后端分发（`ukf_v1/ukf_v2/inekf`）
- 证据：`backend_type_from_string` + `create_backend`。
- 参考：`include/.../norm4_backend_factory.hpp:26`, `:53`

4. 保留现有主链路，未直接改写 `norm4_ukf_backend_v1.hpp`
- 证据：通过 `UkfBackendV1Adapter` 适配 V1。
- 参考：`include/.../norm4_backend_factory.hpp:57`, `include/.../norm4_ukf_backend_v1_adapter.hpp`

5. 运动模型接口抽象完成
- 证据：`IMotionModelBundle` + `NativeProcessModelBundle`。
- 参考：`include/.../norm4_motion_model_bundle.hpp:11`

6. 观测噪声生成接口抽象完成
- 证据：`IMeasurementNoiseModel` + `FixedCartesianNoiseModel`。
- 参考：`include/.../norm4_measurement_noise.hpp:13`, `:26`

7. InEKF 位姿后端主流程已实现并接入 tracker
- 证据：`InvariantPoseBackend` 实现 reset/predict/evaluate/update/commit；tracker 构造时按配置选择。
- 参考：`include/.../norm4_inekf_backend.hpp:25`, `src/.../norm4_tracker_v2.cpp:16-19`

### 3.2 未完全对齐项
1. InEKF 慢结构误差路径未作为默认接线
- 现状：工厂中 INEKF 分支默认创建 `UkfSnapshotStructureProvider`。
- 参考：`include/.../norm4_backend_factory.hpp:71-73`
- 含义：尽管 `SlowStructureErrorUpdaterProvider` 已实现，但默认路径仍是“直接读主状态结构量”，与“优先采用慢结构误差更新路径”的计划目标不一致。

2. 慢结构 provider 的启用策略缺少配置化入口
- 现状：未见 `backend_factory` 根据配置切换 `UkfSnapshotStructureProvider` / `SlowStructureErrorUpdaterProvider`。
- 参考：`include/.../norm4_backend_factory.hpp:66-75`

3. 配置仍显著依赖 `ukf_v1` 命名空间
- 现状：InEKF/UKF v2 评估与 sanity/gate 参数读取仍指向 `config_.norm4_v3.ukf_v1.*`。
- 参考：`src/.../norm4_inekf_backend.cpp:232`, `:322`, `:386`, `:484`, `:626`
- 含义：配置拆分目标（UKF 配置与模型配置分离并引用组合）尚未完成。

4. 运动模型多族能力尚未在工厂层体现
- 现状：`create_default_v2_process_model` 固定 `RotationModel::CV`，translation 走 `cfg.motion.translation_model`，未体现计划中的 yaw: CV/CA/CS 与 xyz: CV/CA/Singer/CS/IMM 全面可配。
- 参考：`include/.../norm4_backend_factory.hpp:49-50`

## 4. 风险清单（按严重度）

### 高风险
1. 计划目标“慢结构误差链路”未默认生效，可能导致上线行为与设计预期不一致
- 证据：INEKF 默认 provider 为 `UkfSnapshotStructureProvider`。
- 位置：`include/.../norm4_backend_factory.hpp:71-73`
- 影响：结构参数更新仍偏“快状态同频更新”，慢时标鲁棒性收益无法保证。

2. 配置耦合 `ukf_v1`，后续扩展/调参存在隐性冲突风险
- 证据：InEKF 关键门限、结构增益、后验 sanity 全部读取 `ukf_v1`。
- 位置：`src/.../norm4_inekf_backend.cpp:232,322,386,484,626`
- 影响：当 `ukf_v2/inekf` 分别调参时易互相干扰，难以做到独立版本管理。

### 中风险
1. `backend_type_from_string` 对未知字符串静默回落 `UKF_V1`
- 位置：`include/.../norm4_backend_factory.hpp:30`
- 影响：配置拼写错误可能被吞掉，导致“以为在跑 inekf/ukf_v2，实际跑 v1”。

2. 运动模型配置维度未完全暴露
- 位置：`include/.../norm4_backend_factory.hpp:33-50`
- 影响：计划中的 CS/IMM 组合难以通过配置直接装配，削弱第二阶段目标。

### 低风险
1. 慢结构 provider 已实现但缺少运行时可观测信息
- 位置：`include/.../norm4_structure_provider.hpp:67-201`
- 影响：调试时难快速确认当前是否进入慢结构更新、是否收敛。

## 5. 建议整改（不改动 v1 主链路前提）
1. 在 `backend_factory` 增加 InEKF 结构 provider 选择开关
- 默认按计划切到 `SlowStructureErrorUpdaterProvider`（保留回退到 `UkfSnapshotStructureProvider` 的选项）。

2. 新增 `norm4_v3.inekf` 与 `norm4_v3.ukf_v2` 独立配置段
- 将 gate/sanity/single_update/dual_update 从 `ukf_v1` 迁出，避免跨后端耦合。

3. 落地“组合式配置”
- 建立 `backend_profile`（后端算法）+ `motion_profile`（xyz/yaw/structure 模型）的引用式组合，避免配置文件爆炸。

4. 工厂层补全运动模型装配矩阵
- 先覆盖 CV/CA/Singer/CS，再接 IMM（可复用 `src/kalmanFilters/filters` 与 ambiguous adapter 现有实现）。

5. 增加最小审计单测/集成验证
- 至少验证：
  - `backend_type` 配置错误时显式报错；
  - InEKF 结构 provider 切换是否生效；
  - `ukf_v2/inekf` 是否仅消费各自配置段。

## 6. 审计结论（供里程碑评审）
- 第二阶段基础框架已搭好，方向正确。
- 当前距离“与计划完全一致”仍有关键差距，尤其是**慢结构路径默认接线**和**配置解耦**两项。
- 建议将上述两项列为下一迭代的硬门槛验收条件。
