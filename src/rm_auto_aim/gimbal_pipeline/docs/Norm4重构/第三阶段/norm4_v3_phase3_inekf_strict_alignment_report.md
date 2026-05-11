# Norm4 V3 第三阶段：InEKF 严格对齐审计与整改记录

## 1. 目标与范围
- 目标：将 `norm4_v3` 中 `inekf` 路径与第二阶段设计文档严格对齐，重点覆盖：
  - yaw 定义域统一（2π 语义一致）；
  - Hypothesis 离散状态（panel/phase）显式建模；
  - InEKF 与 UKF/欧式模型解耦；
  - InEKF 运动/噪声/结构配置独立化；
  - 创新、线性化、协方差传播在不变误差坐标下的一致性。
- 审计对象：
  - `include/max_entropy_tracker/trackers/norm4_v3/**`
  - `src/max_entropy_tracker/trackers/norm4_v3/**`
  - `config/gimbal_pipeline.yaml`
  - `docs/Norm4重构/第二阶段/hypothesis_inekf_slow_structure_derivation.md`

## 2. 结论摘要
- 当前状态：**已完成关键基础整改，但尚未“数学上严格对齐”到完整不变误差 InEKF 版本**。
- 已完成：
  - yaw 主链路统一到 2π wrap 语义；
  - InEKF 拥有独立 runtime 配置入口；
  - Hypothesis 层补充了 hybrid 离散状态承载；
  - InEKF 观测模型从“硬编码 panel 角拼接”改为基于 panel profile 的统一几何表达；
  - InEKF 内部引入独立预测噪声构造与状态回缩接口。
- 未完成（阻塞“严格对齐”）：
  - innovation/H/F/Q/NIS 仍以欧式误差近似为主，尚未完整切换到左/右不变误差坐标体系；
  - retraction 仍是“加法+角度wrap”的一阶近似，不是完整群操作实现；
  - gating 与统计量未完全按 invariant residual 重定义。

## 3. 本阶段已落地整改

### 3.1 yaw 定义域与角度处理统一
- 问题：ukf_v1 可稳定绑定高速旋转目标，但 ukf_v2/inekf 绑定异常，核心诱因之一是 yaw 分解语义不一致（π 半周期与 2π 全周期混用）。
- 处理：
  - 移除/弱化 `decompose_yaw/compose_yaw` 在 norm4_v3 主链路中的半周期依赖；
  - 统一使用 full-yaw + `normalize_angle` / `angle_difference`；
  - UKF sigma 点与观测均值对角度维使用圆均值，减少跨 ±π 边界时的均值偏置。
- 影响：`0/1/2/3` 交替阶段的 alias 误判概率降低，尤其在高速自旋和边界相位切换时更稳定。

### 3.2 Hypothesis 离散状态显式化（hybrid state）
- 问题：`panel_id / k / phase` 分散在不同成员中，commit 时语义不透明，调试难定位。
- 处理：
  - 在 `PredictContext` 引入 `hybrid_prior`；
  - 在 `UkfTrial` 引入 `hybrid_post`；
  - 在 `BackendSnapshot` 补充 `hybrid` 信息；
  - InEKF commit 时同步 `phase_index` 等离散状态。
- 效果：离散/连续状态职责边界更清晰，便于日志、回放、NIS 解释和失败归因。

### 3.3 InEKF 观测几何表达改造
- 问题：旧路径存在 `compose_yaw(k, delta) + panel_id*pi/2` 一类拼接式写法，和文档“群状态作用到 panel 相位”表达不一致。
- 处理：
  - `obs_model_single` 与 Jacobian 改为基于 `PanelProfile`（`phase_offset/z_sign/use_r2`）；
  - 由统一几何模板生成各 panel 预测观测。
- 效果：观测模型更接近“状态 + 结构 + 离散相位”的分层设计，减少硬编码分支漂移。

### 3.4 InEKF 独立 runtime 配置入口
- 新增 `norm4_v3.inekf_runtime` 参数段，支持：
  - `motion_profile` / `noise_profile` / `structure_profile`；
  - `translation_model`、CV/CA/Singer 噪声参数覆盖；
  - spin 相关噪声覆盖（`delta_rate` / `delta_acc`）。
- 工厂逻辑改造后，InEKF 可优先消费独立 profile，而非完全复用 `backend_config`。
- `noise_profile` 已支持路径模式，例如：
  - `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/noise/ypd_ba_low_weight.yaml`

### 3.5 调试可观测性增强
- 增加可选、节流日志（enable/throttle/verbose）；
- 支持在不刷屏的前提下输出 candidate/commit 关键信息；
- 便于对比 `ukf_v1` 正常日志与 `inekf` 异常日志的分歧点（phase、yaw residual、NIS、gate）。

## 4. 仍与设计文档不一致的关键点（严格对齐缺口）

### 4.1 创新定义仍偏欧式
- 现状：大量路径使用 `z_obs - z_pred`（yaw 维 wrap），属于欧式残差框架。
- 设计要求：创新应在左/右不变误差坐标定义（与选定误差形式一致）。
- 风险：在快速旋转、大姿态差、相位切换时，残差统计与理论协方差不完全匹配。

### 4.2 Jacobian/误差传播未完整不变化
- 现状：`H`/`F` 主要是欧式线性化近似。
- 设计要求：误差传播应使用不变误差动力学对应形式，必要处引入 Adjoint 映射一致化。
- 风险：NIS 与 gate 阈值的统计解释会偏离理想模型。

### 4.3 协方差更新仍是标准 EKF 形式
- 现状：`K=PH^TS^-1` + Joseph 形式协方差更新。
- 说明：这在工程上可用，但不等价于完整 InEKF 理论实现。
- 风险：当状态误差非小扰动时，收敛速度与稳态统计可能劣化。

### 4.4 Retraction 仍为一阶近似
- 现状：位置加法 + yaw wrap。
- 设计要求：依据选定群状态（SE(2.5) 表达）定义一致的 `Exp/Log` 回缩与误差注入。
- 风险：强机动和大创新条件下，后验几何一致性受限。

## 5. 关于 SE(2.5) 与运动模型（CV/CA）对齐结论
- 结论 1：InEKF 不应直接复用 `IMotionModelBundle` 的欧式假设作为唯一真源，已开始内聚内部预测与 Q 构造，这是正确方向。
- 结论 2：当前实现具备 CV/CA 分支能力，但仍需补充“群状态 + 误差状态”形式化映射（尤其 yaw-rate/yaw-acc 的代数层表达）。
- 结论 3：若目标是“严格对齐文档”，需要把预测/更新统一重写为“群状态传播 + invariant error propagation + invariant innovation”完整闭环。

## 6. 关于 BA-aware YPD 噪声接口适用性
- `norm4_ba_aware_ypd_noise.hpp` 在当前“观测空间创新（xyz/yaw 或 ypd+yaw）”实现下仍可用。
- 若创新改为严格不变误差坐标，需要：
  - 将测量噪声从原观测空间映射到对应不变残差坐标；
  - 或在 innovation 形成前后加入一致 Jacobian 变换。
- 否则会出现“R 定义空间与残差空间不一致”的统计偏差。

## 7. 配置侧建议（第三阶段验收口径）
- 在 `gimbal_pipeline.yaml` 中保留并强化 `norm4_v3.inekf_runtime` 独立段：
  - 显式声明 `motion_profile/noise_profile/structure_profile`；
  - `translation_model` 与 profile 组合优先级写入注释；
  - 将 InEKF 关键 gate/sanity 阈值从 `ukf_v1` 语义隔离，避免耦合调参。
- 对 `ypd_ba_low_weight.yaml` 做 InEKF 专项标注（适用残差坐标、推荐距离段、推荐 phase 切换速度段）。

## 8. 后续必须完成项（严格对齐清单）
1. 明确采用左不变或右不变误差定义，并在代码接口中固化（类型/注释/日志字段统一）。
2. 重写 innovation 形成逻辑，替换欧式 `z_obs-z_pred` 为 invariant residual。
3. 对应重写 `H/F/Q` 在误差坐标下的表达（含必要 Adjoint）。
4. 重写 retraction/injection 为群操作一致版本，替换“加法+wrap”近似。
5. 更新 NIS/gate 统计解释与阈值基线，建立与 `ukf_v1` 的 A/B 对照回放报告。
6. 将离散 hybrid state（panel/phase）从“附带元信息”升级为 commit 契约的一等公民（日志、快照、回放协议统一）。

## 9. 备注
- 本文档为第三阶段工程审计与整改记录，强调“现状-差距-落地路径”。
- 数学细节基线以第二阶段文档为准：  
  `docs/Norm4重构/第二阶段/hypothesis_inekf_slow_structure_derivation.md`
