# MaxEntropyTracker Binder 包重构设计

## 1. 设计目标

当前 `max_entropy_tracker/association` 中的绑定逻辑存在以下问题：

1. 各 Tracker（如 `AdaptiveArmorTracker`、`OutpostArmorTracker`）内部分别维护一套绑定状态机和跳变判定流程，重复实现较多。
2. `AdaptiveArmorBinder` 已实现但未接入业务主链，抽象能力未发挥。
3. 跳变检测、ID 切换、健康确认、强制重绑等逻辑耦合在 Tracker 内部，不利于后续扩展新机器人类型。

本设计目标是在 `max_entropy_tracker` 下新增 `binder` 包，统一替代 `association` 包中与“装甲板 ID 绑定”相关功能，形成：

- 通用状态机：`BindingFSM`
- 可扩展跳变检测策略：`JumpEventDecoder`（策略模式）
- 机器人专用绑定器：`IDBinder`（单观测/双观测路径）
- 可选统一确认器：`HypothesisScorer`
- 标准化调试与离线验证数据模型

说明：本设计仅定义架构和实现方案，不直接修改现有代码。

## 2. 重构范围与边界

### 2.1 纳入 binder 包的能力

1. 跳变检测（基于 `z jump`、结构先验、`yaw_rate` 方向等）
2. 绑定状态机（锁定、待确认、切换、强制重绑）
3. 单观测/双观测两类 ID 绑定流程
4. 绑定健康度评估和异常连续帧强制重绑机制
5. 调试快照与可离线评估字段

### 2.2 不纳入 binder 包的能力

1. 观测关联（panel associator）
2. UKF 预测更新与运动学约束
3. 几何生成与可视化
4. 上游 detector 的原始观测解析

这些仍由各 Tracker 或其已有模块负责。

### 2.3 `association` 功能重叠与替代目标

当前 `association` 目录组件：

1. `panel_associator.hpp`：观测 panel 关联（yaw/xy/z 组合代价）
2. `height_identifier.hpp`：层级识别（single/dual）
3. `panel_mismatch_detector.hpp`：连续异常纠错（PATCH/REINIT）
4. `oscillation_detector.hpp`：参数振荡检测
5. `adaptive_armor_binder.hpp/.cpp`：绑定状态机与策略

与新 `binder` 的关系建议：

1. **直接迁入并升级到 binder**
   - `adaptive_armor_binder`（并入 `binder/core + pipeline`）
   - `panel_mismatch_detector`（并入 `binder/scorer` 的健康监控分支）
2. **保留在 association（短期），后期按需迁移**
   - `panel_associator`：它更偏“观测关联”而非“绑定决策”
   - `height_identifier`：可先作为 IDBinder 的外部输入，后续再内聚
   - `oscillation_detector`：与结构参数稳定性相关，建议保留在 Tracker 侧

结论：`binder` 可覆盖并替代 `association` 中与“ID 绑定决策”直接相关的能力；关联和滤波参数稳定性能力不强制迁入。

## 3. 目录结构提案

在 `src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/` 下新增：

```text
binder/
  core/
    binding_fsm.hpp
    binding_policy.hpp
    binding_window_counter.hpp
  model/
    binder_types.hpp
    binder_enums.hpp
    geometry_profile.hpp
  decoder/
    jump_event_decoder.hpp
    four_panel_jump_decoder.hpp
    outpost_trilevel_jump_decoder.hpp
    generic_cyclic_jump_decoder.hpp
  id_binder/
    id_binder.hpp
    single_obs_sequence_binder.hpp
    dual_obs_direct_binder.hpp
    hybrid_id_binder.hpp
  scorer/
    hypothesis_scorer.hpp
    null_hypothesis_scorer.hpp
    residual_hypothesis_scorer.hpp
  pipeline/
    binder_pipeline.hpp
  factory/
    binder_factory.hpp
  debug/
    binder_debug_snapshot.hpp
```

实现文件放在 `src/max_entropy_tracker/binder/**.cpp`。

## 4. 机器人统一描述与 Profile 驱动

`JumpEventDecoder`、`IDBinder`、`HypothesisScorer` 都与机器人结构先验强相关。为了避免“每个模块都各自维护先验”，建议将 `gimbal_pipeline/common/robot_description` 作为唯一先验来源，映射到 binder 内部统一 `RobotBindingProfile`。

### 4.1 RobotBindingProfile（来自 robot_description）

建议从 robot_description 归一化得到如下最小字段：

1. `robot_type`
2. `panel_count`
3. `z_offsets`
4. `cyclic_order`（俯视下 panel id 顺序）
5. `has_dual_obs_capability`
6. `jump_signature`（可编码支持的 jump 型别，如 `DZ`、`DOUBLE_DZ`）
7. `height_semantics`（如 `LOWER/UPPER` 或 `HIGH/MIDDLE/LOW`）

约束：

1. Tracker 不直接拼装结构先验，统一向 binder 请求 profile。
2. Decoder/Binder/Scorer 仅消费 `RobotBindingProfile`，不访问 Tracker 私有结构状态。

### 4.2 组合式策略装配原则

不是“每个机器人都手写一套 Decoder/Binder/Scorer”，也不是“一个全通用实现”，而是组合装配：

1. 通用组件优先：`GenericCyclicJumpDecoder`、`HybridIDBinder`、`ResidualHypothesisScorer`
2. 专用组件按需覆盖：如 `OutpostTriLevelJumpDecoder`
3. 仅当通用组件在离线指标不达标时才引入机器人专用实现

这样可以在可扩展和可维护之间保持平衡，避免过度设计。

### 4.3 BinderFactory 装配策略（推荐）

`BinderFactory` 根据 `RobotBindingProfile` 自动装配 pipeline：

1. 4板标准机器人：
   - Decoder: `GenericCyclicJumpDecoder`（或 `FourPanelJumpDecoder`）
   - IDBinder: `HybridIDBinder`
   - Scorer: `ResidualHypothesisScorer`
2. Outpost 3板：
   - Decoder: `OutpostTriLevelJumpDecoder`
   - IDBinder: `HybridIDBinder`（优先 dual，回退 single）
   - Scorer: `ResidualHypothesisScorer`（可选弱化）
3. 未知新类型：
   - Decoder: `GenericCyclicJumpDecoder`
   - IDBinder: `SingleObsSequenceBinder`
   - Scorer: `NullHypothesisScorer`

### 4.4 与 `robot_description` 的集成方式

建议新增 `RobotBindingProfileProvider` 作为适配层，位于 `binder/factory` 或 `gimbal_pipeline/common/robot_description` 扩展目录：

1. 输入：`rm_interfaces::msg::TrackedRobot::robot_type` 或内部 `RobotType`
2. 输出：`RobotBindingProfile`
3. 负责做一次性规范化：
   - 面板顺序方向统一
   - `z_offsets` 维度和排序校验
   - 是否支持 dual obs 标记
4. 提供运行时断言，避免每个 Decoder 再重复防御性判断

## 5. 核心抽象与数据契约

## 5.1 通用输入输出模型（model 层）

`BinderFrameInput`（每帧输入）：

1. 时间与上下文：`timestamp`, `robot_profile`, `obs_count`
2. 候选信息：`candidate_id`, `candidate_prob`, `candidate_margin`
3. 观测信息：`obs_z_values`, `obs_yaw_values`, `z_jump`
4. 动态信息：`yaw_rate_est`, `spin_direction_hint`
5. 几何先验：`panel_count`, `z_offsets`, `cyclic_order`
6. 健康辅助：`same_panel_residual`, `nis`, `has_history`

`JumpDecision`（跳变判决）：

1. `detected`
2. `jump_kind`：`NONE`, `DZ`, `DOUBLE_DZ`, `AMBIGUOUS`, `INVALID`
3. `from_id`, `to_id`
4. `confidence`
5. `evidence_mask`
6. `reason_code`

`BinderOutput`（绑定结果）：

1. `selected_id`
2. `fsm_state`
3. `action`：`HOLD`, `PENDING`, `SWITCH`, `FORCE_REBIND`, `RELOCK`
4. `switch_event`
5. `switch_reason`
6. `binding_confidence`

`BindingHealth`（可选健康评估）：

1. `score`
2. `anomaly_detected`
3. `consecutive_bad_frames`
4. `force_rebind_recommend`

## 5.2 BindingFSM（core 层）

`BindingFSM` 只处理“是否切换”时序，不负责几何判别：

状态：

1. `LOCKED`
2. `PENDING_SWITCH`
3. `UNLOCKED`（可选，用于不可信阶段快速恢复）
4. `LOCKED_NEW`（切换后保护期）

输入：

1. `target_id`
2. `target_confidence`
3. `jump_decision`
4. `binding_health`

输出：

1. `action`
2. `selected_id`
3. `switch_event/reason`

关键策略：

1. 非跳变且健康正常：保持锁定
2. 跳变成立且连续确认达标：切换
3. 健康连续异常：触发强制重绑建议并进入 `UNLOCKED` 或直接 `FORCE_REBIND`

## 5.3 JumpEventDecoder（decoder 层，策略模式）

接口：

```cpp
class JumpEventDecoder {
 public:
  virtual ~JumpEventDecoder() = default;
  virtual JumpDecision decode(const BinderFrameInput& in,
                              const DecoderContext& ctx) = 0;
};
```

实现：

1. `FourPanelJumpDecoder`
2. `OutpostTriLevelJumpDecoder`
3. `GenericCyclicJumpDecoder`（未来扩展）

`OutpostTriLevelJumpDecoder` 需要显式支持：

1. `dz` 与 `2dz` 模式判定
2. 基于 `cyclic_order + yaw_rate_sign` 的 `from_id -> to_id` 解码
3. 单观测时利用历史缓存提高置信度

## 5.4 IDBinder（id_binder 层）

不同机器人可使用不同“ID 确认流程”，统一抽象如下：

```cpp
class IDBinder {
 public:
  virtual ~IDBinder() = default;
  virtual TargetDecision propose(const BinderFrameInput& in,
                                 const JumpDecision& jump,
                                 const BinderContext& ctx) = 0;
};
```

实现分为三类：

1. `SingleObsSequenceBinder`
2. `DualObsDirectBinder`
3. `HybridIDBinder`（按 `obs_count` 自动路由）

### 4.4.1 SingleObsSequenceBinder

适用于“长期单观测”：

1. 记录切换前后 `z`、`z_jump`、`yaw_rate`、历史锁定 ID
2. 基于结构先验和 jump 类型推断候选 `to_id`
3. 输出 `target_id + confidence`

### 4.4.2 DualObsDirectBinder

适用于“同帧双观测”：

1. 同帧高度差直接映射 ID 关系
2. 若双观测判定冲突，降级到 `SingleObsSequenceBinder` 或标注 `AMBIGUOUS`

### 4.4.3 HybridIDBinder

策略：

1. `obs_count >= 2` 优先 `DualObsDirectBinder`
2. 否则走 `SingleObsSequenceBinder`
3. 双观测失败回退单观测路径（可配置）

## 5.5 HypothesisScorer（scorer 层，可选）

职责：

1. 评估“当前锁定 ID”与“观测一致性”
2. 输出健康分数和连续异常计数
3. 异常持续达到阈值触发 `force_rebind_recommend`

注意：`HypothesisScorer` 不是主判据，仅作为确认与故障恢复辅助。

## 6. 绑定流水线（pipeline 层）

`BinderPipeline` 统一 orchestrator：

1. `JumpEventDecoder::decode()`
2. `IDBinder::propose()`
3. `HypothesisScorer::evaluate()`（可选）
4. `BindingFSM::step()`
5. 产出 `BinderOutput + BinderDebugSnapshot`

伪流程：

```text
FrameInput
  -> JumpDecoder
  -> IDBinder(target)
  -> (optional) HypothesisScorer(health)
  -> BindingFSM(action)
  -> BinderOutput
```

## 7. 通用与专用的边界准则（避免过度设计）

为了保持调用链清晰、避免配置与类数量膨胀，建议采用以下边界：

1. **必须通用化**
   - `BindingFSM`
   - `BindingHealthMonitor`
   - `BinderDebugSnapshot`
   - 基础窗口计数和确认逻辑
2. **优先通用，可按需专用**
   - `JumpEventDecoder`
   - `IDBinder`
   - `HypothesisScorer`
3. **仅在以下条件引入专用实现**
   - 通用实现在目标机器人上离线指标（误切换率/切换延迟）持续不达标
   - 结构先验明显不满足通用假设（如 outpost `dz/2dz` 解码）

最小策略：

1. 先上通用实现 + 参数化 profile
2. 观察日志指标
3. 再定点替换一个策略模块，而不是整体重写 pipeline

## 8. 来自 `armor_solver` 的可借鉴设计

`src/rm_auto_aim/armor_solver` 可提供以下参考模式：

1. **状态机职责清晰**
   - `LOST/DETECTING/TRACKING/TEMP_LOST` 切分明确
   - 迁移到 binder 时应保持“状态机只做状态转移，不做复杂几何判定”
2. **事件触发式切换**
   - `handleArmorJump()` 将“跳变事件”和“状态修复动作”解耦
   - 可映射为 `JumpDecision -> BindingFSM Action` 的机制
3. **阈值可配置**
   - `max_match_distance`、`max_match_yaw_diff` 等参数化思路可复用
   - binder 参数也应保持同风格，避免硬编码
4. **异常恢复优先级**
   - 当状态不可信时进行 reset/reinit
   - 对应 binder 中的 `force_rebind` 与 `health-based recovery`
5. **代价函数可组合思想**
   - `Design_of_Loss_Functions_For_ArmorSellection.md` 中多项损失加权思想
   - 可用于 `HypothesisScorer` 的健康分融合（而非主判据）

不建议直接照搬部分：

1. `armor_solver` 的跳变逻辑偏 4板目标追踪语义，不能直接覆盖 outpost 3层先验
2. 其 tracker 与当前 max_entropy tracker 的观测维度和时序约束不同

## 9. 配置体系设计

建议从 `UnifiedConfig` 中新增 `binder` 子配置块：

1. `binder.common`
2. `binder.decoder`
3. `binder.id_binder`
4. `binder.scorer`

示例参数：

1. `confirm_frames`
2. `lock_new_hold_frames`
3. `force_rebind_bad_frames`
4. `jump_dz_tolerance`
5. `jump_2dz_tolerance`
6. `single_obs_history_window`
7. `dual_obs_enable`
8. `scorer_enable`

保留兼容策略：旧 `tracker.jump_binding_*` 与 `outpost.binding_*` 在迁移期映射到新参数，避免一次性破坏现有配置。

## 10. 调试与离线验证设计

新增统一调试快照 `BinderDebugSnapshot`，至少包含：

1. `decoder_name`, `id_binder_name`, `scorer_name`
2. `jump_detected`, `jump_kind`, `jump_confidence`, `from_id`, `to_id`
3. `target_id`, `target_confidence`
4. `fsm_state`, `action`, `switch_reason`
5. `health_score`, `consecutive_bad_frames`, `force_rebind_flag`
6. 关键原始证据：`obs_count`, `z_jump`, `yaw_rate_sign`, `candidate_prob`, `candidate_margin`

离线评估指标：

1. 跳变检测 precision/recall
2. 误切换率（false switch）
3. 切换延迟（真实跳变到确认切换的帧数）
4. 锁定稳定率（静态期 id 波动比例）
5. 强制重绑触发频率与恢复时延

## 11. 与现有 Tracker 的集成方案

## 11.1 AdaptiveArmorTracker

替换点：

1. `reset_jump_binding()`
2. `update_jump_binding()`
3. `update_jump_statistics()`
4. `compute_jump_binding_confidence()`

保留：

1. panel association
2. height identification
3. UKF update

## 11.2 OutpostArmorTracker

替换点（按阶段）：

1. 先将 `z-audit` 逻辑迁入 `OutpostTriLevelJumpDecoder`
2. 再将 `update_binding_state_machine` 迁入 `BindingFSM`
3. 将“冲突重绑计数”迁入 `HypothesisScorer` + FSM 强制重绑

保留：

1. outpost 假设构建与观测模型
2. UKF 更新

## 12. 分阶段实施计划

阶段 1：脚手架与契约落地

1. 新增 `binder/model/core/debug`
2. 提供 `BindingFSM` 和空实现 `NullDecoder/NullBinder/NullScorer`
3. 接入 `robot_description -> RobotBindingProfile` 适配层

阶段 2：接入 4 板路径

1. 实现 `FourPanelJumpDecoder + SingleObsSequenceBinder`
2. 在 `AdaptiveArmorTracker` 以 feature flag 接入
3. 与旧逻辑并行输出调试字段，核对一致性

阶段 3：接入 outpost 路径

1. 实现 `OutpostTriLevelJumpDecoder + HybridIDBinder`
2. 迁移 `OutpostArmorTracker` 绑定主链
3. 校验 `dz/2dz` 判定与 `yaw_rate` 方向规则

阶段 4：统一 scorer 与强制重绑

1. 接入 `ResidualHypothesisScorer`
2. 统一 “连续异常 -> 强制重绑” 机制

阶段 5：收敛与去重

1. 删除 tracker 内重复绑定状态变量
2. 旧 `association` 中与绑定决策直接相关逻辑降级或移除
3. 为 `association` 保留“兼容 facade” 一个版本周期，便于灰度切换

## 13. 调用链示例（简洁版）

以 Tracker 单帧更新为例：

```text
Tracker::update()
  -> build BinderFrameInput (from observation + filter state)
  -> profile = RobotBindingProfileProvider::get(robot_type)
  -> binder_pipeline.step(input, profile)
      -> decoder.decode()
      -> id_binder.propose()
      -> scorer.evaluate()      (optional)
      -> fsm.step()
  -> apply selected_id to UKF update
  -> emit binder debug snapshot
```

该调用链固定且短，便于维护和调试。

## 14. 兼容性与替换策略

为了后期可直接替换 `association`，建议采用“接口镜像 + 渐进切换”：

1. 先在 binder 提供与旧调用近似的 facade（如 `bind(...)` 风格接口）
2. Tracker 先切到 facade，不立刻依赖内部新接口细节
3. 完成行为对齐后，逐步移除 `association/adaptive_armor_binder`
4. 对 `panel_associator`、`height_identifier` 保持独立模块，避免一次性搬迁带来回归风险

替换终态建议：

1. `association` 仅保留“观测关联类”
2. `binder` 统一承接“ID 绑定决策类”
3. 通过 `RobotBindingProfileProvider` 串联两者，避免先验重复定义

## 15. 风险与对策

风险 1：迁移后行为漂移  
对策：旧/新双路并行记录，按日志对比切换事件与最终命中率。

风险 2：参数爆炸  
对策：分层参数命名，默认值按机器人 profile 给定，减少跨模块耦合。

风险 3：调试信息不足  
对策：强制统一输出 `BinderDebugSnapshot` 到 tracker 日志。

## 16. 验收标准

1. 两类 Tracker 均通过统一 `binder` 包完成 ID 绑定主流程。
2. 单观测长序列下，跳变识别稳定，误切换率低于旧实现。
3. 双观测场景下，ID 切换响应更快且无明显抖动。
4. 调试日志可以完整复盘每次切换原因。
5. 删除重复实现后，绑定相关代码总量下降且职责边界清晰。
