# Outpost Jump 事件重构与周期签名增强设计

## 1. 背景与问题定义

当前 outpost V2 链路中，`has_2dz_signature` 长期为 0，导致 Mode 进入 STRUCTURED 的关键证据不足。日志表现为：

1. `jump_detected=1` 可出现；
2. 但 `jump_kind` 很少/几乎不为 `DOUBLE_DZ`；
3. `has_2dz_signature` 依赖 `jump_kind==DOUBLE_DZ`，因此难以触发；
4. 目标存在较长 `TEMP_LOST` 段，单纯依赖“相邻帧 dz 差分”不稳定。

此外，outpost 的结构先验（`2dz, dz, dz` 周期）需要在稳定世界系（通常 odom）下才可靠，应明确坐标系约束。

---

## 2. 目标

1. 将“观测消失→重现”纳入 jump 事件体系，而非仅在连续帧差分上做 dz 检测。
2. 建立“同板连续段”的 z 统计缓存，提升重现后的 ID 恢复稳定性。
3. 将 `has_2dz_signature` 从“单次跳变标签”升级为“窗口结构证据”。
4. 精简 mode/binder 参数，减少强耦合调参。
5. 保持调用链清晰，避免扩大到 UKF 后端改造。

非目标：

1. 不改动 outpost UKF 状态定义；
2. 不改动 gimbal_controller 控制策略；
3. 不在本阶段引入新 tracker 类型。

---

## 3. 现状调用链（关键路径）

1. `OutpostTrackerV2::update(obs)`
2. `ObservationFrontend::build_binding_candidate(...)` 产出 `candidate.z_jump`
3. `OutpostBinderBridge::step(...)` 组织 `BinderFrameInput`
4. `OutpostTriLevelJumpDecoder::decode(...)` 产出 `JumpDecision`
5. `BindingFSM::step(...)` 决定绑定切换
6. `EvidenceFuser` 使用 `jump_detected/jump_kind` 融合 mode 证据

当前薄弱点：

1. `z_jump` 语义主要是“相邻成功观测差分”；
2. `TEMP_LOST` 重现未形成显式事件类型；
3. `has_2dz_signature` 与 `DOUBLE_DZ` 紧耦合；
4. outpost 三板拓扑中，`cyclic_diff` 判别逻辑对 `DOUBLE_DZ` 不友好。

---

## 4. 核心设计

## 4.1 事件语义重构

在 jump 解码前先做观测事件分类（Event Classifier）：

1. `CONTINUITY_SAME_PANEL_CANDIDATE`
2. `REACQUIRE_AFTER_LOSS`
3. `SWITCH_CANDIDATE`
4. `AMBIGUOUS`

建议新增 `TrackEventType`：

```cpp
enum class TrackEventType : uint8_t {
  CONTINUITY = 0,
  REACQUIRE = 1,
  SWITCH_CANDIDATE = 2,
  AMBIGUOUS = 3
};
```

分类依据（按优先级）：

1. `gap_dt > reacquire_gap_dt_gate || lost_frames > reacquire_lost_frames_gate` → `REACQUIRE`
2. 否则若 `yaw_diff < same_panel_yaw_gate && xy_residual < same_panel_xy_gate` → `CONTINUITY`
3. 否则若候选切换代价明显优于当前绑定 → `SWITCH_CANDIDATE`
4. 否则 `AMBIGUOUS`

---

## 4.2 同板段 z 统计缓存（你提出的主机制）

新增 `ZLevelClusterBank`（按面板假设维护）：

1. 每个 panel 维护 `center_z_ema`, `var_ema`, `count`, `last_update_time`；
2. 仅在 `CONTINUITY` 事件下更新；
3. `REACQUIRE` 时使用 `obs.z` 对所有 panel 做似然匹配：
   `cost_i = |obs.z - (center_z_ema_i + z_offset_i)| / sigma_i`

当连续段稳定时，缓存窗口自然形成 `2dz, dz, dz` 幅值分布，弱化单帧噪声影响。

---

## 4.3 2dz 证据升级为窗口签名

`has_2dz_signature` 改为“窗口结构证据”，不再仅依赖某次 `jump_kind`：

1. 维护最近 `N` 个“可信事件”幅值序列（来自 `CONTINUITY/REACQUIRE` 后的有效 dz）；
2. 对 `|dz|` 做双峰一致性检验（小峰≈dz，大峰≈2dz）；
3. 检查顺序一致性（结合 `spin_direction`，检验相位模式）；
4. 输出 `signature_score in [0,1]`；
5. `has_2dz_signature = signature_score >= period_conf_threshold`。

这使 mode 证据从“瞬时单点”升级为“结构化周期”。

---

## 4.4 TEMP_LOST 重现事件参与 JumpDecision

为 `BinderFrameInput` 增加字段：

1. `bool is_reacquired`
2. `double gap_dt`
3. `int lost_frames`
4. `TrackEventType event_type`

`JumpDecision` 增加来源字段：

1. `JumpEvidenceSource source`：`CONTINUITY/REACQUIRE/PERIODIC/COST`
2. `double signature_score`

重现策略：

1. `REACQUIRE` 时先执行“同板复位判别”（避免误判为切换）；
2. 仅当重现观测与当前板统计明显冲突且另一板显著更优时，产生 switch jump；
3. 若冲突不充分，输出 `AMBIGUOUS` 并保持锁定或进入 pending。

---

## 4.5 Mode 侧接口调整（与已做改造一致）

`EvidenceFuser` 输入应增加：

1. `signature_score`
2. `jump_event_detected`（已是脉冲语义）

融合建议：

1. `enter_score` 由结构证据 + jump 事件脉冲 + 签名加成组成；
2. `exit_score` 仅基于结构证据退化，不使用 `1-jump`；
3. `has_2dz_signature` 不再直接绑死 `DOUBLE_DZ` 标签。

---

## 5. 坐标系与先验约束

必须保证 dz 周期先验在稳定世界系下使用（通常 `target_frame=odom`）：

1. 若 `target_frame != odom`，降低/关闭周期签名权重；
2. 在 debug 日志中打印 `target_frame` 与签名状态；
3. 对非 odom 场景保留仅几何/概率路径，避免错误先验。

---

## 6. 参数精简方案

保留核心参数（推荐）：

1. `same_panel_yaw_gate`
2. `same_panel_xy_gate`
3. `reacquire_gap_dt_gate`
4. `reacquire_lost_frames_gate`
5. `z_cluster_ema_alpha`
6. `z_cluster_assign_gate`
7. `period_window`
8. `period_conf_threshold`
9. `force_rebind_bad_frames`

收敛策略：

1. 先固定 `period_window` 与 `reacquire` 门限；
2. 调 `same_panel_*` 保证重现后先“保守同板”；
3. 最后调 `period_conf_threshold` 控制 `has_2dz_signature` 触发频率。

---

## 7. 调试与可观测性

新增 debug 字段建议：

1. `event_type`
2. `is_reacquired/gap_dt/lost_frames`
3. `continuity_score`
4. `reacquire_same_panel_score`
5. `z_cluster_match_costs[3]`
6. `signature_score`
7. `has_2dz_signature`（窗口语义）

关键日志判定：

1. 出现 `REACQUIRE` 时，不应立即高频误切换；
2. 稳定连续段中，`z_cluster` 方差应收敛；
3. `signature_score` 在完整旋转周期后应明显上升。

---

## 8. 实施分期

### Phase 1（低风险）

1. 扩展 `BinderFrameInput` 事件字段；
2. 在 `OutpostTrackerV2/OutpostBinderBridge` 填充 `is_reacquired/gap_dt/lost_frames`；
3. 增加 debug 可观测字段。

### Phase 2（核心）

1. 引入 `Event Classifier`；
2. 引入 `ZLevelClusterBank`；
3. `OutpostTriLevelJumpDecoder` 增加 `REACQUIRE` 分支。

### Phase 3（证据升级）

1. 实现窗口 `2dz` 签名评分；
2. 将 `has_2dz_signature` 切换为窗口语义；
3. 接入 `EvidenceFuser` 的 `signature_score`。

### Phase 4（调参与回放验证）

1. 使用长 TEMP_LOST 视频回放；
2. 对比：切换正确率、误切换率、进入 STRUCTURED 成功率、重现恢复时间；
3. 固化默认参数。

---

## 9. 验收指标

1. `has_2dz_signature` 不再长期为 0（在有效旋转序列中可触发）。
2. `TEMP_LOST` 重现后 1~3 帧内错误切换率下降。
3. mode 进入 STRUCTURED 的触发不依赖单次偶然 jump。
4. 在静止中心+自旋视频中，ID 绑定抖动显著降低。

---

## 10. 风险与回滚

风险：

1. `REACQUIRE` 判别过强导致切换滞后；
2. `z_cluster` 初始化偏差导致早期误导；
3. 非 odom 场景误用周期先验。

缓解：

1. 提供开关：`enable_reacquire_event_decoder`、`enable_periodic_signature`；
2. 初期以日志模式运行（不驱动切换，仅输出评分）；
3. 保留现有 cost decoder 作为 fallback。

回滚：

1. 关闭新事件解码与签名开关；
2. 回到当前 `decode_from_cost + z_audit` 路径。

