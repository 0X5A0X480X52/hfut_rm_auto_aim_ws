# Norm4 优化实现详细规划（Phase 1 - Phase 7）

## 1. 文档范围与目标

本文档给出 `Norm4 tracker` 优化在 `max_entropy_tracker` 体系内的落地实施细化方案，覆盖：

- Phase 1：消息与 ObservationData 打通
- Phase 2：内置 IoU 2D tracker
- Phase 3：SingleArmorProxyManager（含动力学摘要）
- Phase 4：Norm4 相位记忆 + 0101 抑制（首版）
- Phase 5：通用 EvidenceFrame
- Phase 6：Norm4-only Binding soft fusion
- Phase 7：Norm4 通用 pipeline 试点

不包含 Phase 8（SORT/KF 2D tracker 增强）的实现细节。

## 2. 总体实施原则

1. 所有新能力必须“可开关、可回退、可观测”。
2. 2D/Proxy 证据在 Phase 1-7 期间仅作为 soft evidence，不作为硬替代。
3. 不改 Adaptive/Outpost 主链路行为。
4. 优先复用现有模块：`DualRadiusSpinUKF`、`AmbiguousSingleArmorFilterAdapter`、`BinderPipeline`、`ModeFSM`。
5. 每个 Phase 必须具备最小验收闭环：编译通过 + 单元测试/回放验证 + debug 可见性。
6. `Armor2DTracker / Armor3DTracker / RobotTracker` 先在 `max_entropy_tracker` 内共址组织（含 manager + filter）；其中 3D 后端统一收敛到 `PoseTrackerBackend` 包，功能稳定后再做项目结构清理。

## 3. 里程碑与交付物

### 3.1 里程碑定义

- M1（Phase 1 完成）：3D 流程零退化，2D 字段可选透传。
- M2（Phase 2 完成）：2D track id 稳定输出，且不影响旧链路。
- M3（Phase 3 完成）：每个 Track2DId 拥有 single armor proxy 与动力学摘要。
- M4（Phase 4 完成）：可检测并抑制 `0101` 假周期。
- M5（Phase 5 完成）：Norm4 可消费统一 `EvidenceFrame`。
- M6（Phase 6 完成）：Norm4 绑定支持 2D/proxy soft fusion。
- M7（Phase 7 完成）：Norm4 迁移到通用 serial pipeline，保留行为可回退。

### 3.2 统一开关建议

```yaml
tracker:
  norm4_v2:
    enable_common_pipeline: false
    enable_phase_memory: true
    enable_kinematic_anti_pingpong: true
    anti_pingpong:
      min_consistent_frames_to_commit: 3
      jerk_gate: 1.5
      yaw_rate_jump_gate: 2.0
      velocity_dir_cos_min: 0.2
```

## 4. Phase 1：消息与 ObservationData 打通

### 4.1 目标

把图像域观测（bbox/corners/confidence）安全追加到现有消息与内部观测结构，保持旧调用全兼容。

### 4.2 代码改动清单

1. `rm_interfaces/msg/Armor.msg`
- append-only 增加：`detection_confidence`、`has_image_geometry`、`bbox_xywh`、`image_corners` 等字段。

2. `gimbal_pipeline` 消息转换链路
- `msg_converter`：将新增字段复制到 `ObservationData::image`。
- `TFHandler` 或同层处理逻辑：确保 timestamp 与坐标处理不受新增字段影响。

3. `max_entropy_tracker` 数据结构
- `ObservationData` 新增：`std::optional<ImageObservation2D> image`、`std::optional<int> track2d_id`。

### 4.3 验收标准

1. 不带新字段的旧 bag 可直接回放。
2. 全链路编译通过，`Adaptive/Outpost/Norm4` 输出与基线误差在容差内。
3. debug 中可见 `image.valid` 统计。

### 4.4 回滚策略

- 仅保留追加字段，不启用读取逻辑。
- 配置关闭 `enable_2d_tracker` 后行为应与基线一致。

## 5. Phase 2：max_entropy_tracker 内置 IoU 2D tracker

### 5.1 目标

实现轻量 `IoUArmor2DTracker`，输出 `Armor2DTrackEvidence`。

### 5.2 新增文件建议

- `include/max_entropy_tracker/tracking2d/armor_2d_types.hpp`
- `include/max_entropy_tracker/tracking2d/armor_2d_tracker.hpp`
- `include/max_entropy_tracker/tracking2d/iou_2d_tracker.hpp`
- `src/max_entropy_tracker/tracking2d/iou_2d_tracker.cpp`

### 5.3 核心接口

```cpp
class IArmor2DTracker {
 public:
  virtual ~IArmor2DTracker() = default;
  virtual std::vector<Armor2DTrackEvidence> update(
      const std::vector<Armor2DDetection>& detections,
      double timestamp) = 0;
  virtual void reset() = 0;
};
```

### 5.4 实现要点

1. 预测：`bbox += velocity * dt`。
2. 匹配：IOU + center distance gate + Hungarian。
3. 生命周期：`hits/age/missed/confirmed`。
4. 输出：`track_id`、`association_quality`、速度、状态字段。

### 5.5 验收标准

1. 连续视野中 track id 抖动率低于基线阈值。
2. 遮挡短丢后可重连。
3. tracker 输出对旧 3D 路径零强耦合。

## 6. Phase 3：SingleArmorProxyManager（含动力学摘要）

### 6.1 目标

为每个活跃 `Track2DId` 维护一个 `AmbiguousSingleArmorFilterAdapter`，并产出动力学摘要。

### 6.2 新增文件建议

- `include/max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.hpp`
- `src/max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.cpp`
- `include/max_entropy_tracker/pose_tracker_backend/single/ambiguous_single_tracker_adapter.hpp`
- `src/max_entropy_tracker/pose_tracker_backend/single/ambiguous_single_tracker_adapter.cpp`

### 6.3 关键数据

- `SingleArmorTrackEvidence`：`armor_pos/vel/yaw/yaw_rate/z_stats`。
- 动力学摘要：
  - `velocity_var_window`
  - `acc_norm_window`（速度差分估计）
  - `yaw_rate_continuity_score`

### 6.4 更新逻辑

1. 读取 `track2d_id -> observation_index` 映射。
2. `predict(dt)`。
3. 有 3D 观测则 `update`。
4. 丢失帧按 `keep_lost_frames` 保活。
5. 输出证据供后续 Phase 使用。

### 6.5 验收标准

1. ambiguous 输出可直接使用 proxy 结果。
2. 动力学摘要曲线可在 debug 查看。
3. 关闭开关后不影响旧行为。

## 7. Phase 4：Norm4 相位记忆 + 0101 抑制（首版）

### 7.1 目标

引入“序列一致性 + 动力学一致性”联合判别，抑制 `0101` / `1212` 假周期。

### 7.2 新增文件建议

- `include/max_entropy_tracker/trackers/norm4_v2/norm4_phase_sequence_memory.hpp`
- `src/max_entropy_tracker/trackers/norm4_v2/norm4_phase_sequence_memory.cpp`

### 7.3 判定规则

1. `ping_pong_pattern`：窗口内出现 `A B A B`。
2. `opposite_same_height_jump`：`0<->2` 或 `1<->3`。
3. `kinematic_inconsistency`：
- 回跳时速度方向突变（`cos(theta)` 低于阈值）
- jerk 超过阈值
- yaw_rate 不连续

### 7.4 行为策略

1. 命中风险时切换进入 `pending`。
2. 连续 `N` 帧动力学一致才 `commit`。
3. 失败则 `hold bound panel` 或 `single_only`。

### 7.5 验收标准

1. 典型 `0101` bag 中假周期次数明显下降。
2. 不显著增加真实切换延迟（需设阈值）。
3. debug 输出包含：触发原因、抑制时长、commit 次数。

## 8. Phase 5：通用 EvidenceFrame

### 8.1 目标

统一 `Observation + 2D + proxy + 几何/关系` 为 `ArmorEvidenceFrame`，供 Norm4 先行使用。

### 8.2 新增文件建议

- `include/max_entropy_tracker/evidence/evidence_frame.hpp`
- `include/max_entropy_tracker/evidence/evidence_builder.hpp`
- `src/max_entropy_tracker/evidence/evidence_builder.cpp`

### 8.3 构建流程

```text
ObservationData[]
-> Armor2DDetection[]
-> IArmor2DTracker.update()
-> SingleArmorProxyManager.update()
-> geometry/height/relation evidence
-> ArmorEvidenceFrame
```

### 8.4 验收标准

1. `ArmorEvidenceFrame` 字段完整率可统计。
2. Norm4 在不启用新决策时，行为与旧链路一致。
3. build 与运行时开销可控（目标耗时阈值需记录）。

## 9. Phase 6：Norm4-only Binding soft fusion

### 9.1 目标

Norm4 的绑定阶段使用 2D/proxy 软融合，但不改变 binder 的最终职责边界。

### 9.2 改动点

1. `Norm4PanelHypothesisEvaluator`：增加 `track_continuity_cost`、`phase_transition_cost`。
2. `Norm4PanelIdentityResolver`：融合几何/序列/动力学评分。
3. `BindingStage` 输入增加：`phase_confidence`、`ping_pong_risk`。

### 9.3 推荐评分框架

```text
phase_score =
  w_seq * sequence_consistency
+ w_geo * yaw_xy_consistency
+ w_dyn * kinematic_consistency
```

### 9.4 验收标准

1. same-track 场景 panel 跳变减少。
2. `0101 + 动力学冲突` 场景进入低置信切换路径。
3. 关闭融合权重后可回到 Phase 5 行为。

## 10. Phase 7：Norm4 通用 pipeline 试点

### 10.1 目标

将 Norm4 内部改造成通用 serial pipeline，保留 BaseTracker 外壳与回退能力。

### 10.2 新增/改造文件建议

- `include/max_entropy_tracker/pose_tracker_backend/backend_intent.hpp`
- `include/max_entropy_tracker/pose_tracker_backend/backend_execution_plan.hpp`
- `include/max_entropy_tracker/pose_tracker_backend/backend_planner.hpp`
- `include/max_entropy_tracker/pose_tracker_backend/backend_executor.hpp`
- `include/max_entropy_tracker/pipeline/serial_tracker_pipeline.hpp`
- `include/max_entropy_tracker/pipeline/debug_trace.hpp`
- `trackers/norm4_v2/*` 迁移到通用 Evidence/Command 调用路径

### 10.3 目标调用链

```text
ObservationData[]
-> EvidenceBuilder
-> Norm4PanelIdentityResolver
-> BindingStage
-> ModeDecider
-> BackendPlanner
-> BackendExecutor
-> OutputAdapter
```

### 10.4 兼容要求

1. `Norm4ArmorTracker` 仍继承 `BaseTracker`。
2. 对外接口不变。
3. `enable_common_pipeline=false` 时可切回旧实现。

### 10.5 验收标准

1. 编译与集成测试通过。
2. 与基线 bag 对比：跟踪连续性提升，错误跳变下降。
3. CPU 开销在预算内，且无实时性回退。

## 11. 测试计划（Phase 1-7 全局）

### 11.1 测试分层

1. 单元测试
- 2D tracker 匹配与生命周期
- phase memory 检测（ABAB、opposite jump）
- kinematic gate 判定

2. 集成测试
- Norm4 update/predict 主流程
- backend intent/execution-plan 翻译正确性

3. 回放测试
- 匀速旋转
- 高机动转向
- 遮挡重获
- 双板可见/单板可见切换

### 11.2 关键指标

- panel 切换稳定性
- `0101` 假周期事件数
- reacquire 成功率
- 输出 jitter
- 帧耗时 p50/p95/p99

## 12. 风险与缓解

1. 风险：动力学门控过强导致真实切换迟滞。
- 缓解：引入 pending 超时与多阈值档位。

2. 风险：新增证据导致调参复杂。
- 缓解：全部权重可配置，提供 A/B 配置模板。

3. 风险：实时性压力上升。
- 缓解：Phase 4 前后各做 profiling，保留简化路径。

4. 风险：行为偏移难定位。
- 缓解：debug_trace 必须输出每一步证据与决策理由。

## 13. 交付顺序建议（两周节奏示例）

1. 第 1-2 天：Phase 1。
2. 第 3-4 天：Phase 2。
3. 第 5-6 天：Phase 3。
4. 第 7-8 天：Phase 4。
5. 第 9-10 天：Phase 5。
6. 第 11-12 天：Phase 6。
7. 第 13-14 天：Phase 7 + 回归测试与调参。

## 14. 完成定义（DoD）

1. Phase 1-7 对应代码与配置全部落地。
2. 文档中的验收项均有测试记录。
3. 保留一键回退开关。
4. 关键 debug 字段可在日志中直接追踪。
