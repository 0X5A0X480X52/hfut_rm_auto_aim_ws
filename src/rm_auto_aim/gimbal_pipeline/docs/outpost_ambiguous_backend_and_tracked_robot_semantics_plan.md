# OutpostAmbiguousBackend 替换与 TrackedRobot 语义更新详细方案

## 1. 背景与问题

当前 `OutpostTrackerV2` 在 `AMBIGUOUS` 模式下通过 `OutpostAmbiguousBackend` 维护单装甲板观测，但对外仍主要沿用“完整机器人中心态”语义。该设计在工程上可运行，但存在三个核心问题：

1. **滤波语义不一致**  
   `AMBIGUOUS` 模式本质是“单装甲板状态估计”，却被强制映射为完整机器人中心状态，导致含义混杂。

2. **后端复用性不足**  
   当前 `OutpostAmbiguousBackend` 绑定了 outpost 几何（半径、三层高度、panel角），不利于后续复用到 `AdaptiveArmorTracker` 的 ambiguous 后端。

3. **控制链路缺少显式分叉**  
   `gimbal_controller` 默认按“中心+offset重建多装甲板”工作，缺少对“单装甲板表示”的显式处理路径。

---

## 2. 目标与非目标

### 2.1 目标

1. 将 `OutpostAmbiguousBackend` 替换为“**通用单装甲板滤波后端 + 机器人几何适配层**”。
2. 在 `TrackedRobot` 语义中明确区分：
   - `STRUCTURED_ROBOT`（完整机器人）
   - `AMBIGUOUS_SINGLE_ARMOR`（单装甲板降级表示）
3. 在 `gimbal_controller` 内提供单装甲板专用处理链路，避免错误回推完整机器人结构。
4. 保持现有发布/消费接口尽量兼容，支持灰度切换与回退。

### 2.2 非目标

1. 本阶段不强制修改 `rm_interfaces/msg/TrackedRobot.msg` 字段定义。
2. 本阶段不重写 `OutpostStructuredBackend` 与 `OutpostSpinUKF` 主链路。
3. 本阶段不改变选板策略与开火阈值策略本身。

---

## 3. 总体设计原则

1. **robot_type 与表示模式解耦**  
   `robot_type` 仅描述目标类别（OUTPOST_3 / STANDARD_4 等），不描述当前估计是否完整。

2. **语义优先于参数拼凑**  
   ambiguous 下优先表达“当前可置信的单装甲板状态”，而不是强行补齐不可观的完整机器人结构。

3. **接口最小侵入**  
   对上层仍提供 `BaseTracker` 兼容接口；新增语义通过 `robot_description` 统一封装。

4. **渐进式迁移**  
   先完成内部语义和链路分叉，再评估是否需要升级 `rm_interfaces`。

---

## 4. 架构总览

### 4.1 目标调用链

```text
armorsCallback
  -> TrackerManager
    -> OutpostTrackerV2
      -> AMBIGUOUS backend (single-armor IMM adapter)
      -> OutputAdapter / RobotDescriptionBuilder
        -> TrackedRobot(语义化表示)
          -> gimbal_controller
            -> ArmorPositionCalculator(按表示模式分叉)
              -> ArmorSelector / FireAdviceEngine
```

### 4.2 模块分工

1. **滤波层（state estimation）**：只维护单装甲板运动状态，不做完整机器人结构拟合。
2. **语义层（robot_description）**：定义并输出当前 `TrackedRobot` 的表示模式。
3. **控制层（gimbal_controller）**：根据表示模式选择“多装甲板/单装甲板”目标生成路径。

---

## 5. OutpostAmbiguousBackend 替换方案（细化）

## 5.1 现状

现有实现：
- `OutpostAmbiguousBackend` 内部使用 `OutpostAmbiguousKF`
- 通过 `panel_id + radius + z_offset` 将单板状态回推为中心态
- `snapshot()` 输出中心态为主

主要问题：
- 后端强耦合 outpost 三板几何；
- 与“通用 ambiguous 单板后端”目标冲突。

## 5.2 目标方案

引入两层结构：

1. **通用滤波核心（可放在 `src/kalmanFilters`）**  
   `SingleArmorIMMTracker`
   - XY: IMM(CV/CA/CS/CTRV)
   - Z: 1D KF（CA 或 CV）
   - Yaw: 独立 1D KF（CV/CA + unwrap）

2. **gimbal_pipeline 适配层**
   `AmbiguousSingleArmorFilterAdapter`
   - 提供与当前 `outpost_ambiguous_kf.hpp` 等价接口
   - 对接 `ObservationData` 和 `UnifiedConfig`
   - 输出统一单板状态：`armor_position/armor_velocity/armor_yaw/armor_yaw_rate`

## 5.3 对 OutpostAmbiguousBackend 的改造

`OutpostAmbiguousBackend` 保留为 outpost 业务适配器，但不再承载滤波算法本体：

1. 内部成员由 `OutpostAmbiguousKF` 替换为 `AmbiguousSingleArmorFilterAdapter`。
2. `refresh_center_snapshot()` 保留（供结构模式切换与兼容接口使用），但新增“单板真值缓存”：
   - `armor_pos`
   - `armor_vel`
   - `armor_yaw`
   - `armor_yaw_rate`
3. `snapshot()` 仍返回现有 `BackendStateSnapshot`（兼容），但将新增扩展快照接口（见 5.4）。

## 5.4 建议新增后端扩展状态

为避免中心/单板语义混用，建议在 outpost_v2 内新增扩展结构：

```cpp
struct AmbiguousArmorSnapshot {
  Eigen::Vector3d armor_pos;
  Eigen::Vector3d armor_vel;
  double armor_yaw;
  double armor_yaw_rate;
  int panel_id;
  double confidence;
};
```

并在 `OutpostAmbiguousBackend` 增加：
- `const AmbiguousArmorSnapshot & ambiguous_snapshot() const;`

`OutpostTrackerV2` 在 ambiguous 下优先使用该快照填充发布语义。

## 5.5 文件级改动建议（后端部分）

### A. kalmanFilters（新增）

1. `src/kalmanFilters/filters/combined_models/include/combined_models/SingleArmorIMMTracker.h`
2. `src/kalmanFilters/filters/combined_models/src/SingleArmorIMMTracker.cpp`

### B. gimbal_pipeline（新增/修改）

1. 新增  
   `include/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp`  
   `src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp`

2. 修改  
   `include/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp`  
   `src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp`

3. 可选兼容包装  
   `include/max_entropy_tracker/filters/outpost_ambiguous_kf.hpp` 保留为 thin wrapper（避免一次性改动过大）。

---

## 6. TrackedRobot 语义更新方案（细化）

## 6.1 关键语义结论

在 ambiguous 下，`TrackedRobot` 发布的是“**单装甲板降级表示**”，而非“完整机器人状态估计”。

即：
- 目标类别仍是 `OUTPOST_3`（或对应 robot_type）；
- 当前表示模式是 `AMBIGUOUS_SINGLE_ARMOR`。

## 6.2 阶段化接口策略

### Phase A（推荐先落地，无消息变更）

不修改 `rm_interfaces/msg/TrackedRobot.msg`，通过约定表达：

1. `num_armors = 1`
2. `armors_offset.size() == 1`，且 `offset=(0,0,0)`（或与当前装甲板局部定义一致）
3. `center_position / center_velocity / yaw / yaw_velocity` 解释为**当前单装甲板状态**
4. `confidence` 叠加 single-mode 缩放

并在 `robot_description` 内提供统一判别函数，避免外部散落硬编码。

### Phase B（可选，接口增强）

在 `rm_interfaces::msg::TrackedRobot` 增加：
- `uint8 representation_mode`
  - `REP_STRUCTURED_ROBOT = 0`
  - `REP_AMBIGUOUS_SINGLE_ARMOR = 1`

用于跨节点显式语义传递。

## 6.3 robot_description 语义层扩展

建议在 `TrackedRobotUsage` 中新增：

1. `enum class RepresentationMode`
2. `static RepresentationMode inferRepresentationMode(const rm_interfaces::msg::TrackedRobot&)`
3. `static bool isSingleArmorRepresentation(const rm_interfaces::msg::TrackedRobot&)`
4. `static Eigen::Vector3d singleArmorPosition(...)`
5. `static Eigen::Vector3d singleArmorVelocity(...)`
6. `static double singleArmorYaw(...)`

该层作为 gimbal_controller 唯一入口，避免策略层直接猜语义。

## 6.4 Builder 规则调整

`FixedProfileTrackedRobotBuilder` 在 outpost ambiguous 下应遵循：

1. 若 `tracker.is_ambiguous_single_mode()==true`：
   - `num_armors=1`
   - `armors_offset` 只发布当前观测板或零偏移单板
   - `center_*` 填单板状态（来自 `publish_*` 或 ambiguous snapshot）
2. 若 structured：
   - 保持当前完整机器人发布逻辑。

---

## 7. gimbal_controller 单装甲板链路方案

## 7.1 现状风险

当前控制侧会默认将 `TrackedRobot` 解释为“中心+offset可重建多板”。若 ambiguous 输入仍走该逻辑，会引入：

1. 不可观参数被误用；
2. 选板与开火候选构造出现语义偏差。

## 7.2 建议分叉点

在 `ArmorPositionCalculator` 统一分叉：

1. `isSingleArmorRepresentation(robot) == true`
   - `calculate()` 返回单元素向量：`{single_armor_position}`
   - `calculatePredicted(dt)` 返回单元素预测位置
2. 否则走现有 structured 路径（center + offsets）。

这样 `ArmorSelector`、`CurrentPositionStrategy`、`PredictedPositionStrategy`、`FireAdviceEngine` 均可复用现有逻辑（候选集合退化为 1 个）。

## 7.3 FireAdviceEngine 一致性

`FireAdviceEngine` 已按候选列表处理，单板输入时天然退化为单候选。需要确保：

1. 面向过滤（facing filter）在单板下允许配置关闭或弱化；
2. 不再依赖多板候选做“最优板比较”。

---

## 8. 配置与开关设计

建议新增配置项（`UnifiedConfig::OutpostParameters`）：

1. `ambiguous_publish_single_armor_semantics`（bool，默认 `true`）
2. `ambiguous_single_armor_zero_offset`（bool，默认 `true`）
3. `ambiguous_backend_use_imm_adapter`（bool，默认 `false`，灰度开关）

并在 `gimbal_pipeline.yaml` 暴露对应参数。

---

## 9. 迁移步骤（建议执行顺序）

## Phase 1：后端替换准备

1. 引入 `AmbiguousSingleArmorFilterAdapter`（内部可先包现有 `OutpostAmbiguousKF`）。
2. 改 `OutpostAmbiguousBackend` 使用 adapter，不改外部接口。
3. 补齐日志：输出单板状态与中心回推状态的差异诊断。

## Phase 2：TrackedRobot 语义收敛

1. `FixedProfileTrackedRobotBuilder` ambiguous 分支按单板语义发布。
2. `TrackedRobotUsage` 增加表示模式推断函数。
3. `ArmorPositionCalculator` 增加 single-armor 分叉逻辑。

## Phase 3：IMM 核心接入

1. `AmbiguousSingleArmorFilterAdapter` 后端切换到 `SingleArmorIMMTracker`。
2. 开启 `ambiguous_backend_use_imm_adapter=true` 灰度验证。
3. 对比回放日志：中心跳动、yaw连续性、控制稳定性。

## Phase 4：可选消息增强

1. 评估跨节点是否需要显式 `representation_mode` 字段；
2. 若需要，再升级 `rm_interfaces` 并完成上下游适配。

---

## 10. 测试与验收

## 10.1 单元测试

1. `AmbiguousSingleArmorFilterAdapter`  
   - yaw unwrap 连续性  
   - dt 变化鲁棒性  
   - 低速/静止稳定性

2. `TrackedRobotUsage` 表示模式推断  
   - structured / ambiguous 判别正确  
   - 单板 getter 输出一致性

3. `ArmorPositionCalculator`  
   - single 模式输出 1 个候选  
   - structured 模式输出 N 个候选

## 10.2 集成测试（回放）

场景：
1. outpost 原地旋转、中心静止（单观测为主）
2. outpost 双观测出现/消失切换
3. 大 yaw_rate 与低 yaw_rate 切换

观测指标：
1. ambiguous 下发布点抖动（RMS）
2. 结构模式切换次数与抖动
3. 选板连续性与开火建议稳定性

## 10.3 回归检查

1. 非 outpost 目标（标准 4 板）行为不变
2. `outpost.use_tracker_v2=false` 旧链路不受影响
3. 现有配置默认值下系统可编译可运行

---

## 11. 风险与回退

### 11.1 主要风险

1. ambiguous 语义切换后，上游/下游若仍按“中心态”理解，可能引入偏差。
2. IMM 接入初期参数未调稳，可能短时比当前 KF 更抖。
3. 日志字段语义变化导致离线分析脚本需要同步更新。

### 11.2 对策

1. 保留开关：`ambiguous_backend_use_imm_adapter`、`ambiguous_publish_single_armor_semantics`。
2. 在过渡期双写关键调试字段（单板态 + 回推中心态）。
3. 回退路径明确：随时切回当前 `OutpostAmbiguousKF` + 旧发布语义。

---

## 12. 实施清单（文件级）

## 12.1 计划新增

1. `docs/outpost_ambiguous_backend_and_tracked_robot_semantics_plan.md`（本文档）
2. `include/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp`
3. `src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp`
4. （可选）`src/kalmanFilters/filters/combined_models/include/combined_models/SingleArmorIMMTracker.h`
5. （可选）`src/kalmanFilters/filters/combined_models/src/SingleArmorIMMTracker.cpp`

## 12.2 计划修改

1. `include/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp`
2. `src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp`
3. `src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp`
4. `include/gimbal_pipeline/common/robot_description/robot_description_facade.hpp`
5. `src/common/robot_description/tracked_robot_usage.cpp`
6. `src/gimbal_controller/armor_position_calculator.cpp`
7. `include/max_entropy_tracker/core/config.hpp`
8. `src/gimbal_pipeline_node.cpp`（参数声明与读取）

---

## 13. 结论

该方案将 ambiguous 从“隐式退化行为”提升为“显式语义表示”，并通过“通用单装甲板后端 + robot_description 语义层 + controller 分叉”形成清晰调用链：

1. 滤波器只做它能观测到的事（单板状态）；
2. 发布层明确当前表示模式；
3. 控制层按语义选择正确几何链路。

这能显著降低 outpost ambiguous 场景下的语义误差与后续扩展成本，并为 `adaptive_armor_tracker` 复用同一 ambiguous 后端打下基础。

