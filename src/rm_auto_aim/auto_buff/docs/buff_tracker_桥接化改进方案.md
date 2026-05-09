# buff_tracker 桥接化改进方案（单节点内聚版）

日期：2026-05-09

## 1. 背景与问题

当前 `auto_buff` 已具备 ROS2 双节点链路：

1. `buff_detector_node`：图像 -> `RuneTarget/RuneTargetArray`
2. `buff_pose_estimator_node`：`RuneTarget` -> `TrackedRobot`

现状偏差：

- `buff_pose_estimator_node` 中包含了部分状态推断逻辑（roll/掩码估计），与 `buff_tracker` 因子图状态语义存在重复。
- `buff_tracker` 的“真实状态”（`BuffState`、`inactivated_flag[5]`、`TrackState`）尚未在当前主链路中作为唯一语义源使用。

目标偏差：

- 按文档演进方向，`buff_pose_estimator_node` 应收敛为“包装/映射层”，核心状态估计应来自 `buff_tracker`。
- 同时避免新增独立 bridge 进程导致的额外时序与维护开销。

---

## 2. 改进目标

将链路改造为单节点内聚模式：

1. `buff_pose_estimator_node` 内部直接接入 `buff_tracker` 状态
2. 在同一节点内完成状态桥接与 `TrackedRobot` 映射发布
3. 保留现有 `RuneTarget` 推断路径作为可配置 fallback

关键收益：

- 语义唯一源（Single Source of Truth）：`inactivated_flag[5]` 不再二次估计。
- 降低同步风险：状态与掩码在同一节点内一次生成。
- 降低系统复杂度：不新增无意义桥接节点与额外 topic 调度。

---

## 3. 目标架构

目标数据流：

1. `buff_detector_node` 发布 `RuneTarget/RuneTargetArray`
2. `buff_pose_estimator_node`（单节点）：
   - 接入 `buff_tracker` 状态估计结果（主）
   - 可选消费 `RuneTarget`（备）
   - 节点内桥接为统一状态表示
   - 发布 `TrackedRobot`（结构化 5 叶 + 掩码）
3. `gimbal_pipeline::BuffTargetAdapter` 订阅 `TrackedRobot`

说明：

- 不新增 `buff_tracker_state_bridge_node`。
- 如需观测桥接状态，可在 `buff_pose_estimator_node` 内增加“可选调试发布”。

---

## 4. 消息设计

### 4.1 `BuffTrackerState.msg`（建议，节点内/跨节点两用）

字段建议：

- `std_msgs/Header header`
- `string buff_id` (`big_buff` | `small_buff`)
- `uint8 track_state` (`LOST` | `TEMP_LOST` | `TRACKING`)
- `geometry_msgs/Point center_position`
- `float64 center_roll`
- `float64 center_vroll`
- `bool[5] inactivated_flag`
- `float64 radius_m`
- `float64 confidence`
- `uint32 state_seq`

说明：

- `inactivated_flag` 语义直接继承 `buff_tracker` 内部定义，不在 `buff_pose_estimator_node` 中重新解释。
- `state_seq` 用于排查时序跳变。
- 默认不强制外发 `BuffTrackerState` topic；该消息可先作为节点内统一结构与调试产物。

### 4.2 现有消息沿用

- `RuneTarget.msg`：保留扩展字段（`blade_type/blade_slot_hint/confidence/track_id`），作为 fallback 输入。
- `TrackedRobot.msg`：沿用 `engageable_mask/engageable_count` 字段。

---

## 5. 节点职责重构（单节点）

### 5.1 `buff_pose_estimator_node` 收敛为包装层 + 内部桥接层

主路径：

1. 节点内部直接获取 `buff_tracker` 状态（`SmallBuffTarget/BigBuffTarget` 输出）
2. 在节点内填充统一 `BuffTrackerState` 结构（可选调试外发）
3. 基于统一状态直接构建 `TrackedRobot`：
   - `center_pose`：`position + quat(roll,0,0)`
   - `armors_offset`：按 `N=5` 与半径生成
   - `engageable_mask`：由 `inactivated_flag` 一次映射

备用路径：

- 当 `buff_tracker` 状态不可用或失效时，启用 `RuneTarget` 推断路径（可配置开关）。

约束：

- 不新增独立 bridge 进程，避免额外 topic 同步与调度开销。
- 不发布 `AimCommand`，不耦合控制链。
- 发布频率与状态更新时间一致（建议 30~100Hz 可配）。

---

## 6. 分阶段实施

### P1：单节点桥接骨架落地（低风险）

1. 新增 `BuffTrackerState.msg`
2. `buff_pose_estimator_node` 增加 `buff_tracker` 内部状态接入口
3. 节点内统一状态结构打通（可选调试外发 topic）

验收：

- 单节点主链路可稳定生成统一状态并发布 `TrackedRobot`。

### P2：pose_estimator 主路径切换

1. 发布 `TrackedRobot` 优先使用节点内 bridge 状态
2. 保留 `RuneTarget` fallback（默认开启）

验收：

- `TrackedRobot` 中 `engageable_mask` 与 `inactivated_flag` 一致。

### P3：旧逻辑降权与收敛

1. 默认关闭 `RuneTarget` 掩码估计（仅保留应急）
2. 调整文档、launch 默认参数
3. 完成回归与稳定性验证

---

## 7. 兼容与回滚

兼容策略：

1. 若 `buff_tracker` 状态不可用，自动回退 `RuneTarget` 推断链路。
2. 对 `gimbal_pipeline` 下游保持 `TrackedRobot` 接口不变。

回滚开关建议：

- `use_internal_tracker_bridge`（bool）
- `tracker_state_timeout_s`（double）
- `enable_rune_fallback`（bool）

---

## 8. 验收标准

1. `buff_pose_estimator_node` 可稳定获取并桥接 `buff_tracker` 状态（`TRACKING` 下字段连续）。
2. `buff_pose_estimator_node` 在主路径下不再自行估计掩码。
3. `TrackedRobot` 保持 `num_armors=5` 且 `engageable_mask` 正确。
4. `gimbal_pipeline` 无需改消息订阅类型即可消费升级结果。
5. 关闭主路径后 fallback 可恢复当前行为。

---

## 9. 风险与注意事项

1. `buff_tracker` 旧依赖（iceoryx/quill/basic/hardware）尚未 ROS2 化完全，需要在 `buff_pose_estimator_node` 内做好最小依赖隔离。
2. `inactivated_flag` 的“可击打”语义需冻结为统一约定，避免上下游反向解释。
3. 若后续引入跨帧稳定 `blade_slot`，应由 `buff_tracker` 输出，不在包装层推断。
4. 若开启 `BuffTrackerState` 调试外发，需标注为诊断 topic，避免被业务链路误依赖。

---

## 10. 建议实现顺序

1. 先做 `BuffTrackerState.msg + buff_pose_estimator_node` 内桥接结构。
2. 再切 `buff_pose_estimator_node` 主路径。
3. 最后做默认参数切换与文档收敛。
