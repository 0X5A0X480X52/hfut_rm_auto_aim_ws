# Norm4 优化术语收敛与 Tracker 共址梳理

## 1. 术语收敛目标

本文件作为 `Norm4_tracker优化` 目录的统一术语约束，避免同一概念多名、同一模块重复写状态。

核心约束：

1. single armor 状态只由 `SingleArmorProxyManager` 维护。
2. 决策与执行分离：`BackendPlanner` 只产生命令，`BackendExecutor` 只执行命令。
3. `AMBIGUOUS` 模式默认读取 proxy snapshot 发布，不再重复更新 single armor filter。
4. 三类姿态后端统一收敛到 `PoseTrackerBackend` 包（位于 `max_entropy_tracker` 内部）。

## 2. 统一术语映射

旧术语到新术语：

```text
BackendCommandBuilder -> BackendPlanner
BackendManager        -> BackendExecutor
BackendCommand        -> BackendIntent（决策层）/ BackendExecutionPlan（执行层）
```

语义说明：

```text
BackendIntent:
  表达“本帧后端目标与动作意图”

BackendExecutionPlan:
  表达“本帧具体执行路径”
```

## 3. 固定模块职责

```text
Armor2DTracker:
  只做 2D 关联与生命周期，不读写 3D backend

SingleArmorProxyManager（Armor3DTrackerManager）:
  唯一 single armor filter update/reset 持有者
  输出 single armor snapshot + dynamics summary

Binding / Discriminator:
  只做身份与置信决策，不直接调用 backend filter

BackendPlanner:
  把决策翻译成 BackendIntent/BackendExecutionPlan

BackendExecutor:
  执行 structured/outpost 更新，或读取 proxy snapshot 发布
```

## 4. 三层 Tracker 家族共址组织（按当前项目结构）

在功能稳定前，不进行跨项目拆分；先在 `max_entropy_tracker` 内按子目录共址。

```text
include/max_entropy_tracker/
  tracking2d/                 # Armor2DTracker family
    armor_2d_types.hpp
    armor_2d_tracker.hpp
    iou_2d_tracker.hpp
    sort_2d_tracker_adapter.hpp

  pose_tracker_backend/       # 统一姿态后端包（single + structured + outpost）
    backend_intent.hpp
    backend_execution_plan.hpp
    backend_planner.hpp
    backend_executor.hpp
    single/
      ambiguous_single_tracker_adapter.hpp
      single_tracker_manager.hpp
      single_tracker_dynamics_summary.hpp
    robot/
      dual_radius_robot_tracker_adapter.hpp
      outpost_robot_tracker_adapter.hpp
      robot_tracker_manager.hpp
```

`src/max_entropy_tracker/` 保持同名镜像目录。

## 5. 与现有类型的对位关系

```text
Armor2DTracker:
  IoUArmor2DTracker / SortArmor2DTracker

Armor3DTracker:
  AmbiguousSingleArmorFilterAdapter（通过 adapter 方式纳入 PoseTrackerBackend::single）

RobotTracker:
  DualRadiusSpinUKF / OutpostSpinUKF（通过 adapter 方式纳入 PoseTrackerBackend::robot）
```

## 6. 迁移约束（Phase 1-7）

1. 只做“命名与语义收敛 + 目录共址”，不改滤波算法核心行为。
2. 若旧文件仍出现旧名，视为历史描述；以本文件映射为准。
3. 回归时优先验证：
- 无 single armor 重复 update
- AMBIGUOUS 输出来自 proxy snapshot
- structured/outpost 更新路径不触碰 proxy filter state
