# OutpostTrackerV3 审计与修复记录（2026-05-17）

## 1. 审计问题

- OutpostTrackerV3 未接入 `TrackerManager`，运行时无法被创建。
- 配置系统仅支持 `outpost.use_tracker_v2`，V3 参数未暴露。
- V3 在 `AMBIGUOUS` 阶段仅 predict-only，缺少收敛 warmup 机制。
- 周期/相位证据未进入提交前 gate，错误 panel 可能在短窗内通过几何 gate。

## 2. 已完成修复

### 2.1 运行接线

- `TrackerManager` 新增 Outpost V3 创建分支：
  - 优先级：`use_tracker_v3 > use_tracker_v2 > legacy`。

### 2.2 配置暴露

- `OutpostParameters` 增加：
  - `use_tracker_v3`
  - `outpost.v3.*` 全量参数（gate/selector/posterior/mode/prior/noise/warmup/phase_audit）。
- `gimbal_pipeline_node`：
  - 新增参数声明；
  - 新增参数读取与基本 clamp。
- `config/gimbal_pipeline.yaml`：
  - 新增 `outpost.use_tracker_v3`；
  - 新增 `outpost.v3` 配置块。

### 2.3 算法修复

- 新增 warmup 收敛阶段（单观测 3-panel 评分）：
  - 满足 `warmup_frames + min_settle_frames + margin/confidence` 后进入 structured。
- 新增 phase audit gate（trial 前硬门）：
  - 基于相邻帧 `dz` 与 `panel(z_offset)` 预测差值一致性校验。
  - 作为与 `posterior_sanity`、`reconstruction_error` 并列的提交门。

## 3. 启用方式

在 `config/gimbal_pipeline.yaml` 中设置：

```yaml
outpost:
  use_tracker_v3: true
  use_tracker_v2: false
```

并按需调整：

- `outpost.v3.warmup_*`
- `outpost.v3.phase_audit_*`
- `outpost.v3.mode_*`

## 4. 备注

- 当前保持“单观测假设空间”策略，不引入多观测组合。
- `phase_audit_confirm_frames` 已暴露，但当前实现为单帧硬门，后续可升级为多帧确认门。
