# auto_buff 适配器设计

## 1. 目标

在不修改 `tmp/jlu_vision_26/src/auto_buff` 逻辑的前提下，将其输出接入 `gimbal_pipeline`。

## 2. 适配器职责

建议新增：
- `include/gimbal_pipeline/adapters/buff_target_adapter.hpp`
- `src/adapters/buff_target_adapter.cpp`

职责：
1. 订阅 buff 输出（检测/跟踪结果）。
2. 转换为 `rm_interfaces::msg::TrackedRobot(s)`。
3. 提供缓存与时效判定。
4. 按模式开关决定是否参与合并。

## 3. 输出语义映射

推荐先使用单目标语义（低风险）：
- `representation_mode = REP_AMBIGUOUS_SINGLE_ARMOR`
- `num_armors = 1`
- `center_pose.position = 当前选中击打点`
- `center_twist.linear = 击打点速度`
- `armors_offset = [zero offset]`
- `robot_id = "big_buff" | "small_buff"`

备注：
- 即使是单目标语义，也可启用 `FULL_SE3`，为后续扩展保留姿态信息。

## 4. 融合方式

在 `GimbalPipelineNode` 中：
1. 保留 `/armor_detector/armors` 主链路。
2. 新增外部目标订阅与缓存。
3. 在构建 `latest_tracked_robots_` 时合并 buff 目标。
4. 使用 `SetMode` 控制目标白名单。

## 5. 配置项

- `external_targets.enable`
- `external_targets.buff.enable`
- `external_targets.buff.topic`
- `external_targets.buff.timeout_s`
- `external_targets.allowed_ids_by_mode.*`
