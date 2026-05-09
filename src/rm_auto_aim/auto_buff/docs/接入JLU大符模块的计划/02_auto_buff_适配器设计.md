# auto_buff 适配器设计

## 1. 目标

将 `tmp/jlu_vision_26/src/auto_buff` 的算法能力接入 `gimbal_pipeline`，并改造为 ROS2 原生通信与构建体系。

补充约束：
- 不引入 `iceoryx_deps` 规则。
- 不整包迁移 `tools/common_defs`，仅移植必要算法与数据结构。

## 2. 适配器职责

建议新增：
- `include/gimbal_pipeline/adapters/buff_target_adapter.hpp`
- `src/adapters/buff_target_adapter.cpp`

职责：
1. 订阅 buff ROS2 输出（检测/跟踪结果）。
2. 转换为 `rm_interfaces::msg::TrackedRobot(s)`。
3. 提供缓存与时效判定。
4. 按模式开关决定是否参与合并。
5. 与 `SetMode`/目标白名单联动，控制大符/小符启停。

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

## 4. auto_buff ROS2 化拆分

Phase 1（Tracker 优先）：
1. 替换 `hardware/*listener` 为 ROS2 订阅实现。
2. 替换 `transform/tf_listener` + `fast_tf` 为 `tf2_ros`。
3. Tracker 输入对齐 `rm_interfaces` 消息。

Phase 2（Detector）：
1. 图像链路改为 `sensor_msgs/Image`。
2. 模式切换改 ROS2 服务或参数。
3. 输出映射到 Rune 相关 ROS2 消息（优先 `rm_interfaces/msg/RuneTarget`）。

## 5. 融合方式

在 `GimbalPipelineNode` 中：
1. 保留 `/armor_detector/armors` 主链路。
2. 新增外部目标订阅与缓存。
3. 在构建 `latest_tracked_robots_` 时合并 buff 目标。
4. 使用 `SetMode` 控制目标白名单。

## 6. 配置项

- `external_targets.enable`
- `external_targets.buff.enable`
- `external_targets.buff.topic`
- `external_targets.buff.timeout_s`
- `external_targets.allowed_ids_by_mode.*`
