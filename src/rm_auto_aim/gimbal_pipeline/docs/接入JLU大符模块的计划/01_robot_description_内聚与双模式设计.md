# robot_description 内聚与双模式设计

## 1. 当前问题

- 几何逻辑分散在 `fire_advice_engine`、`armor_selector`、`gimbal_pipeline_node` 调试代码中。
- 存在隐式 `yaw` 平面假设，导致 FULL_SE3 扩展困难。
- 同一语义（装甲板世界位置、法向、facing）存在重复实现。

## 2. 内聚目标

将以下能力收敛进 `robot_description`（或同级 geometry 子模块）：

1. 状态标准化
- `normalizeState`
- `syncFullStateFromLegacy`
- `syncLegacyStateFromFull`

2. 几何派生
- `calculateArmorWorldPositionsEigen/Points`
- `calculateArmorWorldNormal`（新增）
- `computeFacingCos`（新增）
- `sampleArmorGeometryAt(dt)`（可选新增）

3. 单目标语义
- `isSingleArmorRepresentation`
- `singleArmorPosition/Velocity/Yaw`
- `singleArmorNormal`（可选新增）

## 3. 双模式定义

新增模式枚举：

- `YAW_PLANE`
  - 使用 `center_position + yaw + armors_offset.position`。
  - 与当前行为一致。

- `FULL_SE3`
  - 使用 `center_pose.orientation` 变换 `armors_offset.position`。
  - 法向由 `center_pose.orientation * armors_offset.orientation` 计算。

## 4. 模式选择策略

默认规则：
- `robot_id in {"big_buff", "small_buff"}` -> `FULL_SE3`
- 其他 -> `YAW_PLANE`

可配置覆盖：
- `robot_description.default_projection_mode`
- `robot_description.full_se3_ids`
- `robot_description.full_se3_robot_types`

## 5. API 草案

建议新增：

- `ProjectionMode resolveProjectionMode(const TrackedRobot&)`
- `std::vector<Eigen::Vector3d> calculateArmorWorldPositionsEigen(..., ProjectionMode mode=Auto)`
- `Eigen::Vector3d calculateArmorWorldNormal(..., ProjectionMode mode=Auto)`
- `double computeFacingCos(const Eigen::Vector3d& center, const Eigen::Vector3d& armor, const Eigen::Vector3d& observer={0,0,0})`

## 6. 兼容策略

- 默认参数保持旧行为。
- 未提供 `full_state_valid` 或姿态非法时，自动降级到 `YAW_PLANE`。
- `armors_offset` 为空时沿用现有 fallback。
