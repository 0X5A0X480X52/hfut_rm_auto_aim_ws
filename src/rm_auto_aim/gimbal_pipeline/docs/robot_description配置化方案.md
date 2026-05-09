# robot_description 配置化方案

## 1. 目标

将 `robot_description` 从“硬编码 builder”升级为“配置驱动 builder”，统一提供：

1. 机器人可击打模块（坐标系内几何定义）。
2. 参数可观测性声明（已知/未知）。
3. 模块有效打击面积（给选板与 fire_advice 概率模型直接使用）。

约束：

- 前期配置源：YAML。
- 后续可平滑扩展：URDF。
- 旧链路保持可用（默认 fallback 到现有 fixed_profile builder）。

---

## 2. 当前现状与改造切入点

当前代码具备可行基础：

- `RobotDescriptionFacade + TrackedRobotBuilderRegistry`：天然可注册新 builder。
- `TrackedRobotUsage`：下游几何计算统一入口（position/normal/facing/projection）。
- `gimbal_controller/fire_advice` 已有独立概率模块（`probability_engine`），可增量接入 profile 参数。

主要缺口：

- `big_buff/small_buff` 目前为单板 profile（`num_armors=1`）。
- 无配置加载器与 schema 校验。
- 无“模块有效面积/参数可观测性”查询 API。

---

## 3. 配置模型（YAML v1）

建议新增：`src/rm_auto_aim/gimbal_pipeline/config/robot_profiles.yaml`

每个 `robot_id` 定义：

- `representation_default`: `structured` | `ambiguous`
- `projection_preferred`: `yaw_plane` | `full_se3`
- `modules`: 可击打模块数组
  - `name`
  - `offset_xyz`（相对 `center_pose`）
  - `orientation_rpy`（可选）
  - `hit_area`
    - `shape`: `rect` | `circle` | `polygon`
    - `size`: 如 `width/height` 或 `radius`
    - `confidence_weight`（给概率融合）
- `known_parameters`
  - 如：`center_pose`, `center_velocity`, `yaw`, `module_orientation`
- `unknown_parameters`
  - 如：`module_spin_phase`, `module_state`

---

## 4. 模块设计

建议新增目录：

- `include/gimbal_pipeline/common/robot_description/profile/`
- `src/common/robot_description/profile/`

核心组件：

1. `RobotProfile`（数据结构）
2. `RobotProfileLoaderYaml`（加载与 schema 校验）
3. `RobotProfileRegistry`（按 `robot_id` 查询）
4. `ConfigDrivenTrackedRobotBuilder`（由 profile 构造 `TrackedRobot`）

`RobotDescriptionFacade` 初始化流程调整：

1. 先加载 profile 配置并注册 config-driven builder。
2. 再注册固定 fallback builder（现有 fixed_profile）。
3. `strict_unknown_reject` 语义保持不变。

---

## 5. 对外 API（供其他模块取参）

在 `TrackedRobotUsage` 外新增 profile 查询接口（建议静态工具类 `RobotProfileUsage`）：

- `getProfile(robot_id)`
- `getModules(robot_id)`
- `getModuleByIndex(robot_id, idx)`
- `getEffectiveHitArea(robot_id, idx)`
- `isKnownParam(robot_id, param_key)`
- `isUnknownParam(robot_id, param_key)`

在运行时：

- 若 `TrackedRobot` 自带 `armors_offset`，优先用消息值。
- 若缺失，则按 profile 生成。
- 若 profile 缺失，走 fixed fallback。

---

## 6. fire_advice 概率新实现取参方式（不改旧链路）

目标：旧 `fire_advice` 保持原行为；新增概率链路按开关启用。

### 6.1 新增参数源抽象

在 `include/gimbal_controller/fire_advice/` 新增：

- `profile_provider.hpp`
- `probability_profile_adapter.hpp`

职责：

- 将 `robot_id + armor_index` 映射到模块 `hit_area/confidence_weight`。
- 将 `known/unknown` 映射为概率模型的置信度修正项（如 sigma 放大系数）。

### 6.2 概率链路接入点

在 `probability_engine` 输入上下文中增加可选字段：

- `target_hit_area`
- `target_confidence_weight`
- `observability_scale`（由 known/unknown 计算）

只在 `controller.fire.probability.enable_profile=true` 时生效；否则沿用旧参数。

### 6.3 兼容策略

- 旧链路：完全不改，配置默认不启用 profile 概率增强。
- 新链路：仅新增读取，不破坏原 `fire_on/off` gate 逻辑。

---

## 7. 分阶段实施

### P1（最小可用）

1. YAML loader + schema 校验。
2. `big_buff/small_buff` 配置化为结构化对象。
3. `BuffTargetAdapter` 去“强制 ambiguous”行为（仅在缺字段时降级）。

### P2（控制链路接入）

1. `RobotProfileUsage` API 落地。
2. `fire_advice` 概率链路接 profile（开关控制）。
3. `selector` 使用 `hit_area/confidence_weight` 加权。

### P3（URDF 扩展）

1. 增加 `RobotProfileLoaderUrdf`。
2. YAML/URDF 二选一或混合覆盖。

---

## 8. 验收标准

1. `big_buff/small_buff` 默认输出 `structured`（非单板）语义。
2. 旧链路关闭 profile 开关时行为一致。
3. 打开 profile 概率增强后，`fire_advice` 抖动下降且可回退。
4. profile 错误配置会在启动阶段给出明确报错（不 silent fallback）。

