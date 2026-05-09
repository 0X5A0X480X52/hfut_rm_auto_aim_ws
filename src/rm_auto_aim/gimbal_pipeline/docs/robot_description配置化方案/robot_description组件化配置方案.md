# robot_description 组件化配置方案

## 1. 概述与目标

将 `robot_description` 从**硬编码 builder**升级为**组件化配置驱动 builder**。核心思路：

1. **基本单元层**：定义可复用的"装甲板/扇叶"形状原语（small_armor、large_armor、big_buff_blade、small_buff_blade）。
2. **装配层**：按机器人类型（norm4、outpost3、BigBuff、SmallBuff、Sentry、Base），通过**布局策略**将原语组装成完整机器人。
3. **配置驱动**：YAML 描述原语库与装配定义，`ConfigDrivenTrackedRobotBuilder` 在运行时根据配置构造 `TrackedRobot`。

约束保持与上一版一致：前期 YAML，后续可扩展 URDF，旧链路 fallback 可用。

---

## 2. 当前实现分析

### 2.1 现有架构

```
RobotDescriptionFacade
  └─ TrackedRobotBuilderRegistry  (robot_id → builder 映射)
       └─ ITrackedRobotBuilderStrategy
            └─ FixedProfileTrackedRobotBuilder  （当前唯一实现）
```

构建流程（[robot_description_facade.cpp:87-133](src/common/robot_description/robot_description_facade.cpp#L87-L133) 与 [fixed_profile_tracked_robot_builder.cpp:31-225](src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp#L31-L225)）：

1. `registerDefaultBuilders()` 硬编码 9 个 robot_id 到 6 种 builder。
2. `FixedProfileTrackedRobotBuilder::buildTrackedRobot()` 从 `BaseTracker` 读取运行时状态（位置/速度/偏航/半径/d_za），用 `generateArmorsOffsetFromProfile()` 生成装甲板偏移。

### 2.2 核心问题

| 问题 | 现状 | 影响 |
|------|------|------|
| 硬编码注册 | robot_id/type/num_armors 写死在 C++ 中 | 新增机器人类型需重新编译 |
| big_buff/small_buff 单板 | `force_single_semantics=true`，num_armors=1 | 无法表示多扇叶结构化语义 |
| 无模块级元数据 | 缺少 hit_area、confidence_weight 等字段 | fire_advice 概率模型只能用全局 armor_width/height |
| 无参数可观测性声明 | known/unknown 参数未结构化 | 概率模型无法区分"已知量"与"估计量"做置信度修正 |
| 装配逻辑与状态估计耦合 | `generateArmorsOffsetFromProfile()` 同时承担布局与参数计算 | 新增布局类型（如旋鼓）需要修改核心工具类 |

### 2.3 现有装甲板偏移生成逻辑

[tracked_robot_usage.cpp:576-623](src/common/robot_description/tracked_robot_usage.cpp#L576-L623) — `generateArmorsOffsetFromProfile()`：

- **N ≠ 3,4**：等间距圆周分布，统一半径 r1，无 z 偏移。
- **N=4**：双半径交替 (r1/r2)，z 交替 ±d_za。
- **N=3**：三层 z 分布 (d_zc+d_za, d_zc, d_zc−d_za)，pitch 反向。

这本质上是**硬编码的三种布局策略**，缺乏对"旋鼓型（buff）"等新布局的扩展能力。

---

## 3. 组件化模型

### 3.1 三层模型

```
┌──────────────────────────────────────────────┐
│  Layer 3: Robot Assembly（机器人装配）        │
│  定义：robot_id → 一组带布局的模块放置        │
│  示例：norm4_standard = 4 × large_armor      │
│         (径向对称, 双半径交替)                │
├──────────────────────────────────────────────┤
│  Layer 2: Module Placement（模块放置）        │
│  定义：一个原语实例 + 相对位置/朝向           │
│  由 LayoutGenerator 根据 tracker 参数生成     │
├──────────────────────────────────────────────┤
│  Layer 1: Primitive Module（基本单元）        │
│  定义：形状、尺寸、有效打击面积、置信度权重   │
│  示例：small_armor, large_armor,              │
│         big_buff_blade, small_buff_blade      │
└──────────────────────────────────────────────┘
```

### 3.2 基本单元库（Primitive Library）

可击打模块的**几何与概率属性**，与具体机器人类型解耦。

> **关键**：大符和小符的打击单元是**圆形**扇叶，而非矩形装甲板。这影响概率模型中 hit_area 的计算方式（圆面积 vs 矩形面积）。

| 原语名称 | 形状 | 尺寸 (m) | 有效面积 (m²) | 置信度权重 | 适用机器人 |
|----------|------|----------|---------------|-----------|-----------|
| `small_armor` | rect | 0.125×0.135 | 0.0169 | 0.8 | norm4_small, sentry |
| `large_armor` | rect | 0.225×0.135 | 0.0304 | 1.0 | norm4_hero, outpost3 |
| `big_buff_blade` | **circle** | radius=0.090 | π×0.09²≈0.0254 | 0.7 | big_buff |
| `small_buff_blade` | **circle** | radius=0.060 | π×0.06²≈0.0113 | 0.6 | small_buff |
| `base_armor` | rect | 0.300×0.200 | 0.0600 | 1.0 | base |

### 3.3 布局策略（Layout Strategy）

每种布局策略定义了**如何从装配参数 + tracker 状态生成模块放置**：

| 布局类型 | 参数 | 模块数 N | 偏移生成规则 | 适用机器人 |
|----------|------|----------|-------------|-----------|
| `radial_symmetric` | 双半径 r1/r2, ±dza | 4 | 0°/90°/180°/270°，半径交替 r1/r2，z 交替 ±dza | norm4, sentry |
| `tri_layer_z` | 单半径 r, dza, dzc | 3 | 同角度、三层 z（dzc+dza, dzc, dzc−dza） | outpost3, base |
| `rotary_drum` | 鼓半径 R_drum, 扇叶角间距 | 5 | 绕 Y/drum 轴 72°均布，扇叶法向径向朝外，半径固定 | big_buff, small_buff |
| `custom_placement` | 显式 [x,y,z,rpy] 列表 | N | 直接使用配置中的显式偏移 | 特殊/自定义机器人 |

关键设计：**布局策略是纯几何函数**，输入为 `(LayoutDef, TrackerDynamicParams) → [ModulePlacement]`。

### 3.4 机器人装配定义

每种机器人 = 一个"原语引用" + "布局策略" + "运行时参数映射"：

| robot_id | 装配名 | 原语 | 布局 | 参数源 | 表示模式 | 投影偏好 |
|----------|-------|------|------|--------|---------|---------|
| `"1"` | hero_4_large | large_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"2"` | standard_4_small | small_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"3"` | standard_4_small | small_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"4"` | standard_4_small | small_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"5"` | standard_4_small | small_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"outpost"` | outpost_3 | large_armor | tri_layer_z | dza/dzc from tracker | structured | yaw_plane |
| `"sentry"` | sentry_4 | small_armor | radial_symmetric | r1/r2/dza from tracker | structured | yaw_plane |
| `"base"` | base_3 | base_armor | tri_layer_z | dza/dzc from tracker | structured | yaw_plane |
| `"big_buff"` | big_buff_5 | big_buff_blade | rotary_drum | R_drum from config, yaw from tracker | structured | full_se3 |
| `"small_buff"` | small_buff_5 | small_buff_blade | rotary_drum | R_drum from config, yaw from tracker | structured | full_se3 |

> **注意**：big_buff/small_buff 从当前 `num_armors=1 + AMBIGUOUS` **升级**为 `num_armors=5 + STRUCTURED`。当 tracker 无法提供多扇叶结构化状态时，降级为单板 ambiguous 模式。

### 3.5 击打约束机制（Engagement Constraints）

不同模块的击打条件差异显著，需要在配置层面声明约束，供 `fire_advice`、`selector`、`probability_engine` 统一查询。

#### 3.5.1 击打模式（Engagement Mode）

| 模式 | 含义 | 典型场景 | 对概率模型的影响 |
|------|------|---------|----------------|
| `static` | 模块随机器人整体运动，自身无独立旋转 | norm4 装甲板、sentry | 模块朝向 = 机器人 yaw + 固定偏置，法向稳定 |
| `rotating` | 模块安装在旋转机构上，相位持续变化 | big_buff/small_buff 扇叶 | 模块朝向 = 鼓转角 + 扇叶偏置，需相位预测；仅在特定相位窗口内可击打 |
| `stationary` | 模块固定在世界坐标系中，完全不运动 | outpost、base | 模块静止，速度为零，可观察性最高 |

#### 3.5.2 约束类型

```yaml
# 击打约束定义（在 assembly 级别声明，可被 downstream 覆盖）
engagement_constraints:

  # ── 朝向可见性约束 ──
  facing:
    mode: front_hemisphere       # "front_hemisphere" | "any" | "normal_cone"
    min_facing_cos: 0.0          # 最小朝向余弦: 0=正面90°半锥, 0.5=正面60°锥
    # normal_cone 模式时的锥角
    cone_half_angle_deg: 45.0    # 仅 mode=normal_cone 时生效

  # ── 可见性约束 ──
  visibility:
    require_visible: true        # 模块必须在相机视野内
    require_stable_tracking: true # 必须在 TRACKING 状态（非 DETECTING/TEMP_LOST）

  # ── 运动状态约束 ──
  motion:
    max_center_speed: 5.0        # 机器人中心最大线速度 (m/s)，超此速度不击打
    max_yaw_rate: 720.0          # 最大 yaw 角速度 (deg/s)

  # ── 距离约束 ──
  distance:
    min_m: 0.5                   # 最小击打距离 (m)
    max_m: 8.0                   # 最大击打距离 (m)

  # ── 旋转相位约束（仅 rotating 模式） ──
  spin_phase:
    active_phase_span_deg: 60.0  # 单个扇叶可击打的相位跨度 (°)
    # 当扇叶法向与"机器人→自车"连线的夹角在此范围内时，判定为可击打
    require_facing_self: true    # 扇叶必须朝向自车方向

  # ── 规则约束（比赛规则相关） ──
  rules:
    cooldown_after_hit_ms: 500   # 被击中后冷却时间
    max_hits_per_burst: 3        # 单次连发最大命中数
    require_armor_id_match: true # armor_id 必须在 bound_armor_ids 中
```

#### 3.5.3 各机器人类型的约束配置

| 机器人 | 击打模式 | 朝向约束 | 特殊约束 |
|--------|---------|---------|---------|
| norm4 (小/英雄) | `static` | front_hemisphere, min_cos=0 | 4 板均等，跟踪态击打 |
| outpost3 | `stationary` | front_hemisphere, min_cos=0.3 | 静止目标，无速度不确定性 |
| base | `stationary` | front_hemisphere, min_cos=0 | 大目标，近距离击打 |
| sentry | `static` | front_hemisphere, min_cos=0 | 同 norm4，可能在坡道上 |
| **big_buff** | **`rotating`** | **normal_cone, cone=45°** | 旋鼓相位约束；扇叶朝向自车时击打；cooldown 500ms |
| **small_buff** | **`rotating`** | **normal_cone, cone=45°** | 同 big_buff，半径更小 |

#### 3.5.4 约束评估流程

```
评估 module_i 是否可击打:
  1. 检查 motion 约束（速度/角速度阈值）
  2. 检查 distance 约束（距离范围）
  3. 检查 visibility 约束（跟踪状态 + 视野内）
  4. 根据 engagement_mode 分支:
     static/stationary:
       5a. 计算 facing_cos = (module_normal · center_to_observer)
       5b. 检查 facing_cos >= min_facing_cos
     rotating:
       5a. 预测 phase(t) = spin_phase + yaw_rate × t
       5b. 若 require_facing_self: 检查 |phase_to_self| < active_phase_span/2
       5c. 若 mode=normal_cone: 检查 normal_angle < cone_half_angle
  6. 检查 rules 约束（cooldown, burst limit）
  → 返回 {engageable: bool, confidence_scale: double}
```

`confidence_scale` 用于概率模型：约束满足越"边缘"（如 min_facing_cos 临界），输出置信度越低。

#### 3.5.5 与现有代码的对应关系

| 约束 | 现有实现位置 | 改造方式 |
|------|------------|---------|
| front_hemisphere 朝向 | `fire_advice_engine` 中 `require_front_face` + `computeFacingCos()` | 从配置读取 `min_facing_cos` 替代硬编码阈值 |
| 跟踪状态过滤 | `FixedProfileTrackedRobotBuilder` 中 `is_tracking()` 分支 | 保持不变，由 tracker 保证 |
| 速度阈值 | `ProbabilityConfig::normal_v_activate_min` | 从 assembly 配置读取 |
| 相位窗口 | **无现成实现** | 新增 `SpinPhaseGate`，在 fire_advice 中接入 |
| 距离约束 | 散落在 selector 和 fire_advice 中 | 统一到 `EngagementConstraintEvaluator` |
| cooldown | **无现成实现** | 新增 `HitCooldownTimer`，按 robot_id 维护 |

#### 3.5.6 激活策略与掩码（新增）

`engagement_constraints` 负责几何/可见性/相位等击打约束；比赛态下的"随机激活几块扇叶"不应混入约束评估，建议新增并行配置 `activation_policy`：

```yaml
activation_policy:
  mode: external_mask              # external_mask | deterministic | random_sim
  module_count: 5
  active_count: 2                  # big_buff=2, small_buff=1
  source: /auto_buff/engagement_state
  mask_field: active_mask
  ttl_ms: 150
```

设计要点：

- 上游 `auto_buff` 始终发布完整五扇叶结构化状态（`num_armors=5`），激活信息通过 `active_mask` 表达。
- `selector/fire_advice` 先按掩码过滤候选扇叶，再应用 `engagement_constraints`。
- 无掩码或掩码超时时，降级为"全不可击打"或"仅保留最近稳定掩码"，由策略参数控制。

---

## 4. YAML 配置 Schema

文件位置：`src/rm_auto_aim/gimbal_pipeline/config/robot_profiles.yaml`

```yaml
# ============================================================
# robot_profiles.yaml — 组件化机器人描述配置
# ============================================================

version: 1

# ── Layer 1: 基本单元库 ───────────────────────────────────
primitives:

  small_armor:
    shape: rect
    size: { width: 0.125, height: 0.135 }
    hit_area: 0.016875
    confidence_weight: 0.8

  large_armor:
    shape: rect
    size: { width: 0.225, height: 0.135 }
    hit_area: 0.030375
    confidence_weight: 1.0

  big_buff_blade:
    shape: circle
    size: { radius: 0.090 }
    hit_area: 0.02545       # π × 0.09²
    confidence_weight: 0.7

  small_buff_blade:
    shape: circle
    size: { radius: 0.060 }
    hit_area: 0.01131       # π × 0.06²
    confidence_weight: 0.6

  base_armor:
    shape: rect
    size: { width: 0.300, height: 0.200 }
    hit_area: 0.06
    confidence_weight: 1.0

# ── Layer 2 & 3: 布局定义与机器人装配 ────────────────────
assemblies:

  # ── Norm4 系列 ──
  hero_4_large:
    robot_type: HERO_4
    representation_default: structured
    projection_preferred: yaw_plane
    engagement_mode: static          # 模块随整体运动，无独立旋转
    layout:
      type: radial_symmetric
      count: 4
      dual_radius: true
      dual_z: true
      radius_source: tracker
      z_offset_source: tracker
    module_ref: large_armor
    engagement_constraints:
      facing: { mode: front_hemisphere, min_facing_cos: 0.0 }
      visibility: { require_visible: true, require_stable_tracking: true }
      motion: { max_center_speed: 5.0, max_yaw_rate: 720.0 }
      distance: { min_m: 0.5, max_m: 8.0 }
    known_parameters:
      - center_pose
      - center_velocity
      - yaw
      - module_orientation
    unknown_parameters:
      - module_spin_phase

  standard_4_small:
    robot_type: STANDARD_4
    representation_default: structured
    projection_preferred: yaw_plane
    engagement_mode: static
    layout:
      type: radial_symmetric
      count: 4
      dual_radius: true
      dual_z: true
      radius_source: tracker
      z_offset_source: tracker
    module_ref: small_armor
    engagement_constraints:
      facing: { mode: front_hemisphere, min_facing_cos: 0.0 }
      visibility: { require_visible: true, require_stable_tracking: true }
      motion: { max_center_speed: 5.0, max_yaw_rate: 720.0 }
      distance: { min_m: 0.5, max_m: 8.0 }
    known_parameters:
      - center_pose
      - center_velocity
      - yaw
      - module_orientation
    unknown_parameters:
      - module_spin_phase

  # ── Outpost3 ──
  outpost_3:
    robot_type: OUTPOST_3
    representation_default: structured
    projection_preferred: yaw_plane
    engagement_mode: stationary       # 固定目标，完全不运动
    layout:
      type: tri_layer_z
      count: 3
      radius_source: tracker
      z_offset_source: tracker
    module_ref: large_armor
    engagement_constraints:
      facing: { mode: front_hemisphere, min_facing_cos: 0.3 }  # 较严格朝向约束
      visibility: { require_visible: true, require_stable_tracking: true }
      motion: { max_center_speed: 0.0, max_yaw_rate: 0.0 }     # 静止
      distance: { min_m: 0.5, max_m: 8.0 }
    known_parameters:
      - center_pose
      - yaw
    unknown_parameters:
      - module_spin_phase
      - center_velocity

  # ── Sentry ──
  sentry_4:
    robot_type: SENTRY
    representation_default: structured
    projection_preferred: yaw_plane
    engagement_mode: static
    layout:
      type: radial_symmetric
      count: 4
      dual_radius: true
      dual_z: true
      radius_source: tracker
      z_offset_source: tracker
    module_ref: small_armor
    engagement_constraints:
      facing: { mode: front_hemisphere, min_facing_cos: 0.0 }
      visibility: { require_visible: true, require_stable_tracking: true }
      motion: { max_center_speed: 3.0, max_yaw_rate: 360.0 }
      distance: { min_m: 0.5, max_m: 8.0 }
    known_parameters:
      - center_pose
      - center_velocity
      - yaw
      - module_orientation
    unknown_parameters:
      - module_spin_phase

  # ── Base ──
  base_3:
    robot_type: BASE
    representation_default: structured
    projection_preferred: yaw_plane
    engagement_mode: stationary       # 固定基地
    layout:
      type: tri_layer_z
      count: 3
      radius_source: tracker
      z_offset_source: tracker
    module_ref: base_armor
    engagement_constraints:
      facing: { mode: front_hemisphere, min_facing_cos: 0.0 }
      visibility: { require_visible: true, require_stable_tracking: true }
      motion: { max_center_speed: 0.0, max_yaw_rate: 0.0 }
      distance: { min_m: 0.3, max_m: 5.0 }    # 基地近距离击打
    known_parameters:
      - center_pose
      - yaw
    unknown_parameters:
      - module_spin_phase
      - center_velocity

  # ── Big Buff（旋鼓型，5 圆形扇叶） ──
  big_buff_5:
    robot_type: UNKNOWN
    representation_default: structured
    projection_preferred: full_se3
    engagement_mode: rotating        # 扇叶随鼓旋转
    layout:
      type: rotary_drum
      count: 5
      angular_pitch_deg: 72.0
      drum_radius: 0.28
      radius_source: config
      normal_direction: radial_outward
    module_ref: big_buff_blade
    engagement_constraints:
      facing: { mode: normal_cone, cone_half_angle_deg: 45.0 }
      visibility: { require_visible: true, require_stable_tracking: false }  # DETECTING 也可击打
      motion: { max_center_speed: 3.0, max_yaw_rate: 1200.0 }
      distance: { min_m: 0.5, max_m: 8.0 }
      spin_phase:
        active_phase_span_deg: 60.0       # 扇叶法向±30°内可击打
        require_facing_self: true
      rules: { cooldown_after_hit_ms: 500 }
    activation_policy:
      mode: external_mask
      module_count: 5
      active_count: 2
      source_topic: /auto_buff/engagement_state
      mask_field: active_mask
      ttl_ms: 150
    kinematic_constraint:
      center_fixed_prior: true
      roll_only: true
      pitch_prior_rad: 0.0
      yaw_prior_rad: 0.0
      center_prior_sigma_m: 0.02
    known_parameters:
      - center_pose
      - center_velocity
      - yaw
    unknown_parameters:
      - module_spin_phase
      - module_state

  # ── Small Buff（旋鼓型，5 圆形扇叶） ──
  small_buff_5:
    robot_type: UNKNOWN
    representation_default: structured
    projection_preferred: full_se3
    engagement_mode: rotating
    layout:
      type: rotary_drum
      count: 5
      angular_pitch_deg: 72.0
      drum_radius: 0.20
      radius_source: config
      normal_direction: radial_outward
    module_ref: small_buff_blade
    engagement_constraints:
      facing: { mode: normal_cone, cone_half_angle_deg: 45.0 }
      visibility: { require_visible: true, require_stable_tracking: false }
      motion: { max_center_speed: 3.0, max_yaw_rate: 1200.0 }
      distance: { min_m: 0.5, max_m: 8.0 }
      spin_phase:
        active_phase_span_deg: 60.0
        require_facing_self: true
      rules: { cooldown_after_hit_ms: 500 }
    activation_policy:
      mode: external_mask
      module_count: 5
      active_count: 1
      source_topic: /auto_buff/engagement_state
      mask_field: active_mask
      ttl_ms: 150
    kinematic_constraint:
      center_fixed_prior: true
      roll_only: true
      pitch_prior_rad: 0.0
      yaw_prior_rad: 0.0
      center_prior_sigma_m: 0.02
    known_parameters:
      - center_pose
      - center_velocity
      - yaw
    unknown_parameters:
      - module_spin_phase
      - module_state

# ── robot_id → 装配 映射 ──────────────────────────────────
id_mapping:
  "1": hero_4_large
  "2": standard_4_small
  "3": standard_4_small
  "4": standard_4_small
  "5": standard_4_small
  outpost: outpost_3
  sentry: sentry_4
  base: base_3
  big_buff: big_buff_5
  small_buff: small_buff_5
```

---

## 5. C++ 模块设计

### 5.1 新增文件结构

```
src/rm_auto_aim/gimbal_pipeline/
  include/gimbal_pipeline/common/robot_description/profile/
    primitive_def.hpp               # PrimitiveDef 数据结构
    robot_assembly_def.hpp          # RobotAssemblyDef, LayoutDef, EngagementConstraints
    layout_generator.hpp            # ILayoutGenerator 接口 + 具体实现
    engagement_constraint_evaluator.hpp  # 击打约束评估器
    robot_profile_registry.hpp      # RobotProfileRegistry
    robot_profile_loader_yaml.hpp   # YAML 加载 + Schema 校验
    config_driven_tracked_robot_builder.hpp  # 新 builder
    robot_profile_usage.hpp         # 对外查询 API
  src/common/robot_description/profile/
    layout_generator.cpp
    engagement_constraint_evaluator.cpp
    robot_profile_registry.cpp
    robot_profile_loader_yaml.cpp
    config_driven_tracked_robot_builder.cpp
    robot_profile_usage.cpp
  config/
    robot_profiles.yaml             # 配置文件
```

### 5.2 核心数据结构

```cpp
// ── primitive_def.hpp ──

enum class PrimitiveShape { RECT, CIRCLE, POLYGON };

struct PrimitiveDef {
  std::string name;
  PrimitiveShape shape;
  double width{0.0};            // rect: width
  double height{0.0};           // rect: height
  double radius{0.0};           // circle: radius
  double hit_area{0.0};         // 有效打击面积 (m²)；circle 自动 π×r²
  double confidence_weight{1.0};// 概率融合权重
};

// ── robot_assembly_def.hpp ──

enum class LayoutType {
  RADIAL_SYMMETRIC,
  TRI_LAYER_Z,
  ROTARY_DRUM,
  CUSTOM_PLACEMENT
};

enum class RadiusSource { TRACKER, CONFIG };

struct LayoutDef {
  LayoutType type;
  int count{0};
  bool dual_radius{false};
  bool dual_z{false};
  RadiusSource radius_source{RadiusSource::TRACKER};
  RadiusSource z_offset_source{RadiusSource::TRACKER};
  double drum_radius{0.0};                  // ROTARY_DRUM 专用
  double angular_pitch_deg{72.0};           // ROTARY_DRUM 专用（五扇叶）
  std::string normal_direction;             // "radial_outward" | "tangential"
  // CUSTOM_PLACEMENT 专用：显式偏移列表
  std::vector<std::array<double, 6>> custom_offsets; // {x,y,z,r,p,y}
};

// ── 击打约束数据结构 ──

enum class EngagementMode { STATIC, ROTATING, STATIONARY };

enum class FacingMode { FRONT_HEMISPHERE, ANY, NORMAL_CONE };

struct FacingConstraint {
  FacingMode mode{FacingMode::FRONT_HEMISPHERE};
  double min_facing_cos{0.0};          // front_hemisphere: 负值=后半球也允许
  double cone_half_angle_deg{45.0};    // normal_cone 专用
};

struct VisibilityConstraint {
  bool require_visible{true};
  bool require_stable_tracking{true};
};

struct MotionConstraint {
  double max_center_speed{5.0};        // m/s，超此速度 reject
  double max_yaw_rate{720.0};          // deg/s
};

struct DistanceConstraint {
  double min_m{0.5};
  double max_m{8.0};
};

struct SpinPhaseConstraint {            // 仅 ROTATING 模式生效
  double active_phase_span_deg{60.0};   // 可击打的相位跨度
  bool require_facing_self{true};
};

struct RuleConstraint {
  double cooldown_after_hit_ms{0.0};
  int max_hits_per_burst{0};           // 0 = 无限制
  bool require_armor_id_match{true};
};

struct EngagementConstraints {
  FacingConstraint facing;
  VisibilityConstraint visibility;
  MotionConstraint motion;
  DistanceConstraint distance;
  SpinPhaseConstraint spin_phase;      // 仅 ROTATING 模式使用
  RuleConstraint rules;
};

struct ActivationPolicy {
  std::string mode{"external_mask"};   // external_mask | deterministic | random_sim
  int module_count{0};                 // 典型值: 5
  int active_count{0};                 // big=2, small=1
  std::string source_topic;            // 例: /auto_buff/engagement_state
  std::string mask_field{"active_mask"};
  int ttl_ms{150};
};

struct KinematicConstraint {
  bool center_fixed_prior{false};      // 是否启用中心固定先验
  bool roll_only{false};               // true: 仅允许 roll 分量
  double pitch_prior_rad{0.0};         // odom 系先验（buff: 0）
  double yaw_prior_rad{0.0};           // odom 系先验（buff: 0）
  double center_prior_sigma_m{0.02};   // 中心固定先验方差（软约束）
};

struct RobotAssemblyDef {
  std::string name;
  uint8_t robot_type;
  std::string representation_default;   // "structured" | "ambiguous"
  std::string projection_preferred;     // "yaw_plane" | "full_se3"
  EngagementMode engagement_mode{EngagementMode::STATIC};
  LayoutDef layout;
  std::string module_ref;               // → PrimitiveDef::name
  EngagementConstraints engagement_constraints;
  ActivationPolicy activation_policy;   // 仅 buff 等需要激活掩码的目标
  KinematicConstraint kinematic_constraint;
  std::vector<std::string> known_parameters;
  std::vector<std::string> unknown_parameters;
};

// 运行时：布局生成器输出的模块放置
struct ModulePlacement {
  int index;
  Eigen::Vector3d offset_xyz;            // 相对 center_pose 的平移
  Eigen::Vector3d orientation_rpy;       // 相对朝向
  const PrimitiveDef * primitive{nullptr}; // 指向原语定义
  bool is_engageable{true};              // 由约束评估器设置
  double constraint_confidence_scale{1.0}; // 约束满足度 (0~1)
};
```

### 5.3 布局生成器接口与实现

```cpp
// ── layout_generator.hpp ──

// 从 tracker 提取的运行时动态参数
struct TrackerDynamicParams {
  double r1{0.0}, r2{0.0};
  double d_za{0.0}, d_zc{0.0};
  double yaw{0.0};
};

class ILayoutGenerator {
public:
  virtual ~ILayoutGenerator() = default;
  virtual std::vector<ModulePlacement> generate(
    const LayoutDef & layout,
    const TrackerDynamicParams & params) const = 0;
};

// 工厂函数
std::unique_ptr<ILayoutGenerator> createLayoutGenerator(LayoutType type);
```

**具体实现映射当前逻辑并扩展新类型**：

| 实现类 | 对应 LayoutType | 说明 |
|--------|----------------|------|
| `RadialSymmetricGenerator` | RADIAL_SYMMETRIC | 迁移自 `generateArmorsOffsetFromProfile` N=4 分支 |
| `TriLayerZGenerator` | TRI_LAYER_Z | 迁移自 `generateArmorsOffsetFromProfile` N=3 分支 |
| `RotaryDrumGenerator` | ROTARY_DRUM | **新增**：旋鼓型扇叶布局 |
| `CustomPlacementGenerator` | CUSTOM_PLACEMENT | **新增**：显式偏移列表 |

`RotaryDrumGenerator` 核心逻辑：
```
for i in 0..count-1:
    angle = i * angular_pitch_deg + yaw_offset
    offset = (drum_radius * cos(angle), drum_radius * sin(angle), 0)
    normal = angle + π     // 扇叶法向径向朝外
```

### 5.4 击打约束评估器

```cpp
// ── engagement_constraint_evaluator.hpp ──

struct EngagementEvalInput {
  Eigen::Vector3d center_position;       // 机器人中心世界坐标
  Eigen::Vector3d center_velocity;       // 机器人中心速度
  Eigen::Vector3d self_position;         // 自车世界坐标
  double yaw{0.0};
  double yaw_rate{0.0};                 // deg/s
  double spin_phase{0.0};              // 旋转模块的当前相位 (仅 ROTATING 模式)
  bool is_tracking{false};
  bool is_visible{false};
};

struct EngagementEvalResult {
  bool engageable{false};
  double confidence_scale{1.0};         // 0=完全不可击打, 1=最佳击打条件
  std::string reject_reason;            // 不满足时记录原因
  double facing_cos{0.0};               // 用于概率模型修正
};

class EngagementConstraintEvaluator {
public:
  explicit EngagementConstraintEvaluator(const EngagementConstraints & constraints,
                                          EngagementMode mode);

  // 对所有模块评估，返回每个模块的结果
  std::vector<EngagementEvalResult> evaluateAll(
    const std::vector<ModulePlacement> & modules,
    const EngagementEvalInput & input) const;

  // 对单个模块评估
  EngagementEvalResult evaluateSingle(
    const ModulePlacement & module,
    const EngagementEvalInput & input) const;

private:
  // 各约束子评估
  bool checkMotion(const EngagementEvalInput & input, std::string & reason) const;
  bool checkDistance(const EngagementEvalInput & input, std::string & reason) const;
  bool checkVisibility(const EngagementEvalInput & input, std::string & reason) const;

  // 朝向评估（模式不同逻辑不同）
  bool checkFacing(const ModulePlacement & module,
                   const EngagementEvalInput & input,
                   double & facing_cos,
                   std::string & reason) const;

  // 旋转相位评估（仅 ROTATING 模式）
  bool checkSpinPhase(const ModulePlacement & module,
                      const EngagementEvalInput & input,
                      std::string & reason) const;

  EngagementConstraints constraints_;
  EngagementMode mode_;
};
```

约束评估逻辑（对应 3.5.4 流程）：

- **motion**：`||velocity|| <= max_center_speed && |yaw_rate| <= max_yaw_rate`
- **distance**：`min_m <= ||center - self|| <= max_m`
- **visibility**：`(!require_visible || is_visible) && (!require_stable_tracking || is_tracking)`
- **facing** (static/stationary)：计算 `(module_normal · center_to_self)`，检查是否 ≥ `min_facing_cos`
- **facing** (rotating)：预测当前相位，检查 `|phase_to_self| < active_phase_span/2`
- **rules**：检查 cooldown 计时器、burst 计数（通过外部状态维护）

### 5.5 配置驱动 Builder

```cpp
// ── config_driven_tracked_robot_builder.hpp ──

class ConfigDrivenTrackedRobotBuilder final : public ITrackedRobotBuilderStrategy {
public:
  ConfigDrivenTrackedRobotBuilder(
    const RobotAssemblyDef & assembly,
    const PrimitiveDef & primitive,
    std::unique_ptr<ILayoutGenerator> layout_gen);

  // ITrackedRobotBuilderStrategy 接口
  std::string strategyName() const override;
  uint8_t robotType() const override;
  int numArmors() const override;
  rm_interfaces::msg::TrackedRobot buildTrackedRobot(
    const TrackedRobotBuildInput & input) const override;

private:
  RobotAssemblyDef assembly_;
  PrimitiveDef primitive_;
  std::unique_ptr<ILayoutGenerator> layout_gen_;
  EngagementConstraintEvaluator constraint_evaluator_;  // 从 assembly 构造
};
```

`buildTrackedRobot()` 流程：

1. 从 `input.tracker` 提取动态参数（位置/速度/偏航/半径/dza/相位）。
2. 调用 `layout_gen_->generate(layout_def, dynamic_params)` 生成 `ModulePlacement[]`。
3. **约束评估**：对每个 module 调用 `constraint_evaluator_.evaluateSingle()`，设置 `is_engageable` 和 `constraint_confidence_scale`。
4. 将 engageable 的 placements 转换为 `armors_offset`（ROS Pose 数组）；不可击打的模块标记但不移除（供 selector 决策）。
5. 若 tracker 处于 ambiguous 模式，降级为单板表示（兼容旧语义）。
6. 填充 robot_type、representation_mode、num_armors 等字段。
7. 将 `primitive_.hit_area`、`primitive_.confidence_weight`、约束评估结果作为扩展字段写入 TrackedRobot（若消息支持，否则通过 `RobotProfileUsage` 侧面查询）。
8. 协方差、置信度等字段与现有 `FixedProfileTrackedRobotBuilder` 保持一致。

### 5.6 Profile Registry 与查询 API

```cpp
// ── robot_profile_registry.hpp ──

class RobotProfileRegistry {
public:
  void loadFromYaml(const std::string & yaml_path);

  const RobotAssemblyDef * findAssembly(const std::string & robot_id) const;
  const PrimitiveDef * findPrimitive(const std::string & name) const;

  std::vector<std::string> supportedRobotIds() const;

private:
  std::unordered_map<std::string, PrimitiveDef> primitives_;
  std::unordered_map<std::string, RobotAssemblyDef> assemblies_;
  std::unordered_map<std::string, std::string> id_to_assembly_; // robot_id → assembly_name
};

// ── robot_profile_usage.hpp ──

class RobotProfileUsage {
public:
  static void initialize(std::shared_ptr<const RobotProfileRegistry> registry);

  static const RobotAssemblyDef * getAssembly(const std::string & robot_id);
  static const PrimitiveDef * getPrimitive(const std::string & robot_id);
  static const EngagementConstraints * getConstraints(const std::string & robot_id);
  static EngagementMode getEngagementMode(const std::string & robot_id);
  static double getEffectiveHitArea(const std::string & robot_id, int module_idx);
  static double getConfidenceWeight(const std::string & robot_id, int module_idx);
  static bool isKnownParam(const std::string & robot_id, const std::string & param_key);
  static bool isUnknownParam(const std::string & robot_id, const std::string & param_key);
  // 快速判断
  static bool isRotatingTarget(const std::string & robot_id);
  static bool isStationaryTarget(const std::string & robot_id);

private:
  static std::shared_ptr<const RobotProfileRegistry> registry_;
};
```

---

## 6. 集成方案

### 6.1 初始化流程变更

当前 `RobotDescriptionFacade` 构造函数直接调用 `registerDefaultBuilders()`。

新流程（[gimbal_pipeline_node.cpp] 初始化段）：

```cpp
// 1. 加载 YAML 配置
auto profile_registry = std::make_shared<RobotProfileRegistry>();
profile_registry->loadFromYaml(yaml_path);
RobotProfileUsage::initialize(profile_registry);

// 2. 注册 config-driven builders 为第一优先级
for (const auto & robot_id : profile_registry->supportedRobotIds()) {
  const auto * assembly = profile_registry->findAssembly(robot_id);
  const auto * primitive = profile_registry->findPrimitive(assembly->module_ref);
  auto layout_gen = createLayoutGenerator(assembly->layout.type);
  auto builder = std::make_shared<ConfigDrivenTrackedRobotBuilder>(
    *assembly, *primitive, std::move(layout_gen));
  facade->registerBuilder(robot_id, builder);
}

// 3. 注册 fixed-profile builders 为第二优先级（fallback）
facade->registerDefaultBuilders();  // 仅在 id 未注册时生效
```

优先规则：`registerBuilder` 若 robot_id 已存在则覆盖（或保持不变，视需求）。建议"先注册优先"：config-driven 先注册，fallback 不覆盖已有条目。

### 6.2 big_buff/small_buff 升级路径

当前 `BuffTargetAdapter`（[buff_target_adapter.cpp:57-86](src/adapters/buff_target_adapter.cpp#L57-L86)）强制将 buff 消息归一化为单板 ambiguous：

```cpp
robot.representation_mode = REP_AMBIGUOUS_SINGLE_ARMOR;
robot.num_armors = 1;
```

升级后行为：

- **auto_buff 始终发布完整五扇叶状态**：`representation_mode = STRUCTURED_ROBOT`，`num_armors = 5`，`armors_offset.size() = 5`。
- **激活信息独立发布**：新增 `BuffEngagementState`（或等效消息），包含 `active_mask`、`active_count`、`mode(big/small)`、`phase/omega`。
- **tracker 提供结构化多扇叶状态**：builder 使用 `ROTARY_DRUM` 布局生成 5 个 armors_offset，几何完整性不受激活数量影响。
- **tracker 处于降级模式**（`is_ambiguous_single_mode() == true`）：builder 回退到单板零偏移 ambiguous 表示。
- `BuffTargetAdapter` 移除强制 `REP_AMBIGUOUS_SINGLE_ARMOR` 行，改为尊重上游 tracker 的结构化输出，并聚合 `active_mask` 到下游可查询缓存。

推荐链路：

1. `auto_buff/tracked_robot_full`：发布完整五扇叶 `TrackedRobot`。
2. `auto_buff/engagement_state`：发布激活掩码与时效（大符随机激活2块，小符随机激活1块由上游实时给定）。
3. `gimbal_pipeline::BuffTargetAdapter`：按时间戳聚合两路消息，对控制侧输出"完整状态 + 激活掩码视图"。

#### 6.2.1 状态模型细化（基于中心固定 + 仅 roll）

大小符统一采用受限状态模型：

- 状态向量：`x = [cx, cy, cz, roll, vroll]`（big_buff 可附加 `a/omega/c/d`）。
- odom 姿态先验：`pitch=0, yaw=0`，仅 `roll` 随时间演化。
- 中心先验：`center_fixed_prior=true`，以软约束形式维持中心稳定（而不是硬编码锁死）。
- 几何重建：`blade_i_roll = roll + i * 72deg`，`i∈[0..4]`。

对应到 `TrackedRobot`：

1. `center_pose.position = [cx,cy,cz]`
2. `center_pose.orientation = quat(roll, 0, 0)`（RPY 顺序）
3. `num_armors=5`，`armors_offset` 全量发布
4. `representation_mode=REP_STRUCTURED_ROBOT`

#### 6.2.2 auto_buff 侧改造步骤

1. 在 tracker 输出层增加 `TrackedRobot(full)` 发布器，替代当前仅 `AimCommand` 的状态暴露。
2. 将 `BuffState` 映射为五扇叶 offset：
   - `position`: 由 `drum_radius` + `roll+i*72deg` 生成
   - `orientation`: `quat(roll+i*72deg, 0, 0)` 或与法向一致的等价表达
3. 新增 `BuffEngagementState` 发布：
   - `active_mask`（5bit）
   - `active_count`（big=2/small=1）
   - `mode`、`stamp`、`ttl_ms`
4. 兼容保留：老 `AimCommand` 输出不删，便于灰度切换。

#### 6.2.3 BuffTargetAdapter 侧改造步骤

1. 删除强制改写为 ambiguous 的逻辑（`representation_mode/num_armors` 不再覆盖）。
2. 增加第二订阅：`/auto_buff/engagement_state`。
3. 以 `stamp + timeout` 聚合两路数据，形成内部快照：
   - `TrackedRobot full_robot`
   - `uint8 active_mask`
   - `bool mask_valid`
4. 对外提供统一查询接口（供 selector/fire_advice）：
   - `latestValidRobot(now)`
   - `latestValidActivationMask(now)`
5. 超时降级策略配置化：
   - `mask_timeout_policy = reject_all | keep_last | fallback_single`
   - 默认 `reject_all`，避免误击发。

#### 6.2.4 对 norm4/outpost3 的兼容边界（仅优化不破坏）

- `norm4/outpost3` 不配置 `activation_policy`，默认空策略，无掩码依赖。
- `kinematic_constraint.roll_only=false`（沿用现有 yaw-plane/full-se3 逻辑）。
- `layout_generator` 的 `RADIAL_SYMMETRIC/TRI_LAYER_Z` 路径不改公式，保持数值一致。
- `fire_advice` 仅在 `isRotatingTarget(robot_id)==true` 时启用相位与掩码分支。
- `strict_unknown_reject` 与 fallback builder 语义不变。

### 6.3 fire_advice 概率链路接入

`ProbabilityEngine::evaluate()` 当前使用 `ProbabilityConfig::armor_width_m/armor_height_m` 作为全局默认装甲板尺寸。

新增接入方式（开关控制）：

```cpp
// 伪代码：在 fire_advice_engine 中
if (config.fire.probability.enable_profile) {
  const auto * primitive = RobotProfileUsage::getPrimitive(robot.robot_id);
  const auto * constraints = RobotProfileUsage::getConstraints(robot.robot_id);

  if (primitive->shape == PrimitiveShape::CIRCLE) {
    // 圆形打击单元（大符/小符）：使用半径计算圆形命中概率
    armor_radius = primitive->radius;
    p_hit = circularHitProbability(error_u, error_v, sigma_u, sigma_v, armor_radius);
  } else {
    // 矩形装甲板：使用宽高计算矩形命中概率
    armor_width = primitive->width;
    armor_height = primitive->height;
    p_hit = rectangularHitProbability(error_u, error_v, sigma_u, sigma_v, armor_width, armor_height);
  }

  // 约束满足度修正
  p_hit *= module.constraint_confidence_scale;

  // observability_scale 由 known/unknown 参数推导
  if (RobotProfileUsage::isRotatingTarget(robot.robot_id)) {
    // 旋转目标：相位不确定性放大 sigma
    sigma_scale = computeRotatingObservabilityScale(robot.robot_id);
    sigma_u *= sigma_scale;
    sigma_v *= sigma_scale;
  }

  // 朝向余弦用于概率加权（正对时 p_hit 高，侧对时低）
  p_hit *= std::max(0.0, engagement_result.facing_cos);

} else {
  armor_width = prob_cfg.armor_width_m;
  armor_height = prob_cfg.armor_height_m;
}
```

**圆形命中概率计算**（区别于矩形的独立维度积分）：

```
circularHitProbability(e_u, e_v, σ_u, σ_v, R):
  // 马氏距离的平方
  d2 = (e_u/σ_u)² + (e_v/σ_v)²
  // 圆内概率 = 卡方分布 CDF (2 自由度)
  return 1 - exp(-d2/2) * (1 + ((R/σ_u)² + (R/σ_v)²)/2 * ... )
  // 或使用 Sigma-Point 变换统一处理:
  for each sigma point (u_i, v_i):
    if (u_i² + v_i² <= R²) p += w_i
  return p
```

### 6.4 约束评估在 fire_advice 中的位置

约束评估器在 `FireAdviceEngine::evaluate()` 中的调用时机：

```
for each candidate robot:
  for each module in robot:
    1. EngagementConstraintEvaluator::evaluateSingle()
       → 快速过滤明显不可击打的模块（距离/速度/跟踪状态）
    2. 若 engageable:
       → 弹道求解 (FlightTimeSolver)
       → 概率计算 (ProbabilityEngine)，传入 constraint_confidence_scale
       → FireAdvisor 决策
    3. 若不 engageable:
       → 跳过，记录 reject_reason
```

这样约束评估器充当**第一道过滤器**，避免对明显无效的模块执行昂贵的弹道求解和概率计算。

### 6.5 兼容策略总结

| 场景 | 行为 |
|------|------|
| 配置 YAML 存在且有效 | 使用 ConfigDrivenTrackedRobotBuilder + EngagementConstraintEvaluator |
| 配置 YAML 缺失或解析失败 | 输出明确错误日志，回退到 FixedProfileTrackedRobotBuilder（无约束评估） |
| robot_id 在配置中未定义 | `strict_unknown_reject=true` 时拒绝；`false` 时回退到 "2" |
| 概率增强开关关闭 | fire_advice 使用旧 `armor_width/height`，约束评估器仅用于过滤（不影响 p_hit 公式） |
| 概率增强开关打开 | 使用 profile 中的形状（rect/circle）、hit_area、confidence_weight、朝向约束

---

## 7. 分阶段实施计划

### P1 — 最小可用（预计 3-5 天）

**目标**：YAML 加载 + 组件化数据结构落地，旧行为不变。

1. **新增数据结构文件**
   - `primitive_def.hpp`：`PrimitiveDef` 结构体
   - `robot_assembly_def.hpp`：`RobotAssemblyDef`、`LayoutDef`、`ModulePlacement`
   - 不依赖任何现有类，纯头文件。

2. **YAML Loader**
   - `robot_profile_loader_yaml.cpp`：解析 YAML → `RobotProfileRegistry`
   - 使用 `yaml-cpp`（ROS 2 默认依赖）。
   - Schema 校验：必填字段检查、枚举值校验、引用完整性（`module_ref` 必须在 primitives 中存在）。

3. **布局生成器**
   - `layout_generator.cpp`：实现 `RadialSymmetricGenerator`、`TriLayerZGenerator`
   - 逻辑直接从 `generateArmorsOffsetFromProfile()` 迁移，行为 1:1 一致。
   - 新增 `RotaryDrumGenerator`（buff 结构化表示的基础）。

4. **单元测试**
   - 对每种布局类型验证生成的 offset 与当前 `generateArmorsOffsetFromProfile` 一致。
   - YAML 加载正确性验证。

### P2 — Builder 集成与概率接入（预计 5-7 天）

**目标**：将 config-driven builder 接入机器人构建主链路，概率模块使用 profile 数据。

1. **ConfigDrivenTrackedRobotBuilder**
   - 实现完整 `buildTrackedRobot()`，替代 `FixedProfileTrackedRobotBuilder` 的同 ID 调用。
   - 处理 ambiguous 降级逻辑。

2. **RobotProfileUsage API 落地**
   - 提供 `getEffectiveHitArea()`、`getConfidenceWeight()`、`isKnownParam()` 等静态方法。
   - 在 `RobotDescriptionFacade` 初始化时注入 registry。

3. **RobotDescriptionFacade 初始化改造**
   - 加载 YAML → 注册 config-driven builders → 注册 fallback builders。
   - `strict_unknown_reject` 语义保持。

4. **BuffTargetAdapter 升级**
   - 移除强制 ambiguous 行为，仅在 tracker 降级时设置 ambiguous。
   - big_buff/small_buff 默认输出 structured (5 扇叶)。
   - 聚合 `active_mask`，并提供超时降级策略。
   - 按 `kinematic_constraint` 校验并同步 `center_pose.orientation`（仅 roll 有效）。

5. **fire_advice 概率链路接入**
   - 在 `ProbabilityConfig` 中增加 `enable_profile: false` 开关。
   - 新增 `profile_provider.hpp` 桥接层。
   - 开关关闭时行为完全不变。

### P3 — URDF 扩展与强化（预计 3-4 天）

**目标**：支持 URDF 作为配置源，YAML/URDF 可混合使用。

1. **RobotProfileLoaderUrdf**
   - 从 URDF 的 `<gazebo>` 插件标签或自定义 `<robot_profile>` 标签提取原语和装配信息。
   - 机械尺寸（鼓半径等）可从 URDF joint 信息自动推导。

2. **混合覆盖策略**
   - YAML 定义逻辑属性（confidence_weight、known/unknown 参数）。
   - URDF 定义几何属性（尺寸、偏移、关节）。
   - 加载时合并：URDF 几何覆盖 YAML 对应字段，YAML 语义字段保持不变。

---

## 8. 验收标准

| # | 标准 | 验证方式 |
|---|------|---------|
| 1 | `big_buff`/`small_buff` 默认输出 `representation_mode=STRUCTURED_ROBOT`，`num_armors=5`，`armors_offset` 包含 5 个圆形扇叶的有效偏移 | 单元测试 + 运行时 rostopic echo |
| 2 | big_buff/small_buff 的 PrimitiveDef 形状为 `CIRCLE`，`radius` 字段正确，概率模型使用圆形命中概率公式 | 单元测试 |
| 3 | 旧链路关闭 profile 概率开关时，`fire_advice` 的 p_hit 与 fire_state 与当前完全一致 | 回归测试（录制 bag 回放对比） |
| 4 | 打开 profile 概率增强后，buff 的 `fire_state` 抖动减少（`fire_state` 翻转频率下降 ≥20%） | 统计对比测试 |
| 5 | `EngagementConstraintEvaluator` 正确过滤静止目标（outpost/base 速度阈值=0）vs 运动目标（norm4/sentry）vs 旋转目标（buff 相位约束） | 单元测试每种模式 |
| 6 | 朝向约束：front_hemisphere 模式下，背对自车的模块 confidence_scale=0 | 单元测试 |
| 7 | 旋转相位约束：扇叶背对自车时 `is_engageable=false`，正对时 `is_engageable=true` | 单元测试（模拟 phase 角度） |
| 8 | 激活掩码约束：`big_buff` 任意时刻仅 2/5 扇叶可击打，`small_buff` 仅 1/5 扇叶可击打；掩码超时触发降级策略 | 消息回放 + 单元测试 |
| 9 | YAML 配置错误（缺失必填字段、无效枚举值、dangling 引用）在启动阶段输出明确错误并禁止构建 | 注入错误配置，检查日志 |
| 10 | `strict_unknown_reject=false` 时未知 robot_id 回退到 STANDARD_4 builder，行为不变 | 单元测试 |
| 11 | YAML 配置缺失时整个系统回退到 FixedProfileTrackedRobotBuilder，行为不变 | 删除 YAML 文件后运行回归测试 |
| 12 | 布局生成器输出与当前 `generateArmorsOffsetFromProfile()` 的 norm4/outpost3 结果数值一致（1e-9 容差） | 单元测试 |
| 13 | buff 的 `center_pose.orientation` 满足 roll-only 约束（pitch/yaw 近零，偏差不超过配置阈值） | bag 回放 + 断言检查 |
| 14 | norm4/outpost3 在关闭 buff 外部输入时控制输出与基线一致 | A/B 回归对比 |

---

## 9. 附录：关键设计决策

### A. 为什么用"原语库 + 装配"而非"每个 robot_id 独立定义"？

- **复用性**：`small_armor` 被 standard_4、sentry 共享；`large_armor` 被 hero_4、outpost3 共享。修改原语一处即可影响所有使用者。
- **一致性**：保证同型装甲板在不同机器人上的 hit_area 等属性一致，避免配置漂移。
- **可扩展**：新增机器人只需选择已有原语 + 已有布局策略 + 指定参数，无需重复定义形状尺寸。例如新增 `norm4_v2` 可直接引用 `small_armor` + `radial_symmetric`。

### B. 为什么 buff 不继续沿用"单板 ambiguous"表示？

当前 `num_armors=1 + REP_AMBIGUOUS_SINGLE_ARMOR` 意味着：
- 下游无法区分"这其实是一个有 5 个扇叶的鼓"与"只有一个装甲板的目标"。
- 概率模型无法利用扇叶间距和旋转相位信息优化命中概率。
- selector 不能按模块粒度做目标选择。

升级为 5 扇叶结构化表示后，概率模型可以：
- 知道每个扇叶的独立位置和法向。
- 按 confidence_weight 对扇叶命中概率加权融合。
- 在扇叶旋转时预测哪个扇叶即将进入可击打姿态。

### C. 为什么布局生成器是独立接口而非在 YAML 中写公式？

- 布局生成涉及三角函数计算（cos/sin），不应在配置文件中表达。
- 接口化允许未来新增布局类型而不修改配置格式。
- 单元测试可以独立验证每个生成器的数值正确性。
- 与 URDF 扩展兼容：URDF 的 joint 链可以映射到 `CUSTOM_PLACEMENT` 布局。

### D. 为什么大符/小符必须用圆形模型？

当前 `ProbabilityEngine::hitProbabilityIndependent()` 使用矩形独立维度积分（`probabilityInside1d` 在 u/v 方向分别计算）。这对于矩形装甲板是正确的，但 buff 扇叶是圆形的：

- **矩形模型**：P_hit = P(|e_u| < W/2) × P(|e_v| < H/2)，假设 u/v 独立
- **圆形模型**：P_hit = P(e_u² + e_v² < R²)，即二维卡方分布 CDF

如果用矩形近似圆形扇叶，会导致：
- 矩形外接圆面积大 27%（正方形外接圆为 π/4 ≈ 27%），高估命中概率
- 矩形内接圆面积小 21%（正方形内接圆为 π/4），低估命中概率
- 更关键的是，矩形积分在角部区域（u→W/2, v→H/2）产生非物理的 P_hit 贡献

因此 profile 中必须区分 `shape: rect` vs `shape: circle`，概率引擎据此选择不同的积分策略。

### E. 为什么约束评估在 builder 中而非在 fire_advice 中独立调用？

约束评估分为两层：

1. **Builder 层（几何层）**：在构造 `TrackedRobot` 时评估静态/几何约束（距离、速度阈值、跟踪状态）。这些是"硬约束"——不满足则模块标记为 `is_engageable=false`。结果写入 TrackedRobot 消息，所有下游共享。
2. **FireAdvice 层（时序层）**：在开火决策时评估动态约束（相位窗口、cooldown、朝向余弦）。这些依赖实时自车状态和弹道时间，需要每帧重新计算。

两层分离的收益：
- Builder 的约束结果可在 selector/visualizer 中复用（显示不可击打模块）。
- 避免 fire_advice 对每个候选重复计算不变的几何约束。
- 相位/时序相关约束天然属于 fire_advice 层（需要预测未来相位）。

---

> 本方案基于对 `robot_description_facade.hpp/cpp`、`fixed_profile_tracked_robot_builder.cpp`、`tracked_robot_usage.cpp`、`TrackedRobot.msg`、`probability_engine.hpp`、`buff_target_adapter.cpp` 的完整代码分析。
