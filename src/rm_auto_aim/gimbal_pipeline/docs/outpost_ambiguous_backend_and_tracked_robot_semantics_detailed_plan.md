# OutpostAmbiguousBackend 替换与 TrackedRobot 语义更新 —— 详细实施方案

> 基于 `outpost_ambiguous_backend_and_tracked_robot_semantics_plan.md` 大纲展开，
> 结合当前 `OutpostTrackerV2` / `binder` / `mode` / `robot_description` 实际代码撰写。

---

## 1. 当前实现分析

### 1.1 数据流现状

```
armorsCallback
  → TrackerManager::get_or_create("outpost")
    → OutpostTrackerV2::update(obs)  (use_tracker_v2=true 时)
      1. ObservationFrontend::select_primary_observation(obs)
      2. ObservationFrontend::build_binding_candidate(...)
      3. OutpostBinderBridge::step(...) → BinderOutput + BinderDebugSnapshot
      4. EvidenceFuser::fuse(...) → ModeEvidence
      5. ModeFSM::step(evidence) → ModeDecision
      6. 路由: AmbiguousBackend::update 或 StructuredBackend::update
      7. OutpostOutputAdapter::update_publish_state(ctx, backend_state)
    → FixedProfileTrackedRobotBuilder::buildTrackedRobot(input)
      → 发布 rm_interfaces::msg::TrackedRobot
        → gimbal_controller / ArmorPositionCalculator
          → calculateArmorWorldPositions via TrackedRobotUsage
```

### 1.2 问题定位（精确到文件/类）

| 问题 | 位置 | 表现 |
|------|------|------|
| 滤波语义不一致 | `outpost_ambiguous_backend.cpp:62-83` | `refresh_center_snapshot()` 将单板状态通过 `radius`/`z_offsets`/`panel_angles` 反向拟合为中心态，语义混杂 |
| 后端耦合 outpost 几何 | `outpost_ambiguous_backend.hpp:32-33` | `z_offsets_` 和 `panel_angles_` 硬编码为 3 板 outpost 专用 |
| 上层无单板感知 | `fixed_profile_tracked_robot_builder.cpp:73-103` | `buildTrackedRobot` 始终按 center + offsets 模式填充，不区分 ambiguous/structured |
| controller 无分叉 | `armor_position_calculator.cpp:24-55` | `calculate()` 直接调用 `calculateArmorWorldPositionsEigen`，无单板路径 |
| 发布语义靠约定 | `base_tracker.hpp:62-63` | `is_ambiguous_single_mode()` 返回 bool，但无配套的 "如何解释 center_* 字段" 机制 |

### 1.3 当前 `BaseTracker` 接口中已存在的语义钩子

```cpp
// base_tracker.hpp - 已定义但下游未充分使用的接口
virtual bool is_ambiguous_single_mode() const;   // 仅 OutpostArmorTracker / V2 重写为 true
virtual int effective_num_armors() const;         // 返回 0 表示用 profile 默认值
virtual double confidence_scale() const;          // ambiguous 下返回 <1.0
virtual std::vector<Pose> build_armors_offset_for_message() const;  // 可返回单板 offset
```

这些接口已经有语义区分意图，但缺少统一的解释层（即 `TrackedRobotUsage` 侧应读取它们并做语义分叉）。

---

## 2. 目标架构

### 2.1 目标调用链

```text
armorsCallback
  → TrackerManager
    → OutpostTrackerV2 (orchestrator, 不变)
      → AMBIGUOUS: OutpostAmbiguousBackend (改造后)
          → AmbiguousSingleArmorFilterAdapter (新增，通用单板 IMM 适配器)
            → SingleArmorIMMTracker (新增，kalmanFilters 层)
          → 输出 AmbiguousArmorSnapshot (新增，单板真值)
      → OutpostOutputAdapter (改造，在 AMBIGUOUS 下优先用单板快照填充 ctx)
    → FixedProfileTrackedRobotBuilder (改造，根据 is_ambiguous_single_mode 分叉)
      → TrackedRobotUsage 语义层 (改造，新增 RepresentationMode 判别)
        → rm_interfaces::msg::TrackedRobot (Phase A 不改消息定义)

  → gimbal_controller / ArmorPositionCalculator (改造，根据表示模式分叉)
      → 单板: 返回 {single_armor_position}
      → 多板: 现有 center + offsets 逻辑
    → ArmorSelector / FireAdviceEngine (复用，单候选退化)
```

### 2.2 模块分层与职责边界

```
┌──────────────────────────────────────────────────────────┐
│ 控制层 (gimbal_controller)                               │
│ ArmorPositionCalculator: isSingleArmorRepresentation?    │
│   → 单板路径  /  多板路径                                │
└───────────────────────┬──────────────────────────────────┘
                        │ rm_interfaces::msg::TrackedRobot
┌───────────────────────┴──────────────────────────────────┐
│ 语义层 (robot_description)                               │
│ TrackedRobotUsage: RepresentationMode 判别 + 访问器      │
│ FixedProfileTrackedRobotBuilder: 按模式填充 TrackedRobot │
└───────────────────────┬──────────────────────────────────┘
                        │ BaseTracker 接口
┌───────────────────────┴──────────────────────────────────┐
│ 编排层 (OutpostTrackerV2)                                │
│ 模式路由 + 后端选择 + 输出适配                            │
└───────┬───────────────────────┬──────────────────────────┘
        │ AMBIGUOUS             │ STRUCTURED
┌───────┴──────────┐  ┌────────┴─────────────────┐
│ AmbiguousBackend │  │ StructuredBackend        │
│ (适配层)         │  │ (OutpostSpinUKF 包装)    │
│   → Adapter      │  └──────────────────────────┘
│     → IMM Core   │
└──────────────────┘
```

---

## 3. Phase A 详细设计（无消息变更，推荐首落）

### 3.1 模块 1：通用单装甲板滤波核心（kalmanFilters 层）

#### 3.1.1 动机

当前 `OutpostAmbiguousKF` 是 outpost 专用的 CV-KF（6+2 维），新设计需要 IMM 以应对前哨站变速旋转场景。将 IMM core 放在 `kalmanFilters` 层便于后续复用至 `AdaptiveArmorTracker` 的 ambiguous 分支。

#### 3.1.2 新建文件

**`src/kalmanFilters/filters/combined_models/include/combined_models/SingleArmorIMMTracker.h`**

```cpp
namespace fyt::auto_aim::kalman {

struct SingleArmorIMMConfig {
  double dt = 0.05;

  // XY 子模型
  bool enable_cv = true;
  bool enable_ca = true;
  bool enable_ctrv = false;  // 可选

  // 过程噪声
  double q_cv_xy = 0.5;
  double q_ca_xy = 1.0;

  // Z 轴 1D KF
  double q_cv_z  = 0.3;
  double q_ca_z  = 0.8;

  // Yaw 1D KF
  double q_cv_yaw = 0.3;
  double q_ca_yaw = 0.6;

  // Markov 转移概率
  double p_stay_cv  = 0.85;
  double p_enter_ca = 0.15;
  double p_stay_ca  = 0.85;
  double p_exit_ca  = 0.15;

  // 观测噪声
  double r_pos_base  = 0.01;
  double r_yaw_base  = 0.03;
};

struct SingleArmorIMMState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  double yaw = 0.0;
  double yaw_rate = 0.0;
  bool initialized = false;
};

class SingleArmorIMMTracker {
 public:
  explicit SingleArmorIMMTracker(const SingleArmorIMMConfig &cfg);

  void initialize(const Eigen::Vector3d &pos, double yaw);
  void predict(double dt);
  void update(const Eigen::Vector3d &pos_meas, double yaw_meas,
              double pos_conf = 1.0, double yaw_conf = 1.0);

  const SingleArmorIMMState &state() const;
  bool initialized() const;

  // IMM 诊断输出
  std::array<double, 3> model_probabilities() const;  // CV / CA / CTRV

 private:
  // IMM 子滤波器实现细节
  // ...
};

}  // namespace fyt::auto_aim::kalman
```

**`src/kalmanFilters/filters/combined_models/src/SingleArmorIMMTracker.cpp`**

实现要点：
1. XY IMM with 2 或 3 子模型（CV/CA/CTRV），每个子模型为 4 维 KF `[x, vx, y, vy]`
2. Z 独立 1D KF `[z, vz]`，CA 模型
3. Yaw 独立 1D KF `[yaw, yaw_rate]`，CV 模型 + angle unwrapping
4. Markov 转移矩阵 + 模型概率混合输出
5. `update()` 中 `pos_conf`/`yaw_conf` 缩放观测噪声 R

#### 3.1.3 与当前 `OutpostAmbiguousKF` 的关系

`SingleArmorIMMTracker` 是纯滤波算法，**不依赖** outpost 几何（radius, z_offsets, panel_angles），不依赖 `UnifiedConfig`，仅依赖 `SingleArmorIMMConfig`。这是替换的关键差异点。

---

### 3.2 模块 2：AmbiguousSingleArmorFilterAdapter（gimbal_pipeline 适配层）

#### 3.2.1 职责

1. 将 `ObservationData` + `UnifiedConfig` 转换为 `SingleArmorIMMTracker` 的输入
2. 对 `SingleArmorIMMTracker` 做生命周期管理（init/predict/update）
3. 输出统一的单板状态（position/velocity/yaw/yaw_rate）
4. 提供与当前 `OutpostAmbiguousKF` 等价的接口，实现对 `OutpostAmbiguousBackend` 的**无痛替换**
5. 通过 `ambiguous_backend_use_imm_adapter` 开关控制是否使用 IMM 核心（未开启时内部包装 `OutpostAmbiguousKF` 作为回退）

#### 3.2.2 新建文件

**`include/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp`**

```cpp
namespace fyt::auto_aim {

class AmbiguousSingleArmorFilterAdapter {
 public:
  explicit AmbiguousSingleArmorFilterAdapter(const UnifiedConfig &config,
                                              double dt = 0.05);

  void initialize(const ObservationData &obs);
  void predict(double dt);
  void update(const ObservationData &obs,
              double position_confidence = 1.0,
              double yaw_confidence = 1.0);

  bool initialized() const;

  Eigen::Vector3d armor_position() const;
  Eigen::Vector3d armor_velocity() const;
  double armor_yaw() const;
  double armor_yaw_rate() const;

 private:
  void build_imm_config_from_unified();
  double clamp_conf(double c);
  double unwrap_yaw(double yaw_meas);

  UnifiedConfig config_;
  double dt_ = 0.05;

  // 开关控制
  bool use_imm_ = false;

  // 回退：当前 OutpostAmbiguousKF
  std::unique_ptr<OutpostAmbiguousKF> legacy_kf_;

  // 新核心：通用 IMM
  std::unique_ptr<kalman::SingleArmorIMMTracker> imm_tracker_;
  kalman::SingleArmorIMMConfig imm_cfg_;
};

}  // namespace fyt::auto_aim
```

**`src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp`**

实现关键逻辑：

```cpp
void AmbiguousSingleArmorFilterAdapter::update(
    const ObservationData &obs, double pos_conf, double yaw_conf) {
  if (use_imm_) {
    // 从 ObservationData 提取单板位姿
    Eigen::Vector3d pos(obs.armor_position.x(), obs.armor_position.y(), obs.armor_position.z());
    double yaw = obs.armor_yaw;
    imm_tracker_->update(pos, yaw, clamp_conf(pos_conf), clamp_conf(yaw_conf));
  } else {
    legacy_kf_->update(obs, pos_conf, yaw_conf);
  }
}
```

#### 3.2.3 设计细节

- `build_imm_config_from_unified()` 从 `UnifiedConfig::OutpostParameters` 和 `UnifiedConfig::MotionModelParameters` 中提取参数构建 `SingleArmorIMMConfig`
- `use_imm_` 开关读取 `config_.outpost.ambiguous_backend_use_imm_adapter`
- 接口与 `OutpostAmbiguousKF` 完全对齐（相同方法名+签名），确保替换 `OutpostAmbiguousBackend` 内部成员时的 diff 最小

---

### 3.3 模块 3：改造 OutpostAmbiguousBackend

#### 3.3.1 目标改动

1. **内部成员替换**：`OutpostAmbiguousKF kf_` → `AmbiguousSingleArmorFilterAdapter filter_`
2. **新增单板真值缓存**：在 `refresh_center_snapshot()` 同步计算并缓存 `armor_pos`/`armor_vel`/`armor_yaw`/`armor_yaw_rate`
3. **新增扩展快照结构**：`AmbiguousArmorSnapshot`，对外暴露单板原始状态
4. **保留 `BackendStateSnapshot` 接口**：`snapshot()` 仍返回 center-centric 快照，兼容现有 `OutpostOutputAdapter` / `OutpostTrackerV2` 调用

#### 3.3.2 改造后的头文件

**修改 `include/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp`**：

```cpp
namespace fyt::auto_aim::outpost_v2 {

// 新增：单板原始状态快照
struct AmbiguousArmorSnapshot {
  Eigen::Vector3d armor_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_vel = Eigen::Vector3d::Zero();
  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;
  int panel_id = -1;
  double confidence = 1.0;
};

class OutpostAmbiguousBackend : public IOutpostBackend {
 public:
  explicit OutpostAmbiguousBackend(const UnifiedConfig &cfg);

  // IOutpostBackend 接口（不变）
  void reset(const ObservationData &obs, int panel_id) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

  // 新增：单板原始状态访问
  const AmbiguousArmorSnapshot &ambiguous_snapshot() const;

 private:
  int sanitize_panel_id(int panel_id) const;
  void refresh_center_snapshot();

  UnifiedConfig cfg_;

  // 核心改动：OutpostAmbiguousKF → AmbiguousSingleArmorFilterAdapter
  AmbiguousSingleArmorFilterAdapter filter_;

  int current_panel_id_ = 0;
  BackendStateSnapshot state_;
  AmbiguousArmorSnapshot armor_snap_;  // 新增

  // outpost 几何参数（仅在 center<->armor 转换时使用，不再参与滤波）
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{0.06, 0.0, -0.06};
  std::array<double, 3> panel_angles_{0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 3.3.3 改造后的实现要点

**修改 `src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp`**：

```cpp
void OutpostAmbiguousBackend::refresh_center_snapshot() {
  if (!filter_.initialized()) return;

  // 1. 缓存单板原始状态
  armor_snap_.armor_pos = filter_.armor_position();
  armor_snap_.armor_vel = filter_.armor_velocity();
  armor_snap_.armor_yaw = filter_.armor_yaw();
  armor_snap_.armor_yaw_rate = filter_.armor_yaw_rate();
  armor_snap_.panel_id = sanitize_panel_id(current_panel_id_);

  // 2. 兼容：回推为中心态（仅用于 BackendStateSnapshot）
  const int pid = armor_snap_.panel_id;
  state_.panel_id = pid;

  state_.center_pos.x() = armor_snap_.armor_pos.x() - radius_ * std::cos(armor_snap_.armor_yaw);
  state_.center_pos.y() = armor_snap_.armor_pos.y() - radius_ * std::sin(armor_snap_.armor_yaw);
  state_.center_pos.z() = armor_snap_.armor_pos.z() - z_offsets_[pid];

  state_.center_yaw = normalize_angle(armor_snap_.armor_yaw - panel_angles_[pid]);
  state_.yaw_rate = armor_snap_.armor_yaw_rate;

  const Eigen::Vector3d tangential(
      -armor_snap_.armor_yaw_rate * radius_ * std::sin(armor_snap_.armor_yaw),
       armor_snap_.armor_yaw_rate * radius_ * std::cos(armor_snap_.armor_yaw), 0.0);
  state_.center_vel = armor_snap_.armor_vel - tangential;
}
```

**关键设计决策**：
- `radius_`/`z_offsets_`/`panel_angles_` 仍然留在 `OutpostAmbiguousBackend` 中，因为它们是 outpost **适配层**需要的几何知识
- 但它们不再被传入滤波核心（`filter_` 内部不依赖这些参数）
- 这样 `AmbiguousSingleArmorFilterAdapter` → `SingleArmorIMMTracker` 可用于其他机器人类型

---

### 3.4 模块 4：OutpostOutputAdapter 改造

#### 3.4.1 目标

在 AMBIGUOUS 模式下，`update_publish_state()` 优先使用 `AmbiguousArmorSnapshot`（单板真值）填充 `OutpostRuntimeContext`，而不是使用 center-centric 的 `BackendStateSnapshot`。

#### 3.4.2 现状

当前 `OutpostOutputAdapter::update_publish_state()` 接收 `BackendStateSnapshot`，始终按中心态填充 ctx：
```cpp
ctx.center_pos = snap.center_pos;   // center-centric
ctx.center_vel = snap.center_vel;
ctx.center_yaw = snap.center_yaw;
ctx.yaw_rate = snap.yaw_rate;
```

#### 3.4.3 改造方案

需修改 `OutpostOutputAdapter` 的签名，使其在 AMBIGUOUS 下接收额外参数：

**方案 A**（推荐）：在 `OutpostOutputAdapter::update_publish_state` 增加可选参数

```cpp
struct PublishStateInput {
  const BackendStateSnapshot *backend_snap = nullptr;
  const AmbiguousArmorSnapshot *armor_snap = nullptr;  // optional, AMBIGUOUS only
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;
};

void OutpostOutputAdapter::update_publish_state(
    OutpostRuntimeContext *ctx, const PublishStateInput &input) const;
```

实现分叉逻辑：
```cpp
if (input.mode == mode::TrackMode::AMBIGUOUS && input.armor_snap != nullptr) {
  // 单板语义：center 字段填充单板状态
  ctx->publish_pos = input.armor_snap->armor_pos;
  ctx->publish_vel = input.armor_snap->armor_vel;
  ctx->center_yaw = input.armor_snap->armor_yaw;
  ctx->yaw_rate  = input.armor_snap->armor_yaw_rate;
  ctx->selected_panel_id = input.armor_snap->panel_id;
} else if (input.backend_snap != nullptr) {
  // 结构化语义：center 字段填充中心状态
  ctx->publish_pos = input.backend_snap->center_pos;
  ctx->publish_vel = input.backend_snap->center_vel;
  ctx->center_yaw   = input.backend_snap->center_yaw;
  ctx->yaw_rate     = input.backend_snap->yaw_rate;
}
```

这保持了向后兼容——`OutpostStructuredBackend` 不需要提供 `AmbiguousArmorSnapshot`。

---

### 3.5 模块 5：TrackedRobot 语义层（`TrackedRobotUsage` 扩展）

#### 3.5.1 新增：表示模式枚举与判别

**修改 `include/gimbal_pipeline/common/robot_description/robot_description_facade.hpp`**，在 `TrackedRobotUsage` 中新增：

```cpp
class TrackedRobotUsage {
 public:
  // === 新增：表示模式 ===
  enum class RepresentationMode {
    STRUCTURED_ROBOT = 0,         // 完整机器人中心 + 多装甲板
    AMBIGUOUS_SINGLE_ARMOR = 1,   // 单装甲板降级表示
    UNKNOWN = 2,
  };

  // 根据 TrackedRobot 消息字段推断表示模式
  // 规则：
  //   - num_armors == 1 && armors_offset.size() <= 1 → AMBIGUOUS_SINGLE_ARMOR
  //   - num_armors >= 3 && armors_offset.size() >= 3 → STRUCTURED_ROBOT
  //   - 其他 → UNKNOWN
  static RepresentationMode inferRepresentationMode(
      const rm_interfaces::msg::TrackedRobot &robot);

  static bool isSingleArmorRepresentation(
      const rm_interfaces::msg::TrackedRobot &robot);

  // 单板语义访问器：当 isSingleArmorRepresentation 为 true 时，
  // center_position / center_velocity / yaw / yaw_velocity 被解释为单板状态
  static Eigen::Vector3d singleArmorPosition(
      const rm_interfaces::msg::TrackedRobot &robot);
  static Eigen::Vector3d singleArmorVelocity(
      const rm_interfaces::msg::TrackedRobot &robot);
  static double singleArmorYaw(
      const rm_interfaces::msg::TrackedRobot &robot);

  // === 已有接口不变 ===
  // ...
};
```

#### 3.5.2 实现

**修改 `src/common/robot_description/tracked_robot_usage.cpp`**：

```cpp
TrackedRobotUsage::RepresentationMode TrackedRobotUsage::inferRepresentationMode(
    const rm_interfaces::msg::TrackedRobot &robot) {
  if (robot.num_armors == 1 && robot.armors_offset.size() <= 1) {
    return RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
  }
  if (robot.num_armors >= 3 && robot.armors_offset.size() >= 3) {
    return RepresentationMode::STRUCTURED_ROBOT;
  }
  // 回退：基于 robot_type 判别
  if (robot.robot_type == rm_interfaces::msg::TrackedRobot::OUTPOST_3) {
    // OUTPOST 默认按单板处理（安全性优先）
    return RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
  }
  return RepresentationMode::STRUCTURED_ROBOT;
}

bool TrackedRobotUsage::isSingleArmorRepresentation(
    const rm_interfaces::msg::TrackedRobot &robot) {
  return inferRepresentationMode(robot) == RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
}

// 单板语义下，center_* 字段按 single armor 解释
Eigen::Vector3d TrackedRobotUsage::singleArmorPosition(
    const rm_interfaces::msg::TrackedRobot &robot) {
  return centerPosition(robot);  // 语义等价，但调用方意图明确
}

Eigen::Vector3d TrackedRobotUsage::singleArmorVelocity(
    const rm_interfaces::msg::TrackedRobot &robot) {
  return linearVelocity(robot);
}

double TrackedRobotUsage::singleArmorYaw(
    const rm_interfaces::msg::TrackedRobot &robot) {
  return yaw(robot);
}
```

---

### 3.6 模块 6：FixedProfileTrackedRobotBuilder 改造

#### 3.6.1 目标

在 ambiguous 模式下，发布 `num_armors=1`，`armors_offset` 只含单板偏移（或零偏移），`center_*` 字段填充单板状态。

#### 3.6.2 改造点

**修改 `src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp`** 中的 `buildTrackedRobot()` 方法：

在填充 `msg.center_position` / `msg.center_velocity` / `msg.yaw` 的逻辑附近添加分叉：

```cpp
// === 在现有 center_position 填充逻辑处插入分叉 ===

if (input.tracker.is_ambiguous_single_mode()) {
  // ── AMBIGUOUS 单板语义 ──
  // center_* 字段直接填充单板状态（与 BaseTracker 输出的语义对齐）
  const auto pos = input.tracker.get_center_position();
  msg.center_position.x = pos.x();
  msg.center_position.y = pos.y();
  msg.center_position.z = pos.z();
  const auto pub_vel = input.tracker.get_publish_velocity();
  msg.center_velocity.x = pub_vel.x();
  msg.center_velocity.y = pub_vel.y();
  msg.center_velocity.z = pub_vel.z();
  msg.yaw = input.tracker.get_yaw();

  // 单板模式下 num_armors = 1
  msg.num_armors = 1;

  // armors_offset 只发布单板偏移（优先使用 tracker 提供的 runtime offsets）
  const auto runtime_offsets = input.tracker.build_armors_offset_for_message();
  if (!runtime_offsets.empty()) {
    // tracker 已提供正确的单板 offset
    msg.armors_offset = runtime_offsets;
  } else {
    // fallback: 零偏移单板
    geometry_msgs::msg::Pose single_offset;
    single_offset.position.x = 0.0;
    single_offset.position.y = 0.0;
    single_offset.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    single_offset.orientation = tf2::toMsg(q);
    msg.armors_offset = {single_offset};
  }

  // 单板模式下 radius 填 0（无意义）
  msg.radius = 0.0;
  msg.radius_2 = 0.0;
  msg.d_za = 0.0;
  msg.d_zc = 0.0;

} else {
  // ── STRUCTURED 完整机器人语义（现有逻辑不变）──
  // ... 现有代码 ...
}

// 置信度缩放（已有逻辑，ambiguous 下 confidence_scale 返回 <1.0）
msg.confidence *= input.tracker.confidence_scale();
```

**注意**：`input.tracker.get_center_position()` 在 `OutpostTrackerV2` 中需要保证在 AMBIGUOUS 下返回的是**单板位置**而非中心位置。这需要在 `OutpostTrackerV2` 中确保：
- AMBIGUOUS: `get_center_position()` 返回 `ctx_.publish_pos`（在 output_adapter 改造后是单板位置）
- STRUCTURED: `get_center_position()` 返回 `ctx_.center_pos`（中心位置）

---

### 3.7 模块 7：ArmorPositionCalculator 分叉

#### 3.7.1 目标

在 `AmorPositionCalculator` 中检测单板表示模式，走专用的单板路径，避免错误使用 center + offsets 逻辑。

#### 3.7.2 改造方案

**修改 `src/rm_auto_aim/gimbal_pipeline/src/gimbal_controller/armor_position_calculator.cpp`**：

```cpp
std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculate(
    const rm_interfaces::msg::TrackedRobot &robot) const {

  const auto normalized_robot =
      fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(robot);

  // === 新增：单板表示模式检测 ===
  if (fyt::auto_aim::robot_description::TrackedRobotUsage::isSingleArmorRepresentation(
          normalized_robot)) {
    // 单板路径：center_position 即单板世界坐标，无需 offset 变换
    const Eigen::Vector3d armor_pos =
        fyt::auto_aim::robot_description::TrackedRobotUsage::singleArmorPosition(
            normalized_robot);
    return {armor_pos};  // 单元素向量
  }

  // === 现有多板路径不变 ===
  // ... 使用 TrackedRobotUsage::calculateArmorWorldPositionsEigen ...
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculatePredicted(
    const rm_interfaces::msg::TrackedRobot &robot, double dt) const {

  const auto normalized_robot =
      fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(robot);

  // === 新增：单板预测 ===
  if (fyt::auto_aim::robot_description::TrackedRobotUsage::isSingleArmorRepresentation(
          normalized_robot)) {
    const Eigen::Vector3d armor_pos =
        fyt::auto_aim::robot_description::TrackedRobotUsage::singleArmorPosition(
            normalized_robot);
    const Eigen::Vector3d armor_vel =
        fyt::auto_aim::robot_description::TrackedRobotUsage::singleArmorVelocity(
            normalized_robot);
    return {armor_pos + armor_vel * dt};
  }

  // === 现有多板预测路径不变 ===
  // ...
}
```

#### 3.7.3 下游影响分析

| 下游模块 | 影响 | 处理方式 |
|----------|------|----------|
| `ArmorSelector` | 候选集退化为 1 个 | 天然兼容，选板逻辑退化为直接选第一个 |
| `CurrentPositionStrategy` | 目标位置为单板坐标 | 天然兼容 |
| `PredictedPositionStrategy` | 预测位置为单板坐标 | 天然兼容 |
| `FireAdviceEngine` | 单候选 | 需要确保面向过滤（facing filter）在单板下可配置关闭；不再做多板比较 |
| `BestArmorStrategy` | 多板择优 | 单候选下退化为恒等 |

**需确认**：`FireAdviceEngine` 中的面向过滤逻辑是否依赖装甲板朝向（从 offset pose 计算）。在 AMBIGUOUS 下，单板 offset 为 (0,0,0)，朝向数据可能不可靠。需增加配置项允许关闭面向过滤。

---

### 3.8 模块 8：OutpostTrackerV2 适配改动

#### 3.8.1 变更点

`OutpostTrackerV2` 主编排器需要配合以下变更：

1. **更新 `update_publish_state` 调用**：传递 `PublishStateInput` 而非仅 `BackendStateSnapshot`
2. **确保 `get_center_position()` 语义正确**：
   - AMBIGUOUS: 返回 `ctx_.publish_pos`（单板位置）
   - STRUCTURED: 返回 `ctx_.center_pos`（中心位置）

这需要在 `sync_runtime_from_backend()` 和 `update()` 中做对应调整。

#### 3.8.2 具体改动

**`src/max_entropy_tracker/trackers/outpost_tracker_v2.cpp`** 中的 `update()` 末尾：

```cpp
// 步骤 8: 输出适配
void OutpostTrackerV2::update_publish_cache() {
  if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
    const auto &armor_snap = ambiguous_backend_.ambiguous_snapshot();
    outpost_v2::PublishStateInput input;
    input.mode = mode::TrackMode::AMBIGUOUS;
    input.armor_snap = &armor_snap;
    output_adapter_.update_publish_state(&ctx_, input);
  } else {
    const auto &backend_snap = structured_backend_.snapshot();
    outpost_v2::PublishStateInput input;
    input.mode = mode::TrackMode::STRUCTURED;
    input.backend_snap = &backend_snap;
    output_adapter_.update_publish_state(&ctx_, input);
  }
}
```

以及 `get_center_position()` 中的语义路由（需确认当前实现是否已经正确）。

---

### 3.9 模块 9：配置扩展

#### 3.9.1 新增配置项

**修改 `include/max_entropy_tracker/core/config.hpp`** 的 `OutpostParameters`：

```cpp
struct OutpostParameters {
  // ... 现有字段 ...

  // === 新增：ambiguous 语义与后端控制 ===
  bool ambiguous_publish_single_armor_semantics = true;  // 发布侧启用单板语义
  bool ambiguous_single_armor_zero_offset = true;        // 单板 offset 填 (0,0,0)
  bool ambiguous_backend_use_imm_adapter = false;        // 灰度：IMM 核心开关
};
```

#### 3.9.2 YAML 参数暴露

**修改 `src/rm_bringup/config/leg/gimbal_pipeline.yaml`** 的 `outpost` 段：

```yaml
outpost:
  use_tracker_v2: true

  # === 新增 ===
  ambiguous_publish_single_armor_semantics: true
  ambiguous_single_armor_zero_offset: true
  ambiguous_backend_use_imm_adapter: false
```

#### 3.9.3 参数读取

**修改 `src/rm_auto_aim/gimbal_pipeline/src/gimbal_pipeline_node.cpp`**：

```cpp
// 在声明区新增
declare_parameter("outpost.ambiguous_publish_single_armor_semantics", true);
declare_parameter("outpost.ambiguous_single_armor_zero_offset", true);
declare_parameter("outpost.ambiguous_backend_use_imm_adapter", false);

// 在读取区新增
c.outpost.ambiguous_publish_single_armor_semantics =
    get_parameter("outpost.ambiguous_publish_single_armor_semantics").as_bool();
c.outpost.ambiguous_single_armor_zero_offset =
    get_parameter("outpost.ambiguous_single_armor_zero_offset").as_bool();
c.outpost.ambiguous_backend_use_imm_adapter =
    get_parameter("outpost.ambiguous_backend_use_imm_adapter").as_bool();
```

---

### 3.10 模块 10：FireAdviceEngine 适配

#### 3.10.1 需要确认的风险点

在 `gimbal_controller` 中（`src/rm_auto_aim/gimbal_controller/` 独立包或 `gimbal_pipeline` 包内），`FireAdviceEngine` 的"面向过滤"逻辑可能依赖装甲板法向量（从 offset pose 的 orientation 中提取）。

单板模式下 offset pose 的 orientation 可能是 (0,0,0,1)，这会给出错误的法向量。

#### 3.10.2 建议改动

在 `FireAdviceEngine` 的面向过滤逻辑中增加条件：

```cpp
// 如果只有一个候选且处于单板表示模式，弱化或跳过面向过滤
if (candidates.size() == 1 &&
    TrackedRobotUsage::isSingleArmorRepresentation(robot)) {
  // 跳过面向过滤，或使用宽松阈值
  facing_threshold = std::cos(90.0 * M_PI / 180.0);  // 90° 全通
}
```

**注意**：此改动需要确认 `FireAdviceEngine` 的实际文件位置后再实施。当前 `gimbal_controller` 有两个位置：
- `src/rm_auto_aim/gimbal_controller/`（独立包）
- `src/rm_auto_aim/gimbal_pipeline/src/gimbal_controller/`（gimbal_pipeline 内）

---

## 4. 文件级变更清单

### 4.1 新增文件

| # | 文件路径 | 职责 | Phase |
|---|---------|------|-------|
| 1 | `src/kalmanFilters/.../SingleArmorIMMTracker.h` | 通用单板 IMM 滤波核心（kalmanFilters 层） | 3 |
| 2 | `src/kalmanFilters/.../SingleArmorIMMTracker.cpp` | IMM 核心实现 | 3 |
| 3 | `include/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp` | gimbal_pipeline 适配器 | 1 |
| 4 | `src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp` | 适配器实现 | 1 |

### 4.2 修改文件

| # | 文件路径 | 改动内容 | Phase |
|---|---------|---------|-------|
| 5 | `include/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp` | 新增 `AmbiguousArmorSnapshot`、替换 `kf_` 为 `filter_`、新增 `ambiguous_snapshot()` | 1 |
| 6 | `src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp` | `refresh_center_snapshot()` 同步缓存单板状态；构造函数/Method 适配 adapter | 1 |
| 7 | `include/max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.hpp` | 新增 `PublishStateInput` 结构；修改 `update_publish_state` 签名 | 2 |
| 8 | `src/max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.cpp` | 实现 AMBIGUOUS/STRUCTURED 分叉填充逻辑 | 2 |
| 9 | `include/gimbal_pipeline/common/robot_description/robot_description_facade.hpp` | `TrackedRobotUsage` 新增 `RepresentationMode` 枚举、`inferRepresentationMode`、`isSingleArmorRepresentation`、单板访问器 | 2 |
| 10 | `src/common/robot_description/tracked_robot_usage.cpp` | 实现新增的表示模式判别和单板访问器 | 2 |
| 11 | `src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp` | `buildTrackedRobot()` 增加 `is_ambiguous_single_mode()` 分叉 | 2 |
| 12 | `src/gimbal_controller/armor_position_calculator.cpp` | `calculate()` / `calculatePredicted()` 增加 `isSingleArmorRepresentation` 单板路径 | 2 |
| 13 | `src/max_entropy_tracker/trackers/outpost_tracker_v2.cpp` | `update_publish_cache()` 传递 `PublishStateInput`；确认 `get_center_position()` 语义路由 | 1-2 |
| 14 | `include/max_entropy_tracker/core/config.hpp` | `OutpostParameters` 新增 3 个配置字段 | 1 |
| 15 | `src/gimbal_pipeline_node.cpp` | 声明 + 读取新增参数 | 1 |
| 16 | `src/rm_bringup/config/leg/gimbal_pipeline.yaml` | 暴露新增参数 | 1 |
| 17 | `FireAdviceEngine` 对应文件 | 单板下面向过滤弱化 | 2 |
| 18 | `CMakeLists.txt`（gimbal_pipeline） | 添加新 .cpp 编译目标 | 1,3 |

---

## 5. 分阶段执行计划

### Phase 1：后端适配层落地（不动发布语义）

**目标**：引入 `AmbiguousSingleArmorFilterAdapter`，改造 `OutpostAmbiguousBackend` 使用 adapter（先用 legacy KF 路径），补齐配置开关。**对外行为不变**。

**具体步骤**：

1. 创建 `ambiguous_single_armor_filter_adapter.hpp/cpp`
   - 内部包装 `OutpostAmbiguousKF`（`use_imm_ = false` 路径）
   - 接口与 `OutpostAmbiguousKF` 对齐
2. 修改 `outpost_ambiguous_backend.hpp`
   - 新增 `AmbiguousArmorSnapshot` 结构体
   - 替换 `OutpostAmbiguousKF kf_` 为 `AmbiguousSingleArmorFilterAdapter filter_`
   - 新增 `AmbiguousArmorSnapshot armor_snap_` 缓存
   - 新增 `ambiguous_snapshot()` 方法
3. 修改 `outpost_ambiguous_backend.cpp`
   - `refresh_center_snapshot()` 同时写入 `armor_snap_`
   - 构造函数传参适配
   - `predict/update/reset` 调用适配
4. 修改 `config.hpp` — 新增 3 个开关字段
5. 修改 `gimbal_pipeline_node.cpp` — 声明和读取新参数
6. 修改 `gimbal_pipeline.yaml` — 暴露默认值
7. **验证**：编译通过，`use_tracker_v2=true` 行为不变（回放对比）

### Phase 2：TrackedRobot 语义收敛 + Controller 分叉

**目标**：发布侧正确标注单板语义，控制侧走单板路径。通过开关控制新旧语义。

**具体步骤**：

1. 修改 `robot_description_facade.hpp` — 新增 `RepresentationMode`、`inferRepresentationMode`、`isSingleArmorRepresentation`、单板访问器
2. 修改 `tracked_robot_usage.cpp` — 实现上述方法
3. 修改 `outpost_output_adapter.hpp/cpp` — 新增 `PublishStateInput`，实现分叉填充
4. 修改 `outpost_tracker_v2.cpp` — `update_publish_cache()` 传递 `PublishStateInput`
5. 修改 `fixed_profile_tracked_robot_builder.cpp` — `is_ambiguous_single_mode()` 分叉发布
6. 修改 `armor_position_calculator.cpp` — 单板路径分叉
7. 修改 `FireAdviceEngine` — 单板下面向过滤处理
8. **验证**：
   - `ambiguous_publish_single_armor_semantics=false` → 行为不变
   - `ambiguous_publish_single_armor_semantics=true` → 单板路径生效
   - 回放对比 center jitter、选板连续性

### Phase 3：IMM 核心接入（灰度）

**目标**：将 adapter 后端从 legacy KF 切换到 `SingleArmorIMMTracker`，灰度验证滤波性能。

**具体步骤**：

1. 创建 `SingleArmorIMMTracker.h/cpp`（kalmanFilters 层）
2. 修改 `AmbiguousSingleArmorFilterAdapter` — 实例化并使用 IMM tracker（`use_imm_ = true` 路径）
3. 更新 CMakeLists.txt
4. 开启 `ambiguous_backend_use_imm_adapter=true` 进行灰度测试
5. **验证**：
   - yaw 连续性（unwrap 正确）
   - center 抖动量对比（IMM vs KF）
   - 低速/静止场景稳定性
   - 切换时瞬态行为

### Phase 4（可选）：消息增强

**目标**：评估是否需要 `representation_mode` 字段跨节点显式传递。

**触发条件**：
- 有非 outpost 的目标也需要 single-armor 表示
- 跨节点语义推断出现歧义
- 需要升级 `rm_interfaces` 包

---

## 6. 详细测试计划

### 6.1 单元测试

| 测试模块 | 测试文件（建议） | 测试点 |
|----------|-----------------|--------|
| `AmbiguousSingleArmorFilterAdapter` | `test_ambiguous_adapter.cpp` | (1) initialize/predict/update 等价于 `OutpostAmbiguousKF`; (2) `use_imm_` 开关; (3) 空观测保护 |
| `TrackedRobotUsage::inferRepresentationMode` | `test_representation_mode.cpp` | (1) num_armors=1 + single offset → AMBIGUOUS; (2) num_armors=3 + 3 offset → STRUCTURED; (3) 边界: 空 offset/缺字段 |
| `ArmorPositionCalculator` | `test_armor_position_calc.cpp` | (1) single 模式输出 1 候选; (2) structured 模式输出 N 候选; (3) 预测位置计算正确 |
| `SingleArmorIMMTracker` | `test_single_armor_imm.cpp` | (1) yaw unwrap 连续性（跨 ±π 跳变）; (2) dt 变化鲁棒性; (3) 静止收敛 |

### 6.2 集成测试（回放）

| 场景 | 输入特征 | 观测指标 |
|------|---------|---------|
| 原地旋转 | 中心静止、单观测持续 ≥ N 帧 | center RMS jitter; publish_pos 与观测一致性 |
| 双观测切换 | 双观测出现→消失→出现 | 模式切换延迟; 切换帧间跳跃量 |
| 大 yaw_rate | spin_rate > 8 rad/s | IMM 跟踪残差 vs KF 残差; 预测偏差 |
| 低 yaw_rate | spin_rate < 1 rad/s | 静止稳定性; 误切换率 |
| 标准 4 板 | 非 outpost 目标 | 行为回归：与旧链路一致 |

### 6.3 回归检查清单

- [ ] `outpost.use_tracker_v2 = false` 下，旧 `OutpostArmorTracker` 行为不变
- [ ] `ambiguous_publish_single_armor_semantics = false` 下，V2 发布语义与改造前一致
- [ ] 标准 4 板 tracker（`AdaptiveArmorTracker` 等）不受影响
- [ ] `gimbal_pipeline.yaml` 默认值下系统可编译、可运行
- [ ] 离线分析脚本（如依赖 debug_snapshot 字段）运行正常

---

## 7. 风险矩阵

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| single/structured 语义切换后上游误读 center_* 字段 | 中 | 高——控制偏差 | `RepresentationMode` 提供显式判别函数; Phase A 不改消息字段，仅通过 num_armors=1 暗示 |
| IMM 参数未调稳导致短时抖动增大 | 高 | 中——影响开火稳定性 | `ambiguous_backend_use_imm_adapter` 开关; 默认关; 回放充分对比后开启 |
| `OutpostOutputAdapter` 接口变更导致编译失败 | 低 | 低 | 新增 `PublishStateInput` 而非修改函数签名（default = nullptr 兼容） |
| 单板 offset=(0,0,0) 导致 ArmorSelector 行为异常 | 中 | 中 | 单板模式下候选集已退化为 1；验证 `ArmorSelector` 单候选路径 |
| `FireAdviceEngine` 面向过滤依赖 offset orientation | 中 | 中 | 单板下关闭或弱化面向过滤；增加配置开关 |

---

## 8. 回退方案

任何时候可通过以下步骤回退到改造前状态：

1. 设置 `outpost.ambiguous_backend_use_imm_adapter: false`
2. 设置 `outpost.ambiguous_publish_single_armor_semantics: false`
3. 确认 `outpost.use_tracker_v2` 设置为所需值（true=V2 旧语义, false=旧 OutpostArmorTracker）
4. 此时所有行为等价于改造前

**代码级回退**：
- `AmbiguousSingleArmorFilterAdapter` 在 `use_imm_=false` 时内部使用 legacy `OutpostAmbiguousKF`，行为与改造前 `OutpostAmbiguousBackend` 直接使用 `OutpostAmbiguousKF` 等价
- `OutpostOutputAdapter` 的 `PublishStateInput.armor_snap == nullptr` 时走原有 center-centric 路径

---

## 9. 附录：关键接口速查

### 9.1 AmbiguousArmorSnapshot（新增）

```cpp
struct AmbiguousArmorSnapshot {
  Eigen::Vector3d armor_pos;        // 单板世界坐标
  Eigen::Vector3d armor_vel;        // 单板速度
  double armor_yaw;                  // 单板 yaw（装甲板法向量朝向）
  double armor_yaw_rate;            // yaw 角速度
  int panel_id;                     // 面板 ID（0/1/2）
  double confidence;                // 滤波置信度（从 pos_conf 聚合）
};
```

### 9.2 PublishStateInput（新增）

```cpp
struct PublishStateInput {
  const BackendStateSnapshot *backend_snap = nullptr;
  const AmbiguousArmorSnapshot *armor_snap = nullptr;
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;
};
```

### 9.3 RepresentationMode（新增）

```cpp
enum class RepresentationMode {
  STRUCTURED_ROBOT = 0,
  AMBIGUOUS_SINGLE_ARMOR = 1,
  UNKNOWN = 2,
};
```

### 9.4 现有接口语义映射（Phase A 约定）

| `TrackedRobot` 字段 | STRUCTURED | AMBIGUOUS_SINGLE_ARMOR |
|---------------------|------------|------------------------|
| `num_armors` | ≥3 | 1 |
| `armors_offset.size()` | ≥3 | 1（zero offset 或当前板 offset） |
| `center_position` | 机器人中心 | 单板世界坐标 |
| `center_velocity` | 中心速度 | 单板速度 |
| `yaw` | 机器人朝向 | 单板朝向 |
| `yaw_velocity` | 机器人角速度 | 单板角速度 |
| `radius` | 主半径 | 0.0（无意义） |
| `radius_2` | 副半径 | 0.0（无意义） |
| `d_za` | 半高度差 | 0.0（无意义） |
| `confidence` | 原始 | × `single_mode_confidence_scale`（~0.7） |

---

## 10. 结论

本详细方案将原大纲的 13 节内容展开为 **10 个具体模块**的改造方案，覆盖 4 个新增文件 + 14 个修改文件，分 3 个 Phase 逐步落地。

核心链路为：
```
SingleArmorIMMTracker (滤波核心)
  → AmbiguousSingleArmorFilterAdapter (适配层)
    → OutpostAmbiguousBackend (业务适配层，保留几何但解耦滤波)
      → OutpostOutputAdapter (分叉填充 publish 语义)
        → FixedProfileTrackedRobotBuilder (分叉发布单板/多板)
          → TrackedRobotUsage::isSingleArmorRepresentation (语义判别)
            → ArmorPositionCalculator (单板/多板路径分叉)
```

每层职责清晰，回退路径明确，可在不改变 `rm_interfaces` 消息定义的前提下完成全部语义升级。
