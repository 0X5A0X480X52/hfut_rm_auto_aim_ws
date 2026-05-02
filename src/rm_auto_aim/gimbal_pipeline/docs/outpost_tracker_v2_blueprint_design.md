# OutpostTrackerV2 文件级详细蓝图设计

## 1. 背景与目标

当前 `OutpostArmorTracker` 已承载过多职责（观测选择、假设打分、ID 绑定、模式切换、周期证据、滤波更新、输出组装），导致：

1. 文件体量大，修改风险高；
2. 新旧链路并存时分支复杂度进一步上升；
3. `AMBIGUOUS/STRUCTURED` 模式切换逻辑与 `binder` 逻辑耦合在单类中，难复用。

本设计目标：

1. 在 `trackers/` 下新增一个全新 `OutpostTrackerV2`，不继续扩展旧类；
2. 将链路拆分为“编排层 + 子模块”；
3. 明确 `ModeFSM` 与 `binder` 的边界；
4. 支持“信息不足降级输出 + 信息充分结构输出”的双模式机制；
5. 支持后续在 `AMBIGUOUS` 与 `STRUCTURED` 使用不同滤波后端。

---

## 2. ModeFSM 放置决策

### 2.1 备选方案

1. **放进 `binder` 内部**
   - 优点：调用短；
   - 缺点：`binder` 语义膨胀（从“ID 绑定”变成“全链路模式控制”），后续其他 tracker 复用困难。

2. **与 `binder` 并列独立模块（推荐）**
   - 优点：职责清晰，`mode` 可跨 tracker 复用，`binder` 可保持纯绑定职责；
   - 缺点：需要定义 `binder -> mode` 的证据接口。

### 2.2 结论

`ModeFSM` 应与 `binder` **并列**，放在 `max_entropy_tracker/mode` 下。  
两者通过稳定数据契约耦合，而非实现耦合：

1. `ModeFSM` 只消费 `ModeEvidence`；
2. `ModeEvidence` 中可包含 `BinderDebugSnapshot` 派生字段；
3. `binder` 不依赖 `mode`，避免反向耦合。

---

## 3. 总体架构与调用链

### 3.1 架构分层

```text
OutpostTrackerV2 (orchestrator)
  ├─ ObservationFrontend       (观测选择/候选生成)
  ├─ BinderBridge              (调用 binder pipeline)
  ├─ EvidenceFuser + ModeFSM   (证据融合 + 模式迁移)
  ├─ BackendRouter
  │    ├─ AmbiguousBackend     (轻量跟踪后端)
  │    └─ StructuredBackend    (结构化 UKF 后端)
  ├─ StateTransfer             (跨模式状态桥接)
  └─ OutputAdapter             (统一输出与debug快照)
```

### 3.2 单帧调用链（update）

1. `ObservationFrontend::select_primary_observation(obs)`
2. `ObservationFrontend::build_binding_candidate(...)`
3. `BinderBridge::step(candidate, state_hint)` 得到 `BinderOutput + BinderDebugSnapshot`
4. `EvidenceFuser::fuse(...)` 生成 `ModeEvidence`
5. `ModeFSM::step(evidence)` 得到 `ModeDecision`
6. 若模式切换，调用 `StateTransfer`
7. 路由到 `AmbiguousBackend` 或 `StructuredBackend` 执行 `update`
8. `OutputAdapter` 统一构造发布状态和 debug 信息

### 3.3 关键思想

1. `AMBIGUOUS`：不承诺 ID，输出降级估计，防止错误结构绑定污染状态；
2. `STRUCTURED`：在证据充足后才启用结构先验与稳定 ID；
3. 切换通过滞回阈值 + 连续帧确认，避免抖动。

---

## 4. 目录与文件蓝图

> 下述为**新增**文件建议；旧 `OutpostArmorTracker` 保留用于回退。

### 4.1 `mode` 模块（与 binder 并列）

#### 4.1.1 `include/max_entropy_tracker/mode/mode_enums.hpp`
职责：模式状态与迁移原因枚举。

```cpp
namespace fyt::auto_aim::mode {

enum class TrackMode { AMBIGUOUS = 0, STRUCTURED = 1 };

enum class TransitionReason {
  NONE = 0,
  STRONG_EVIDENCE_ENTER = 1,
  WEAK_EVIDENCE_EXIT = 2,
  FORCED_REBIND_EXIT = 3,
  BACKEND_DIVERGENCE_EXIT = 4
};

}  // namespace fyt::auto_aim::mode
```

#### 4.1.2 `include/max_entropy_tracker/mode/mode_types.hpp`
职责：`ModeFSM` 输入输出契约。

```cpp
namespace fyt::auto_aim::mode {

struct ModeEvidence {
  double timestamp = 0.0;
  int obs_count = 0;
  bool has_dual_obs = false;

  bool jump_detected = false;
  int jump_kind = 0;  // binder::JumpKind cast
  double jump_confidence = 0.0;
  double candidate_margin = 0.0;

  double binder_health_score = 1.0;
  int binder_bad_frames = 0;
  bool binder_force_rebind = false;

  double entropy_norm = 1.0;
  double max_prob = 0.0;

  bool z_audit_strong = false;
  bool has_2dz_signature = false;
};

struct ModeDecision {
  TrackMode mode = TrackMode::AMBIGUOUS;
  bool switched = false;
  TransitionReason reason = TransitionReason::NONE;
  double confidence = 0.0;
};

struct ModeDebugSnapshot {
  bool valid = false;
  TrackMode mode = TrackMode::AMBIGUOUS;
  double enter_score = 0.0;
  double exit_score = 0.0;
  int enter_counter = 0;
  int exit_counter = 0;
  TransitionReason last_reason = TransitionReason::NONE;
};

}  // namespace fyt::auto_aim::mode
```

#### 4.1.3 `include/max_entropy_tracker/mode/mode_fsm.hpp`
职责：模式状态机本体。

```cpp
namespace fyt::auto_aim::mode {

struct ModeFSMConfig {
  int enter_confirm_frames = 3;
  int exit_confirm_frames = 4;
  int min_dwell_frames = 6;
  double enter_threshold = 0.72;
  double exit_threshold = 0.45;
};

class ModeFSM {
 public:
  explicit ModeFSM(const ModeFSMConfig &cfg);

  void reset(TrackMode init_mode);
  ModeDecision step(const ModeEvidence &ev);

  TrackMode mode() const;
  const ModeDebugSnapshot &debug_snapshot() const;

 private:
  ModeFSMConfig cfg_;
  TrackMode mode_ = TrackMode::AMBIGUOUS;
  int enter_counter_ = 0;
  int exit_counter_ = 0;
  int dwell_counter_ = 0;
  ModeDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::mode
```

#### 4.1.4 `include/max_entropy_tracker/mode/evidence_fuser.hpp`
职责：融合观测质量、binder诊断、统计信息为 `ModeEvidence`。

```cpp
namespace fyt::auto_aim::mode {

struct EvidenceFuserConfig {
  double w_jump = 0.30;
  double w_dual = 0.20;
  double w_margin = 0.20;
  double w_health = 0.20;
  double w_entropy = 0.10;
};

class EvidenceFuser {
 public:
  explicit EvidenceFuser(const EvidenceFuserConfig &cfg);

  ModeEvidence fuse(
      double timestamp,
      int obs_count,
      double entropy_norm,
      double max_prob,
      double candidate_margin,
      bool z_audit_strong,
      const binder::BinderDebugSnapshot &binder_dbg) const;

  double compute_enter_score(const ModeEvidence &ev) const;
  double compute_exit_score(const ModeEvidence &ev) const;

 private:
  EvidenceFuserConfig cfg_;
};

}  // namespace fyt::auto_aim::mode
```

#### 4.1.5 实现文件

1. `src/max_entropy_tracker/mode/mode_fsm.cpp`
2. `src/max_entropy_tracker/mode/evidence_fuser.cpp`

---

### 4.2 OutpostTrackerV2 主体与子模块

#### 4.2.1 `include/max_entropy_tracker/trackers/outpost_tracker_v2.hpp`
职责：主编排器（薄类），实现 `BaseTracker` 接口。

```cpp
namespace fyt::auto_aim {

class OutpostTrackerV2 : public BaseTracker {
 public:
  explicit OutpostTrackerV2(const UnifiedConfig &config, double dt = 0.05,
                            bool enable_oscillation = false);

  void initialize(const std::vector<ObservationData> &obs,
                  double r1 = 0.15, double r2 = 0.20,
                  double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;
  SpinFilterInterface &spin_filter() override;
  const SpinFilterInterface &spin_filter() const override;
  ManeuverResult assess_maneuver() const override;

  Eigen::Vector3d get_publish_velocity() const override;
  bool is_ambiguous_single_mode() const override;
  int effective_num_armors() const override;
  double confidence_scale() const override;
  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

  const outpost_v2::OutpostDebugSnapshotV2 &debug_snapshot() const;

 private:
  bool step_with_observation(const ObservationData &obs,
                             int obs_count,
                             double dt_for_update);
  void apply_mode_decision(const mode::ModeDecision &decision,
                           const ObservationData &obs);
  void update_publish_cache();

  UnifiedConfig config_;
  std::unique_ptr<outpost_v2::ObservationFrontend> obs_frontend_;
  std::unique_ptr<outpost_v2::OutpostBinderBridge> binder_bridge_;
  std::unique_ptr<mode::EvidenceFuser> evidence_fuser_;
  std::unique_ptr<mode::ModeFSM> mode_fsm_;
  std::unique_ptr<outpost_v2::IOutpostBackend> ambiguous_backend_;
  std::unique_ptr<outpost_v2::IOutpostBackend> structured_backend_;
  outpost_v2::BackendRouter router_;
  outpost_v2::OutpostRuntimeContext ctx_;
  outpost_v2::OutpostDebugSnapshotV2 debug_;
};

}  // namespace fyt::auto_aim
```

#### 4.2.2 `src/max_entropy_tracker/trackers/outpost_tracker_v2.cpp`
职责：实现主调用链、生命周期和模式切换编排。

---

### 4.3 `outpost_v2` 子目录（细粒度组件）

#### 4.3.1 `include/max_entropy_tracker/trackers/outpost_v2/outpost_runtime_context.hpp`
职责：运行时共享状态（去中心化成员变量）。

```cpp
namespace fyt::auto_aim::outpost_v2 {

struct OutpostRuntimeContext {
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;
  int selected_panel_id = -1;
  int bound_panel_id = -1;
  double binding_confidence = 0.0;

  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;

  Eigen::Vector3d publish_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d publish_vel = Eigen::Vector3d::Zero();
  double entropy_norm = 1.0;
  double max_prob = 0.0;

  std::optional<double> last_timestamp;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.2 `include/max_entropy_tracker/trackers/outpost_v2/outpost_debug_snapshot.hpp`
职责：V2 debug 快照定义。

```cpp
namespace fyt::auto_aim::outpost_v2 {

struct OutpostDebugSnapshotV2 {
  bool valid = false;
  int mode = 0;  // mode::TrackMode cast
  int selected_panel_id = -1;
  int bound_panel_id = -1;
  double binding_confidence = 0.0;

  int obs_count = 0;
  double entropy_norm = 1.0;
  double max_prob = 0.0;

  binder::BinderDebugSnapshot binder_dbg;
  mode::ModeDebugSnapshot mode_dbg;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.3 `include/max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.hpp`
职责：观测选择、候选构造、观测质量计算。

```cpp
namespace fyt::auto_aim::outpost_v2 {

struct BindingCandidate {
  int candidate_panel_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double selected_yaw_err = 0.0;
  double selected_xy_residual = 0.0;
  double z_jump = std::numeric_limits<double>::quiet_NaN();
  bool has_z_jump = false;
};

class ObservationFrontend {
 public:
  explicit ObservationFrontend(const UnifiedConfig &cfg);

  const ObservationData *select_primary_observation(
      const std::vector<ObservationData> &obs,
      const OutpostRuntimeContext &ctx) const;

  BindingCandidate build_binding_candidate(
      const ObservationData &obs,
      const OutpostRuntimeContext &ctx) const;

 private:
  UnifiedConfig cfg_;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.4 `include/max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.hpp`
职责：`ObservationFrontend` 与 `binder pipeline` 适配层。

```cpp
namespace fyt::auto_aim::outpost_v2 {

class OutpostBinderBridge {
 public:
  OutpostBinderBridge(const UnifiedConfig &cfg,
                      const binder::RobotBindingProfile &profile);

  void reset(int init_panel_id, std::optional<double> obs_z);

  binder::BinderOutput step(const ObservationData &obs,
                            int obs_count,
                            const BindingCandidate &candidate,
                            const OutpostRuntimeContext &ctx);

  const binder::BinderDebugSnapshot &debug_snapshot() const;

 private:
  binder::RobotBindingProfile profile_;
  std::unique_ptr<binder::BinderPipeline> pipeline_;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.5 `include/max_entropy_tracker/trackers/outpost_v2/outpost_backend_interface.hpp`
职责：后端统一接口 + 快照定义。

```cpp
namespace fyt::auto_aim::outpost_v2 {

struct BackendStateSnapshot {
  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;
  double radius = 0.0;
  int panel_id = -1;
};

struct BackendUpdateHint {
  int panel_id = -1;
  double position_confidence = 1.0;
  bool enforce_panel_constraint = false;
};

class IOutpostBackend {
 public:
  virtual ~IOutpostBackend() = default;

  virtual void reset(const ObservationData &obs, int panel_id) = 0;
  virtual void predict(double dt) = 0;
  virtual bool update(const ObservationData &obs,
                      const BackendUpdateHint &hint) = 0;
  virtual BackendStateSnapshot snapshot() const = 0;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.6 `include/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp`
职责：`AMBIGUOUS` 轻量后端（建议接 `kalmanFilters` 基础模型）。

```cpp
namespace fyt::auto_aim::outpost_v2 {

class OutpostAmbiguousBackend : public IOutpostBackend {
 public:
  explicit OutpostAmbiguousBackend(const UnifiedConfig &cfg);

  void reset(const ObservationData &obs, int panel_id) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

 private:
  UnifiedConfig cfg_;
  // 例如: basic_models::CV_KF / Singer_KF 的包装实例
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.7 `include/max_entropy_tracker/trackers/outpost_v2/outpost_structured_backend.hpp`
职责：`STRUCTURED` 结构后端（包装 `OutpostSpinUKF`）。

```cpp
namespace fyt::auto_aim::outpost_v2 {

class OutpostStructuredBackend : public IOutpostBackend {
 public:
  OutpostStructuredBackend(const UnifiedConfig &cfg, double dt);

  void reset(const ObservationData &obs, int panel_id) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

  OutpostSpinUKF &ukf();
  const OutpostSpinUKF &ukf() const;

 private:
  UnifiedConfig cfg_;
  OutpostSpinUKF ukf_;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.8 `include/max_entropy_tracker/trackers/outpost_v2/outpost_state_transfer.hpp`
职责：跨模式状态桥接（避免切换瞬间跳变）。

```cpp
namespace fyt::auto_aim::outpost_v2 {

class OutpostStateTransfer {
 public:
  static void ambiguous_to_structured(
      const BackendStateSnapshot &amb_state,
      int panel_id_hint,
      OutpostStructuredBackend *structured);

  static void structured_to_ambiguous(
      const BackendStateSnapshot &st_state,
      OutpostAmbiguousBackend *ambiguous);
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.9 `include/max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.hpp`
职责：统一生成 Tracker 输出语义（发布位姿、速度、armors_offset）。

```cpp
namespace fyt::auto_aim::outpost_v2 {

class OutpostOutputAdapter {
 public:
  explicit OutpostOutputAdapter(const UnifiedConfig &cfg);

  void update_publish_state(OutpostRuntimeContext *ctx,
                            const BackendStateSnapshot &backend_state) const;

  std::vector<geometry_msgs::msg::Pose> build_armors_offset(
      const OutpostRuntimeContext &ctx) const;

 private:
  UnifiedConfig cfg_;
};

}  // namespace fyt::auto_aim::outpost_v2
```

#### 4.3.10 对应实现文件

1. `src/max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.cpp`
2. `src/max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.cpp`
3. `src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp`
4. `src/max_entropy_tracker/trackers/outpost_v2/outpost_structured_backend.cpp`
5. `src/max_entropy_tracker/trackers/outpost_v2/outpost_state_transfer.cpp`
6. `src/max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.cpp`

---

## 5. 配置扩展蓝图

### 5.1 `core/config.hpp`（新增字段建议）

```cpp
struct OutpostParameters {
  // ...
  bool use_tracker_v2 = false;

  // 模式FSM
  int mode_enter_confirm_frames = 3;
  int mode_exit_confirm_frames = 4;
  int mode_min_dwell_frames = 6;
  double mode_enter_threshold = 0.72;
  double mode_exit_threshold = 0.45;

  // 证据融合
  double mode_weight_jump = 0.30;
  double mode_weight_dual = 0.20;
  double mode_weight_margin = 0.20;
  double mode_weight_health = 0.20;
  double mode_weight_entropy = 0.10;
};
```

### 5.2 `gimbal_pipeline_node.cpp`（新增参数）

1. 声明：`declare_parameter("outpost.use_tracker_v2", false);`
2. 读取：`c.outpost.use_tracker_v2 = get_parameter(...).as_bool();`
3. 对 `mode_*` 参数做范围钳制。

### 5.3 `tracker_manager.hpp`（实例化切换）

`get_or_create()` 中：

```cpp
if (robot_id == "outpost") {
  if (config_.outpost.use_tracker_v2) {
    t = std::make_unique<OutpostTrackerV2>(config_, dt_, enable_osc_);
  } else {
    t = std::make_unique<OutpostArmorTracker>(config_, dt_, enable_osc_);
  }
}
```

---

## 6. 实现步骤（分阶段）

### Phase 0：骨架落地（可编译，无行为切换）

1. 新增 `mode` 与 `outpost_v2` 头/源文件骨架；
2. `OutpostTrackerV2` 实现最小 `BaseTracker` 接口；
3. `use_tracker_v2` 参数接入但默认关闭。

### Phase 1：影子链路（Shadow Mode）

1. 仍由旧 tracker 输出；
2. V2 并行运行仅记录 debug（不参与发布）；
3. 对比 `selected_id/switch_event/center_jitter`。

### Phase 2：灰度切换

1. 对 `outpost.use_tracker_v2=true` 目标启用 V2；
2. 保留旧 tracker 回退开关；
3. 重点调 `ModeFSM` 阈值与确认帧数。

### Phase 3：清理与收敛

1. 稳定后逐步下线旧 outpost 绑定分支；
2. 保留旧类一段时间用于回归对照；
3. 最终统一 debug 字段到 V2。

---

## 7. 测试与验收

### 7.1 单元测试建议

1. `test_mode_fsm.cpp`
   - 进入/退出滞回；
   - 最小驻留帧；
   - 强制回退分支。
2. `test_outpost_binder_bridge.cpp`
   - `BinderFrameInput` 映射正确性；
   - `jump_kind`/`switch_reason` 透传。
3. `test_outpost_state_transfer.cpp`
   - A->S / S->A 切换后状态连续性。

### 7.2 集成测试建议

1. 固定视频回放（含单观测长段 + 明显 2dz 跳变）；
2. 指标：
   - center 抖动（RMS）；
   - 错误切换率；
   - 模式切换次数与停留时长；
   - 从 AMBIGUOUS 收敛到 STRUCTURED 的平均帧数。

### 7.3 验收门槛（建议）

1. `use_tracker_v2=false` 行为与现网一致；
2. `use_tracker_v2=true` 在 outpost 测试集中：
   - center 抖动不高于旧链路；
   - 错误ID绑定率下降；
   - 异常后可回到 AMBIGUOUS 并再次收敛。

---

## 8. 风险与控制

1. **模式抖动风险**：使用 enter/exit 滞回 + dwell 限制；
2. **切换瞬态跳变**：必须实现 `StateTransfer`；
3. **过度依赖单一证据**：`ModeEvidence` 必须多源融合，避免仅凭 2dz 一票否决/通过；
4. **复杂度反弹**：`OutpostTrackerV2` 只做编排，禁止把策略逻辑回灌主类。

---

## 9. 结论

1. 新建 `OutpostTrackerV2` 是必要且可控的重构路径；
2. `ModeFSM` 应作为与 `binder` 并列模块，而非塞入 `binder` 内部；
3. 通过“稳定接口耦合（证据输入）+ 实现解耦（模块并列）”，可以兼顾可维护性与演进速度。

