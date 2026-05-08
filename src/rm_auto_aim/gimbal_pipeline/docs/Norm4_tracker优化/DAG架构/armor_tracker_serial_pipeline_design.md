# 基于 DAG 思想的串行 Armor Tracker 架构迁移设计

> 目标：在不过度设计的前提下，吸收 Evidence DAG / Discriminator / Tracker Manager / Debug 系统的核心思想，先将当前 `Norm4ArmorTracker` 风格实现迁移为一个**固定调用链的串行 Pipeline**。  
> 后续如果复杂度继续上升，再平滑演进为通用 DAG Runtime。

---

## 1. 设计背景

当前跟踪系统已经具备一定分层雏形，例如：

```text
ObservationFrontend
→ Norm4BinderBridge
→ EvidenceFuser + ModeFSM
→ AmbiguousBackend / StructuredBackend
→ OutputAdapter
```

这说明系统并不是完全单体式结构，已经存在前端候选生成、绑定、模式决策、后端更新和输出适配等职责拆分。

前面讨论过一种更通用的架构：

```text
TrackerManager
+ Evidence DAG
+ Discriminator DAG
+ Voting / Ensemble
+ BackendDecision
+ DebugBus
```

这种设计在扩展性、可调试性和可组合性上很强，但对当前工程阶段可能存在明显的过度设计风险，主要体现在：

```text
1. GraphRuntime、NodeMeta、PortBundle、DAG compile 等基础设施开发成本较高；
2. 当前判别器数量还不够多，不一定需要动态 DAG；
3. 实车链路对实时性、可控性、易调试性要求更高；
4. 当前实现已经有固定顺序的分层调用链，先整理串行版本收益更直接；
5. 过早引入通用图后端可能导致调试复杂度上升。
```

因此，当前更合理的策略是：

```text
先做“DAG 思想指导下的串行架构”，
而不是一开始做“完整 DAG 框架”。
```

---

## 2. 总体迁移目标

目标不是立即实现通用 DAG Runtime，而是先形成如下结构：

```text
ArmorTrackerPipeline
  ├── TrackerManager
  ├── EvidenceExtractor
  ├── DiscriminatorChain
  ├── BackendModeDecider
  ├── BackendUpdater
  ├── OutputAdapter
  └── DebugRecorder
```

每帧仍然串行执行，但每一阶段的输入输出清晰可见：

```text
FrameInput
→ 2D Track Stage
→ PnP Evidence Stage
→ 3D Evidence Stage
→ Binding / Discriminator Stage
→ Backend Mode Decision Stage
→ Backend Update Stage
→ Output Stage
→ Debug Trace
```

相比完整 DAG，这种串行版本有几个优点：

```text
1. 实现成本低；
2. 调用顺序稳定；
3. 易于保持当前 Norm4 行为；
4. 便于逐步插入 2D tracker / evidence / debug；
5. 后续仍可平滑演进为 DAG Runtime；
6. 更容易做实时性分析和 P99 延迟监控。
```

---

## 3. 核心设计原则

### 3.1 先固定调用链，后通用 DAG

当前阶段不实现：

```text
GraphRuntime
NodeMeta
PortBundle
DAG compile
拓扑排序
通用节点注册
YAML 动态配置节点
```

当前阶段先实现：

```text
Serial Pipeline
统一数据结构
明确阶段边界
TrackerManager 图外管理
Evidence / Discriminator / Decision 串行调用
DebugTrace 串行记录
```

---

### 3.2 分离 tracker_id、panel_id、armor_id

必须明确区分：

```text
tracker_id：图像域 / 临时轨迹编号
panel_id：结构候选编号，例如 0~3 panel 假设
armor_id：机器人内部真实装甲板编号
robot_track_id：完整机器人对象编号
```

不要让 `tracker_id` 直接等价于 `panel_id` 或 `armor_id`。

建议定义：

```cpp
using Track2DId = int;
using Track3DId = int;
using RobotTrackId = int;
using ArmorId = int;
using PanelId = int;
```

---

### 3.3 Evidence 不直接修改 Tracker

Evidence 阶段只负责提取证据，例如：

```text
2D 连续性
PnP 位姿
3D 速度
yaw 观测
高度均值和方差
重投影误差
双板关系
```

Evidence 不应该直接创建、删除、升级 tracker。

---

### 3.4 Discriminator 不直接更新 UKF

判别器只输出：

```text
armor_id 概率
panel_id 候选
稳定性判断
置信度
原因 reason
```

它不应该直接调用：

```text
AmbiguousSingleArmorFilterAdapter::update()
DualRadiusSpinUKF::update()
```

后端更新统一由 `TrackerManager / BackendUpdater` 执行。

---

### 3.5 BackendDecision 显式化

所有后端更新都应该由一个显式决策结构驱动：

```text
是否只更新单装甲板后端
是否创建完整机器人对象
是否更新 DualRadiusSpinUKF
是否发布完整机器人状态
是否发布单装甲板状态
```

这样 debug 时可以直接解释：

```text
为什么当前帧没有创建 DualRadiusSpinUKF？
为什么当前帧发布的是单板状态？
为什么当前帧从 AMBIGUOUS 切换到了 STRUCTURED？
```

---

## 4. 推荐串行 Pipeline

### 4.1 高层流程

```cpp
void ArmorTrackerPipeline::update(const FrameInput& input) {
  debug_.beginFrame(input);

  // 1. 2D tracker 更新或读取
  auto track2d_result =
      tracker_manager_.update2D(input.detections, input.timestamp);

  // 2. PnP 证据生成
  auto pnp_evidence =
      evidence_extractor_.buildPnPEvidence(track2d_result, input);

  // 3. 3D / 高度 / 关系证据生成
  auto evidence_frame =
      evidence_extractor_.buildAllEvidence(track2d_result, pnp_evidence);

  // 4. armor_id / panel_id / 稳定性判别
  auto disc_result =
      discriminator_chain_.evaluate(evidence_frame);

  // 5. 后端模式决策
  auto decision =
      mode_decider_.decide(evidence_frame, disc_result);

  // 6. 根据 decision 更新 ambiguous 或 structured backend
  tracker_manager_.applyDecision(decision, evidence_frame);

  // 7. 生成输出
  auto output =
      output_adapter_.makeOutput(tracker_manager_.snapshot(), decision);

  // 8. 发布给 target_selector / gimbal controller
  output_adapter_.publish(output);

  debug_.endFrame(output);
}
```

---

### 4.2 阶段划分

```text
Stage 1: Input Normalize
  输入检测器原始结果
  输出统一 Detection2D / FrameInput

Stage 2: 2D Track Stage
  基于 SORT-like 逻辑维护 tracker_id
  输出 Track2DEvidence

Stage 3: PnP Evidence Stage
  对带 tracker_id 的 2D 观测做 PnP
  输出 PnPEvidence

Stage 4: 3D Evidence Stage
  结合单板 3D tracker / 结构化 tracker 状态
  输出 Track3DEvidence

Stage 5: Relation Evidence Stage
  统计多块装甲板之间的高度差、距离差、yaw 相位差
  输出 RelationEvidence

Stage 6: Discriminator Stage
  基于证据判断 armor_id / panel_id / 稳定性
  输出 DiscriminatorResult

Stage 7: Backend Mode Decision Stage
  判断 AMBIGUOUS / STRUCTURED / REACQUIRE / LOST
  输出 BackendDecision

Stage 8: Backend Update Stage
  由 TrackerManager 执行后端更新
  更新 AmbiguousSingleArmorFilterAdapter 或 DualRadiusSpinUKF

Stage 9: Output Stage
  输出单装甲板状态或完整机器人状态
```

---

## 5. 关键数据结构草案

### 5.1 FrameInput

```cpp
struct FrameInput {
  uint64_t frame_id = 0;
  double timestamp = 0.0;

  std::vector<Detection2D> detections;
};
```

---

### 5.2 Detection2D

```cpp
struct Detection2D {
  int detection_id = -1;

  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;

  int class_id = -1;
  double confidence = 0.0;

  double timestamp = 0.0;
};
```

---

### 5.3 Track2DEvidence

```cpp
struct Track2DEvidence {
  Track2DId tracker_id = -1;

  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;

  cv::Point2f center;
  cv::Point2f velocity_2d;

  int class_id = -1;
  double detection_confidence = 0.0;
  double track_confidence = 0.0;

  int age = 0;
  int hit_count = 0;
  int miss_count = 0;

  double timestamp = 0.0;
};
```

---

### 5.4 PnPEvidence

```cpp
struct PnPEvidence {
  Track2DId tracker_id = -1;

  Eigen::Vector3d position_cam;
  Eigen::Vector3d position_odom;

  double armor_yaw = 0.0;
  double armor_pitch = 0.0;
  double armor_roll = 0.0;

  double reprojection_error = 0.0;
  double pnp_confidence = 0.0;

  double timestamp = 0.0;
};
```

---

### 5.5 Track3DEvidence

```cpp
struct Track3DEvidence {
  Track2DId tracker_id = -1;

  Eigen::Vector3d position_odom;
  Eigen::Vector3d velocity_odom;

  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;

  double z_mean = 0.0;
  double z_var = 0.0;

  double innovation_norm = 0.0;
  double stability = 0.0;

  double timestamp = 0.0;
};
```

---

### 5.6 RelationEvidence

```cpp
struct RelationEvidence {
  Track2DId tracker_id_a = -1;
  Track2DId tracker_id_b = -1;

  double height_diff = 0.0;
  double distance_xy = 0.0;
  double yaw_phase_diff = 0.0;

  double relation_confidence = 0.0;
};
```

---

### 5.7 EvidenceFrame

```cpp
struct EvidenceFrame {
  uint64_t frame_id = 0;
  double timestamp = 0.0;

  std::vector<Track2DEvidence> track2d;
  std::vector<PnPEvidence> pnp;
  std::vector<Track3DEvidence> track3d;
  std::vector<RelationEvidence> relation;
};
```

---

### 5.8 DiscriminatorResult

```cpp
struct DiscriminatorResult {
  Track2DId tracker_id = -1;

  ArmorId reference_armor_id = -1;
  PanelId reference_panel_id = -1;

  std::array<double, 4> armor_id_prob{};
  std::array<double, 4> panel_id_prob{};

  double confidence = 0.0;
  bool stable = false;

  std::string source;
  std::string reason;
};
```

---

### 5.9 BackendMode

```cpp
enum class BackendMode {
  NO_TARGET,
  TWO_D_ONLY,
  AMBIGUOUS_SINGLE_ARMOR,
  STRUCTURED_ROBOT,
  REACQUIRE,
  LOST
};
```

---

### 5.10 BackendDecision

```cpp
struct BackendDecision {
  BackendMode mode = BackendMode::NO_TARGET;

  Track2DId tracker_id = -1;
  RobotTrackId robot_track_id = -1;

  ArmorId reference_armor_id = -1;
  PanelId reference_panel_id = -1;

  bool update_single_armor = false;
  bool create_structured_robot = false;
  bool update_structured_robot = false;

  bool publish_single_armor_state = false;
  bool publish_robot_state = false;

  double armor_id_confidence = 0.0;
  double structure_confidence = 0.0;
  double motion_confidence = 0.0;
  double output_confidence = 0.0;

  std::string reason;
};
```

---

### 5.11 TargetCandidate

```cpp
enum class TargetStateType {
  SINGLE_ARMOR,
  STRUCTURED_ROBOT
};

struct TargetCandidate {
  TargetStateType type = TargetStateType::SINGLE_ARMOR;

  Track2DId tracker_id = -1;
  RobotTrackId robot_track_id = -1;
  ArmorId reference_armor_id = -1;

  Eigen::Vector3d aim_position_odom;
  Eigen::Vector3d aim_velocity_odom;

  std::optional<Eigen::Vector3d> robot_center_odom;
  std::optional<double> robot_center_yaw;
  std::optional<double> robot_yaw_rate;

  double confidence = 0.0;
  double prediction_horizon_validity = 0.0;
  double timestamp = 0.0;
};
```

---

## 6. TrackerManager 设计

### 6.1 职责

`TrackerManager` 是状态资产管理器，负责：

```text
1. 维护 2D tracker；
2. 维护单装甲板 3D tracker；
3. 维护完整机器人结构化 tracker；
4. 处理 tracker 生命周期；
5. 执行 BackendDecision；
6. 对外提供 snapshot；
7. 不负责 evidence 判别逻辑。
```

---

### 6.2 管理的对象

```text
2DTrackerPool
  维护 SORT-like 2D tracker

SingleArmor3DTrackerPool
  维护 AmbiguousSingleArmorFilterAdapter

StructuredRobotTrackerPool
  维护 DualRadiusSpinUKF

TrackLifecycleManager
  维护 NEW / ACTIVE / AMBIGUOUS / STRUCTURED / LOST / DEAD 状态
```

---

### 6.3 生命周期建议

```cpp
enum class TrackLifecycle {
  NEW,
  ACTIVE_2D,
  AMBIGUOUS_3D,
  STRUCTURED_ROBOT,
  REACQUIRE,
  LOST,
  DEAD
};
```

典型转换：

```text
NEW
→ ACTIVE_2D
→ AMBIGUOUS_3D
→ STRUCTURED_ROBOT

STRUCTURED_ROBOT
→ REACQUIRE
→ AMBIGUOUS_3D
→ LOST
→ DEAD
```

---

### 6.4 applyDecision

```cpp
void TrackerManager::applyDecision(
    const BackendDecision& decision,
    const EvidenceFrame& evidence) {
  switch (decision.mode) {
    case BackendMode::AMBIGUOUS_SINGLE_ARMOR:
      updateSingleArmorTracker(decision, evidence);
      break;

    case BackendMode::STRUCTURED_ROBOT:
      if (decision.create_structured_robot) {
        createStructuredRobotTracker(decision, evidence);
      }
      if (decision.update_structured_robot) {
        updateStructuredRobotTracker(decision, evidence);
      }
      break;

    case BackendMode::REACQUIRE:
      handleReacquire(decision, evidence);
      break;

    case BackendMode::LOST:
      markLost(decision.tracker_id);
      break;

    default:
      break;
  }
}
```

---

## 7. EvidenceExtractor 设计

### 7.1 职责

`EvidenceExtractor` 负责将当前帧观测和 tracker snapshot 转换为统一证据。

```text
Detection2D / Track2D
→ PnPEvidence
→ Track3DEvidence
→ RelationEvidence
→ EvidenceFrame
```

---

### 7.2 推荐接口

```cpp
class EvidenceExtractor {
public:
  std::vector<PnPEvidence> buildPnPEvidence(
      const std::vector<Track2DEvidence>& track2d,
      const FrameInput& input);

  EvidenceFrame buildAllEvidence(
      const std::vector<Track2DEvidence>& track2d,
      const std::vector<PnPEvidence>& pnp);
};
```

---

### 7.3 与当前实现的对应

当前 `ObservationFrontend::build_binding_candidate`、`assign_dual_observations`、`PanelAssociator::associate_panel` 可以先保留，只是不要继续把它们视为“最终 ID 绑定器”，而是视为：

```text
PanelCandidateEvidence 生成器
```

也就是说，当前候选生成逻辑可以继续用，只是输出更显式。

---

## 8. DiscriminatorChain 设计

### 8.1 初始阶段不要拆太细

第一版不建议马上实现很多判别器。可以先包装当前 `Norm4BinderBridge`：

```text
BindingDiscriminator
```

它继续使用当前 binder pipeline，只是输出统一的 `DiscriminatorResult`。

---

### 8.2 后续可扩展的判别器

等串行框架稳定后，再逐步加入：

```text
HeightDiscriminator
YawPhaseDiscriminator
MotionConsistencyDiscriminator
TemporalContinuityDiscriminator
RelationLayoutDiscriminator
ReprojectionQualityDiscriminator
```

---

### 8.3 推荐接口

```cpp
class DiscriminatorChain {
public:
  std::vector<DiscriminatorResult> evaluate(
      const EvidenceFrame& evidence);
};
```

---

### 8.4 简单融合策略

第一版可以先不用复杂 ensemble，只做：

```text
优先 binder 结果
辅助高度置信
辅助 yaw residual
辅助 PnP reprojection error
```

后续再升级为：

```text
weighted voting
Bayesian fusion
rule-based ensemble
```

---

## 9. BackendModeDecider 设计

### 9.1 职责

`BackendModeDecider` 根据证据和判别器输出决定当前应该：

```text
只保持 2D 观测
更新单装甲板后端
创建完整机器人对象
更新完整机器人对象
降级到单装甲板模式
标记丢失
```

---

### 9.2 推荐接口

```cpp
class BackendModeDecider {
public:
  BackendDecision decide(
      const EvidenceFrame& evidence,
      const std::vector<DiscriminatorResult>& disc_results);
};
```

---

### 9.3 创建 DualRadiusSpinUKF 的建议条件

不建议单帧高置信就创建结构化后端。可以要求：

```text
1. 2D tracker 连续稳定若干帧；
2. PnP reprojection error 低；
3. armor_id / panel_id 判别连续稳定；
4. z_mean / z_var 满足高度先验；
5. yaw phase 与结构模型一致；
6. 如果有双板观测，双板几何关系成立。
```

示例规则：

```text
track_age >= 5
miss_count == 0
pnp_reprojection_error < threshold
armor_id_confidence > 0.75 for 3 consecutive frames
structure_confidence > 0.70
```

双板观测有效时可以适当降低稳定帧数要求。

---

## 10. BackendUpdater / 后端更新策略

在串行版本中，`BackendUpdater` 可以合并进 `TrackerManager::applyDecision`，不必单独抽类。

### 10.1 AMBIGUOUS_SINGLE_ARMOR

```text
更新 AmbiguousSingleArmorFilterAdapter
输出单块装甲板状态
可用于短期瞄准，但预测能力有限
```

### 10.2 STRUCTURED_ROBOT

```text
创建或更新 DualRadiusSpinUKF
输出完整机器人中心、yaw、yaw_rate、r1/r2/dza
可支持更强预测和装甲板切换
```

### 10.3 降级策略

当结构证据恶化时：

```text
STRUCTURED_ROBOT
→ REACQUIRE
→ AMBIGUOUS_SINGLE_ARMOR
```

降级时保留单板状态，避免目标输出中断。

---

## 11. OutputAdapter 设计

### 11.1 职责

`OutputAdapter` 将内部 tracker 状态转换为 target_selector / 云台控制可用状态。

---

### 11.2 两类输出

```text
SINGLE_ARMOR:
  aim_position = 单块装甲板预测位置
  aim_velocity = 单块装甲板速度
  无完整 robot_center
  confidence 较低或预测 horizon 较短

STRUCTURED_ROBOT:
  aim_position = 根据完整机器人状态预测的目标装甲板位置
  robot_center / center_yaw / yaw_rate 可用
  confidence 较高，可支持装甲板切换和更长预测
```

---

### 11.3 推荐接口

```cpp
class OutputAdapter {
public:
  TargetCandidate makeOutput(
      const TrackerSnapshot& snapshot,
      const BackendDecision& decision);

  void publish(const TargetCandidate& target);
};
```

---

## 12. DebugRecorder 设计

### 12.1 串行版 Debug 足够先用

当前阶段不必立即上完整 DebugBus，可以先做一个简单的串行 `DebugRecorder`。

它记录：

```text
每帧输入数量
每个阶段耗时
2D tracker 数量
PnP reprojection error
候选 panel_id / armor_id
DiscriminatorResult
BackendDecision
最终输出类型
模式切换原因
```

---

### 12.2 推荐接口

```cpp
class DebugRecorder {
public:
  void beginFrame(const FrameInput& input);

  void recordStageTime(
      const std::string& stage_name,
      double duration_ms);

  void recordEvidenceFrame(
      const EvidenceFrame& evidence);

  void recordDiscriminatorResults(
      const std::vector<DiscriminatorResult>& results);

  void recordBackendDecision(
      const BackendDecision& decision);

  void endFrame(const TargetCandidate& output);
};
```

---

### 12.3 Debug 输出建议

实车运行默认只开启轻量 debug：

```text
frame_id
stage time
tracker_id
mode
confidence
final armor_id
decision reason
```

离线调试时再开启：

```text
完整 evidence
完整 discriminator score
JSONL
debug image
marker
Graphviz
```

---

## 13. 与完整 DAG Runtime 的关系

串行版本并不否定后续 DAG 化。

相反，它应该为后续 DAG 化打基础：

```text
EvidenceFrame
DiscriminatorResult
BackendDecision
TargetCandidate
DebugTrace
```

这些数据结构在未来 DAG Runtime 中仍然可以复用。

后续如果需要 DAG，可以把当前串行阶段映射为：

```text
EvidenceExtractor::buildPnPEvidence
→ PnPEvidenceNode

EvidenceExtractor::buildAllEvidence
→ EvidenceNodeGroup

DiscriminatorChain::evaluate
→ DiscriminatorGraph

BackendModeDecider::decide
→ BackendDecisionNode

OutputAdapter::makeOutput
→ TargetAdapterNode
```

---

## 14. 何时再考虑完整 DAG

只有当出现以下需求时，再考虑完整 DAG Runtime：

```text
1. 判别器数量明显增加；
2. 经常需要启停或替换判别器；
3. 同一套 evidence 要输入多套策略；
4. 希望通过 YAML 配置 pipeline；
5. 需要自动生成调用链图；
6. 需要离线批量实验不同节点组合；
7. 串行 pipeline 已经变得难以维护。
```

否则，固定串行 pipeline 已经足够。

---

## 15. 实时性注意事项

串行版本相比完整 DAG 更容易控制实时性。

建议：

```text
1. 不要拆成多个 ROS2 node；
2. 不要每帧动态构造大量对象；
3. EvidenceFrame 内部 vector 预留容量；
4. Debug 分 OFF / LIGHT / FULL；
5. 实车默认 LIGHT；
6. 重点统计 P95 / P99 延迟；
7. 图像 overlay、JSON、marker 不要常开；
8. PnP / BA / 推理 / 可视化仍然是主要耗时来源。
```

90 FPS 下单帧预算约为：

```text
1 / 90 ≈ 11.1 ms
```

串行框架本身不应成为主要瓶颈。真正需要关注的是：

```text
检测推理
PnP / BA
图像拷贝
ROS topic 发布
debug image
Foxglove / RViz 订阅
```

---

## 16. 推荐分阶段迁移路线

### 阶段 1：整理当前 Norm4 为串行 Pipeline

目标：

```text
不改变算法行为，只整理调用链。
```

改造内容：

```text
1. 新建 ArmorTrackerPipeline；
2. 将当前 update 流程拆成固定阶段；
3. 引入 DebugRecorder 记录阶段耗时；
4. 保持原有 ObservationFrontend / BinderBridge / ModeFSM / Backend 行为。
```

---

### 阶段 2：显式化数据结构

目标：

```text
引入 EvidenceFrame / DiscriminatorResult / BackendDecision。
```

改造内容：

```text
1. 当前 BindingCandidate 转换为 EvidenceFrame 的一部分；
2. binder 输出转换为 DiscriminatorResult；
3. ModeFSM 输出转换为 BackendDecision；
4. OutputAdapter 根据 BackendDecision 输出目标状态。
```

---

### 阶段 3：引入 2D tracker evidence

目标：

```text
先让 2D tracker_id 可见，但暂时不强行参与决策。
```

改造内容：

```text
1. 增加 SORT-like 2D tracker；
2. 输出 Track2DEvidence；
3. PnP 结果带 tracker_id；
4. Debug 中观察 tracker_id 与 panel_id / armor_id 的关系。
```

---

### 阶段 4：包装 BinderBridge 为 BindingDiscriminator

目标：

```text
把当前 binder 从流程内部逻辑变成一个标准判别器。
```

改造内容：

```text
1. BindingDiscriminator::evaluate(EvidenceFrame)；
2. 输出 DiscriminatorResult；
3. 保持原有 binder 状态机逻辑。
```

---

### 阶段 5：TrackerManager 统一管理后端

目标：

```text
AmbiguousSingleArmorFilterAdapter 和 DualRadiusSpinUKF 由 TrackerManager 管理。
```

改造内容：

```text
1. SingleArmor3DTrackerPool；
2. StructuredRobotTrackerPool；
3. applyDecision 统一执行后端更新；
4. 降级 / 升级 / lost 逻辑集中管理。
```

---

### 阶段 6：逐步加入额外判别器

目标：

```text
增加更多可解释证据，但不破坏主流程。
```

可加入：

```text
HeightDiscriminator
YawPhaseDiscriminator
MotionConsistencyDiscriminator
RelationLayoutDiscriminator
ReprojectionQualityDiscriminator
```

---

### 阶段 7：复杂度足够高后再 DAG 化

目标：

```text
如果串行 pipeline 难以维护，再迁移到 DAG Runtime。
```

此时已有结构可直接映射到 DAG：

```text
Stage → Node
EvidenceFrame → Port data
DiscriminatorResult → Node output
BackendDecision → Decision node output
DebugRecorder → DebugBus
```

---

## 17. 最终建议

当前最合理的架构升级路线是：

```text
先迁移到串行 Pipeline，
而不是完整 DAG Runtime。
```

最终串行版目标：

```text
调用链清晰；
数据结构统一；
tracker_id / panel_id / armor_id 分离；
Evidence 显式化；
Discriminator 显式化；
BackendDecision 显式化；
TrackerManager 统一管理后端；
Debug 能解释每一帧决策；
实时性风险可控。
```

一句话总结：

```text
先做“DAG 思想指导下的串行版本”，
等判别器和证据节点数量明显增加后，
再考虑升级为通用 DAG Runtime。
```
