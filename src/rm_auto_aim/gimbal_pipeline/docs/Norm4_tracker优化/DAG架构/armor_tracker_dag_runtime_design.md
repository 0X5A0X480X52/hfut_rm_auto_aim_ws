# Armor Tracker 内部 DAG Runtime 与接口模型包设计方案

## 1. 文档目的

本文档整理一套面向装甲板跟踪、证据提取、判别融合和目标输出的进程内 DAG 架构设计方案。该方案参考 ROS2 的接口包、节点元数据、回调执行和图结构思想，但不直接把每个内部模块拆成 ROS2 Node，而是在一个 ROS2 Component 或普通 C++ 模块内部实现轻量级 DAG Runtime。

该设计主要服务于以下目标：

1. 将 2D tracker、3D tracker、Evidence、Discriminator、Voting、BackendDecision、Debug 等模块解耦。
2. 避免手写固定调用链导致后续扩展困难。
3. 让每个图节点只关心自己的输入、输出和回调逻辑。
4. 通过统一接口支持调试、复盘、图结构导出和节点替换。
5. 保持 Tracker Manager 作为唯一状态资产管理器，避免 Evidence 或 Discriminator 直接修改滤波器状态。

最终目标是形成如下抽象：

```text
业务类 = 算法实现
GraphNodeAdapter = 节点外壳
NodeMeta = 节点接口描述
PortSpec = 输入输出约定
PortBundle = 数据传递容器
GraphRuntime = 调度器
TrackerManager = 图外状态资产管理器
DebugBus = 图运行观测系统
```

---

## 2. 设计背景与动机

当前装甲板跟踪系统中通常存在多种不同层级的逻辑：

```text
2D 检测
→ 2D 数据关联
→ PnP 解算
→ 3D 单板跟踪
→ 结构化机器人跟踪
→ armor_id 判别
→ 目标选择
→ 云台控制
```

如果将这些逻辑全部写在一个 tracker 类内部，会出现几个问题：

1. **调用链固定**：新增一个判别器、证据节点或调试输出时，需要修改主流程代码。
2. **职责混杂**：数据关联、证据提取、ID 判别、滤波器更新、状态发布容易混在一起。
3. **调试困难**：很难回答“这一帧为什么选择这个 armor_id”“为什么没创建完整机器人对象”。
4. **状态污染风险**：如果 Evidence 或 Discriminator 直接更新 tracker，错误判别会反向污染上游状态。
5. **扩展成本高**：从规则判别器切换到投票融合、贝叶斯融合或学习式判别器时，主框架容易被重写。

因此需要一个更统一的内部图执行模型。

---

## 3. 总体架构

推荐架构如下：

```text
ROS2 Component / Tracker Node
  └── TrackerPipeline
        ├── TrackerManager
        │     ├── 2DTrackerPool
        │     ├── SingleArmor3DTrackerPool
        │     └── StructuredRobotTrackerPool
        │
        ├── PreUpdateGraphRuntime
        │     ├── DetectionSourceNode
        │     ├── Track2DEvidenceNode
        │     ├── PnPEvidenceNode
        │     ├── Track3DEvidenceNode
        │     ├── RelationEvidenceNode
        │     ├── HeightDiscriminatorNode
        │     ├── YawPhaseDiscriminatorNode
        │     ├── MotionDiscriminatorNode
        │     ├── ArmorIdVotingNode
        │     └── BackendModeDecisionNode
        │
        ├── TrackerCommandApplier
        │
        ├── PostUpdateGraphRuntime
        │     ├── TargetCandidateAdapterNode
        │     ├── DebugImageNode
        │     ├── MarkerNode
        │     └── RosPublisherSinkNode
        │
        └── DebugBus
```

核心边界：

```text
TrackerManager：负责状态资产管理和滤波器更新
DAG Runtime：负责证据计算、判别融合和决策生成
DebugBus：负责记录调用链、节点输入输出和耗时
OutputAdapter：负责把内部状态转为 target_selector / gimbal_controller 可消费的消息
```

---

## 4. 为什么 TrackerManager 不放进普通 DAG

TrackerManager 拥有强副作用：

```text
创建 tracker
删除 tracker
升级 tracker
降级 tracker
更新 2D Kalman tracker
更新 AmbiguousSingleArmorFilterAdapter
更新 DualRadiusSpinUKF
维护生命周期
维护历史统计
```

如果将 TrackerManager 也作为普通 DAG 节点，容易出现状态读写时序不一致：

```text
EvidenceNode A 读取更新前状态
DiscriminatorNode B 读取更新后状态
OutputNode C 又触发二次更新
```

因此建议：

```text
TrackerManager = 图外状态管理器
DAG = 每帧基于 snapshot 的纯计算图
TrackerCommand = DAG 对 TrackerManager 的唯一写接口
```

即 DAG 节点可以读取 snapshot，但不能直接调用 tracker 的 update、reset、create 或 delete。

---

## 5. 两阶段图执行模型

推荐将每帧处理拆为两个图：

```text
PreUpdateGraph:
  Detection
  → Evidence
  → Discriminator
  → Fuser
  → BackendDecision
  → TrackerCommand

TrackerManager.applyCommands()

PostUpdateGraph:
  UpdatedSnapshot
  → TargetCandidateAdapter
  → Debug / Marker / ROS Output
```

完整流程：

```text
FrameInput
  ↓
snapshot_before
  ↓
PreUpdateGraph
  ↓
TrackerCommandBatch
  ↓
TrackerManager.applyCommands()
  ↓
snapshot_after
  ↓
PostUpdateGraph
  ↓
TargetCandidate / Debug Output
```

这样可以保证：

1. Evidence / Discriminator 总是基于同一个冻结快照进行计算。
2. 所有状态变更统一由 TrackerManager 执行。
3. 输出发布基于更新后的状态。
4. Debug 系统可以清晰记录 before / decision / after 的完整链路。

---

## 6. 类 ROS2 的 model / interface 包设计

建议新建一个仅包含数据模型和节点接口的包，例如：

```text
armor_tracker_model
```

该包不直接实现具体算法，只约定不同节点之间传递什么数据、节点如何声明输入输出、图运行时如何调度节点。

推荐目录结构：

```text
armor_tracker_model/
├── include/armor_tracker_model/
│   ├── core/
│   │   ├── timestamp.hpp
│   │   ├── ids.hpp
│   │   ├── geometry.hpp
│   │   └── quality.hpp
│   │
│   ├── data/
│   │   ├── detection_2d.hpp
│   │   ├── track_2d.hpp
│   │   ├── pnp_evidence.hpp
│   │   ├── track_3d.hpp
│   │   ├── relation_evidence.hpp
│   │   ├── discriminator_output.hpp
│   │   ├── backend_decision.hpp
│   │   ├── tracker_command.hpp
│   │   └── target_candidate.hpp
│   │
│   ├── graph/
│   │   ├── node_kind.hpp
│   │   ├── node_meta.hpp
│   │   ├── port_spec.hpp
│   │   ├── port_bundle.hpp
│   │   ├── node_context.hpp
│   │   ├── graph_node_adapter.hpp
│   │   └── graph_registry.hpp
│   │
│   └── debug/
│       ├── debug_event.hpp
│       ├── debug_bus.hpp
│       └── debug_scope.hpp
```

业务包可以拆成：

```text
armor_tracker_nodes       // 具体 Evidence / Discriminator / Fuser 节点
armor_tracker_runtime     // GraphRuntime / Scheduler
armor_tracker_backends    // 2D tracker / Ambiguous / DualRadiusSpinUKF 管理
armor_tracker_ros         // ROS2 topic / marker / debug image 适配
```

---

## 7. 核心 ID 类型约定

必须一开始就区分不同语义的 ID：

```cpp
using Track2DId = int;       // 图像域临时轨迹编号
using Track3DId = int;       // 单装甲板 3D 轨迹编号
using RobotTrackId = int;    // 完整机器人对象编号
using ArmorId = int;         // 机器人内部真实装甲板编号
using DetectionId = int;     // 当前帧检测编号
```

需要明确：

```text
tracker_id ≠ panel_id ≠ armor_id ≠ robot_track_id
```

其中：

- `tracker_id`：2D 或 3D 跟踪器内部的临时编号。
- `panel_id`：结构化几何模型中的候选面板编号。
- `armor_id`：最终希望判别出的机器人内部装甲板编号。
- `robot_track_id`：完整机器人对象 tracker 的编号。

---

## 8. 数据模型设计

### 8.1 2D 检测数据

```cpp
struct Detection2D {
  DetectionId detection_id = -1;

  Rect2f bbox;
  std::array<Point2f, 4> keypoints;

  int class_id = -1;
  double confidence = 0.0;

  double timestamp = 0.0;
};

struct Detection2DBatch {
  std::vector<Detection2D> items;
};
```

### 8.2 2D Track Evidence

```cpp
struct Track2DEvidence {
  Track2DId tracker_id = -1;

  Rect2f bbox;
  std::array<Point2f, 4> keypoints;

  Point2f center;
  Point2f velocity_2d;

  int class_id = -1;
  double detection_confidence = 0.0;
  double track_confidence = 0.0;

  int age = 0;
  int hit_count = 0;
  int miss_count = 0;

  double timestamp = 0.0;
};

struct Track2DEvidenceBatch {
  std::vector<Track2DEvidence> items;
};
```

### 8.3 PnP Evidence

```cpp
struct PnPEvidence {
  Track2DId tracker_id = -1;

  Vec3 position_cam;
  Vec3 position_odom;

  double armor_yaw = 0.0;
  double armor_pitch = 0.0;
  double armor_roll = 0.0;

  double reprojection_error = 0.0;
  double pnp_confidence = 0.0;

  double timestamp = 0.0;
};

struct PnPEvidenceBatch {
  std::vector<PnPEvidence> items;
};
```

### 8.4 3D Track Evidence

```cpp
struct Track3DEvidence {
  Track2DId tracker_id = -1;
  Track3DId track3d_id = -1;

  Vec3 position_odom;
  Vec3 velocity_odom;

  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;

  double z_mean = 0.0;
  double z_var = 0.0;

  double innovation_norm = 0.0;
  double track_confidence = 0.0;

  double timestamp = 0.0;
};

struct Track3DEvidenceBatch {
  std::vector<Track3DEvidence> items;
};
```

### 8.5 Relation Evidence

```cpp
struct RelationEvidence {
  Track2DId tracker_id_a = -1;
  Track2DId tracker_id_b = -1;

  double height_diff = 0.0;
  double distance_diff = 0.0;
  double yaw_phase_diff = 0.0;

  double confidence = 0.0;
};

struct RelationEvidenceBatch {
  std::vector<RelationEvidence> items;
};
```

### 8.6 Discriminator 输出

```cpp
struct DiscriminatorOutput {
  Track2DId tracker_id = -1;

  std::string source_name;

  std::array<double, 4> id_score{};
  std::array<double, 4> id_prob{};

  int best_id = -1;
  double confidence = 0.0;

  bool reliable = false;
  std::string reason;
};

struct DiscriminatorOutputBatch {
  std::vector<DiscriminatorOutput> items;
};
```

### 8.7 ArmorIdDecision

```cpp
struct ArmorIdDecision {
  Track2DId tracker_id = -1;

  ArmorId best_armor_id = -1;
  std::array<double, 4> armor_id_prob{};

  double confidence = 0.0;
  bool stable = false;

  std::string reason;
};

struct ArmorIdDecisionBatch {
  std::vector<ArmorIdDecision> items;
};
```

### 8.8 BackendDecision

```cpp
enum class BackendMode {
  NO_TARGET,
  TWO_D_ONLY,
  AMBIGUOUS_SINGLE_ARMOR,
  STRUCTURED_ROBOT,
  REACQUIRE
};

struct BackendDecision {
  BackendMode mode = BackendMode::NO_TARGET;

  Track2DId tracker_id = -1;
  RobotTrackId robot_track_id = -1;
  ArmorId reference_armor_id = -1;

  bool create_structured_robot = false;
  bool update_structured_robot = false;
  bool update_single_armor = false;

  bool publish_robot_state = false;
  bool publish_single_armor_state = false;

  double armor_id_confidence = 0.0;
  double structure_confidence = 0.0;
  double output_confidence = 0.0;

  std::string reason;
};
```

### 8.9 TrackerCommand

```cpp
enum class TrackerCommandType {
  UPDATE_2D_TRACK,
  CREATE_SINGLE_ARMOR_TRACK,
  UPDATE_SINGLE_ARMOR_TRACK,
  CREATE_STRUCTURED_ROBOT,
  UPDATE_STRUCTURED_ROBOT,
  DOWNGRADE_TO_AMBIGUOUS,
  DELETE_TRACK
};

struct TrackerCommand {
  TrackerCommandType type;

  Track2DId tracker_id = -1;
  RobotTrackId robot_track_id = -1;
  ArmorId reference_armor_id = -1;

  BackendDecision decision;
  double confidence = 0.0;
  std::string reason;
};

struct TrackerCommandBatch {
  std::vector<TrackerCommand> commands;
};
```

### 8.10 TargetCandidate

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

  Vec3 aim_position_odom;
  Vec3 aim_velocity_odom;

  std::optional<Vec3> robot_center_odom;
  std::optional<double> robot_center_yaw;
  std::optional<double> robot_yaw_rate;

  double confidence = 0.0;
  double prediction_horizon_validity = 0.0;
  double timestamp = 0.0;
};

struct TargetCandidateBatch {
  std::vector<TargetCandidate> items;
};
```

---

## 9. 节点元数据设计

### 9.1 NodeKind

```cpp
enum class NodeKind {
  SOURCE,
  EVIDENCE,
  DISCRIMINATOR,
  FUSER,
  DECISION,
  ADAPTER,
  SINK,
  DEBUG
};
```

### 9.2 PortSpec

```cpp
struct PortSpec {
  std::string name;
  std::string type_name;
  bool optional = false;
};
```

也可以使用 `std::type_index` 做更强类型检查：

```cpp
struct PortSpec {
  std::string name;
  std::type_index type;
  bool optional = false;
};
```

### 9.3 NodeMeta

```cpp
struct NodeMeta {
  std::string name;
  NodeKind kind;
  std::string version;

  std::vector<PortSpec> inputs;
  std::vector<PortSpec> outputs;

  bool stateful = false;
  bool realtime_critical = true;
};
```

示例：

```cpp
NodeMeta HeightDiscriminatorMeta() {
  return {
    .name = "HeightDiscriminator",
    .kind = NodeKind::DISCRIMINATOR,
    .version = "1.0.0",
    .inputs = {
      {"track3d_evidence", "Track3DEvidenceBatch", false}
    },
    .outputs = {
      {"height_disc_output", "DiscriminatorOutputBatch", false}
    },
    .stateful = false,
    .realtime_critical = true
  };
}
```

---

## 10. PortBundle 设计

DAG 中的数据传递建议使用强类型 `PortBundle`。

```cpp
class PortBundle {
public:
  template <typename T>
  void set(const std::string& key, std::shared_ptr<const T> value);

  template <typename T>
  std::shared_ptr<const T> get(const std::string& key) const;

  bool has(const std::string& key) const;
};
```

为了减少复制，建议 port 中传递 `shared_ptr<const T>`：

```text
节点输出后不再修改数据
下游节点只读输入
大型 batch 不反复复制
Debug 系统也可以安全引用
```

---

## 11. GraphNodeAdapter：组合式节点外壳

核心思想：业务类不需要继承复杂基类，只需要组合一个 `GraphNodeAdapter`，并注册一个 tick 回调。

### 11.1 回调类型

```cpp
enum class NodeStatus {
  OK,
  SKIPPED,
  WARNING,
  ERROR
};

struct NodeContext {
  uint64_t frame_id = 0;
  double timestamp = 0.0;
};

using NodeTickCallback = std::function<NodeStatus(
    const NodeContext& ctx,
    const PortBundle& inputs,
    PortBundle& outputs
)>;
```

### 11.2 GraphNodeAdapter

```cpp
class GraphNodeAdapter {
public:
  GraphNodeAdapter(NodeMeta meta, NodeTickCallback callback)
      : meta_(std::move(meta)), callback_(std::move(callback)) {}

  const NodeMeta& meta() const {
    return meta_;
  }

  NodeStatus tick(
      const NodeContext& ctx,
      const PortBundle& inputs,
      PortBundle& outputs) {
    return callback_(ctx, inputs, outputs);
  }

private:
  NodeMeta meta_;
  NodeTickCallback callback_;
};
```

这样 GraphRuntime 只认识 `GraphNodeAdapter`，不需要知道业务类具体是什么。

---

## 12. 业务类通过组合成为图节点

以高度判别器为例：

```cpp
class HeightDiscriminator {
public:
  HeightDiscriminator()
      : node_(
          makeMeta(),
          [this](const NodeContext& ctx,
                 const PortBundle& inputs,
                 PortBundle& outputs) {
            return this->onTick(ctx, inputs, outputs);
          }) {}

  GraphNodeAdapter& node() {
    return node_;
  }

private:
  static NodeMeta makeMeta() {
    return {
      .name = "HeightDiscriminator",
      .kind = NodeKind::DISCRIMINATOR,
      .version = "1.0.0",
      .inputs = {
        {"track3d_evidence", "Track3DEvidenceBatch", false}
      },
      .outputs = {
        {"height_disc_output", "DiscriminatorOutputBatch", false}
      },
      .stateful = false,
      .realtime_critical = true
    };
  }

  NodeStatus onTick(
      const NodeContext& ctx,
      const PortBundle& inputs,
      PortBundle& outputs) {
    auto evidence = inputs.get<Track3DEvidenceBatch>("track3d_evidence");

    DiscriminatorOutputBatch result;

    for (const auto& item : evidence->items) {
      result.items.push_back(scoreHeight(item));
    }

    outputs.set("height_disc_output",
                std::make_shared<DiscriminatorOutputBatch>(std::move(result)));

    return NodeStatus::OK;
  }

private:
  GraphNodeAdapter node_;
};
```

这样 `HeightDiscriminator` 仍然是一个普通 C++ 类，只是通过组合获得了 DAG 节点能力。

---

## 13. GraphRuntime 设计

GraphRuntime 负责：

```text
1. 注册节点
2. 注册边
3. 校验 port 类型
4. 检查有向无环图
5. 拓扑排序
6. 每帧执行 tick
7. 收集 debug trace
8. 输出最终结果
```

### 13.1 基本接口

```cpp
class GraphRuntime {
public:
  void addNode(std::shared_ptr<GraphNodeAdapter> node);

  void connect(
      const std::string& from_node,
      const std::string& from_port,
      const std::string& to_node,
      const std::string& to_port);

  bool compile();

  GraphResult run(const GraphInput& input);
};
```

### 13.2 每帧执行伪代码

```cpp
GraphResult GraphRuntime::run(const GraphInput& input) {
  NodeContext ctx;
  ctx.frame_id = input.frame_id;
  ctx.timestamp = input.timestamp;

  PortBundle blackboard;
  blackboard.merge(input.initial_ports);

  for (auto& node : topo_order_) {
    PortBundle node_inputs = blackboard.collect(node->meta().inputs);
    PortBundle node_outputs;

    debug_bus_->onNodeBegin(node->meta(), ctx);

    auto status = node->tick(ctx, node_inputs, node_outputs);

    debug_bus_->onNodeEnd(node->meta(), ctx, status, node_outputs);

    if (status == NodeStatus::ERROR) {
      return GraphResult::error(node->meta().name);
    }

    blackboard.merge(node_outputs);
  }

  return GraphResult::fromBlackboard(blackboard);
}
```

---

## 14. 图节点示例

### 14.1 Track2DEvidenceNode

```cpp
class Track2DEvidenceNode {
public:
  Track2DEvidenceNode()
      : node_(makeMeta(), [this](auto&& ctx, auto&& in, auto&& out) {
          return this->onTick(ctx, in, out);
        }) {}

  GraphNodeAdapter& node() { return node_; }

private:
  static NodeMeta makeMeta() {
    return {
      .name = "Track2DEvidenceNode",
      .kind = NodeKind::EVIDENCE,
      .version = "1.0.0",
      .inputs = {
        {"tracker_snapshot", "TrackerSnapshot", false},
        {"detections_2d", "Detection2DBatch", false}
      },
      .outputs = {
        {"track2d_evidence", "Track2DEvidenceBatch", false}
      },
      .stateful = false,
      .realtime_critical = true
    };
  }

  NodeStatus onTick(const NodeContext& ctx,
                    const PortBundle& in,
                    PortBundle& out) {
    auto snapshot = in.get<TrackerSnapshot>("tracker_snapshot");
    auto detections = in.get<Detection2DBatch>("detections_2d");

    auto evidence = buildTrack2DEvidence(*snapshot, *detections);

    out.set("track2d_evidence",
            std::make_shared<Track2DEvidenceBatch>(std::move(evidence)));

    return NodeStatus::OK;
  }

private:
  GraphNodeAdapter node_;
};
```

### 14.2 ArmorIdVotingNode

```cpp
class ArmorIdVotingNode {
public:
  ArmorIdVotingNode()
      : node_(makeMeta(), [this](auto&& ctx, auto&& in, auto&& out) {
          return this->onTick(ctx, in, out);
        }) {}

  GraphNodeAdapter& node() { return node_; }

private:
  static NodeMeta makeMeta() {
    return {
      .name = "ArmorIdVotingNode",
      .kind = NodeKind::FUSER,
      .version = "1.0.0",
      .inputs = {
        {"height_disc_output", "DiscriminatorOutputBatch", true},
        {"yaw_phase_disc_output", "DiscriminatorOutputBatch", true},
        {"motion_disc_output", "DiscriminatorOutputBatch", true},
        {"temporal_disc_output", "DiscriminatorOutputBatch", true}
      },
      .outputs = {
        {"armor_id_decision", "ArmorIdDecisionBatch", false}
      },
      .stateful = false,
      .realtime_critical = true
    };
  }

  NodeStatus onTick(const NodeContext& ctx,
                    const PortBundle& in,
                    PortBundle& out) {
    auto decision = fuseAllDiscriminators(in);

    out.set("armor_id_decision",
            std::make_shared<ArmorIdDecisionBatch>(std::move(decision)));

    return NodeStatus::OK;
  }

private:
  GraphNodeAdapter node_;
};
```

### 14.3 BackendModeDecisionNode

```cpp
class BackendModeDecisionNode {
public:
  BackendModeDecisionNode()
      : node_(makeMeta(), [this](auto&& ctx, auto&& in, auto&& out) {
          return this->onTick(ctx, in, out);
        }) {}

  GraphNodeAdapter& node() { return node_; }

private:
  static NodeMeta makeMeta() {
    return {
      .name = "BackendModeDecisionNode",
      .kind = NodeKind::DECISION,
      .version = "1.0.0",
      .inputs = {
        {"armor_id_decision", "ArmorIdDecisionBatch", false},
        {"track3d_evidence", "Track3DEvidenceBatch", false},
        {"relation_evidence", "RelationEvidenceBatch", true}
      },
      .outputs = {
        {"backend_decision", "BackendDecision", false},
        {"tracker_commands", "TrackerCommandBatch", false}
      },
      .stateful = false,
      .realtime_critical = true
    };
  }

  NodeStatus onTick(const NodeContext& ctx,
                    const PortBundle& in,
                    PortBundle& out) {
    auto decision = makeBackendDecision(in);
    auto commands = buildTrackerCommands(decision);

    out.set("backend_decision",
            std::make_shared<BackendDecision>(decision));
    out.set("tracker_commands",
            std::make_shared<TrackerCommandBatch>(std::move(commands)));

    return NodeStatus::OK;
  }

private:
  GraphNodeAdapter node_;
};
```

---

## 15. TrackerManager 与 DAG 的交互

### 15.1 Snapshot 读取

TrackerManager 提供只读快照：

```cpp
class TrackerManager {
public:
  TrackerSnapshot makeSnapshot(double timestamp) const;
  void applyCommands(const TrackerCommandBatch& commands);
};
```

Snapshot 中可以包含：

```cpp
struct TrackerSnapshot {
  uint64_t frame_id = 0;
  double timestamp = 0.0;

  std::vector<Track2DSnapshot> tracks_2d;
  std::vector<SingleArmor3DSnapshot> single_armor_tracks;
  std::vector<StructuredRobotSnapshot> robot_tracks;
};
```

### 15.2 Command 写入

DAG 只输出命令：

```cpp
TrackerCommandBatch commands = result.get<TrackerCommandBatch>("tracker_commands");
tracker_manager.applyCommands(commands);
```

TrackerManager 内部根据命令决定：

```text
更新 2D tracker
创建 AmbiguousSingleArmorFilterAdapter
更新 AmbiguousSingleArmorFilterAdapter
创建 DualRadiusSpinUKF
更新 DualRadiusSpinUKF
Structured 降级为 Ambiguous
删除失效 tracker
```

---

## 16. Debug 系统与 DAG 的结合

统一 DAG 的一个重要好处是 Debug 系统可以天然接入。

DebugBus 可以记录：

```text
节点开始执行
节点结束执行
节点输入摘要
节点输出摘要
节点耗时
节点状态
节点错误
图结构
每帧决策链路
```

### 16.1 DebugEvent

```cpp
enum class DebugEventType {
  FRAME_BEGIN,
  FRAME_END,
  NODE_BEGIN,
  NODE_END,
  NODE_ERROR,
  TRACKER_COMMAND,
  BACKEND_DECISION,
  TARGET_OUTPUT
};

struct DebugEvent {
  DebugEventType type;

  uint64_t frame_id = 0;
  double timestamp = 0.0;

  std::string node_name;
  std::string message;

  std::unordered_map<std::string, double> scalars;
  std::unordered_map<std::string, std::string> strings;
};
```

### 16.2 DebugBus

```cpp
class IDebugBus {
public:
  virtual ~IDebugBus() = default;
  virtual void emit(const DebugEvent& event) = 0;
  virtual bool enabled(DebugEventType type) const = 0;
};

class NoopDebugBus : public IDebugBus {
public:
  void emit(const DebugEvent&) override {}
  bool enabled(DebugEventType) const override { return false; }
};
```

### 16.3 DebugSink

```cpp
class IDebugSink {
public:
  virtual ~IDebugSink() = default;
  virtual void consume(const DebugEvent& event) = 0;
};
```

可选 Sink：

```text
ConsoleDebugSink
RosTopicDebugSink
JsonFileDebugSink
CsvMetricDebugSink
GraphvizDebugSink
ImageOverlayDebugSink
FoxgloveDebugSink
```

### 16.4 可重构调用链

基于 NodeMeta 和 DebugEvent，可以自动得到：

```text
Frame 1024
  Track2DEvidenceNode           OK    0.21 ms
  PnPEvidenceNode               OK    0.32 ms
  Track3DEvidenceNode           OK    0.09 ms
  HeightDiscriminator           OK    0.03 ms
  YawPhaseDiscriminator         OK    0.04 ms
  ArmorIdVotingNode             OK    0.02 ms
  BackendModeDecisionNode       OK    0.05 ms
```

这可以直接回答：

```text
某一帧哪个 Evidence 出了异常？
哪个 Discriminator 给了高分？
Voting 是怎么融合的？
BackendDecision 为什么创建 / 不创建 DualRadiusSpinUKF？
最终为什么发布单装甲板状态而不是完整机器人状态？
```

---

## 17. YAML 图配置

建议支持通过 YAML 描述 DAG，便于切换不同判别器组合。

示例：

```yaml
nodes:
  - name: Track2DEvidenceNode
    type: Track2DEvidenceNode
    kind: EVIDENCE

  - name: PnPEvidenceNode
    type: PnPEvidenceNode
    kind: EVIDENCE

  - name: HeightDiscriminator
    type: HeightDiscriminator
    kind: DISCRIMINATOR

  - name: YawPhaseDiscriminator
    type: YawPhaseDiscriminator
    kind: DISCRIMINATOR

  - name: ArmorIdVotingNode
    type: ArmorIdVotingNode
    kind: FUSER

  - name: BackendModeDecisionNode
    type: BackendModeDecisionNode
    kind: DECISION

edges:
  - from: Track2DEvidenceNode.track2d_evidence
    to: PnPEvidenceNode.track2d_evidence

  - from: PnPEvidenceNode.pnp_evidence
    to: HeightDiscriminator.pnp_evidence

  - from: PnPEvidenceNode.pnp_evidence
    to: YawPhaseDiscriminator.pnp_evidence

  - from: HeightDiscriminator.height_disc_output
    to: ArmorIdVotingNode.height_disc_output

  - from: YawPhaseDiscriminator.yaw_phase_disc_output
    to: ArmorIdVotingNode.yaw_phase_disc_output

  - from: ArmorIdVotingNode.armor_id_decision
    to: BackendModeDecisionNode.armor_id_decision
```

通过配置可以切换：

```text
只用 HeightDiscriminator
Height + YawPhase
Height + YawPhase + Motion
规则融合
加权投票融合
贝叶斯融合
学习式融合
```

---

## 18. 与 ROS2 的对应关系

| ROS2 概念 | 进程内 DAG 对应概念 |
|---|---|
| package | `armor_tracker_model` |
| msg | `Detection2D`, `PnPEvidence`, `BackendDecision` |
| node | `GraphNodeAdapter` |
| topic | `PortSpec` |
| callback | `NodeTickCallback` |
| executor | `GraphRuntime` |
| parameter | `NodeConfig` |
| launch graph | DAG YAML 配置文件 |
| rqt_graph | Graphviz / DebugGraph |

建议整体仍作为一个 ROS2 Component 运行：

```text
外部：ROS2 topic / service / parameter / lifecycle
内部：in-process DAG Runtime
```

不要把每个 Evidence / Discriminator 都做成独立 ROS2 node，否则会带来：

```text
频繁序列化
额外调度延迟
时间同步复杂度
debug 链路分散
90 FPS 下实时性难控
```

---

## 19. 节点设计原则

### 19.1 节点尽量无状态

推荐 Evidence / Discriminator / Fuser 尽量设计为纯函数式节点：

```text
输入 evidence
输出 evidence / score / decision
```

真正的历史状态放到：

```text
TrackerManager
TemporalStateCache
DebugRecorder
```

### 19.2 有状态节点必须显式标记

如果某个节点维护历史窗口或内部滤波状态，应标记：

```cpp
NodeMeta {
  .stateful = true
}
```

这样 Debug 和 Replay 系统才知道该节点不是纯函数。

### 19.3 DAG 内不要出现环

如果需要上一帧结果影响当前帧，不要在图里做反馈环，而是通过 snapshot 注入：

```text
上一帧状态
→ TrackerManager snapshot
→ 当前帧 DAG 输入
```

### 19.4 所有写操作都通过 Command

例如：

```text
创建完整机器人对象
更新 DualRadiusSpinUKF
更新 AmbiguousSingleArmorFilterAdapter
降级 structured robot
删除 tracker
```

都应通过：

```cpp
TrackerCommandBatch
```

间接执行。

### 19.5 输出必须带置信度和 reason

例如：

```cpp
struct ArmorIdDecision {
  int tracker_id;
  int best_armor_id;
  std::array<double, 4> prob;
  double confidence;
  bool stable;
  std::string reason;
};
```

这对后续排查问题非常关键。

---

## 20. 与现有 Norm4 / Adaptive 实现的迁移关系

当前系统中已有部分结构可直接迁移：

```text
ObservationFrontend
Norm4BinderBridge
EvidenceFuser + ModeFSM
Norm4AmbiguousBackend
Norm4StructuredBackend
BackendUpdateHint
Norm4OutputAdapter
```

建议迁移方式：

1. `ObservationFrontend` 包装为 Evidence 节点或 CandidateEvidence 节点。
2. `Norm4BinderBridge` 包装为 TemporalBindingDiscriminator 节点。
3. `EvidenceFuser + ModeFSM` 包装为 BackendModeDecisionNode 或 ModeDiscriminator 节点。
4. `BackendUpdateHint` 保留，作为 TrackerCommand 的一部分。
5. `Norm4AmbiguousBackend` 与 `Norm4StructuredBackend` 移入 TrackerManager 管理。
6. `Norm4OutputAdapter` 包装为 PostUpdateGraph 中的 TargetCandidateAdapterNode。
7. `AdaptiveArmorTracker` 作为 legacy 对照，不建议作为新架构主线。

---

## 21. 推荐迁移步骤

### 阶段 1：引入 model 包

先创建：

```text
armor_tracker_model
```

定义：

```text
ID 类型
Evidence 数据结构
DiscriminatorOutput
BackendDecision
TrackerCommand
TargetCandidate
NodeMeta / PortSpec / GraphNodeAdapter
```

此阶段不改主流程。

### 阶段 2：包装现有模块为 GraphNodeAdapter

优先包装：

```text
ObservationFrontend → CandidateEvidenceNode
Norm4BinderBridge → TemporalBindingDiscriminatorNode
EvidenceFuser + ModeFSM → BackendModeDecisionNode
Norm4OutputAdapter → TargetCandidateAdapterNode
```

此阶段允许仍由旧主流程调用，但开始输出统一数据结构。

### 阶段 3：引入 GraphRuntime

将固定调用链改为：

```text
GraphRuntime.addNode()
GraphRuntime.connect()
GraphRuntime.compile()
GraphRuntime.run()
```

先跑最小图：

```text
CandidateEvidenceNode
→ TemporalBindingDiscriminatorNode
→ BackendModeDecisionNode
```

### 阶段 4：将 Backend 移入 TrackerManager

把：

```text
AmbiguousSingleArmorFilterAdapter
DualRadiusSpinUKF
```

统一由 TrackerManager 管理，DAG 只输出 TrackerCommand。

### 阶段 5：加入 2D Tracker Evidence

补上 PnP 前的：

```text
Detection2D
→ Track2DEvidence
→ PnPEvidence
```

让后续 3D evidence 带有稳定的 `tracker_id`。

### 阶段 6：加入更多 Discriminator

逐步加入：

```text
HeightDiscriminator
YawPhaseDiscriminator
MotionConsistencyDiscriminator
RelationDiscriminator
TemporalConsistencyDiscriminator
```

然后用 `ArmorIdVotingNode` 融合。

### 阶段 7：完善 Debug 系统

加入：

```text
DebugBus
JsonTraceSink
GraphvizSink
RosDebugTopicSink
DebugImageSink
```

实现离线复盘和调用链重构。

---

## 22. 最终总结

推荐最终架构为：

```text
TrackerManager 保持图外，作为唯一状态资产管理器；
Evidence / Discriminator / Fuser / Decision / Output / Debug 统一放入进程内 DAG；
每个业务类通过组合持有 GraphNodeAdapter；
GraphNodeAdapter 持有 NodeMeta 和 TickCallback；
GraphRuntime 根据 NodeMeta 和 PortSpec 自动调度节点；
DAG 只输出 TrackerCommand，不直接修改 tracker；
DebugBus 基于 DAG 自动记录节点输入输出、耗时和决策链路。
```

一句话概括：

```text
这相当于在 ROS2 Component 内部实现一个轻量级、强类型、可调试、可配置的 DAG executor；业务类只负责算法，节点元数据和回调让它被组合进图中运行。
```

