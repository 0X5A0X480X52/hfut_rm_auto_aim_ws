# 三层 Tracker 家族下沉到 gimbal_pipeline/src 子模块的可行性调研

## 1. 结论

可行，且风险可控。

基于当前实现结构，建议将三层 tracker 家族先在 `src/rm_auto_aim/gimbal_pipeline/src` 内部共址成子模块，使用 adapter 复用现有滤波链路，不立即改动核心滤波算法。

同时，在 `src/rm_auto_aim/gimbal_pipeline/src/common/robot_description` 可引入“代理式状态估计”侧车模块，用于消费同一批滤波能力（先以只读/弱耦合方式接入），后续再决定是否提升为主路径。

## 2. 现状调研证据（代码结构）

### 2.1 目录层面

当前已存在：

```text
src/gimbal_pipeline/src/max_entropy_tracker/
  association/
  binder/
  filters/
  mode/
  trackers/
```

`CMakeLists.txt` 使用 `file(GLOB_RECURSE "src/*.cpp" "src/**/*.cpp")`，因此新增 `src` 子目录会自动编译，无需额外手动加源文件列表。

### 2.2 现有滤波器与追踪器关系

- single-armor：`AmbiguousSingleArmorFilterAdapter`
- structured-robot：`DualRadiusSpinUKF`、`OutpostSpinUKF`
- tracker 外壳：`BaseTracker` / `TrackerManager`

说明：已有滤波器能力是稳定资产，适合被 adapter 包装后纳入三层 tracker 家族。

### 2.3 robot_description 可扩展性

`common/robot_description` 已有策略模式（`ITrackedRobotBuilderStrategy` + registry + facade）。
这天然支持新增“代理式估计 builder/estimator provider”，且不破坏现有 builder。

## 3. 建议的 src 子模块布局（先共址）

建议在 `src/rm_auto_aim/gimbal_pipeline/src/max_entropy_tracker` 下新增：

```text
tracker_family/
  armor2d/
    detectors_to_2d_observations.cpp
    iou_armor2d_tracker.cpp
    sort_armor2d_tracker_adapter.cpp          # 后续可选
    armor2d_tracker_manager.cpp

  armor3d_single/
    ambiguous_single_tracker_adapter.cpp      # 包 AmbiguousSingleArmorFilterAdapter
    single_tracker_proxy_manager.cpp
    single_tracker_dynamics_summary.cpp

  robot3d_structured/
    dual_radius_robot_tracker_adapter.cpp     # 包 DualRadiusSpinUKF
    outpost_robot_tracker_adapter.cpp         # 包 OutpostSpinUKF
    robot_tracker_manager.cpp

  backend/
    backend_intent.cpp
    backend_execution_plan.cpp
    backend_planner.cpp
    backend_executor.cpp
```

对应头文件先放 `include/max_entropy_tracker/tracker_family/...`。

## 4. 适配现有链路方式

### 4.1 关键原则

1. adapter 只做接口翻译与生命周期管理，不改滤波数学。
2. 单板状态唯一写入口保持在 single tracker manager（防重复 update）。
3. structured robot tracker 与 single tracker 状态隔离，避免交叉写。

### 4.2 最小接口草案

```cpp
class IArmor2DTracker {
 public:
  virtual ~IArmor2DTracker() = default;
  virtual std::vector<Armor2DTrackEvidence> update(
      const std::vector<Armor2DDetection>& detections,
      double timestamp) = 0;
};

class IArmor3DSingleTracker {
 public:
  virtual ~IArmor3DSingleTracker() = default;
  virtual void predict(double dt) = 0;
  virtual void update(const ObservationData& obs, double pos_conf, double yaw_conf) = 0;
  virtual SingleArmorState snapshot() const = 0;
};

class IRobotStructuredTracker {
 public:
  virtual ~IRobotStructuredTracker() = default;
  virtual void predict(std::optional<double> dt) = 0;
  virtual bool update_single(const ObservationData& obs, int panel_id,
                             binder::HeightLabel h, double pos_conf,
                             double bind_conf) = 0;
  virtual bool update_dual(const std::vector<ObservationData>& obs,
                           const std::vector<int>& panel_ids,
                           const std::vector<binder::HeightLabel>& hs,
                           double pos_conf, double bind_conf) = 0;
  virtual RobotStructuredSnapshot snapshot() const = 0;
};
```

### 4.3 Backend 语义固定

- `BackendPlanner`：`Evidence/Binding/Mode` -> `BackendIntent`
- `BackendExecutor`：`BackendIntent` -> `BackendExecutionPlan` -> 执行

特别约束：

```text
target == PROXY_SNAPSHOT 时只读取 single proxy snapshot，不调用 single filter update
```

## 5. robot_description 中“代理式状态估计”可行方案

目标：让 `common/robot_description` 也能用同类滤波能力进行状态估计或修正，但不直接侵入 tracker 主循环。

### 5.1 推荐接入方式（侧车）

新增子模块：

```text
src/common/robot_description/estimation/
  tracked_robot_state_proxy.hpp/.cpp
  tracked_robot_estimator_registry.hpp/.cpp
  strategy/filter_backed_tracked_robot_builder.hpp/.cpp
```

思路：

1. 输入：`TrackedRobotBuildInput`（已有）+ 可选 tracker snapshot。
2. `TrackedRobotStateProxy` 内部维护轻量滤波状态（按 robot_id）。
3. 在 build 阶段做“输出态平滑/补全/一致性修正”，不反向修改 `BaseTracker`。
4. 通过 facade registry 按 robot_type/robot_id 选择是否启用。

### 5.2 为什么可行

- 现有 builder 策略就是注入点，增量小。
- `TrackedRobotUsage` 已提供标准化访问器与 predict 辅助。
- 可先只做 post-build 校正，不改变主追踪拓扑。

## 6. 分阶段落地（建议）

### P0 结构落位（无行为变化）

1. 建立 `tracker_family/*` 目录和空壳接口。
2. 添加 adapter 壳文件，内部直接转调现有 filter。
3. 编译通过，单元测试不变。

### P1 单板代理管理迁移

1. `SingleArmorProxyManager` 下沉到 `tracker_family/armor3d_single`。
2. Norm4 ambiguous 输出改为读取 proxy snapshot。
3. 验证无重复 update。

### P2 structured tracker manager 迁移

1. `DualRadiusSpinUKF` / `OutpostSpinUKF` 走 `robot3d_structured` adapter。
2. 引入 `BackendPlanner/Executor`。
3. 保持 API 与现有 `BaseTracker` 外壳兼容。

### P3 robot_description 侧车试点

1. 增加 `estimation/` 子模块。
2. 只在一个 robot_type 上灰度启用 filter-backed builder。
3. 观察输出 jitter、延迟、一致性收益。

## 7. 风险与规避

1. 风险：目录迁移引发 include 路径混乱。
- 规避：先新建 adapter + 转发，不立即移动旧 filter 文件。

2. 风险：single/structured 双路径重复更新。
- 规避：以 manager 写权限为准，代码层禁止 executor 更新 single filter。

3. 风险：robot_description 侧车影响实时性。
- 规避：先做可关闭开关 + 固定上限耗时。

4. 风险：术语不一致导致实现偏差。
- 规避：统一采用 `BackendIntent/BackendExecutionPlan/Planner/Executor` 术语。

## 8. 实施可行性评级

- 工程可行性：高
- 侵入风险：中低（采用 adapter 迁移）
- 性能风险：中（需 profile）
- 回滚难度：低（可配置开关 + 保留旧路径）

总体建议：按“共址 + adapter + 分阶段灰度”推进，当前阶段值得实施。
