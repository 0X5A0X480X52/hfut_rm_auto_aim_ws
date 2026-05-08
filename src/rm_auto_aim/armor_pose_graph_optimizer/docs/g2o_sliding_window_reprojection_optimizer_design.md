# g2o 滑窗重投影优化模块设计

## 1. 背景与目标

`armor_detector` 和 `armor_detector_nn` 当前都以 PnP 作为装甲板三维位姿估计的基础。传统 `armor_detector` 已有基于 g2o 的单帧 yaw BA，`armor_detector_nn` 已有手写的 `single_yaw` 与 `sliding_window` 位姿精修链路。两者都已经具备关键点、相机内参、PnP 初值、姿态回退等基础，但优化实现分散，接口不统一。

本模块拟设计为一个独立公共库：`armor_pose_graph_optimizer`，为后续两个 detector 共用提供统一的 g2o 滑窗重投影优化能力。

核心目标：

1. 统一单帧与滑窗装甲板位姿精修接口。
2. 使用 g2o 实现重投影误差优化，降低引入 GTSAM 的依赖成本。
3. 支持 `armor_detector_nn` 现有 `IPoseRefiner` 链路平滑接入。
4. 支持传统 `armor_detector` 从旧 `BaSolver` 迁移到公共优化器。
5. 保留 PnP 作为兜底，任何优化失败不得阻塞检测发布。

非目标：

1. 本模块不负责装甲板检测、分类、角点提取。
2. 本模块不负责整车状态跟踪、装甲板绑定、目标选择。
3. 本模块不直接发布 ROS topic，只提供 C++ 库接口。
4. 本模块不替代后续 `armor_tracker` 或 `robot_pose_estimator` 的整车级状态估计。

## 2. 模块位置与目录规划

建议目录：

```text
src/rm_auto_aim/armor_pose_graph_optimizer/
├── CMakeLists.txt
├── package.xml
├── docs/
│   └── g2o_sliding_window_reprojection_optimizer_design.md
├── include/armor_pose_graph_optimizer/
│   ├── optimizer_config.hpp
│   ├── optimizer_types.hpp
│   ├── armor_pose_graph_optimizer.hpp
│   ├── single_frame_optimizer.hpp
│   ├── sliding_window_optimizer.hpp
│   ├── track_window_manager.hpp
│   └── g2o/
│       ├── vertices.hpp
│       └── edges.hpp
├── src/
│   ├── single_frame_optimizer.cpp
│   ├── sliding_window_optimizer.cpp
│   ├── track_window_manager.cpp
│   └── g2o/
│       ├── vertices.cpp
│       └── edges.cpp
└── test/
    ├── test_reprojection_edge.cpp
    ├── test_single_frame_optimizer.cpp
    └── test_sliding_window_optimizer.cpp
```

首阶段可以只创建文档与接口设计，后续再补齐 CMake、package、头文件和实现。

## 3. 现有基础调研结论

### 3.1 `armor_detector`

现状：

1. 已依赖 g2o，当前 `BaSolver` 使用 g2o 做单帧 yaw BA。
2. 输入来自传统灯条检测结果，`Armor::landmarks()` 当前可产生 6 点观测。
3. `ArmorPoseEstimator::extractArmorPoses()` 中先 PnP，再可选 BA。
4. 输出为 `rm_interfaces::msg::Armors`。

接入基础：良好，但需要统一输入抽象，尤其是支持 4 点和 6 点观测。

主要改造点：

1. 在 `ArmorPoseEstimator` 中构造公共 `ArmorPoseObservation`。
2. 用公共优化器替换或并列旧 `BaSolver`。
3. 增加配置参数，例如 `pose_refine.mode`、`pose_refine.use_graph_optimizer`。
4. 传统 detector 暂无稳定 `track_id`，首阶段建议只接单帧优化；滑窗优化需要后续补 track 关联。

### 3.2 `armor_detector_nn`

现状：

1. 已有 `ArmorPoseEstimatorAdapter`，输入为 `ArmorDetection`。
2. 已有 `PoseEstimate`、`IPoseRefiner`、`SingleYawRefiner`、`SlidingWindowRefiner`。
3. `ArmorDetection` 已包含 `track_id`、关键点置信度、时间戳。
4. 配置已有 `pose.refiner.mode`，当前支持 `none | single_yaw | sliding_window`。

接入基础：非常好，建议作为首个落地点。

主要改造点：

1. 新增 `G2oSingleFrameRefiner` 和 `G2oSlidingWindowRefiner` 适配 `IPoseRefiner`。
2. `pose.refiner.mode` 扩展为：`none | single_yaw | sliding_window | g2o_single | g2o_window`。
3. 用公共优化器输出填充现有 `PoseEstimate`。
4. 保留现有回退链：`g2o_window -> g2o_single -> single_yaw -> pnp` 或保守地 `g2o_window -> single_yaw -> pnp`。

### 3.3 `Tracker-manager` 复用调研与决策

本模块内部需要维护 `Tracker-manager`，用于给滑窗重投影优化提供稳定 `track_id` 和窗口生命周期管理。已对以下两处现有实现进行复用性调研：

1. `src/rm_auto_aim/armor_detector_nn` 内部 tracker
2. `src/kalmanFilters/muit_obj_tracker`

结论：

1. `armor_detector_nn` 内部 tracker 可直接复用，建议首版优先采用。
2. `muit_obj_tracker` 不建议首版直接复用，建议后续作为可选后端适配。

#### 3.3.1 `armor_detector_nn` tracker 复用评估

关键文件：

1. `src/rm_auto_aim/armor_detector_nn/include/armor_detector_nn/core/tracker/itracker_strategy.hpp`
2. `src/rm_auto_aim/armor_detector_nn/include/armor_detector_nn/core/tracker/tracker_types.hpp`
3. `src/rm_auto_aim/armor_detector_nn/src/core/tracker/internal_iou_tracker_strategy.cpp`

现有能力：

1. `associate(detections, stamp)` 直接输出带 `track_id` 的检测结果。
2. 采用 IoU + 匈牙利匹配 + 中心距离门限，适合 detector 级实时关联。
3. `TrackState` 包含 `id/age/hits/missed/last_stamp/velocity`，与滑窗管理需求高度重合。
4. 已在 `armor_detector_nn` 主链路实际使用，行为可观测可回归。

复用成本：

1. 小。主要是类型适配（当前输入为 `ArmorDetection`）。
2. 不需要引入新增重依赖。

建议：

1. 在 `armor_pose_graph_optimizer` 内部复用该策略思路，落地为默认关联后端。
2. 命名建议：`InternalIouAssociator`。

#### 3.3.2 `muit_obj_tracker` 复用评估

关键文件：

1. `src/kalmanFilters/muit_obj_tracker/include/muit_obj_tracker/tracker/itracker.hpp`
2. `src/kalmanFilters/muit_obj_tracker/include/muit_obj_tracker/tracker/tracker_manager.hpp`
3. `src/kalmanFilters/muit_obj_tracker/src/tracker/tracker_manager.cpp`
4. `src/kalmanFilters/muit_obj_tracker/CMakeLists.txt`

现有能力：

1. 提供 `SORT`、`POINT` 等实现，具备 KF 预测更新框架。
2. 通过 `TrackerManager` 支持运行时切换算法。
3. 有测试与配置体系，工程完整度较高。

主要问题：

1. 接口风格为 `predict/update/getTracks`，与本模块更适合的 `associate` 一步输出风格不一致。
2. 依赖较重：`models/basic_models/combined_models`，会显著增加新模块耦合。
3. 数据结构偏通用（bbox/3D/feature/corners 混合），首版仅为 g2o 滑窗提供 track_id 时收益不高。

结论：

1. 不建议首版直接复用 `muit_obj_tracker` 作为默认 tracker-manager。
2. 可在二期通过 adapter 形式接入，作为可选高阶运动先验后端。

#### 3.3.3 复用落地方案

为避免优化器与具体 tracker 强耦合，建议 `Tracker-manager` 拆为两层：

1. `TrackAssociator`：负责当前帧 `observation -> track_id`。
2. `TrackWindowManager`：负责按 `track_id` 管理滑窗缓存、过期清理、重置。

建议默认路径：

1. `TrackAssociator` 默认后端复用 `armor_detector_nn` 的 `InternalIoUTrackerStrategy` 思路。
2. `TrackWindowManager` 在 `armor_pose_graph_optimizer` 内部自实现并与 g2o 输入结构对齐。

二期可选路径：

1. 增加 `MuitObjTrackerAssociatorAdapter`，将 `muit_obj_tracker` 适配为 `TrackAssociator` 后端。

#### 3.3.4 接口草案（用于本模块）

```cpp
namespace armor_pose_graph_optimizer {

struct TrackAssocObservation {
  cv::Rect2f bbox;
  cv::Point2f center;
  float confidence{0.0F};
  std::string armor_type;
  std::string armor_number;
};

struct TrackAssocResult {
  TrackAssocObservation obs;
  int track_id{-1};
  bool matched{false};
  bool confirmed{false};
};

class ITrackAssociator {
public:
  virtual ~ITrackAssociator() = default;
  virtual std::vector<TrackAssocResult> associate(
      const std::vector<TrackAssocObservation>& observations,
      const rclcpp::Time& stamp) = 0;
  virtual void reset() = 0;
};

class TrackWindowManager {
public:
  void pushObservation(int track_id, const ArmorPoseObservation& obs);
  std::vector<ArmorPoseObservation> getWindow(int track_id) const;
  void removeTrack(int track_id);
  void pruneExpired(const rclcpp::Time& now);
  void reset();
};

}  // namespace armor_pose_graph_optimizer
```

约束：

1. `TrackAssociator` 不持有 g2o 状态，仅做关联。
2. `TrackWindowManager` 不做关联，仅维护窗口。
3. 优化失败只回退 pose，不影响 `track_id` 连续性。
4. 模式切换或跟踪丢失时支持按 `track_id` 精细重置窗口。

## 4. 总体架构

```text
Detector keypoints + class/type + CameraInfo + PnP result
        |
        v
ArmorPoseObservation
        |
        v
armor_pose_graph_optimizer
        |
        +--> SingleFrameOptimizer
        |       +--> g2o vertices / edges
        |
        +--> SlidingWindowOptimizer
                +--> TrackWindowManager(track_id)
                +--> fixed window g2o batch solve
        |
        v
ArmorPoseOptimizationResult
        |
        +--> armor_detector_nn: convert to PoseEstimate
        +--> armor_detector: convert to rm_interfaces::msg::Armor pose
```

模块只处理“单个装甲板实例的相机系位姿精修”。滑窗以 `track_id` 区分不同装甲板轨迹，每条轨迹维护独立窗口。

## 5. 公共数据结构设计

### 5.1 观测输入

```cpp
namespace armor_pose_graph_optimizer {

struct ArmorPoseObservation {
  int track_id{-1};
  rclcpp::Time stamp{};

  std::string armor_number;
  std::string armor_type;  // small | large

  // Canonical image/object correspondences. Supports 4 or 6 points.
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;
  std::vector<double> keypoint_sigma_px;

  cv::Mat camera_matrix;       // CV_64F 3x3
  cv::Mat dist_coeffs;         // CV_64F 1xN, optional

  // PnP initial value in camera frame.
  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};
  cv::Mat rvec;
  cv::Mat tvec;

  // Optional attitude relation used by yaw/pitch/roll constrained modes.
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};

  // Optional fixed geometry assumptions.
  double fixed_pitch_rad{0.0};
  double fixed_roll_rad{0.0};
  bool use_fixed_pitch_roll{false};
};

}  // namespace armor_pose_graph_optimizer
```

设计要点：

1. `image_points` 和 `object_points` 不固定点数，支持 `armor_detector_nn` 的 4 点与 `armor_detector` 的 6 点。
2. `keypoint_sigma_px` 可选；若为空，使用统一像素噪声。
3. `R_imu_camera` 用于保持与旧 BA / NN refiner 的 yaw 定义一致。
4. `use_fixed_pitch_roll` 用于单帧 yaw-only 和轻量滑窗模式。

### 5.2 优化输出

```cpp
struct ArmorPoseOptimizationResult {
  bool valid{false};
  enum class Mode {
    PNP_FALLBACK,
    G2O_SINGLE_POSE,
    G2O_SINGLE_YAW,
    G2O_WINDOW_POSE,
    G2O_WINDOW_XYZ_YAW
  } mode{Mode::PNP_FALLBACK};

  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  cv::Mat rvec;
  cv::Mat tvec;

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  double reproj_error_raw_px{0.0};
  double reproj_error_refined_px{0.0};
  double pose_delta_m{0.0};
  double yaw_delta_rad{0.0};
  double quality_score{0.0};

  std::string reason;
};
```

## 6. 优化模式设计

### 6.1 单帧 yaw-only 模式

用途：替代传统 `BaSolver` 和 NN `SingleYawRefiner`。

状态变量：

```text
yaw: 1 DoF
```

固定量：

1. PnP 平移 `t_camera_armor`。
2. pitch / roll 使用装甲板先验，例如默认 pitch = 15 deg, roll = 0 deg。
3. `R_imu_camera` 用于从 yaw/pitch/roll 构造 `R_camera_armor`。

边：

1. `EdgeYawReprojection`：每个关键点 2 维像素残差。
2. 可选 `EdgeYawPrior`：弱约束 yaw 不偏离 PnP 初值太远。

优点：快，适合 detector 主链路实时兜底。

缺点：不修正平移，无法改善 PnP 深度抖动。

### 6.2 单帧 Pose3 模式

用途：在关键点质量较好时优化完整 `SE3` 位姿。

状态变量：

```text
pose_camera_armor: SE3, 6 DoF
```

边：

1. `EdgePoseReprojection`：每个关键点 2 维像素残差。
2. `EdgePosePrior`：PnP 位姿弱先验，避免平面 PnP 退化导致深度漂移。

优点：可修正平移和旋转。

风险：平面目标完整 Pose3 优化数值敏感，必须使用 PnP prior 和质量门控。

建议：首版默认关闭，作为实验模式。

### 6.3 滑窗 XYZ + yaw 模式

用途：替代 `armor_detector_nn` 当前手写滑窗 BA。

每帧状态变量：

```text
state_k = [tx, ty, tz, yaw]
```

固定量：

1. 每帧 pitch / roll。
2. 每帧 `R_imu_camera`。
3. 物体点模型。

边：

1. `EdgeXyzYawReprojection`：每帧每关键点重投影残差。
2. `EdgeTranslationPrior`：PnP 平移弱先验。
3. `EdgeYawPrior`：PnP yaw 弱先验。
4. `EdgeTranslationSmooth`：相邻帧平移平滑。
5. `EdgeYawSmooth`：相邻帧 yaw 平滑。
6. 可选 `EdgeConstantVelocity`：后续加入速度变量时使用。

推荐作为 `armor_detector_nn` 首个滑窗落地模式。

### 6.4 滑窗 Pose3 模式

每帧状态变量：

```text
pose_k: SE3
```

边：

1. `EdgePoseReprojection`。
2. `EdgePosePrior`。
3. `EdgePoseSmooth` 或 `g2o::EdgeSE3`。

优点：表达统一。

风险：平面目标全姿态可观性弱，容易产生不合理 roll/pitch 或深度漂移。

建议：作为二期实验模式，不作为默认。

## 7. g2o 顶点与边设计

### 7.1 顶点

#### VertexYaw

沿用传统 `armor_detector` 中已有思路。

```cpp
class VertexYaw : public g2o::BaseVertex<1, double> {
public:
  void setToOriginImpl() override;
  void oplusImpl(const double* update) override;
};
```

`oplusImpl` 需要做角度 wrap，避免 yaw 越界。

#### VertexXyzYaw

```cpp
class VertexXyzYaw : public g2o::BaseVertex<4, Eigen::Matrix<double, 4, 1>> {
public:
  // estimate = [tx, ty, tz, yaw]
  void setToOriginImpl() override;
  void oplusImpl(const double* update) override;
};
```

#### VertexPoseSE3

可直接使用 `g2o::VertexSE3Expmap` 或 `g2o::VertexSE3`。若使用 Sophus 统一项目风格，也可以自定义 SE3 顶点，但首版建议复用 g2o 标准顶点降低实现成本。

### 7.2 重投影边

#### EdgeYawReprojection

连接：`VertexYaw`

误差：

```text
error = observed_uv - project(K, R_camera_armor(yaw, fixed_pitch, fixed_roll) * P_obj + t_pnp)
```

维度：2。

雅可比：

1. 首版可使用数值雅可比，风险低。
2. 性能不足时补解析雅可比。

#### EdgeXyzYawReprojection

连接：`VertexXyzYaw`

误差：

```text
error = observed_uv - project(K, R_camera_armor(yaw, fixed_pitch, fixed_roll) * P_obj + t_xyz)
```

维度：2。

信息矩阵：

```text
Information = diag(1 / sigma_u^2, 1 / sigma_v^2)
```

关键点置信度映射：

```text
sigma_kp = sigma_kp_min + (1 - confidence) * sigma_kp_scale
```

#### EdgePoseReprojection

连接：`VertexPoseSE3`

误差：

```text
error = observed_uv - project(K, pose_camera_armor * P_obj)
```

维度：2。

### 7.3 先验边

#### EdgeTranslationPrior

连接：`VertexXyzYaw`

误差：

```text
error = t_current - t_pnp
```

维度：3。

#### EdgeYawPrior

连接：`VertexYaw` 或 `VertexXyzYaw`

误差：

```text
error = wrap(yaw_current - yaw_pnp)
```

维度：1。

#### EdgePosePrior

连接：`VertexPoseSE3`

误差：

```text
error = log(pose_pnp^-1 * pose_current)
```

维度：6。

### 7.4 帧间平滑边

#### EdgeTranslationSmooth

连接：`VertexXyzYaw(k-1)` 与 `VertexXyzYaw(k)`。

误差：

```text
error = t_k - t_{k-1}
```

维度：3。

#### EdgeYawSmooth

误差：

```text
error = wrap(yaw_k - yaw_{k-1})
```

维度：1。

后续可扩展为匀速模型：

```text
error = x_k - (x_{k-1} + v_{k-1} * dt)
```

但首版不建议引入速度变量，保持窗口轻量。

## 8. 优化器接口设计

### 8.1 单帧优化器

```cpp
class SingleFrameOptimizer {
public:
  explicit SingleFrameOptimizer(const OptimizerConfig& config);

  ArmorPoseOptimizationResult optimizeYawOnly(
      const ArmorPoseObservation& obs);

  ArmorPoseOptimizationResult optimizePoseSE3(
      const ArmorPoseObservation& obs);
};
```

### 8.2 滑窗优化器

```cpp
class SlidingWindowOptimizer {
public:
  explicit SlidingWindowOptimizer(const OptimizerConfig& config);

  ArmorPoseOptimizationResult optimize(
      const std::vector<ArmorPoseObservation>& window);
};
```

### 8.3 轨迹窗口管理

```cpp
class TrackWindowManager {
public:
  explicit TrackWindowManager(const OptimizerConfig& config);

  void push(const ArmorPoseObservation& obs);
  std::vector<ArmorPoseObservation> getWindow(int track_id) const;
  void removeTrack(int track_id);
  void pruneExpired(const rclcpp::Time& now);
};
```

### 8.4 高层 facade

```cpp
class ArmorPoseGraphOptimizer {
public:
  explicit ArmorPoseGraphOptimizer(const OptimizerConfig& config);

  ArmorPoseOptimizationResult refineSingle(
      const ArmorPoseObservation& obs);

  ArmorPoseOptimizationResult refineWindow(
      const ArmorPoseObservation& obs);

  void resetTrack(int track_id);
  void resetAll();
};
```

## 9. 配置设计

```cpp
struct OptimizerConfig {
  std::string mode{"xyz_yaw_window"};

  int window_size{8};
  int min_window_size{4};
  double max_time_span_ms{300.0};
  double max_solver_time_ms{2.0};
  int max_iterations{10};

  double pixel_sigma{2.0};
  double huber_delta{3.0};

  double prior_sigma_xy{0.08};
  double prior_sigma_z{0.15};
  double prior_sigma_yaw_rad{0.35};

  double smooth_sigma_xy{0.05};
  double smooth_sigma_z{0.10};
  double smooth_sigma_yaw_rad{0.10};

  double max_reproj_error_px{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_rad{20.0 * M_PI / 180.0};

  bool use_robust_kernel{true};
  std::string robust_kernel{"huber"};
  bool require_positive_depth{true};
  bool require_finite{true};
};
```

`armor_detector_nn` 配置映射建议：

```yaml
pose:
  refiner:
    mode: "g2o_window"  # none | single_yaw | sliding_window | g2o_single | g2o_window
  g2o:
    window_size: 8
    min_window_size: 4
    max_time_span_ms: 300.0
    max_solver_time_ms: 2.0
    max_iterations: 10
    pixel_sigma: 2.0
    huber_delta: 3.0
    prior_sigma_xy: 0.08
    prior_sigma_z: 0.15
    prior_sigma_yaw: 0.35
    smooth_sigma_xy: 0.05
    smooth_sigma_z: 0.10
    smooth_sigma_yaw: 0.10
    max_reproj_error: 3.0
    max_pose_delta_m: 0.20
    max_yaw_delta_deg: 20.0
```

传统 `armor_detector` 配置建议：

```yaml
use_ba: true
pose_refine:
  backend: "g2o_legacy"  # g2o_legacy | graph_single | graph_window
  mode: "graph_single"
```

首版传统 detector 建议只启用 `graph_single`，避免没有 track_id 时滑窗窗口不稳定。

## 10. 质量门控与回退链

所有优化结果必须经过质量门控。

门控条件：

1. 输出数值必须 finite。
2. 所有投影点深度必须为正。
3. 平均重投影误差不得超过 `max_reproj_error_px`。
4. 优化后平移相对 PnP 不得超过 `max_pose_delta_m`。
5. 优化后 yaw 相对 PnP 不得超过 `max_yaw_delta_rad`。
6. 若求解耗时超过预算，允许提前停止，但结果仍需过门控。

推荐回退链：

`armor_detector_nn`：

```text
g2o_window -> g2o_single -> existing_single_yaw -> pnp
```

传统 `armor_detector`：

```text
graph_single -> legacy BaSolver -> pnp
```

若后续传统 detector 获得稳定 track_id：

```text
graph_window -> graph_single -> legacy BaSolver -> pnp
```

## 11. 与 `armor_detector_nn` 的接入方案

新增适配器类：

```cpp
class G2oPoseRefiner : public IPoseRefiner {
public:
  G2oPoseRefiner(const OptimizerConfig& graph_config,
                 const SingleYawConfig& sy_config,
                 const GateConfig& gate);

  PoseEstimate refine(
      const PoseEstimate& pnp_result,
      const std::array<cv::Point2f, 4>& image_points,
      const std::array<cv::Point3f, 4>& object_points,
      const cv::Mat& K,
      const cv::Mat& D) override;
};
```

接入点：

1. `ArmorDetectorNNNode::initializeParameters()` 增加 `pose.g2o.*` 参数。
2. `ArmorDetectorNNNode` 初始化 refiner 时新增：

```cpp
if (config_.pose.refiner.mode == "g2o_window") {
  pose_estimator_adapter_->setRefiner(std::make_shared<G2oPoseRefiner>(...));
}
```

3. `ArmorPoseEstimatorAdapter::estimate()` 无需大改，因为它已经通过 `IPoseRefiner` 调用精修器。
4. `PoseEstimate::mode` 可扩展：

```cpp
G2O_SINGLE_VALID
G2O_WINDOW_VALID
```

若不希望改 enum，可先复用 `SINGLE_BA_VALID` 和 `SW_BA_VALID`，但不利于调试统计。

## 12. 与 `armor_detector` 的接入方案

传统 detector 没有 `IPoseRefiner` 接口，建议两步走。

### 12.1 短期接入

在 `ArmorPoseEstimator` 中新增成员：

```cpp
std::unique_ptr<armor_pose_graph_optimizer::ArmorPoseGraphOptimizer> graph_optimizer_;
bool use_graph_optimizer_{false};
```

在 `extractArmorPoses()` 中：

1. PnP 成功后构造 `ArmorPoseObservation`。
2. 若 `use_graph_optimizer_` 开启，调用 `refineSingle()`。
3. 结果通过门控则覆盖 `R` 和 `t`。
4. 否则保留 legacy `BaSolver` 或 PnP。

### 12.2 中期接入

抽象传统 detector 的位姿估计接口，使其与 NN adapter 更接近：

```cpp
class IArmorPoseRefinerAdapter {
public:
  virtual rm_interfaces::msg::Armor refine(...) = 0;
};
```

这样后续可统一 detector 与 detector_nn 的 refiner 配置和调试指标。

## 13. CMake 与依赖规划

模块依赖：

1. `ament_cmake_auto`
2. `rclcpp`
3. `OpenCV`
4. `Eigen3`
5. `Sophus`，可选，用于 SE3/yaw 计算保持项目风格
6. `G2O`
7. `fmt`
8. `rm_utils`

CMake 链接建议：

```cmake
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  g2o_core
  g2o_stuff
  g2o_solver_dense
  g2o_solver_csparse
  g2o_types_sba
  g2o_types_slam3d
  fmt::fmt
)
```

注意：当前 `armor_detector` 内已有 `FindG2O.cmake`，新模块可复用或上移一个公共 `cmake_modules/FindG2O.cmake`，避免两个包各自维护。

## 14. 数值与坐标约定

必须统一以下约定，否则两个 detector 的优化结果会出现方向不一致：

1. 相机系位姿输出保持与现有 `rm_interfaces::msg::Armor.pose` 一致。
2. `object_points` 由调用方传入，优化器不假设左右上下顺序，但要求 `image_points[i]` 与 `object_points[i]` 严格对应。
3. `armor_detector_nn` 当前四点顺序为 `left_bottom, left_top, right_top, right_bottom`。
4. 传统 `armor_detector` 当前六点顺序为 `left_bottom, left_center, left_top, right_top, right_center, right_bottom`。
5. yaw-only 模式必须使用与旧 `BaSolver` 一致的 yaw 提取和 `R_imu_camera` 语义。
6. 若使用畸变参数，重投影边应支持畸变投影；首版可简化为使用 OpenCV `projectPoints` 计算误差，后续再实现解析投影。

## 15. 实现策略建议

### Phase 0：文档与接口

1. 创建模块目录与设计文档。
2. 明确公共数据结构、配置、接入点。

### Phase 1：单帧 yaw-only g2o

1. 实现 `VertexYaw`。
2. 实现 `EdgeYawReprojection`。
3. 实现 `SingleFrameOptimizer::optimizeYawOnly()`。
4. 接入 `armor_detector_nn` 的 `g2o_single` 模式。
5. 与现有 `single_yaw` 输出对比。

验收：

1. 平均耗时小于 1 ms。
2. 优化失败自动回退。
3. 与现有 single_yaw 结果方向一致。

### Phase 2：滑窗 XYZ + yaw g2o

1. 实现 `VertexXyzYaw`。
2. 实现 `EdgeXyzYawReprojection`。
3. 实现 prior/smooth edges。
4. 实现 `TrackWindowManager`。
5. 接入 `armor_detector_nn` 的 `g2o_window` 模式。

验收：

1. 平均耗时不超过配置预算，如 2 ms。
2. 平移与 yaw 输出较 PnP 更平滑。
3. 重投影误差不劣于 PnP。
4. ID 切换或 track 丢失时窗口能正确重置。

### Phase 3：传统 `armor_detector` 单帧接入

1. 传统 detector 构造 6 点 `ArmorPoseObservation`。
2. 接入 `graph_single`。
3. 保留 legacy `BaSolver` 回退。

验收：

1. 输出 topic 不变。
2. 与旧 BA 相比 yaw 稳定性不下降。
3. 6 点输入正常工作。

### Phase 4：Pose3 实验模式

1. 实现 `EdgePoseReprojection`。
2. 实现 Pose prior。
3. 离线 bag 评估是否值得开启。

## 16. 测试计划

### 16.1 单元测试

1. `test_reprojection_edge.cpp`
   - 给定已知 pose 和 object point，检查投影误差接近 0。
   - 对 yaw 增量做数值扰动，检查误差方向合理。

2. `test_single_frame_optimizer.cpp`
   - 生成合成装甲板点。
   - 加入 1-3 px 噪声。
   - 检查优化后 yaw error 下降。

3. `test_sliding_window_optimizer.cpp`
   - 生成连续运动轨迹。
   - 加入随机像素噪声和少量 outlier。
   - 检查窗口输出平滑度和误差。

### 16.2 集成测试

1. 对同一段 bag 同时运行 `pnp`、`single_yaw`、`g2o_single`、`g2o_window`。
2. 记录：
   - 每帧耗时。
   - 平均重投影误差。
   - yaw 抖动标准差。
   - 平移抖动标准差。
   - 回退次数。
   - track reset 次数。

### 16.3 实车/视频验证

1. 静止装甲板：检查输出稳定性。
2. 横向移动目标：检查平移延迟与平滑性。
3. 旋转目标：检查 yaw 跟随性。
4. 遮挡/误检：检查回退与窗口重置。

## 17. 风险与对策

1. 风险：滑窗优化引入延迟或超时。
   - 对策：设置 `max_solver_time_ms`，超时回退上一层结果。

2. 风险：平面目标深度漂移。
   - 对策：PnP translation prior 必须开启，Z 方向 prior 可强于 XY。

3. 风险：track_id 切换导致窗口污染。
   - 对策：track_id 变化自动新建窗口；检测到 pose delta 异常时清空该 track 窗口。

4. 风险：传统 detector 6 点与 NN 4 点顺序不一致。
   - 对策：统一由调用方传入一一对应的 object/image vectors，模块不重排。

5. 风险：g2o 手写边数值错误。
   - 对策：首版用数值雅可比或 OpenCV 投影校验；补充单元测试。

6. 风险：优化结果比 PnP 更差。
   - 对策：严格质量门控，永远允许 PnP fallback。

## 18. 推荐默认策略

首版推荐默认：

```text
armor_detector_nn: pose.refiner.mode = g2o_single 或 single_yaw，g2o_window 实验开启
armor_detector: pose_refine.backend = graph_single，legacy BaSolver 作为 fallback
```

稳定后推荐：

```text
armor_detector_nn: g2o_window -> g2o_single -> pnp
armor_detector: graph_single -> pnp
```

不建议首版默认启用 Pose3 全自由度滑窗。对于平面装甲板，`XYZ + yaw + fixed pitch/roll` 是更稳妥的工程折中。

## 19. 后续扩展

1. 加入速度变量，实现 CV 模型滑窗。
2. 引入 per-armor-type 的 pitch/roll 先验。
3. 对 `robot_pose_estimator` 暴露更稳定的观测协方差。
4. 与角点精修模块共享关键点质量评分。
5. 将 detector 级滑窗输出与 tracker 级整车状态估计解耦评估，避免重复滤波。

## 20. 总结

用 g2o 实现滑窗重投影优化是当前工程最现实的方案。它复用已有 g2o 依赖，避免新引入 GTSAM，同时可以覆盖 detector 级位姿精修的主要需求。

建议优先落地 `armor_detector_nn`，因为其 refiner 接口、track_id、配置链路已经齐备；随后将公共优化器以单帧模式接入传统 `armor_detector`。整个模块必须始终坚持可开关、可回退、质量门控和不阻塞发布的原则。
