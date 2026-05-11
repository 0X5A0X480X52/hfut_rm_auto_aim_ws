# Armor PnP Refiner 修正版设计文档

> 原文档：`g2o 滑窗重投影优化模块设计`  
> 本修正版将模块从“g2o 滑窗重投影优化模块 / pose graph optimizer”重新定位为 **PnP 微调包**：输入 PnP 结果，输出 PnP 风格结果；内部可以使用 g2o 单帧优化、滑动窗口优化和短期数据关联，但这些实现细节对 detector 与后续 pipeline 透明。

---

## 0. 修正摘要

相较于原设计，本文档做如下关键修正：

1. **模块重定位**：从 `armor_pose_graph_optimizer` 调整为 `armor_pnp_refiner`，语义上明确为 PnP 后处理 / PnP 微调包。
2. **接口重定位**：对外采用 `PnP in -> PnP out` 的透明注入式接口，使用方仍按 PnP 结果消费。
3. **内部短期关联保留在包内**：由于 `armor_detector` 当前未实现 track_id，`armor_detector_nn` 已实现 track_id，未来还可能有其他 detector，因此内部实现轻量 `ShortTermAssociator` 更统一。
4. **内部关联语义修正**：内部关联只生成 `refine_track_id`，只服务于滑窗微调，不具有系统级 tracker 语义。
5. **优化目标修正**：滑窗优化不输出最终目标状态，只输出当前帧更高质量的 PnP 观测。
6. **平滑模型修正**：默认由一阶平滑改为二阶有限差分平滑，三阶有限差分作为可选弱正则。
7. **窗口参数修正**：90 FPS 左右建议默认 `3~5` 帧，`max_time_span_ms` 建议约 `60 ms`，不建议默认 8 帧 / 300 ms。
8. **协方差与置信度补充**：输出 `R_xyz_yaw`、`confidence`、`GOOD / DEGRADED / REJECT` 状态，便于后端 UKF/InEKF 使用。
9. **风险修正**：补充 Hessian 协方差过度自信、固定 pitch/roll 系统偏差、重复平滑、内部关联窗口污染、优化失败影响主流程等风险与对策。

---

## 1. 背景与目标

`armor_detector` 和 `armor_detector_nn` 当前都以 PnP 作为装甲板三维位姿估计的基础。

现有情况：

1. 传统 `armor_detector` 已有基于 g2o 的单帧 yaw BA，但没有稳定的 detector 内部 track_id。
2. `armor_detector_nn` 已有 `single_yaw` 与 `sliding_window` 位姿精修链路，并且已有 detector 级 `track_id`。
3. 两个 detector 都具备关键点、相机内参、PnP 初值、姿态回退等基础信息。
4. 当前优化实现分散，接口不统一，难以复用到未来其他 detector。

因此，本模块重新设计为独立公共库：

```text
armor_pnp_refiner
```

核心目标：

1. 统一单帧与滑窗 PnP 微调接口。
2. 对外保持 `PnP in -> PnP out` 语义，便于透明接入现有 detector。
3. 内部使用 g2o 实现重投影误差优化，降低引入 GTSAM 的依赖成本。
4. 内部实现轻量短期数据关联与滑窗缓存，使无 track_id 的 detector 也能使用滑窗优化。
5. 支持 `armor_detector_nn` 已有 `IPoseRefiner` 链路平滑接入。
6. 支持传统 `armor_detector` 从旧 `BaSolver` 迁移到公共 PnP 微调器。
7. 输出 refined PnP、协方差、置信度和诊断信息。
8. 保留原始 PnP 作为兜底，任何优化失败不得阻塞检测发布。

---

## 2. 模块定位

### 2.1 推荐定位

本模块是：

```text
Detector 级 PnP 微调包 / PnP Refinement Package
```

它接收 detector 已经计算出的 PnP 初值，并结合图像关键点、相机内参、时间戳和可选 track 信息，对 PnP 结果进行单帧或滑窗微调。

对外语义：

```text
PnP result in
    ↓
PnP refiner
    ↓
PnP-like result out
```

### 2.2 本模块负责

1. 接收 PnP 初值。
2. 接收关键点 2D/3D 对应关系。
3. 接收相机内参和畸变参数。
4. 可选接收 detector 外部 track_id。
5. 在包内部完成短期数据关联。
6. 维护每个 `refine_track_id` 的短期滑窗。
7. 执行单帧 yaw-only / 单帧 xyz-yaw / 滑窗 xyz-yaw 优化。
8. 输出 refined PnP pose。
9. 输出当前帧 `xyz + yaw` 协方差。
10. 输出结果置信度和质量状态。
11. 优化失败时透明回退原始 PnP。

### 2.3 本模块不负责

1. 不负责装甲板检测。
2. 不负责装甲板分类。
3. 不负责关键点提取。
4. 不负责整车状态跟踪。
5. 不负责装甲板绑定到机器人中心。
6. 不负责目标选择。
7. 不负责弹道解算。
8. 不直接发布 ROS topic。
9. 不替代后续 `armor_tracker`、`robot_pose_estimator`、`gimbal_pipeline`。
10. 不输出系统级 `target_id` 或 `robot_id`。

---

## 3. 总体架构

### 3.1 外部数据流

```text
Detector
  ↓
keypoints / bbox / class / confidence
  ↓
PnP Solver
  ↓
raw PnP result
  ↓
ArmorPnpRefiner
  ↓
refined PnP result
  ↓
原 detector 输出链路
```

对现有流程的修改尽量保持为一行注入：

```cpp
pnp_result = pnp_refiner->refine(pnp_input);
```

若优化失败：

```text
refine failed -> return original PnP result
```

也就是说：

```text
PnP 成功，refine 失败，不影响发布。
```

### 3.2 内部架构

```text
ArmorPnpRefiner
  ├── SingleFrameRefiner
  │     ├── yaw-only refine
  │     └── xyz-yaw refine, optional
  │
  ├── SlidingWindowRefiner
  │     ├── ShortTermAssociator
  │     ├── PnpRefineWindowManager
  │     └── G2oWindowOptimizer
  │
  ├── QualityGate
  │     ├── finite check
  │     ├── positive depth check
  │     ├── reprojection check
  │     ├── pose delta check
  │     ├── covariance check
  │     └── confidence check
  │
  └── FallbackManager
        └── refined -> single -> raw PnP
```

---

## 4. 包名与目录规划

建议包名：

```text
armor_pnp_refiner
```

建议目录：

```text
src/rm_auto_aim/armor_pnp_refiner/
├── CMakeLists.txt
├── package.xml
├── docs/
│   └── armor_pnp_refiner_revised_design.md
├── include/armor_pnp_refiner/
│   ├── pnp_refiner_config.hpp
│   ├── pnp_refiner_types.hpp
│   ├── armor_pnp_refiner.hpp
│   ├── single_frame_refiner.hpp
│   ├── sliding_window_refiner.hpp
│   ├── short_term_associator.hpp
│   ├── pnp_refine_window_manager.hpp
│   ├── quality_gate.hpp
│   └── g2o/
│       ├── vertices.hpp
│       └── edges.hpp
├── src/
│   ├── armor_pnp_refiner.cpp
│   ├── single_frame_refiner.cpp
│   ├── sliding_window_refiner.cpp
│   ├── short_term_associator.cpp
│   ├── pnp_refine_window_manager.cpp
│   ├── quality_gate.cpp
│   └── g2o/
│       ├── vertices.cpp
│       └── edges.cpp
└── test/
    ├── test_reprojection_edge.cpp
    ├── test_single_frame_refiner.cpp
    ├── test_sliding_window_refiner.cpp
    ├── test_short_term_associator.cpp
    └── test_quality_gate.cpp
```

如果短期内不想重命名包，也可以先保留原包名 `armor_pose_graph_optimizer`，但建议文档和类名先切换到 PnP refiner 语义，避免后续继续扩散旧概念。

---

## 5. 公共数据结构设计

### 5.1 PnP 输入

虽然对外语义是 `PnP in`，但重投影优化不能只传 `xyz + yaw`，还必须传图像观测和相机模型。

```cpp
namespace armor_pnp_refiner {

struct PnpRefineInput {
  // 原始 PnP 结果，默认在 camera frame 下。
  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};

  // 从 q_camera_armor 提取或由调用方提供。
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  // OpenCV solvePnP 原始输出，可选保留，便于兼容旧流程。
  cv::Mat rvec;
  cv::Mat tvec;

  // 2D-3D 对应点。
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;

  // 每个 keypoint 的像素噪声或置信度映射结果。
  // 为空时使用 config.pixel_sigma。
  std::vector<double> keypoint_sigma_px;

  // 相机模型。
  cv::Mat camera_matrix;  // CV_64F 3x3
  cv::Mat dist_coeffs;    // CV_64F 1xN，可为空

  // 检测辅助信息，用于内部短期关联。
  rclcpp::Time stamp{};
  cv::Rect2f bbox{};
  cv::Point2f center{};
  double detection_confidence{1.0};
  std::string armor_number;
  std::string armor_type;  // small | large

  // 可选外部 track_id。
  // armor_detector_nn 可传入；传统 armor_detector 可不传。
  std::optional<int> external_track_id{std::nullopt};

  // yaw/pitch/roll 约定。
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};
  bool use_fixed_pitch_roll{true};
  double fixed_pitch_rad{0.0};
  double fixed_roll_rad{0.0};
};

}  // namespace armor_pnp_refiner
```

设计要点：

1. 默认输出与输入都保持 `camera frame`，便于对 detector 透明。
2. `image_points[i]` 必须与 `object_points[i]` 严格一一对应。
3. 支持 4 点和 6 点输入，不在优化器内部重排点顺序。
4. `external_track_id` 存在时优先使用；不存在时内部关联器生成 `refine_track_id`。
5. `R_imu_camera` 与 `fixed_pitch_roll` 用于兼容旧 yaw-only BA 的语义。

### 5.2 PnP 输出

```cpp
namespace armor_pnp_refiner {

enum class RefineMode {
  PNP_FALLBACK,
  G2O_SINGLE_YAW,
  G2O_SINGLE_XYZ_YAW,
  G2O_WINDOW_XYZ_YAW,
  G2O_WINDOW_POSE3_EXPERIMENTAL
};

enum class RefineStatus {
  GOOD,
  DEGRADED,
  REJECTED
};

struct PnpRefineOutput {
  bool valid{false};       // 是否有可用 pose。fallback PnP 也可以 valid。
  bool refined{false};     // 是否成功使用优化结果，而非原始 PnP。

  RefineMode mode{RefineMode::PNP_FALLBACK};
  RefineStatus status{RefineStatus::REJECTED};

  // 输出仍是 PnP 风格 pose，默认 camera frame。
  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};
  cv::Mat rvec;
  cv::Mat tvec;

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  // 当前帧 [x, y, z, yaw] 协方差，camera frame。
  Eigen::Matrix4d covariance_xyz_yaw{Eigen::Matrix4d::Identity()};

  // 质量评分。
  double confidence{0.0};
  double quality_score{0.0};

  // 诊断信息。
  int refine_track_id{-1};
  int window_size{0};
  int num_points{0};
  int num_inliers{0};

  double reproj_error_raw_px{0.0};
  double reproj_error_refined_px{0.0};
  double chi2_per_dof{0.0};
  double condition_number{0.0};

  double pose_delta_m{0.0};
  double yaw_delta_rad{0.0};
  double cost_before{0.0};
  double cost_after{0.0};
  double solve_time_ms{0.0};

  std::string reason;
};

}  // namespace armor_pnp_refiner
```

设计要点：

1. `valid=true, refined=false` 表示返回原始 PnP fallback。
2. `refined=true` 表示优化结果通过质量门控。
3. `covariance_xyz_yaw` 即使在 fallback 时也应提供一个保守值。
4. 后端滤波器应使用 `status/confidence/covariance` 决定是否吸收该观测。

---

## 6. 配置设计

```cpp
struct PnpRefinerConfig {
  std::string mode{"g2o_window"};
  // none | g2o_single_yaw | g2o_single_xyz_yaw | g2o_window

  // 窗口设置。
  int window_size{5};
  int min_window_size{3};
  double max_time_span_ms{60.0};
  double max_time_gap_ms{35.0};

  // 求解设置。
  double max_solver_time_ms{2.0};
  int max_iterations{5};
  bool use_robust_kernel{true};
  std::string robust_kernel{"huber"};

  // 像素噪声。
  double pixel_sigma{1.5};
  double pixel_sigma_min{0.8};
  double pixel_sigma_max{5.0};
  double huber_delta_px{3.0};

  // PnP prior。
  double prior_sigma_xy{0.08};
  double prior_sigma_z{0.15};
  double prior_sigma_yaw_rad{0.10};

  // 二阶平滑，默认开启。
  double acc_sigma_xy{0.15};
  double acc_sigma_z{0.25};
  double acc_sigma_yaw_rad{0.12};

  // 三阶有限差分弱正则，默认关闭。
  bool enable_jerk_smooth{false};
  double jerk_sigma_xy{0.30};
  double jerk_sigma_z{0.45};
  double jerk_sigma_yaw_rad{0.25};

  // 内部短期关联。
  bool use_external_track_id_if_available{true};
  bool enable_internal_association{true};
  double iou_match_threshold{0.30};
  double center_distance_threshold_px{80.0};
  bool require_same_armor_number{true};
  bool require_same_armor_type{true};
  int min_confirm_hits{2};
  int max_missed_frames{3};

  // 质量门控。
  double max_reproj_error_px{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_rad{20.0 * M_PI / 180.0};
  double max_chi2_per_dof{5.0};
  double max_condition_number{1e7};

  // 协方差下限。
  double min_var_x{0.01 * 0.01};
  double min_var_y{0.01 * 0.01};
  double min_var_z{0.02 * 0.02};
  double min_var_yaw{0.01 * 0.01};

  // 置信度门槛。
  double good_confidence{0.75};
  double reject_confidence{0.40};

  bool require_positive_depth{true};
  bool require_finite{true};
};
```

### 6.1 默认窗口参数解释

在 90 FPS 左右：

```text
frame interval ≈ 11.1 ms
3 帧窗口覆盖 ≈ 22.2 ms
5 帧窗口覆盖 ≈ 44.4 ms
```

因此首版建议：

```yaml
window_size: 5
min_window_size: 3
max_time_span_ms: 60.0
max_iterations: 5
max_solver_time_ms: 2.0
```

不建议默认：

```yaml
window_size: 8
max_time_span_ms: 300.0
```

因为 300 ms 对自瞄实时链路已经偏长，容易引入历史观测污染和运动滞后。

---

## 7. 内部短期数据关联设计

### 7.1 为什么保留在包内部

内部短期关联保留在包内是合理的，原因如下：

1. `armor_detector` 当前没有 track_id。
2. `armor_detector_nn` 当前有 track_id。
3. 后续可能出现新的 detector，不一定都有 track_id。
4. 如果每个 detector 各自实现滑窗关联，会导致行为不一致。
5. PnP 微调本身需要滑窗缓存，因此把短期关联与窗口管理封装在包内更统一。

### 7.2 语义边界

内部关联只服务于 PnP 微调，输出 `refine_track_id`。

它不是：

```text
system target_id
robot_id
global track_id
armor_tracker track_id
```

它只是：

```text
refine_window_id
```

也就是：

```text
当前检测观测
  ↓
匹配到一个短期 refine 窗口
  ↓
给滑窗 BA 提供最近 3~5 帧观测
```

### 7.3 外部 track_id 与内部关联的关系

推荐逻辑：

```cpp
if (input.external_track_id.has_value() &&
    config.use_external_track_id_if_available) {
  refine_track_id = input.external_track_id.value();
} else if (config.enable_internal_association) {
  refine_track_id = associator.associate(input);
} else {
  use_single_frame_only();
}
```

这意味着：

```text
armor_detector_nn:
  已有 track_id -> 直接传 external_track_id

armor_detector:
  无 track_id -> 内部 ShortTermAssociator 生成 refine_track_id

未来 detector:
  有 track_id 就传，没有就内部关联
```

### 7.4 ShortTermAssociator 输入

```cpp
struct RefineAssocObservation {
  rclcpp::Time stamp;
  cv::Rect2f bbox;
  cv::Point2f center;
  double confidence{1.0};
  std::string armor_number;
  std::string armor_type;
  Eigen::Vector3d t_camera_armor;
  double yaw_rad{0.0};
};
```

### 7.5 关联评分

建议综合：

```text
score = w_iou * IoU
      - w_center * normalized_center_distance
      - w_pose * normalized_pose_delta
      + w_cls * class_consistency
```

基础首版可以只用：

1. IoU。
2. 中心距离。
3. armor_number/type 一致性。
4. 时间间隔。

关联门控：

```text
IoU > iou_match_threshold
或 center_distance < center_distance_threshold_px
且 armor_number/type 满足配置要求
且 dt < max_time_gap_ms
```

### 7.6 PnpRefineWindowManager

```cpp
class PnpRefineWindowManager {
public:
  explicit PnpRefineWindowManager(const PnpRefinerConfig& config);

  void push(int refine_track_id, const PnpRefineInput& input);
  std::vector<PnpRefineInput> getWindow(int refine_track_id) const;
  void resetTrack(int refine_track_id);
  void pruneExpired(const rclcpp::Time& now);
  void resetAll();
};
```

窗口重置条件：

1. `refine_track_id` 切换。
2. 时间间隔超过 `max_time_gap_ms`。
3. PnP pose delta 异常。
4. armor_number/type 发生不允许的变化。
5. 连续优化失败超过阈值。
6. 检测质量长期低于阈值。

---

## 8. 优化模式设计

### 8.1 none / PnP fallback

不做优化，直接返回原始 PnP，但仍可填充保守协方差。

用途：

1. 关闭微调进行对照实验。
2. 优化异常时回退。
3. 窗口不足时兜底。

### 8.2 单帧 yaw-only 模式

状态变量：

```text
yaw: 1 DoF
```

固定量：

1. PnP 平移 `t_camera_armor`。
2. pitch / roll 使用固定先验或 PnP 提取值。
3. `R_imu_camera` 用于保持旧 BA / NN refiner 的 yaw 语义。

残差：

```text
r_proj = u_observed - project(K, R_camera_armor(yaw, pitch, roll) * P_obj + t_pnp)
```

优点：

1. 快。
2. 稳定。
3. 适合作为兜底。
4. 可替代旧 `BaSolver` 和 NN `SingleYawRefiner`。

缺点：

1. 不修正平移。
2. 无法改善深度抖动。
3. 对 pitch/roll 先验有依赖。

### 8.3 单帧 xyz-yaw 模式

状态变量：

```text
x = [tx, ty, tz, yaw]
```

固定量：

1. pitch。
2. roll。
3. object model。

残差：

1. 重投影残差。
2. PnP translation prior。
3. yaw prior。

用途：

1. 关键点质量较好时优化当前帧 `xyz + yaw`。
2. 作为滑窗不足时的中间兜底。

风险：

1. 单帧平面目标 `z/yaw` 耦合较强。
2. 必须有 PnP prior 和质量门控。

### 8.4 滑窗 xyz-yaw 模式

推荐主模式。

每帧状态：

```text
x_i = [tx_i, ty_i, tz_i, yaw_i]
```

窗口：

```text
X = {x_0, x_1, ..., x_{N-1}}
```

输出只取当前帧：

```text
z_refined = x_{N-1}
```

也就是 trailing window，不使用未来帧，不引入额外等待延迟。

### 8.5 Pose3 模式

完整 SE3 优化作为实验模式，不建议首版默认开启。

风险：

1. 平面目标完整姿态可观性弱。
2. roll/pitch 容易漂移。
3. 深度可能不稳定。
4. 系统行为不如 `xyz + yaw + fixed pitch/roll` 可控。

---

## 9. g2o 顶点与边设计

### 9.1 VertexYaw

```cpp
class VertexYaw : public g2o::BaseVertex<1, double> {
public:
  void setToOriginImpl() override;
  void oplusImpl(const double* update) override;
};
```

`oplusImpl` 需要做角度 wrap：

```cpp
_estimate = normalizeAngle(_estimate + update[0]);
```

### 9.2 VertexXyzYaw

```cpp
class VertexXyzYaw
    : public g2o::BaseVertex<4, Eigen::Matrix<double, 4, 1>> {
public:
  // estimate = [tx, ty, tz, yaw]
  void setToOriginImpl() override;
  void oplusImpl(const double* update) override;
};
```

更新方式：

```cpp
_estimate[0] += update[0];
_estimate[1] += update[1];
_estimate[2] += update[2];
_estimate[3] = normalizeAngle(_estimate[3] + update[3]);
```

### 9.3 EdgeYawReprojection

连接：

```text
VertexYaw
```

误差：

```text
error = observed_uv - project(K, R_camera_armor(yaw, fixed_pitch, fixed_roll) * P_obj + t_pnp)
```

维度：2。

首版可使用数值雅可比，后续根据性能补解析雅可比。

### 9.4 EdgeXyzYawReprojection

连接：

```text
VertexXyzYaw
```

误差：

```text
error = observed_uv - project(K, R_camera_armor(yaw, fixed_pitch, fixed_roll) * P_obj + t_xyz)
```

信息矩阵：

```text
Information = diag(1 / sigma_u^2, 1 / sigma_v^2)
```

关键点置信度映射：

```text
sigma_kp = clamp(pixel_sigma_base / confidence,
                 pixel_sigma_min,
                 pixel_sigma_max)
```

或：

```text
sigma_kp = sigma_min + (1 - confidence) * sigma_scale
```

### 9.5 EdgeTranslationPrior

连接：

```text
VertexXyzYaw
```

误差：

```text
r_t = t_current - t_pnp
```

维度：3。

建议 z 方向 prior 略强于 xy，因为平面 PnP 深度方向更容易漂移，但也不能过强，否则无法改善 PnP 深度抖动。

### 9.6 EdgeYawPrior

误差：

```text
r_yaw = wrap(yaw_current - yaw_pnp)
```

维度：1。

### 9.7 EdgeSecondOrderSmooth

连接：

```text
VertexXyzYaw(k-2), VertexXyzYaw(k-1), VertexXyzYaw(k)
```

误差：

```text
r_acc_t = t_k - 2 t_{k-1} + t_{k-2}
r_acc_yaw = wrap(yaw_k - 2 yaw_{k-1} + yaw_{k-2})
```

维度：4。

这是推荐默认平滑边。

它比原先一阶平滑更合适，因为一阶平滑：

```text
r = x_k - x_{k-1}
```

会隐含鼓励零速度，目标运动时容易引入滞后。

二阶平滑更接近短窗口内的局部匀速 / 匀加速度弱约束，主要抑制高频抖动。

### 9.8 EdgeThirdOrderSmooth，可选

连接：

```text
VertexXyzYaw(k-3), VertexXyzYaw(k-2), VertexXyzYaw(k-1), VertexXyzYaw(k)
```

误差：

```text
r_jerk_t = t_k - 3 t_{k-1} + 3 t_{k-2} - t_{k-3}
r_jerk_yaw = wrap(yaw_k - 3 yaw_{k-1} + 3 yaw_{k-2} - yaw_{k-3})
```

三阶平滑只作为弱正则，不建议显式估计 jerk 状态。

---

## 10. 滑窗优化目标细化

### 10.1 变量

对窗口内 N 帧：

```text
X = {x_0, x_1, ..., x_{N-1}}
x_i = [t_i, yaw_i]
t_i = [tx_i, ty_i, tz_i]
```

不建议首版显式优化：

```text
velocity
acceleration
jerk
yaw_rate
yaw_acc
yaw_jerk
```

原因是 3~5 帧窗口太短，高阶变量容易吸收关键点噪声。

### 10.2 总目标函数

```text
min_X
  Σ ρ(||r_proj||²)
+ λ_pnp   Σ ||r_pnp||²
+ λ_acc   Σ ||r_acc||²
+ λ_jerk  Σ ||r_jerk||²
```

其中：

1. `r_proj` 是每帧每个关键点的重投影误差。
2. `r_pnp` 是 PnP 初值弱先验。
3. `r_acc` 是二阶有限差分平滑。
4. `r_jerk` 是可选三阶有限差分弱正则。
5. `ρ` 是 Huber 或 Cauchy robust kernel。

### 10.3 重投影残差

对于第 i 帧第 j 个关键点：

```text
P_cam_ij = R_camera_armor(yaw_i, pitch_i, roll_i) * P_obj_j + t_i
u_hat_ij = project(K, D, P_cam_ij)
r_proj_ij = u_obs_ij - u_hat_ij
```

代价：

```text
E_proj = Σ r_proj_ijᵀ W_ij r_proj_ij
```

其中：

```text
W_ij = diag(1 / sigma_ij², 1 / sigma_ij²)
```

### 10.4 PnP 先验残差

```text
r_pnp_i = [
  (tx_i - tx_pnp_i) / sigma_prior_x,
  (ty_i - ty_pnp_i) / sigma_prior_y,
  (tz_i - tz_pnp_i) / sigma_prior_z,
  wrap(yaw_i - yaw_pnp_i) / sigma_prior_yaw
]
```

作用：

1. 防止平面目标优化跑飞。
2. 防止窗口内少量错误关键点将 pose 拉崩。
3. 保证优化结果不会过度偏离 PnP 初值。

### 10.5 二阶平滑残差

```text
r_acc_i = [
  (t_i - 2t_{i-1} + t_{i-2}) / sigma_acc_t,
  wrap(yaw_i - 2yaw_{i-1} + yaw_{i-2}) / sigma_acc_yaw
]
```

作用：

1. 抑制短窗口内的高频抖动。
2. 避免一阶平滑带来的零速度偏置。
3. 对 90 FPS、3~5 帧窗口而言，足以提供短时运动正则。

### 10.6 三阶平滑残差

仅在高速变速旋转或二阶平滑仍存在明显滞后时开启。

```text
r_jerk_i = [
  (t_i - 3t_{i-1} + 3t_{i-2} - t_{i-3}) / sigma_jerk_t,
  wrap(yaw_i - 3yaw_{i-1} + 3yaw_{i-2} - yaw_{i-3}) / sigma_jerk_yaw
]
```

注意：

1. 三阶项权重应弱。
2. 不建议显式输出 jerk。
3. 不建议让三阶模型成为硬约束。

---

## 11. 求解流程

```cpp
PnpRefineOutput SlidingWindowRefiner::refine(const PnpRefineInput& input) {
  // 1. 确定 refine_track_id
  int id = resolveRefineTrackId(input);

  // 2. 更新窗口
  window_manager_.push(id, input);
  auto window = window_manager_.getWindow(id);

  // 3. 窗口不足则单帧兜底
  if (window.size() < config_.min_window_size) {
    return single_frame_refiner_.refine(input);
  }

  // 4. 检查窗口稳定性
  if (!isWindowStable(window)) {
    window_manager_.resetTrack(id);
    return single_frame_refiner_.refine(input);
  }

  // 5. 构造 g2o 图
  G2oGraph graph;
  addXyzYawVertices(graph, window);
  addReprojectionEdges(graph, window);
  addPnpPriorEdges(graph, window);
  addSecondOrderSmoothEdges(graph, window);

  if (config_.enable_jerk_smooth) {
    addThirdOrderSmoothEdges(graph, window);
  }

  // 6. 求解
  double cost_before = graph.computeCost();
  bool ok = graph.optimize(config_.max_iterations, config_.max_solver_time_ms);
  double cost_after = graph.computeCost();

  if (!ok) {
    return fallbackToSingleOrPnp(input, "g2o optimize failed");
  }

  // 7. 获取当前帧结果
  auto z_current = graph.getCurrentXyzYaw();

  // 8. 计算协方差与质量指标
  auto metrics = graph.computeQualityMetrics();
  auto R = covariance_estimator_.computeCurrentMarginal(graph, metrics);

  // 9. 质量门控
  auto output = makeOutput(input, z_current, R, metrics);
  return quality_gate_.filterOrFallback(input, output);
}
```

---

## 12. 当前帧协方差设计

### 12.1 目标

输出当前帧：

```text
z = [x, y, z, yaw]
R_xyz_yaw ∈ R^{4x4}
```

该协方差表示 detector 级 PnP 微调观测的不确定性，可供后端 UKF/InEKF 使用。

### 12.2 Hessian 信息矩阵

优化收敛后，在最终线性化点上：

```text
H = Jᵀ W J
```

其中：

1. `J` 是残差对窗口状态的雅可比。
2. `W` 是残差信息矩阵。
3. `H` 是窗口状态的信息矩阵。

窗口状态分为旧帧和当前帧：

```text
X = [X_old, x_current]
```

Hessian 分块：

```text
H = [ H_oo  H_oc
      H_co  H_cc ]
```

当前帧边缘信息矩阵：

```text
Ω_c = H_cc - H_co H_oo^{-1} H_oc
```

当前帧协方差：

```text
Σ_c = Ω_c^{-1}
```

即：

```text
R_ba = Σ_c
```

### 12.3 残差尺度修正

Hessian 协方差通常容易偏乐观，因此需要按残差尺度修正：

```text
s² = max(1.0, chi2 / dof)
R_ba_scaled = s² · R_ba
```

其中：

```text
chi2 = Σ rᵀ W r
dof = residual_dim - effective_state_dim
```

### 12.4 协方差下限

必须设置下限：

```text
R_floor = diag([
  min_var_x,
  min_var_y,
  min_var_z,
  min_var_yaw
])
```

最终：

```text
R_final = R_floor + R_ba_scaled
```

### 12.5 可选附加噪声

后续可扩展：

```text
R_final = R_floor
        + R_ba_scaled
        + R_time
        + R_assoc
        + R_system
```

其中：

1. `R_time` 表示时间同步 / 延迟误差。
2. `R_assoc` 表示内部关联不确定性。
3. `R_system` 表示固定 pitch/roll、外参、装甲板尺寸等系统偏差保守项。

首版可以只实现：

```text
R_final = R_floor + s² R_ba
```

---

## 13. 置信度设计

### 13.1 综合形式

置信度不应只看重投影误差，建议综合：

```text
confidence =
  c_reproj^0.30
* c_condition^0.20
* c_inlier^0.15
* c_improvement^0.15
* c_association^0.10
* c_covariance^0.10
```

首版可简化为：

```text
confidence =
  c_reproj^0.40
* c_condition^0.25
* c_inlier^0.20
* c_improvement^0.15
```

### 13.2 重投影置信度

```text
c_reproj = exp(-0.5 * max(0, chi2_per_dof - 1))
```

或根据 RMSE：

```text
c_reproj = exp(-reproj_rmse² / sigma_reproj_ref²)
```

### 13.3 条件数置信度

```text
c_condition = clamp(
  log(kappa_bad / kappa) / log(kappa_bad / kappa_good),
  0,
  1
)
```

建议：

```text
kappa_good = 1e3
kappa_bad  = 1e7
```

### 13.4 Inlier 置信度

```text
c_inlier = num_inliers / num_points
```

inlier 可根据 robust kernel 权重或单点重投影误差判断。

### 13.5 优化收益置信度

```text
improve_ratio = (cost_before - cost_after) / max(cost_before, eps)
c_improvement = clamp(improve_ratio / expected_improve_ratio, 0, 1)
```

注意：初值已经很好时，收益小不一定表示错误，所以该项权重不能太高。

### 13.6 关联置信度

若使用外部 track_id 且窗口连续：

```text
c_association = 1.0
```

若使用内部关联：

```text
c_association = association_score_normalized
```

若刚建立窗口或刚发生重置：

```text
c_association = 0.5 ~ 0.7
```

### 13.7 协方差置信度

可根据协方差对角线是否过大或条件数是否异常给出：

```text
c_covariance = clamp(log(var_bad / var_trace) / log(var_bad / var_good), 0, 1)
```

---

## 14. GOOD / DEGRADED / REJECT 判定

### 14.1 GOOD

条件示例：

```text
confidence >= 0.75
reproj_error_refined_px <= 2.0 px
chi2_per_dof <= 3.0
condition_number <= 1e6
pose_delta_m <= max_pose_delta_m
yaw_delta_rad <= max_yaw_delta_rad
```

处理：

```text
使用 refined PnP
使用正常 R_final
```

### 14.2 DEGRADED

条件示例：

```text
0.40 < confidence < 0.75
或 reproj_error_refined_px 略高
或 condition_number 偏大
或窗口刚建立
```

处理：

```text
可以使用 refined PnP
但放大 R_final
```

例如：

```text
R_final = R_final / max(confidence, 0.2)
```

或：

```text
R_final *= 2 ~ 5
```

### 14.3 REJECTED

条件示例：

```text
confidence <= 0.40
Hessian 不可逆
输出非 finite
投影深度为负
reproj error 明显异常
pose delta 过大
yaw delta 过大
窗口关联不稳定
求解超时且结果未通过门控
```

处理：

```text
回退到单帧 refine 或原始 PnP
```

---

## 15. 回退链设计

推荐回退链：

```text
g2o_window
  -> g2o_single_yaw 或 g2o_single_xyz_yaw
  -> existing_single_yaw，可选
  -> raw PnP
```

对于 `armor_detector_nn`：

```text
g2o_window -> g2o_single -> existing_single_yaw -> pnp
```

对于传统 `armor_detector`：

```text
g2o_window, 若开启内部关联
  -> g2o_single
  -> legacy BaSolver
  -> pnp
```

重要原则：

```text
refine failed != detection failed
```

只要 PnP 成功，最终就必须有可发布结果。

---

## 16. 高层接口设计

### 16.1 ArmorPnpRefiner

```cpp
class ArmorPnpRefiner {
public:
  explicit ArmorPnpRefiner(const PnpRefinerConfig& config);

  PnpRefineOutput refine(const PnpRefineInput& input);

  void reset();
  void resetRefineTrack(int refine_track_id);

private:
  PnpRefinerConfig config_;
  ShortTermAssociator associator_;
  PnpRefineWindowManager window_manager_;
  SingleFrameRefiner single_frame_refiner_;
  SlidingWindowRefiner sliding_window_refiner_;
  QualityGate quality_gate_;
};
```

### 16.2 detector 侧调用示例

```cpp
PnpRefineInput input;
input.t_camera_armor = pnp_result.t;
input.q_camera_armor = pnp_result.q;
input.yaw_rad = pnp_result.yaw;
input.pitch_rad = pnp_result.pitch;
input.roll_rad = pnp_result.roll;

input.image_points = detection.keypoints;
input.object_points = armor_model.object_points;
input.keypoint_sigma_px = detection.keypoint_sigmas;
input.camera_matrix = K;
input.dist_coeffs = D;

input.stamp = stamp;
input.bbox = detection.bbox;
input.center = detection.center;
input.armor_number = detection.number;
input.armor_type = detection.type;
input.detection_confidence = detection.confidence;

if (detection.has_track_id) {
  input.external_track_id = detection.track_id;
}

auto refined = pnp_refiner_->refine(input);

if (refined.valid) {
  usePose(refined);
} else {
  usePose(pnp_result);
}
```

---

## 17. 与 armor_detector_nn 的接入方案

`armor_detector_nn` 已有 `PoseEstimate`、`IPoseRefiner` 和 detector 级 track_id，因此优先接入。

新增适配器：

```cpp
class G2oPnpRefinerAdapter : public IPoseRefiner {
public:
  explicit G2oPnpRefinerAdapter(const PnpRefinerConfig& config);

  PoseEstimate refine(
      const PoseEstimate& pnp_result,
      const ArmorDetection& detection,
      const CameraModel& camera_model) override;

private:
  std::shared_ptr<armor_pnp_refiner::ArmorPnpRefiner> refiner_;
};
```

配置扩展：

```yaml
pose:
  refiner:
    mode: "g2o_window"  # none | single_yaw | sliding_window | g2o_single | g2o_window
  pnp_refiner:
    window_size: 5
    min_window_size: 3
    max_time_span_ms: 60.0
    max_solver_time_ms: 2.0
    max_iterations: 5
    pixel_sigma: 1.5
    huber_delta_px: 3.0
    prior_sigma_xy: 0.08
    prior_sigma_z: 0.15
    prior_sigma_yaw: 0.10
    acc_sigma_xy: 0.15
    acc_sigma_z: 0.25
    acc_sigma_yaw: 0.12
    enable_internal_association: true
    use_external_track_id_if_available: true
```

接入策略：

1. 优先使用 `ArmorDetection.track_id` 作为 `external_track_id`。
2. 若 `track_id` 缺失或无效，可启用内部关联兜底。
3. 输出填充现有 `PoseEstimate`。
4. 建议扩展 `PoseEstimate::mode`：

```cpp
G2O_SINGLE_VALID
G2O_WINDOW_VALID
G2O_PNP_FALLBACK
```

---

## 18. 与传统 armor_detector 的接入方案

传统 `armor_detector` 没有统一 `IPoseRefiner`，建议短期直接注入：

1. PnP 成功后构造 `PnpRefineInput`。
2. 如果开启 `use_pnp_refiner`，调用 `ArmorPnpRefiner::refine()`。
3. `refined.valid && refined.refined` 时覆盖原 pose。
4. 否则保留 legacy `BaSolver` 或原始 PnP。

示例配置：

```yaml
use_ba: true
pose_refine:
  backend: "pnp_refiner"  # legacy_g2o | pnp_refiner
  mode: "g2o_single"      # g2o_single | g2o_window
  enable_internal_association: true
```

对于传统 detector：

1. 短期建议先启用 `g2o_single_yaw`。
2. 滑窗模式可以依赖内部 `ShortTermAssociator`。
3. 如果传统 detector 后续也实现 track_id，则传入 `external_track_id` 即可。

---

## 19. CMake 与依赖规划

模块依赖：

1. `ament_cmake_auto`
2. `rclcpp`
3. `OpenCV`
4. `Eigen3`
5. `g2o`
6. `fmt`
7. `rm_utils`，可选

CMake 示例：

```cmake
cmake_minimum_required(VERSION 3.8)
project(armor_pnp_refiner)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(G2O REQUIRED)

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
  fmt::fmt
)

ament_auto_package(
  INSTALL_TO_SHARE
  docs
)
```

注意：如果项目已有 `FindG2O.cmake`，建议移动到公共 cmake modules，避免多个包各自维护。

---

## 20. 数值与坐标约定

必须统一以下约定：

1. 输入输出默认都是 `camera frame` 下的 `armor pose`。
2. `image_points[i]` 与 `object_points[i]` 必须一一对应。
3. `armor_detector_nn` 4 点顺序由调用方保证。
4. 传统 `armor_detector` 6 点顺序由调用方保证。
5. 优化器内部不重排点。
6. yaw-only 和 xyz-yaw 模式必须明确 yaw 的定义。
7. `fixed_pitch/roll` 必须与旧 BA / NN refiner 一致。
8. 如果使用畸变参数，重投影边应支持畸变投影。
9. 首版可使用 OpenCV `projectPoints` 验证投影正确性，后续再补解析投影。

### 20.1 yaw 构造建议

推荐将 yaw 构造封装为统一函数：

```cpp
Eigen::Matrix3d buildCameraArmorRotation(
    double yaw,
    double pitch,
    double roll,
    const Eigen::Matrix3d& R_imu_camera,
    const YawConvention& convention);
```

避免不同 detector 中 yaw 方向、坐标轴顺序不一致。

---

## 21. 测试计划

### 21.1 单元测试

1. `test_reprojection_edge.cpp`
   - 给定已知 pose 和 object point，检查投影误差接近 0。
   - 对 yaw / xyz 做数值扰动，检查误差方向合理。

2. `test_single_frame_refiner.cpp`
   - 生成合成装甲板点。
   - 加入 1~3 px 噪声。
   - 检查优化后 yaw error 下降。
   - 检查异常输入能 fallback。

3. `test_sliding_window_refiner.cpp`
   - 生成连续运动轨迹。
   - 加入随机像素噪声和少量 outlier。
   - 检查窗口输出平滑度和误差。
   - 检查窗口不足时回退单帧。

4. `test_short_term_associator.cpp`
   - 检查 IoU / center distance 匹配。
   - 检查 armor_number/type 不一致时是否拒绝。
   - 检查遮挡后是否能重建窗口。

5. `test_quality_gate.cpp`
   - 检查非 finite、负深度、reproj error 过大、pose delta 过大时 reject。
   - 检查 DEGRADED 时协方差放大。

### 21.2 集成测试

对同一段 bag 同时运行：

```text
pnp
single_yaw
g2o_single_yaw
g2o_window
```

记录：

1. 每帧耗时。
2. 平均重投影误差。
3. yaw 抖动标准差。
4. x/y/z 抖动标准差。
5. 输出协方差均值。
6. confidence 分布。
7. fallback 次数。
8. window reset 次数。
9. 内部关联 ID 切换次数。

### 21.3 实车 / 视频验证

1. 静止装甲板：检查输出稳定性。
2. 横向移动目标：检查平移延迟与平滑性。
3. 旋转目标：检查 yaw 跟随性。
4. 遮挡 / 误检：检查 fallback 与窗口重置。
5. 远距离小目标：检查协方差是否合理变大。
6. 高速机动：检查二阶平滑是否引入滞后。

---

## 22. 实现阶段规划

### Phase 0：重命名与接口整理

1. 明确包定位为 `armor_pnp_refiner`。
2. 定义 `PnpRefineInput` / `PnpRefineOutput` / `PnpRefinerConfig`。
3. 定义 `ArmorPnpRefiner` facade。
4. 保留原文档，新增本修正版文档。

### Phase 1：单帧 yaw-only g2o

1. 实现 `VertexYaw`。
2. 实现 `EdgeYawReprojection`。
3. 实现 `SingleFrameRefiner::refineYawOnly()`。
4. 接入 `armor_detector_nn` 的 `g2o_single` 模式。
5. 与现有 `single_yaw` 对比。

验收：

1. 平均耗时小于 1 ms。
2. 优化失败自动回退。
3. 与现有 yaw 方向一致。

### Phase 2：单帧 xyz-yaw

1. 实现 `VertexXyzYaw`。
2. 实现 `EdgeXyzYawReprojection`。
3. 实现 translation/yaw prior。
4. 输出简单协方差与 confidence。

验收：

1. 重投影误差不劣于 PnP。
2. pose delta 在可控范围内。
3. 对异常关键点能 reject。

### Phase 3：滑窗 xyz-yaw

1. 实现 `ShortTermAssociator`。
2. 实现 `PnpRefineWindowManager`。
3. 实现二阶平滑边。
4. 实现滑窗 g2o batch solve。
5. 输出当前帧 refined PnP。

验收：

1. 平移与 yaw 输出较 PnP 更平滑。
2. 90 FPS 下耗时不超过预算。
3. track 切换 / 遮挡 / 丢帧时窗口能正确重置。

### Phase 4：协方差与置信度

1. 实现当前帧 marginal covariance。
2. 实现 `chi2/dof` 缩放。
3. 实现协方差下限。
4. 实现 `GOOD / DEGRADED / REJECTED`。
5. 将 `R_xyz_yaw` 传给后端 tracker，可选。

### Phase 5：传统 armor_detector 接入

1. 传统 detector 构造 `PnpRefineInput`。
2. 接入 `g2o_single`。
3. 实验启用内部关联下的 `g2o_window`。
4. 保留 legacy `BaSolver` 回退。

### Phase 6：扩展模式

1. 三阶有限差分弱正则。
2. `R_time` / `R_assoc` / `R_system`。
3. Pose3 实验模式。
4. 与后端 UKF/InEKF 的 NIS 一致性检查。

---

## 23. 风险与修正对策

### 风险 1：包定位过大，和 tracker / pose estimator 混淆

修正：

```text
将包定位为 armor_pnp_refiner。
输入 PnP，输出 PnP。
内部关联只服务于短期滑窗，不输出系统级 track。
```

### 风险 2：内部关联污染滑窗

对策：

1. 优先使用外部 `track_id`。
2. 内部关联必须有 IoU / center / class / time gap 门控。
3. pose delta 异常时清空窗口。
4. 连续失败时 reset track。
5. 关联置信度低时输出 DEGRADED 或回退单帧。

### 风险 3：一阶平滑导致零速度偏置

修正：

```text
默认不使用 x_k - x_{k-1} 作为主要平滑项。
改用二阶有限差分。
```

### 风险 4：窗口过长导致延迟与历史污染

修正：

```text
90 FPS 默认 3~5 帧。
max_time_span_ms 默认约 60 ms。
```

### 风险 5：平面目标深度漂移

对策：

1. PnP translation prior 必须开启。
2. z prior 可略强于 xy。
3. 输出必须经过 pose delta 与 positive depth 门控。
4. 协方差中 z 方向需要保守下限。

### 风险 6：固定 pitch / roll 引入系统偏差

对策：

1. 明确固定 pitch/roll 的坐标约定。
2. 将该偏差反映到 `R_system` 或协方差下限中。
3. 后续可按 armor_type / 距离 / 视角估计 pitch/roll 偏差。
4. Pose3 只作为实验模式，不默认开启。

### 风险 7：Hessian 协方差过度自信

对策：

1. 使用 `chi2/dof` 缩放。
2. 加协方差下限。
3. 条件数差时放大 R。
4. DEGRADED 时进一步 inflate R。
5. 不直接把裸 Hessian inverse 交给后端。

### 风险 8：滑窗 BA 与后端 UKF/InEKF 重复平滑

对策：

1. 滑窗 BA 只输出当前帧观测，不维护长期状态。
2. 运动正则设置为弱约束。
3. 输出协方差要反映滑窗平滑带来的相关性和不确定性。
4. 后端根据 `R` 和 `confidence` 决定吸收程度。

### 风险 9：优化结果比 PnP 更差

对策：

1. 严格质量门控。
2. cost_after 不应明显大于 cost_before。
3. reproj error 不应劣化。
4. pose delta/yaw delta 不得超限。
5. 永远允许 PnP fallback。

### 风险 10：g2o 手写边数值错误

对策：

1. 首版可使用数值雅可比。
2. 使用 OpenCV `projectPoints` 对投影进行对照。
3. 单元测试覆盖零误差、扰动方向、finite check。
4. 后续再优化解析雅可比。

---

## 24. 推荐默认策略

首版推荐：

```text
armor_detector_nn:
  mode = g2o_single_yaw 或 g2o_window 实验开启
  external_track_id 优先

armor_detector:
  mode = g2o_single_yaw
  g2o_window 实验开启，依赖内部 ShortTermAssociator
```

稳定后推荐：

```text
armor_detector_nn:
  g2o_window -> g2o_single -> existing_single_yaw -> pnp

armor_detector:
  g2o_window -> g2o_single -> legacy BaSolver -> pnp
```

不建议首版默认开启：

```text
Pose3 全自由度滑窗
显式速度 / 加速度 / jerk 状态
长窗口 8 帧以上
300 ms 时间跨度
强运动先验
```

---

## 25. 总结

修正后的模块应被理解为：

```text
armor_pnp_refiner 是 detector 后处理模块。
它接收 PnP 初值和图像观测，输出 refined PnP pose、协方差和置信度。
内部可以维护短期数据关联和滑窗，但该关联只服务于 PnP 微调，不具有系统级 tracker 语义。
```

最终对外使用方式保持简单：

```cpp
auto refined = pnp_refiner->refine(pnp_input);
```

对 detector 和后续 pipeline 来说，它仍然是一个 PnP 结果，只是质量更高、带有协方差和置信度。

推荐最终实现路线：

```text
单帧 yaw-only
  ↓
单帧 xyz-yaw
  ↓
内部短期关联 + 3~5 帧滑窗 xyz-yaw
  ↓
当前帧 marginal covariance
  ↓
confidence / GOOD-DEGRADED-REJECT
  ↓
接入后端 UKF/InEKF 的观测 R
```

这样既能统一多个 detector 的 PnP 微调逻辑，又不会把该包扩展成一个职责过重的 tracker 或整车状态估计器。
