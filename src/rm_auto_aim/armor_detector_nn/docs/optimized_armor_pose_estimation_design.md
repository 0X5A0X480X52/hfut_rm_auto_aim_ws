# 基于 YOLO-Pose、区域传统精修、滑窗 BA 与实时降级机制的装甲板位姿估计方案设计

## 1. 方案定位

本文设计一个适用于 RoboMaster 装甲板定位与自瞄系统的前端位姿估计方案。该方案面向以下问题：

- YOLO-Pose 能稳定检测装甲板与角点，但角点存在像素级漂移；
- 单帧 PnP 对角点漂移敏感，容易造成 `xyz` 和 `yaw` 抖动；
- 纯传统视觉在光照复杂、大角度、遮挡、数字识别方面不够鲁棒；
- 纯神经网络对标注数据、模型容量、量化精度和 GPU/NPU 依赖较强；
- 滑窗 BA 与局部角点精修虽然能提高精度，但若串行执行可能影响实时性；
- 高速旋转目标存在面板切换、ID switch、yaw wrap、观测不连续等风险。

因此，本方案采用：

```text
神经网络粗检测
    +
区域传统角点精修
    +
PnP 初值
    +
单帧降自由度 BA
    +
异步滑窗 BA
    +
超时降级机制
```

总体思想是：

```text
主链路保证实时性：
    YOLO-Pose → PnP → 单帧 BaSolver → 立即可输出

增强链路提高精度：
    局部角点精修 → 精修后 PnP → per-track 滑窗 BA

异常或超时：
    退回 PnP + 单帧 BaSolver
```

最终该方案不是用滑窗 BA 替代 PnP，而是将滑窗 BA 作为 **可开关、可降级、可计时的增强模块**。

---

## 2. 可参考的开源设计思想

开源技术文档中涉及的几个思想对本方案具有较高参考价值。

### 2.1 三种检测路线

| 方案 | 思路 | 优点 | 缺点 |
|---|---|---|---|
| 纯传统视觉 | 灰度二值化 → 找轮廓 → 配对灯条 → 传统方法找角点 | 不依赖 GPU/NPU；角点精度可控；光照稳定时可靠 | 光照鲁棒性弱；大角度遮挡困难；无法直接识别数字 |
| 纯神经网络 | 完全依赖 YOLO-Pose 输出角点、颜色、标号 | 端到端简单；鲁棒性强；能同时识别数字 | 需要大量数据；量化后角点精度可能下降；依赖 GPU/NPU |
| 神经网络 + 区域传统优化 | YOLO 找大致区域和角点，再在局部灯条区域精修角点 | 兼顾神经网络鲁棒性与传统方法精度 | 实现复杂；需要维护两套路径 |

本方案采用第三种路线：

```text
YOLO-Pose 负责“找得到”
传统局部优化负责“找得准”
BA / UKF 负责“估得稳”
```

### 2.2 PCA 灯条端点精修

传统视觉中直接使用 `minAreaRect` 顶点或 YOLO-Pose 关键点作为角点，容易受到二值化阈值、曝光、网络回归误差的影响。开源方案中的 PCA 灯条端点矫正可作为局部精修模块参考：

```text
灯条 ROI
    ↓
提取亮区点集
    ↓
PCA 得到灯条主方向
    ↓
沿主方向搜索明暗交界
    ↓
得到更稳定的灯条端点
```

其优势是：

```text
角点不再完全依赖二值化阈值或网络回归，
而是利用灯条整体分布和物理亮度边界。
```

### 2.3 降自由度 PnP / 单帧 yaw 优化

开源文档中提到固定 `pitch / roll`，只优化 `yaw` 的设计，与已有 `BaSolver` 思想一致：

```text
PnP 给出 tvec 和初始姿态
固定 pitch ≈ 15°
固定 roll ≈ 0°
固定 tvec
只优化 yaw
最小化 4 个角点重投影误差
```

其目标函数为：

```text
cost(yaw) = Σ_i || u_i - project(R(yaw) · R_pitch · P_i + tvec) ||²
```

这类一维优化具有：

- 搜索空间小；
- 不易发散；
- 实时性好；
- 适合作为主链路中的轻量兜底增强。

但它不能修正 `tvec` 的漂移，因此本方案进一步引入滑窗 `[x, y, z, yaw]` 优化作为增强链路。

---

## 3. 总体架构

### 3.1 总体流程

```text
Camera Image
    ↓
YOLO-Pose / OpenCV Detector
    ↓
bbox + corners + corner confidence + armor number
    ↓
IoU / SORT-like Tracker
    ↓
track_id 绑定
    ↓
主链路：
    PnP(raw corners)
        ↓
    Single-frame BaSolver(raw corners, t_pnp)
        ↓
    等待异步结果 max_wait_ms
        ├── 异步结果及时且质量正常 → 输出 Refined + Sliding BA
        └── 超时 / 异常 → 输出 PnP + Single-frame BA
    ↓
UKF / IMM
    ↓
MPC / Fire Decision
```

异步增强链路：

```text
Async Task
    ↓
ROI corner refinement
    ↓
PnP(refined corners)
    ↓
per-track sliding window
    ↓
Sliding Window BA
    ↓
quality check
    ↓
publish async result
```

### 3.2 分层职责

| 层级 | 功能 |
|---|---|
| Detector | 给出粗 bbox、粗角点、颜色、数字、置信度 |
| Tracker | 在图像空间绑定 `track_id`，避免不同目标混入同一窗口 |
| 主估计链路 | PnP + 单帧 BaSolver，保证实时输出 |
| 异步增强链路 | 局部角点精修 + 滑窗 BA，提高估计质量 |
| 降级状态机 | 根据质量、耗时和一致性决定输出 BA / 单帧 BA / PnP / LOST |
| UKF / IMM | 预测、延迟补偿、丢帧处理、多运动模型切换 |
| MPC | 使用预测状态控制云台 |

---

## 4. 主链路设计：实时兜底路径

主链路必须保证在每一帧都能快速给出可用结果，不能被局部精修或滑窗 BA 阻塞。

### 4.1 主链路步骤

```text
1. 接收当前检测结果
2. 使用 raw corners 做 PnP
3. 使用单帧 BaSolver 修正 yaw
4. 推送异步增强任务
5. 最多等待 max_wait_ms
6. 若异步结果可用且质量正常，输出异步 BA
7. 否则输出 PnP + 单帧 BaSolver
```

### 4.2 输出优先级

```text
Level 3: Refined corners + Sliding Window BA
Level 2: Raw corners + Single-frame BaSolver
Level 1: Raw PnP
Level 0: LOST
```

对应状态：

```cpp
enum class EstimateMode {
  NORMAL_SW_BA,          // 精修 + 滑窗 BA
  DEGRADED_SINGLE_BA,    // PnP + 单帧 BA
  DEGRADED_PNP,          // 原始 PnP
  LOST
};
```

### 4.3 最大等待时间

假设 YOLO 推理耗时约 `30 ms`，目标帧率约 `30 Hz`，单帧周期约 `33 ms`。增强结果等待时间不宜过长。

推荐：

```text
max_wait_ms = 1 ~ 3 ms
```

若控制周期较宽松，可设为：

```text
max_wait_ms = 5 ms
```

不建议超过 `5 ms`，否则增强链路的耗时抖动会传递到主链路。

### 4.4 主链路伪代码

```cpp
PoseResult processFrame(const Detection& det) {
  PoseResult result;

  // 1. Raw PnP
  PnpResult pnp = solvePnP(det.raw_corners);
  if (!pnp.ok) {
    result.valid = false;
    result.mode = EstimateMode::LOST;
    return result;
  }

  // 2. Single-frame BaSolver
  SingleBaResult single_ba = singleBaSolver.optimizeYaw(
      det.raw_corners,
      pnp.t,
      pnp.R,
      det.armor_type,
      det.armor_number
  );

  PoseCandidate fallback;
  if (single_ba.ok) {
    fallback.t = pnp.t;
    fallback.yaw = single_ba.yaw;
    fallback.mode = EstimateMode::DEGRADED_SINGLE_BA;
  } else {
    fallback.t = pnp.t;
    fallback.yaw = pnp.yaw;
    fallback.mode = EstimateMode::DEGRADED_PNP;
  }

  // 3. Push async task
  EstimationTask task;
  task.frame_id = det.frame_id;
  task.timestamp = det.timestamp;
  task.track_id = det.track_id;
  task.raw_corners = det.raw_corners;
  task.corner_conf = det.corner_conf;
  task.t_pnp = pnp.t;
  task.yaw_pnp = fallback.yaw;
  task.armor_type = det.armor_type;
  task.armor_number = det.armor_number;

  async_optimizer.pushLatest(task);

  // 4. Wait async result with timeout
  AsyncBaResult async_res;
  if (async_optimizer.tryGetResult(
          det.frame_id,
          det.track_id,
          max_wait_ms,
          async_res) &&
      async_res.quality_ok) {
    result.valid = true;
    result.mode = EstimateMode::NORMAL_SW_BA;
    result.t = async_res.t_ba;
    result.yaw = async_res.yaw_ba;
    result.reproj_error = async_res.reproj_error;
    return result;
  }

  // 5. Timeout or bad async result, fallback
  result.valid = true;
  result.mode = fallback.mode;
  result.t = fallback.t;
  result.yaw = fallback.yaw;
  return result;
}
```

---

## 5. 异步增强链路设计：BA + 精修线程

### 5.1 异步线程职责

异步线程负责高精度但非强实时的部分：

```text
1. 接收最新任务
2. 局部角点精修
3. 精修后重新 PnP
4. 根据 track_id 推入对应滑窗
5. 执行滑窗 BA
6. 质量评估
7. 发布结果
```

### 5.2 任务结构

```cpp
struct EstimationTask {
  uint64_t frame_id;
  double timestamp;
  int track_id;

  std::array<cv::Point2f, 4> raw_corners;
  std::array<double, 4> corner_conf;

  Eigen::Vector3d t_pnp;
  double yaw_pnp;

  ArmorType armor_type;
  std::string armor_number;

  cv::Rect2f bbox;
  cv::Mat image_roi;  // 可选，注意避免大规模 clone
};
```

### 5.3 结果结构

```cpp
struct AsyncBaResult {
  uint64_t frame_id;
  double timestamp;
  int track_id;

  Eigen::Vector3d t_ba;
  double yaw_ba;

  double reproj_error;
  double chi2;
  bool quality_ok;
};
```

### 5.4 最新任务队列

为避免异步线程落后，队列不应无限堆积。推荐只保留最新任务：

```text
queue_size = 1
新任务到来时覆盖旧任务
```

伪代码：

```cpp
class LatestTaskQueue {
public:
  void push(const EstimationTask& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_task_ = task;
    has_task_ = true;
    cv_.notify_one();
  }

  bool popLatest(EstimationTask& task) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return has_task_ || stop_; });

    if (stop_) return false;

    task = latest_task_;
    has_task_ = false;
    return true;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  EstimationTask latest_task_;
  bool has_task_ = false;
  bool stop_ = false;
};
```

### 5.5 异步线程伪代码

```cpp
void AsyncOptimizer::workerLoop() {
  while (running_) {
    EstimationTask task;
    if (!queue_.popLatest(task)) {
      continue;
    }

    AsyncBaResult result;
    result.frame_id = task.frame_id;
    result.timestamp = task.timestamp;
    result.track_id = task.track_id;

    // 1. Local corner refinement
    RefineResult refined = cornerRefiner.refine(task);

    const auto& corners =
        refined.ok ? refined.corners : task.raw_corners;

    // 2. Refined PnP
    PnpResult pnp_refined = solvePnP(corners);
    if (!pnp_refined.ok) {
      result.quality_ok = false;
      publishResult(result);
      continue;
    }

    // 3. Push to per-track sliding window
    FrameData frame;
    frame.timestamp = task.timestamp;
    frame.corners = corners;
    frame.corner_conf = task.corner_conf;
    frame.t_pnp = pnp_refined.t;
    frame.yaw_pnp = pnp_refined.yaw;
    frame.armor_type = task.armor_type;
    frame.armor_number = task.armor_number;

    auto& solver = getSolver(task.track_id);
    solver.pushFrame(frame);

    // 4. Sliding window BA
    Eigen::Vector3d t_ba;
    double yaw_ba = 0.0;
    bool ok = solver.optimize(t_ba, yaw_ba);

    // 5. Quality check
    result.quality_ok =
        ok &&
        solver.lastMeanReprojError() < ba_reproj_threshold &&
        (t_ba - pnp_refined.t).norm() < pose_delta_threshold &&
        std::isfinite(yaw_ba);

    result.t_ba = t_ba;
    result.yaw_ba = yaw_ba;
    result.reproj_error = solver.lastMeanReprojError();
    result.chi2 = solver.lastChi2();

    publishResult(result);
  }
}
```

---

## 6. 区域传统角点精修模块

### 6.1 输入与输出

输入：

```text
YOLO-Pose 粗角点
bbox / rotated rect
原始图像或局部 ROI
armor color / armor number
```

输出：

```text
refined corners[4]
refine_ok
refine_quality
```

### 6.2 推荐流程

```text
YOLO 粗角点
    ↓
估计左右灯条局部 ROI
    ↓
灰度化 / 颜色通道增强 / 局部二值化
    ↓
提取亮区点集
    ↓
PCA 得到灯条主方向
    ↓
沿主方向搜索明暗交界
    ↓
得到左右灯条上下端点
    ↓
角点排序与几何检查
```

### 6.3 PCA 精修核心

对单个灯条 ROI 内的亮区点集 `Q = {q_j}`：

```text
1. 计算点集均值 μ
2. 计算协方差矩阵 Cov
3. PCA 第一主成分 v1 作为灯条主方向
4. 在 μ ± α v1 方向搜索灰度梯度最大或明暗边界
5. 得到灯条两个端点
```

### 6.4 精修失败条件

以下情况应直接退回 YOLO 原始角点：

```text
ROI 太小
亮区点数量不足
PCA 主方向不稳定
灯条长宽比异常
明暗边界搜索失败
精修后四边形面积异常
精修后角点重投影误差更大
```

### 6.5 性能控制

角点精修是最可能成为新瓶颈的模块，因此必须限制范围：

```text
只对 confirmed track 精修
优先只对当前主目标精修
限制 ROI 最大尺寸
避免全图 findContours
避免频繁 Mat clone
精修耗时超过阈值则跳过
```

推荐阈值：

```text
corner_refine_time_threshold = 2 ~ 3 ms
```

---

## 7. PnP 与单帧降自由度 BaSolver

### 7.1 Raw PnP

PnP 使用检测角点或精修角点求解：

```text
object_points[4] + image_points[4] + K + D
    ↓
t_pnp, R_pnp
```

对于平面装甲板，推荐考虑：

```text
SOLVEPNP_IPPE
```

若工程中已有稳定实现，也可沿用当前 `solvePnP` 流程。

### 7.2 单帧 BaSolver

单帧 BaSolver 作为主链路轻量增强模块。

优化变量：

```text
yaw
```

固定量：

```text
t = t_pnp
pitch = ±15°
roll = 0°
object_points
camera intrinsics
```

目标函数：

```text
min_yaw Σ_i || u_i - project(R_yaw · R_pitch · P_i + t_pnp) ||²
```

其作用：

```text
降低完整 PnP 姿态中的不稳定性
为后续滑窗提供更可靠 yaw 初值
作为异步超时时的降级输出
```

### 7.3 单帧 yaw + t 优化的可选增强

如果发现 `t_pnp` 抖动明显，可引入轻量单帧 `[x, y, z, yaw]` 优化：

```text
min_{t,yaw}
Σ_i || u_i - project(R_yaw · R_pitch · P_i + t) ||²
+ λ || t - t_pnp ||²
+ λ_yaw || yaw - yaw_pnp ||²
```

但若该优化放在主链路中，需严格限制迭代次数与耗时。第一版建议主链路只优化 yaw。

---

## 8. Per-track 滑窗 BA 建模

### 8.1 每个 track 独立滑窗

```text
track_id = 1 → SlidingWindowBaSolver_1
track_id = 2 → SlidingWindowBaSolver_2
...
```

这样可以避免不同目标或不同装甲板观测混入同一窗口。

### 8.2 窗口状态

窗口大小为 `N`，第 `k` 帧状态：

```text
x_k = [tx_k, ty_k, tz_k, yaw_k]^T
```

固定：

```text
pitch = ±15°
roll = 0°
```

推荐：

```text
N = 5
高速旋转时 N = 3 ~ 4
```

### 8.3 总目标函数

```text
E = E_reproj + λ_p E_prior + λ_m E_motion
```

展开：

```text
E =
Σ_k Σ_i ρ( || u_{k,i} - π(K, R(yaw_k) R_pitch P_i + t_k) ||²_{W_{k,i}} )
+
λ_p Σ_k || x_k - x_pnp_k ||²_{Σ_prior^{-1}}
+
λ_m Σ_k || x_k - x_{k-1} ||²_{Σ_motion^{-1}}
```

其中：

- `E_reproj`：角点重投影误差；
- `E_prior`：PnP 弱先验，防止深度漂移；
- `E_motion`：帧间连续性约束；
- `ρ`：鲁棒核，推荐 Huber；
- `W`：角点置信度对应的信息矩阵。

### 8.4 角点权重

```cpp
double sigma = sigma_min + (1.0 - conf) * sigma_scale;
double weight = 1.0 / (sigma * sigma);
```

推荐初值：

```text
sigma_min = 1.0 px
sigma_scale = 5.0 px
huber_delta = 3.0 px
```

### 8.5 PnP 先验权重

```text
sigma_prior_xy  = 0.05 ~ 0.10 m
sigma_prior_z   = 0.10 ~ 0.20 m
sigma_prior_yaw = 0.20 ~ 0.50 rad
```

### 8.6 运动约束权重

建议根据 `dt` 自适应：

```cpp
double sigma_t   = base_sigma_t   + max_linear_speed * dt;
double sigma_yaw = base_sigma_yaw + max_yaw_rate     * dt;
```

推荐：

```text
base_sigma_xy   = 0.03 ~ 0.08 m
base_sigma_z    = 0.05 ~ 0.15 m
base_sigma_yaw  = 0.05 ~ 0.15 rad
max_yaw_rate    = 20 ~ 25 rad/s
```

---

## 9. g2o 顶点与边设计

### 9.1 VertexPoseYawT

```cpp
class VertexPoseYawT final : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override {
    _estimate.setZero();
  }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Eigen::Vector4d> du(update);
    _estimate += du;
    _estimate[3] = std::atan2(std::sin(_estimate[3]), std::cos(_estimate[3]));
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

  Eigen::Vector3d t() const { return _estimate.head<3>(); }
  double yaw() const { return _estimate[3]; }
};
```

### 9.2 EdgeReprojectionYawT

```cpp
class EdgeReprojectionYawT final
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPoseYawT> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeReprojectionYawT(
      const Eigen::Vector3d& object_point,
      const Eigen::Matrix3d& K,
      const Sophus::SO3d& R_camera_imu,
      const Sophus::SO3d& R_pitch)
      : object_point_(object_point),
        K_(K),
        R_camera_imu_(R_camera_imu),
        R_pitch_(R_pitch) {}

  void computeError() override {
    const auto* v = static_cast<const VertexPoseYawT*>(_vertices[0]);

    Eigen::Vector3d t = v->t();
    double yaw = v->yaw();

    Sophus::SO3d R_yaw =
        Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, yaw));

    Eigen::Vector3d pc =
        (R_camera_imu_ * R_yaw * R_pitch_) * object_point_ + t;

    if (pc.z() <= 1e-6) {
      _error = Eigen::Vector2d(1e3, 1e3);
      return;
    }

    double inv_z = 1.0 / pc.z();

    Eigen::Vector2d proj;
    proj.x() = K_(0, 0) * pc.x() * inv_z + K_(0, 2);
    proj.y() = K_(1, 1) * pc.y() * inv_z + K_(1, 2);

    _error = _measurement - proj;
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  Eigen::Vector3d object_point_;
  Eigen::Matrix3d K_;
  Sophus::SO3d R_camera_imu_;
  Sophus::SO3d R_pitch_;
};
```

### 9.3 EdgePriorYawT

```cpp
class EdgePriorYawT final
    : public g2o::BaseUnaryEdge<4, Eigen::Vector4d, VertexPoseYawT> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override {
    const auto* v = static_cast<const VertexPoseYawT*>(_vertices[0]);

    _error = v->estimate() - _measurement;
    _error[3] = std::atan2(std::sin(_error[3]), std::cos(_error[3]));
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};
```

### 9.4 EdgeSmoothYawT

```cpp
class EdgeSmoothYawT final
    : public g2o::BaseBinaryEdge<4, Eigen::Vector4d,
                                 VertexPoseYawT, VertexPoseYawT> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override {
    const auto* v0 = static_cast<const VertexPoseYawT*>(_vertices[0]);
    const auto* v1 = static_cast<const VertexPoseYawT*>(_vertices[1]);

    _error = v1->estimate() - v0->estimate();
    _error[3] = std::atan2(std::sin(_error[3]), std::cos(_error[3]));
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};
```

### 9.5 图规模

以 `N = 5` 为例：

```text
顶点：
5 个 VertexPoseYawT
总变量：20 维

边：
20 条重投影边
5 条 PnP 先验边
4 条帧间平滑边
```

该规模较小，一般不会成为主要实时瓶颈。

---

## 10. Tracker 与滑窗实例管理

### 10.1 TrackState

```cpp
struct TrackState {
  int id;
  cv::Rect2f bbox;
  cv::RotatedRect armor_rect;

  int age = 0;
  int hit_count = 0;
  int miss_count = 0;

  ArmorType armor_type;
  std::string armor_number;

  SlidingWindowBaSolver solver;

  Eigen::Vector3d last_t;
  double last_yaw = 0.0;
};
```

### 10.2 关联规则

关联不应只看 IoU，建议使用：

```text
IoU
中心距离
armor number
armor type
PnP 位置门控
bbox 面积变化
yaw / yaw_rate 一致性
```

推荐策略：

```text
armor number 不一致 → 强惩罚或拒绝
armor type 不一致 → 强惩罚
IoU 极低且中心距离过大 → 拒绝
PnP 空间位置跳变过大 → 拒绝
```

### 10.3 Track 生命周期

```text
Tentative:
    新检测创建 track，但暂不稳定输出
    连续命中 min_hits 次后转 Confirmed

Confirmed:
    正常维护滑窗并输出

Missing:
    短暂丢失，保留窗口
    miss_count 超过 max_age 删除

Deleted:
    删除 track 与对应 solver
```

推荐参数：

```text
min_hits = 2 ~ 3
max_age = 3 ~ 5
```

---

## 11. 高速旋转目标的特殊处理

装甲板绕运动轴心旋转时，真实状态可能连续，但图像观测会出现不连续：

```text
面板切换
遮挡
角点顺序变化
ID switch
yaw 过 ±π 边界
采样率不足造成 aliasing
```

### 11.1 yaw unwrap

必须处理：

```cpp
double unwrapYaw(double prev, double curr) {
  double diff = curr - prev;
  while (diff > M_PI)  diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return prev + diff;
}
```

### 11.2 yaw_rate gating

```cpp
double omega = wrapAngle(yaw_k - yaw_prev) / dt;
if (std::abs(omega) > omega_max) {
  markAsSuspicious();
}
```

推荐：

```text
omega_max = 20 ~ 25 rad/s
```

### 11.3 面板切换重置

以下情况建议重置滑窗：

```text
armor number 变化
armor type 变化
track_id 重新分配
连续丢失后重捕获
bbox 面积突变
yaw 突变异常
角点几何异常
```

### 11.4 自适应窗口

```text
高速旋转 / 观测不稳定：N = 3 ~ 4
正常运动：N = 5
低速稳定：N = 5 ~ 8
```

窗口越大，平滑越强，但滞后和窗口污染风险越高。

---

## 12. 质量评估指标

### 12.1 单帧 PnP 质量

```text
PnP 是否成功
PnP 重投影误差
t.z 是否为正
角点是否构成合理四边形
bbox 面积是否过小
角点平均置信度
```

推荐阈值：

```text
pnp_reproj_error > 5 px → 可疑
mean_corner_conf < 0.4 → 可疑
t.z <= 0 → 无效
```

### 12.2 角点精修质量

```text
ROI 是否足够大
亮区点数量是否足够
PCA 主方向是否稳定
精修后角点几何是否合理
精修后 PnP 重投影误差是否下降
```

若精修后质量变差，应直接退回 raw corners。

### 12.3 滑窗 BA 质量

```text
BA 平均重投影误差
chi2
是否 NaN
优化前后 t / yaw 偏差
是否超时
```

推荐阈值：

```text
ba_mean_reproj_error > 4 px → 可疑
|t_ba - t_pnp| > 0.15 ~ 0.30 m → 可疑
|yaw_ba - yaw_pnp| > 0.3 ~ 0.5 rad → 可疑
ba_time > 5 ms → 超时降级
```

### 12.4 关联质量

```text
IoU
中心距离
armor number / type 一致性
bbox 面积变化
miss_count
ID switch
```

推荐：

```text
IoU < 0.1 且中心距离大 → 可疑
bbox_area_ratio > 2.0 或 < 0.5 → 可疑
armor number 改变 → 重置窗口
```

---

## 13. 降级机制

### 13.1 状态定义

```cpp
enum class EstimateMode {
  NORMAL_SW_BA,
  DEGRADED_SINGLE_BA,
  DEGRADED_PNP,
  LOST
};
```

### 13.2 状态切换

```text
NORMAL_SW_BA:
    异步 refined + sliding BA 按时返回且质量正常

DEGRADED_SINGLE_BA:
    异步结果超时或质量异常
    但 PnP + 单帧 BaSolver 正常

DEGRADED_PNP:
    单帧 BaSolver 异常
    但 PnP 正常

LOST:
    PnP 失败或检测/track 不可靠
```

### 13.3 异常计数

```cpp
if (bad_metric) {
  bad_count++;
} else {
  bad_count = std::max(0, bad_count - 1);
}

if (bad_count >= 2) {
  mode = EstimateMode::DEGRADED_SINGLE_BA;
}
```

### 13.4 恢复计数

```cpp
if (good_metric) {
  good_count++;
} else {
  good_count = 0;
}

if (mode != EstimateMode::NORMAL_SW_BA && good_count >= 3) {
  mode = EstimateMode::NORMAL_SW_BA;
}
```

推荐：

```text
bad_count_threshold = 2
good_count_threshold = 3
```

---

## 14. 窗口冻结、重置与删除

### 14.1 冻结窗口

适用于轻微异常：

```text
角点置信度低
当前帧重投影误差略大
精修失败
异步 BA 超时
```

动作：

```text
当前帧不加入窗口
保留历史窗口
输出 PnP + 单帧 BaSolver
```

### 14.2 重置窗口

适用于严重异常：

```text
armor number 变化
track_id 重新分配
连续丢失后重捕获
角点顺序错误
bbox 面积剧烈突变
yaw unwrap 失败
BA 结果 NaN
```

动作：

```text
清空窗口
用当前 PnP / 单帧 BA 重新初始化
输出降级结果
```

### 14.3 删除窗口

适用于目标消失：

```text
miss_count > max_age
```

动作：

```text
删除 track
删除对应 SlidingWindowBaSolver
```

---

## 15. 实时性分析与性能保护

### 15.1 模块耗时风险

| 模块 | 瓶颈风险 | 说明 |
|---|---|---|
| PnP | 低 | 4 点 PnP 很轻 |
| 单帧 yaw BA | 低 | 一维优化或小规模 g2o |
| 滑窗 BA | 中低 | N=5 时规模很小 |
| 局部角点精修 | 中高 | OpenCV ROI、PCA、轮廓、边缘搜索可能吃 CPU |
| 多目标全量优化 | 高 | 多 track 同时 refine + BA 会叠加 |
| ROS2 图像传输 / Foxglove | 高 | 大图像消息和可视化可能拖慢 |

### 15.2 耗时统计

必须加入 profiling：

```cpp
time_yolo_preprocess
time_yolo_inference
time_yolo_postprocess
time_tracker
time_pnp
time_single_ba
time_corner_refine
time_sliding_ba
time_total
```

日志示例：

```text
[PerceptionProfile]
pre=1.2ms infer=30.4ms post=2.8ms
pnp=0.2ms single_ba=0.1ms
refine=1.6ms sw_ba=1.4ms
total=37.7ms
```

### 15.3 超时策略

```text
corner_refine > 3 ms → 跳过精修
sliding_ba > 5 ms → 异步结果标记超时
main wait > max_wait_ms → 退回 PnP + 单帧 BA
total pipeline 超过目标周期 → 进入 FAST 模式
```

### 15.4 模式分级

#### FAST 模式

```text
YOLO-Pose
    ↓
PnP
    ↓
单帧 BaSolver
    ↓
UKF
```

不做精修，不做滑窗 BA。

#### NORMAL 模式

```text
YOLO-Pose
    ↓
PnP + 单帧 BaSolver
    ↓
异步精修 + 滑窗 BA
    ↓
按时返回则使用 BA，否则降级
```

#### DEBUG / FULL 模式

```text
对多个 confirmed track 做完整 refine + BA
输出详细 residual / profiling
```

仅用于离线调试或性能评估。

---

## 16. 推荐参数初值

```text
max_wait_ms = 2 ~ 3 ms

single_ba_iterations = 10 ~ 20

sliding_window_size = 5
sliding_ba_iterations = 5 ~ 10

async_queue_size = 1

huber_delta = 3.0 px

sigma_kp_min = 1.0 px
sigma_kp_scale = 5.0 px

sigma_prior_xy = 0.05 ~ 0.10 m
sigma_prior_z = 0.10 ~ 0.20 m
sigma_prior_yaw = 0.20 ~ 0.50 rad

sigma_smooth_xy = 0.03 ~ 0.08 m
sigma_smooth_z = 0.05 ~ 0.15 m
sigma_smooth_yaw = 0.05 ~ 0.15 rad

pnp_reproj_error_th = 5.0 px
ba_reproj_error_th = 4.0 px

pose_delta_t_th = 0.15 ~ 0.30 m
pose_delta_yaw_th = 0.3 ~ 0.5 rad

yaw_rate_max = 20 ~ 25 rad/s

bad_count_threshold = 2
good_count_threshold = 3

corner_refine_time_threshold = 2 ~ 3 ms
sliding_ba_time_threshold = 5 ms
```

---

## 17. 推荐实现顺序

### 阶段 1：主链路兜底

```text
PnP(raw corners)
单帧 BaSolver yaw 优化
输出 DEGRADED_SINGLE_BA
加入耗时统计
```

目标：

```text
保证实时性和基础稳定性
```

### 阶段 2：异步框架

```text
LatestTaskQueue
AsyncOptimizer 线程
frame_id / track_id 对齐
max_wait_ms 超时退回
```

目标：

```text
建立不阻塞主链路的增强通道
```

### 阶段 3：滑窗 BA

```text
VertexPoseYawT
EdgeReprojectionYawT
EdgePriorYawT
EdgeSmoothYawT
per-track SlidingWindowBaSolver
```

目标：

```text
验证滑窗对 PnP 抖动的抑制效果
```

### 阶段 4：区域角点精修

```text
YOLO ROI 裁剪
PCA 灯条主方向
明暗边界搜索
精修失败回退
```

目标：

```text
从源头降低角点观测误差
```

### 阶段 5：质量评估与降级状态机

```text
PnP 质量
refine 质量
BA 质量
tracker 质量
运动一致性
冻结 / 重置 / 删除窗口
```

目标：

```text
避免滑窗被错误观测污染
```

### 阶段 6：高速旋转增强

```text
yaw unwrap
yaw_rate gating
自适应窗口 N
面板切换检测
可选 yaw_rate 状态
接入 IMM
```

目标：

```text
适配装甲板绕轴高速旋转场景
```

---

## 18. 主要隐患与应对

| 隐患 | 表现 | 应对 |
|---|---|---|
| detector 系统性偏差 | 输出稳定但整体偏 | 改标定、训练、畸变补偿、角点精修 |
| 局部精修失败 | 角点被修坏 | 质量检查，失败回退 raw corners |
| 滑窗污染 | BA 输出跳变或发散 | 异常帧门控、冻结/重置窗口 |
| ID switch | 不同目标进入同一窗口 | IoU + number + pose gating |
| 高速旋转滞后 | 输出落后真实目标 | 小窗口、自适应平滑、yaw_rate 模型 |
| 深度漂移 | z 慢慢偏 | PnP 弱先验 |
| yaw wrap 错误 | 角度突然跳变 | yaw unwrap |
| 精修成为瓶颈 | CPU 占用上升 | ROI 限制、只处理主目标、超时跳过 |
| 异步结果过期 | 使用旧帧结果 | frame_id / track_id / timestamp 校验 |
| 队列堆积 | BA 输出越来越滞后 | queue_size = 1，只保留最新任务 |

---

## 19. 最终推荐架构

```text
主线程：

YOLO-Pose / Detector
    ↓
Tracker association
    ↓
PnP(raw corners)
    ↓
Single-frame BaSolver(raw corners)
    ↓
push async task
    ↓
wait max_wait_ms
    ├── async result ready + quality ok
    │       output NORMAL_SW_BA
    └── timeout / bad
            output DEGRADED_SINGLE_BA or DEGRADED_PNP


异步线程：

latest task
    ↓
ROI corner refinement
    ↓
PnP(refined corners)
    ↓
per-track SlidingWindowBaSolver
    ↓
quality check
    ↓
publish AsyncBaResult


后端：

PoseResult
    ↓
UKF / IMM
    ↓
delay compensation
    ↓
MPC / Fire decision
```

---

## 20. 总结

结合开源方案与当前新设计，推荐采用如下定位：

```text
PnP + 单帧 BaSolver 是实时安全底座；
区域角点精修是前端精度增强；
滑窗 BA 是时序稳定增强；
降级状态机是工程可靠性保障；
UKF / IMM 是预测与控制接口。
```

最终系统不应设计成：

```text
YOLO → 精修 → BA → 输出
```

这种强串行链路，而应设计成：

```text
YOLO → PnP → 单帧 BA → 可立即输出
              ↓
        异步精修 + 滑窗 BA
              ↓
        按时且可靠才替换输出
```

一句话总结：

**把“精修 + 滑窗 BA”做成异步增强模块，把“PnP + 单帧 BaSolver”作为实时兜底输出，是在精度、实时性和鲁棒性之间更均衡的工程方案。**
