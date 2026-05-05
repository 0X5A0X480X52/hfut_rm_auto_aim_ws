# Phase 4: PnP Output Armors

## 1. Overview

**Goal:** Convert `ArmorDetection` keypoints to full `rm_interfaces/msg/Armors` messages via PnP pose estimation, matching the legacy `armor_detector` output semantics. This is the phase where `armor_detector_nn` becomes a functional drop-in replacement — downstream tracker and gimbal nodes receive Armors messages indistinguishable from the old detector.

**Precondition:** Phase 3 complete (postprocess produces valid `ArmorDetection` objects with bbox, keypoints, labels).

---

## 2. Data Flow (Phase 4)

```
ArmorDetection (keypoints in image coords, publish_type, publish_number)
  → ArmorPoseEstimatorAdapter
    → select object points (small vs large)
    → solvePnP / IPPE
    → optional bundle adjustment (BA)
  → fill rm_interfaces::msg::Armor
    → number, type, pose, distance_to_image_center
  → assemble Armors message with original Image header
  → publish /armor_detector/armors
  → publish /armor_detector/marker (visualization)
```

---

## 3. Keypoint-to-Landmark Convention

### 3.1 Canonical Order (reaffirmed)

From Phase 3, keypoints are in canonical order:

```
kpt0: left_bottom  (of the left light bar)
kpt1: left_top
kpt2: right_top    (of the right light bar)
kpt3: right_bottom
```

This matches the legacy `Armor::landmarks()` for `N_LANDMARKS == 4`:
```cpp
{left_light.bottom, left_light.top, right_light.top, right_light.bottom}
```

### 3.2 Object Points (3D)

The object coordinate system is centered on the armor plate:

```
Origin: center of the armor plate
X-axis: pointing forward (out from the plate, toward the camera when head-on)
Y-axis: pointing left (from the plate's perspective)
Z-axis: pointing up

Small armor (width=0.133m, height=0.050m):
  kpt0 (left_bottom):  (0,  w/2, -h/2) = (0,  0.0665, -0.025)
  kpt1 (left_top):     (0,  w/2,  h/2) = (0,  0.0665,  0.025)
  kpt2 (right_top):    (0, -w/2,  h/2) = (0, -0.0665,  0.025)
  kpt3 (right_bottom): (0, -w/2, -h/2) = (0, -0.0665, -0.025)

Large armor (width=0.225m, height=0.050m):
  kpt0 (left_bottom):  (0,  0.1125, -0.025)
  kpt1 (left_top):     (0,  0.1125,  0.025)
  kpt2 (right_top):    (0, -0.1125,  0.025)
  kpt3 (right_bottom): (0, -0.1125, -0.025)
```

This uses the exact same `Armor::buildObjectPoints()` from the legacy `armor_detector` ([types.hpp:116](../../armor_detector/include/armor_detector/types.hpp)).

### 3.3 Selection Rule

| `ArmorDetection.publish_type` | Object points used |
|:--|:--|
| `"small"` | `buildObjectPoints(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)` |
| `"large"` | `buildObjectPoints(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT)` |
| `"invalid"` | skip PnP, mark Armor.type as `"invalid"` |

---

## 4. ArmorPoseEstimatorAdapter

### 4.1 Header: `include/armor_detector_nn/core/armor_pose_estimator_adapter.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_ARMOR_POSE_ESTIMATOR_ADAPTER_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_POSE_ESTIMATOR_ADAPTER_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>

namespace fyt::auto_aim {

// Result of a single-armor pose estimation.
struct PoseEstimate {
  bool valid{false};
  Eigen::Vector3d translation;    // in camera frame
  Eigen::Quaterniond rotation;    // camera → armor
  cv::Mat rvec;                   // Rodrigues vector
  cv::Mat tvec;                   // translation vector
  double reprojection_error{0.0};
};

class ArmorPoseEstimatorAdapter {
public:
  explicit ArmorPoseEstimatorAdapter(const PoseConfig& config);

  // Estimate pose for a single armor detection.
  PoseEstimate estimate(
    const ArmorDetection& detection,
    const sensor_msgs::msg::CameraInfo& camera_info);

  // Batch estimation for multiple detections.
  std::vector<PoseEstimate> estimateBatch(
    const std::vector<ArmorDetection>& detections,
    const sensor_msgs::msg::CameraInfo& camera_info);

  // Return canonical 3D object points for a given armor type.
  static std::vector<cv::Point3f>
  getObjectPoints(const std::string& publish_type,
                  double small_w, double small_h,
                  double large_w, double large_h);

  // Compute distance from bbox center to image center (in pixels).
  // Must match legacy PnPSolver::calculateDistanceToCenter() semantics.
  static double distanceToImageCenter(
    const cv::Point2f& center,
    const cv::Point2f& image_center);

  const PoseConfig& config() const { return config_; }

private:
  // Run PnP + optional BA on one set of image/object point correspondences.
  PoseEstimate solvePnP(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs);

  PoseConfig config_;
};

} // namespace fyt::auto_aim

#endif
```

### 4.2 Implementation Outline

**`estimate(detection, camera_info)`:**
1. If `detection.publish_type == "invalid"` → return invalid `PoseEstimate`.
2. Get object points: `getObjectPoints(detection.publish_type, ...)`.
3. Extract image points from `detection.keypoints` in canonical order.
4. Parse `camera_info` → `camera_matrix` (3x3) and `dist_coeffs` (1x5 or 1x8).
5. Call `solvePnP(image_points, object_points, K, D)`.
6. If `valid`:
   - Convert `rvec` → `Eigen::Quaterniond` via `Sophus::SO3`.
   - Set `translation` from `tvec`.
7. Return `PoseEstimate`.

**`getObjectPoints()`** is static so it can be called without an adapter instance. This is useful for testing and for future strategies that may want to override object points.

**`distanceToImageCenter()`**:

```cpp
double ArmorPoseEstimatorAdapter::distanceToImageCenter(
    const cv::Point2f& center, const cv::Point2f& image_center)
{
  return cv::norm(center - image_center);
}
```

The legacy `PnPSolver::calculateDistanceToCenter()` returns this value in pixels, normalized by image dimensions. We must check the exact legacy formula and replicate it. If the legacy code divides by `max(width,height)/2`, this phase implements the same normalization.

### 4.3 PnP Solver Selection

```cpp
PoseEstimate ArmorPoseEstimatorAdapter::solvePnP(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
  PoseEstimate result;

  if (config_.pnp_method == "ippe") {
    // IPPE: specialized for planar objects, returns two solutions.
    // We select the one with the lower reprojection error.
    std::vector<cv::Mat> rvecs, tvecs;
    try {
      cv::solvePnPGeneric(object_points, image_points,
                          camera_matrix, dist_coeffs,
                          rvecs, tvecs,
                          false,  // useExtrinsicGuess
                          cv::SOLVEPNP_IPPE);
      if (!rvecs.empty()) {
        // IPPE returns 2 solutions.
        // Selection rule:
        // 1) keep candidates with tvec.z > 0 (in front of camera),
        // 2) among valid candidates, pick the one with minimum reprojection error,
        // 3) if none satisfy z > 0, fallback to global minimum reprojection error.
        int best = -1;
        double best_err = std::numeric_limits<double>::max();
        std::vector<double> reproj_errors(rvecs.size());
        for (size_t i = 0; i < rvecs.size(); ++i) {
          // Compute reprojection error
          std::vector<cv::Point2f> projected;
          cv::projectPoints(object_points, rvecs[i], tvecs[i],
                            camera_matrix, dist_coeffs, projected);
          double err = 0.0;
          for (size_t j = 0; j < image_points.size(); ++j) {
            err += cv::norm(image_points[j] - projected[j]);
          }
          reproj_errors[i] = err;
          const double z_in_camera = tvecs[i].at<double>(2);
          if (z_in_camera > 0.0 && err < best_err) {
            best_err = err;
            best = static_cast<int>(i);
          }
        }
        if (best < 0) {
          for (size_t i = 0; i < reproj_errors.size(); ++i) {
            if (reproj_errors[i] < best_err) {
              best_err = reproj_errors[i];
              best = static_cast<int>(i);
            }
          }
        }
        result.rvec = rvecs[best];
        result.tvec = tvecs[best];
        result.reprojection_error = reproj_errors[best];
        result.valid = true;
      }
    } catch (const cv::Exception& e) {
      // Fall through, result.valid stays false.
    }
  } else {
    // Default: SOLVEPNP_ITERATIVE
    cv::Mat rvec, tvec;
    if (cv::solvePnP(object_points, image_points,
                     camera_matrix, dist_coeffs,
                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
      result.rvec = rvec;
      result.tvec = tvec;
      result.valid = true;
    }
  }

  if (result.valid) {
    result.translation = Eigen::Vector3d(
      result.tvec.at<double>(0),
      result.tvec.at<double>(1),
      result.tvec.at<double>(2));
    cv::Mat R;
    cv::Rodrigues(result.rvec, R);
    Eigen::Matrix3d eigen_R;
    cv::cv2eigen(R, eigen_R);
    result.rotation = Eigen::Quaterniond(eigen_R);
  }

  return result;
}
```

---

## 5. Bundle Adjustment (BA) Integration

### 5.1 Existing BA

The current `armor_detector` package includes `ba_solver.hpp` / `ba_solver.cpp`. Phase 4 reuses this solver as-is:

```cpp
// In ArmorPoseEstimatorAdapter, after initial PnP:
if (config_.use_ba && result.valid) {
  // Feed the initial PnP solution as the BA initial guess.
  // BA refines using the same image/object point correspondences.
  result = ba_solver_->refine(result, image_points, object_points,
                              camera_matrix, dist_coeffs);
}
```

### 5.2 BA Abstraction (reserved interface)

If the BA implementation changes between phases or platforms, encapsulate behind an interface:

```cpp
class IBundleAdjuster {
public:
  virtual ~IBundleAdjuster() = default;
  virtual PoseEstimate refine(
    const PoseEstimate& initial,
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& K, const cv::Mat& D) = 0;
};
```

The `ArmorPoseEstimatorAdapter` holds a `std::unique_ptr<IBundleAdjuster>`. If `use_ba: false`, the pointer is null and the step is skipped.

---

## 6. Mapping to `rm_interfaces/msg/Armors`

### 6.1 `Armors` Message Structure

```cpp
// rm_interfaces/msg/Armors
std_msgs/Header header
Armor[] armors

// rm_interfaces/msg/Armor
string number
string type
float32 distance_to_image_center
geometry_msgs/Pose pose
```

`rm_interfaces/msg/Armor` currently has no `confidence` field. If confidence needs to be exposed later, it should be done via:

- debug topic, or
- explicit `rm_interfaces` message evolution in a separate compatibility-reviewed change.

### 6.2 Filling Logic

```cpp
rm_interfaces::msg::Armors buildArmorsMsg(
    const std_msgs::msg::Header& header,
    const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses,
    const cv::Point2f& image_center)
{
  rm_interfaces::msg::Armors msg;
  msg.header = header;   // CRITICAL: use original Image header, NOT current time

  size_t n = detections.size();
  msg.armors.resize(n);

  for (size_t i = 0; i < n; ++i) {
    auto& armor = msg.armors[i];
    armor.number = detections[i].publish_number;
    armor.type   = detections[i].publish_type;

    if (poses[i].valid) {
      armor.pose.position.x = poses[i].translation.x();
      armor.pose.position.y = poses[i].translation.y();
      armor.pose.position.z = poses[i].translation.z();
      armor.pose.orientation.x = poses[i].rotation.x();
      armor.pose.orientation.y = poses[i].rotation.y();
      armor.pose.orientation.z = poses[i].rotation.z();
      armor.pose.orientation.w = poses[i].rotation.w();
    }
    // else: pose remains at identity/origin (invalid marker)

    armor.distance_to_image_center = static_cast<float>(
        ArmorPoseEstimatorAdapter::distanceToImageCenter(
            detections[i].center, image_center));
  }

  return msg;
}
```

**Key constraint:** `msg.header.stamp` must equal the original `Image.header.stamp`. This ensures downstream trackers compute correct `dt` values for prediction and update steps. See [Phase 7 (scheduling) — not in scope] for how this is maintained under async/batch inference.

---

## 7. MarkerArray Publishing

### 7.1 Visualization

Publish to `/armor_detector/marker` for RViz visualization. One marker per detected armor:

- **Armor plate outline:** Four line segments connecting the 4 keypoints in order: kpt0→kpt1→kpt2→kpt3→kpt0.
- **Text label:** Above the armor showing `publish_number` + `confidence`.
- **Pose arrow:** From the plate center pointing along the normal (derived from PnP rotation).

Color coding:
- Red team armors: red marker
- Blue team armors: blue marker

The existing `armor_detector_node.cpp` already implements `publishMarkers()`. Phase 4 follows the same pattern but uses the 4 detected keypoints (not light-bar derived corners).

### 7.2 Marker coordinate frame

Markers are published in the camera optical frame (`camera_info.header.frame_id`). The pose from PnP is already in this frame.

---

## 8. Full Pipeline (Node Level)

### 8.1 `imageCallback` (Phase 4 final form)

```cpp
void ArmorDetectorNNNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  if (current_mode_ == DetectMode::DISABLED || !detector_ || !detector_->isInitialized()) {
    return;
  }

  auto t0 = std::chrono::steady_clock::now();

  // 1. cv_bridge (copy policy)
  // Jetson release path should avoid unconditional deep copy.
  cv::Mat frame;
  if (config_.runtime.copy_policy == CopyPolicy::ALWAYS_COPY ||
      (config_.runtime.copy_policy == CopyPolicy::COPY_ON_WRITE_DEBUG && debug_)) {
    frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  } else {
    frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  }

  // 2. detect (preprocess → infer → decode → NMS → label map → color filter)
  auto results = detector_->detectBatch({frame}, {img_msg->header});

  auto t1 = std::chrono::steady_clock::now();

  if (results.empty()) {
    publishEmptyArmors(img_msg->header);
    return;
  }

  auto& fd = results[0];

  // 3. PnP pose estimation
  std::vector<PoseEstimate> poses;
  if (cam_info_ && pose_estimator_adapter_) {
    poses = pose_estimator_adapter_->estimateBatch(
      fd.detections, *cam_info_);
  } else {
    // No camera info yet — publish armors without pose.
    poses.resize(fd.detections.size());
  }

  auto t2 = std::chrono::steady_clock::now();

  // 4. Build and publish Armors message
  auto armors_msg = buildArmorsMsg(
    fd.header, fd.detections, poses, cam_center_);
  armors_pub_->publish(armors_msg);

  // 5. Visualization markers
  if (marker_pub_) {
    publishMarkers(fd.detections, poses);
  }

  // 6. Debug image
  if (debug_ && result_img_pub_) {
    // If the frame is shared, clone once before drawing to avoid mutating shared memory.
    if (config_.runtime.copy_policy != CopyPolicy::ALWAYS_COPY &&
        frame.u && frame.u->refcount > 1) {
      frame = frame.clone();
    }
    publishDebugImage(frame, fd.detections, poses);
  }

  // 7. Profiler
  if (profiler_) {
    ProfilerEntry entry;
    entry.preprocess_ms  = detector_->lastProfiler().preprocess_ms;
    entry.infer_ms       = detector_->lastProfiler().infer_ms;
    entry.decode_ms      = detector_->lastProfiler().decode_ms;
    entry.nms_ms         = detector_->lastProfiler().nms_ms;
    entry.pose_ms        = std::chrono::duration<double, std::milli>(t2 - t1).count();
    entry.total_ms       = std::chrono::duration<double, std::milli>(t2 - t0).count();
    entry.raw_candidates = detector_->lastProfiler().raw_candidates;
    entry.after_conf     = detector_->lastProfiler().after_conf;
    entry.after_nms      = detector_->lastProfiler().after_nms;
    entry.published      = static_cast<int>(fd.detections.size());
    profiler_->record(entry);
  }

  // 8. Heartbeat
  heartbeat_->beat();
}
```

### 8.2 New Node Members

```cpp
std::unique_ptr<ArmorPoseEstimatorAdapter> pose_estimator_adapter_;
std::unique_ptr<DebugDrawer> debug_drawer_;
```

Initialized in the constructor after `detector_->initialize()`:

```cpp
pose_estimator_adapter_ = std::make_unique<ArmorPoseEstimatorAdapter>(config_.pose);
debug_drawer_ = std::make_unique<DebugDrawer>();
```

---

### 8.3 Jetson 内存路径约束

Phase 4 对 Jetson 的约束是：

- 非 debug 模式不做每帧 `toCvCopy`。
- debug 绘制采用 copy-on-write，仅在发布 debug 图时克隆。
- `detectBatch` 接口接收 `const cv::Mat&` 或轻量视图，避免多次容器拷贝。
- 禁止在 `imageCallback` 内进行额外 `cv::resize/cvtColor` 临时深拷贝，统一放到 `Preprocessor` 或 GPU preprocess。

## 9. Integration Testing with Legacy Tracker

### 9.1 Compatibility Checklist

| Legacy field | NN source | Verification |
|:--|:--|:--|
| `Armors.header.stamp` | `Image.header.stamp` | Identical timestamp in echoed messages |
| `Armors.armors[i].number` | `LabelMap.publish_number` | Match legacy numbering (1-5, outpost, sentry) |
| `Armors.armors[i].type` | `LabelMap.publish_type` | "small" or "large" only |
| `Armors.armors[i].pose` | PnP result | Pose in camera optical frame |
| `Armors.armors[i].distance_to_image_center` | Euclidean distance from bbox center to cam center | Same formula as legacy |
| Topic name | hardcoded `armor_detector/armors` | Downstream subscribes without remap |
| `set_mode` service | same name and semantics | RED/BLUE/DISABLED |

### 9.2 Rosbag Regression Test

1. Record a rosbag with the legacy `armor_detector` running.
2. Replay the same rosbag with `armor_detector_nn`.
3. For each frame, compare:
   - Number of detections (may differ — NN vs classical pipeline).
   - Detection centers (within reasonable tolerance).
   - Pose estimates (within reasonable tolerance, allowing for different PnP noise profiles).
   - Distance to image center ordering (used by tracker to select the "nearest" target).

The goal is not pixel-perfect identical output, but that the downstream tracker produces stable gimbal commands of comparable quality.

---

## 10. CMakeLists.txt Additions

```cmake
list(APPEND CORE_SOURCES
  src/core/armor_pose_estimator_adapter.cpp
)
```

If the BA solver is reused from `armor_detector`:
```cmake
# Option A: link against armor_detector library
find_package(armor_detector REQUIRED)
target_link_libraries(${PROJECT_NAME} armor_detector::armor_detector)

# Option B (recommended): extract BA solver into rm_utils, depend on that.
# This avoids a circular-ish dependency and keeps the solver implementation DRY.
```

---

## 11. Error Handling

| Scenario | Behavior |
|:--|:--|
| No `camera_info` received yet | Publish armors with pose fields zeroed; log warning once |
| `detection.publish_type == "invalid"` | Skip PnP, publish zero pose |
| PnP fails (all points colinear, bad keypoints) | `PoseEstimate.valid = false`, publish zero pose |
| `dist_coeffs` empty in CameraInfo | Use zero distortion (5x1 zeros) |
| BA fails to converge | Fall back to initial PnP estimate |

---

## 12. Profiler

`pose_ms` is now populated. The `Profiler` summary printed periodically (every 100 frames or on demand via debug topic) includes all Phase 2–4 metrics.

---

## 13. Reserved Interfaces

### 13.1 PnP Solver Selection

The config `pose.pnp_method` currently supports `"ippe"` and `"iterative"`. The switch is implemented via a lookup table in `solvePnP()`, making it trivial to add:
- `"epnp"` — good for non-planar configurations.
- `"sqpnp"` — faster, more robust alternative.

### 13.2 Multi-Landmark Armor (N_LANDMARKS > 4)

If a future model outputs 6 or more keypoints per armor (e.g., adding midpoint keypoints), the object points construction must adapt. The `getObjectPoints()` method is parameterized by `publish_type`; a future change to `Armor::N_LANDMARKS` can be reflected here without touching the pipeline:

```cpp
// Reserved: support for N_LANDMARKS=6
if (use_extended_landmarks) {
  return Armor::buildObjectPoints<cv::Point3f>(w, h);  // uses the N=6 version
} else {
  return Armor::buildObjectPoints<cv::Point3f>(w, h);  // uses the N=4 version
}
```

### 13.3 Alternative Object Point Sources

If armor sizes become parameterized per class (e.g., different width for sentry), extend config:

```yaml
pose:
  class_specific_sizes:     # reserved, not in Phase 4
    1: { width: 0.133, height: 0.050 }
    outpost: { width: 0.225, height: 0.050 }
    sentry: { width: 0.133, height: 0.050 }
```

`getObjectPoints()` would take the `publish_number` and look up sizes from this map.

### 13.4 Multi-Camera Support

The current design assumes one `camera_info` subscriber. For multi-camera rigs, the camera_info is keyed by `frame_id` from the image header. The adapter already takes `camera_info` as a parameter (not stored as member state), so multi-camera support requires only a lookup map in the node.

---

## 14. Test Plan

### 14.1 `test_pose_adapter.cpp`

| Test | Verification |
|:--|:--|
| Synthetic keypoints + known camera → correct translation | Forward-project object points, run PnP, verify recovery within 1mm |
| publish_type "small" → uses small object points | Verify getObjectPoints returns 0.133 × 0.050 |
| publish_type "large" → uses large object points | Verify getObjectPoints returns 0.225 × 0.050 |
| publish_type "invalid" → PoseEstimate.valid == false | No crash, zero pose |
| IPPE returns 2 solutions → picks one with positive Z | Verify tvec.z > 0 |
| BA enabled → reprojection error decreases | Compare before/after reprojection error |
| distance_to_image_center matches legacy formula | Validate against known center/bbox pairs |

### 14.2 `test_interface_compatibility.cpp` (extended)

| Test | Verification |
|:--|:--|
| Armors.header == Image.header | Compare timestamps |
| Armor.number in legacy label set | Filter out unexpected strings |
| Armor.type in {small, large} | No invalid types published |
| Armor.pose orientation is valid quaternion | Norm ≈ 1.0, w ≥ 0 |

---

## 15. Acceptance Criteria

1. PnP produces valid poses for synthetic keypoints with known camera parameters.
2. Object point selection distinguishes small vs large armor.
3. `buildArmorsMsg()` fills all fields with correct types.
4. `Armors.header.stamp` matches the input `Image.header.stamp` exactly.
5. Markers publish and display correctly in RViz.
6. Profiler records `pose_ms` for each frame.
7. Node publishes valid Armors messages with real ONNX model on sample images.
8. Downstream tracker receives and processes Armors without crashing.
9. Mode switching (RED→DISABLE→BLUE) correctly filters and resumes publishing.
10. Camera info arriving late (after first images) does not crash — armor still publishes with zero pose until camera_info arrives.
