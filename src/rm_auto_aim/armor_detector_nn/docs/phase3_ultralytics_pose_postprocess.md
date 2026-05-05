# Phase 3: Ultralytics Pose Postprocessing

## 1. Overview

**Goal:** Transform raw `output0` tensor `[1, 26, 8400]` into filtered, label-mapped `ArmorDetection` objects with correct image-space coordinates. Wire the full preprocess→infer→postprocess pipeline into the node, and publish the result image with debug overlays.

**Precondition:** Phase 2 complete (ONNX backend + preprocessor produce valid tensors).

---

## 2. Data Flow (Phase 3)

```
output0 [1, 26, 8400]
  → UltralyticsPoseDecodeStrategy::decode()
    → conf_threshold filter
    → bbox cxcywh → xyxy
    → keypoint extraction + remap
    → coordinate restore (letterbox → original image)
  → NMS (class-aware or class-agnostic)
  → LabelMap::filterByColor(DetectMode)
  → max_detections clamp
  → debug drawing (bbox, keypoints, label, confidence)
  → FrameDetections.detections populated
```

---

## 3. UltralyticsPoseDecodeStrategy

### 3.1 Header: `include/armor_detector_nn/postprocess/ultralytics_pose_decode_strategy.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_ULTRALYTICS_POSE_DECODE_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_ULTRALYTICS_POSE_DECODE_STRATEGY_HPP_

#include "armor_detector_nn/postprocess/decode_strategy.hpp"
#include <vector>

namespace fyt::auto_aim {

class UltralyticsPoseDecodeStrategy : public IDecodeStrategy {
public:
  explicit UltralyticsPoseDecodeStrategy();
  ~UltralyticsPoseDecodeStrategy() override = default;

  std::vector<RawDetection> decode(
    const std::vector<TensorOutput>& outputs,
    const ImageMeta& image_meta,
    const PostprocessConfig& config) override;

private:
  // Decode one candidate at index i from the flat [C, N] layout.
  // Returns true if the candidate passes conf_threshold.
  bool decodeCandidate(
    const float* data,
    int index,
    int num_candidates,
    const PostprocessConfig& config,
    RawDetection& detection);

  // Convert bbox from [cx, cy, w, h] model-space to [x, y, w, h] image-space.
  void restoreBbox(
    RawDetection& detection,
    const ImageMeta& meta,
    const PostprocessConfig& config);

  // Restore keypoints from model-space to original image coordinates.
  // Applies keypoint_remap to reorder model kpts into canonical order.
  void restoreKeypoints(
    RawDetection& detection,
    const ImageMeta& meta,
    const PostprocessConfig& config);
};

} // namespace fyt::auto_aim

#endif
```

### 3.2 Tensor Layout

The model output has shape `[1, 26, 8400]` with `output_layout: "channels_first"`. The 26 channels are:

```
Index  Content
0..3   bbox: cx, cy, w, h   (model-space, before sigmoid/exp decode if the ONNX
                               export already applied the head activations)
4..17  class scores: 14 classes (B1..BS, R1..RS)
18..25 keypoints: kpt0_x, kpt0_y, kpt1_x, kpt1_y, kpt2_x, kpt2_y, kpt3_x, kpt3_y
```

**Important:** The ONNX export from Ultralytics typically applies the final head activations (sigmoid for bbox center + keypoints, exp for bbox wh). We must verify this against the actual model output. The OpenVINO IR metadata shows `<model_type value="YOLO"/>`, and the NNCF quantization ignored scope for `.*model.32/.*/Add, Sub, Mul, Div` and the DFL layer — this suggests the export includes the full post-head processing. We include a configurable flag to handle both cases:

```yaml
postprocess:
  head_already_applied: true   # true if ONNX export includes detect head (default for ultralytics export)
```

### 3.3 decode() Algorithm

```
Input:  output_data = outputs[0].host_data  // [1*26*8400] = 218400 floats
        image_meta (scale, pad from letterbox)
        config (offsets, thresholds)

Algorithm:

1. Interpret data as C=26 rows, N=8400 columns.
   Each column i is one candidate.

2. For i in 0..8399:
   a. Read class scores from col i, rows [class_offset ... class_offset+num_classes-1].
   b. Find max class score and its index.
   c. If max_score < conf_threshold → skip.
   d. Read bbox from col i, rows [bbox_offset ... bbox_offset+3]:
        cx, cy, w, h
   e. Read keypoints from col i, rows [keypoint_offset ... keypoint_offset + num_keypoints*keypoint_dims - 1]:
        kp[0..3] = (x, y)
   f. Apply keypoint_remap to reorder into canonical order.
   g. Restore coordinates from model-space to original image-space.
   h. Append RawDetection { class_id, max_score, 1.0, max_score, bbox, keypoints }.

3. Apply NMS to the collected detections.

4. Clamp to max_detections (keep highest confidence).

5. Return the result.
```

### 3.4 Coordinate Restoration

Model bbox is in the letterbox-padded coordinate space (0..640 for this model). We need to restore to the original image:

```
For bbox (cx, cy, w, h):
    cx_orig = (cx - pad_left) / scale_x
    cy_orig = (cy - pad_top)  / scale_y
    w_orig  = w / scale_x
    h_orig  = h / scale_y

    bbox.x = cx_orig - w_orig/2
    bbox.y = cy_orig - h_orig/2
    bbox.width  = w_orig
    bbox.height = h_orig

For each keypoint (kx, ky):
    kx_orig = (kx - pad_left) / scale_x
    ky_orig = (ky - pad_top)  / scale_y
```

The base class `IDecodeStrategy` receives `ImageMeta` which carries `scale_x`, `scale_y`, `pad_left`, `pad_top` from the preprocessor. This decouples the decode strategy from any knowledge of how the model input was prepared (letterbox, stretch, center-crop, etc.).

### 3.5 Keypoint Remapping

The canonical keypoint order is:
```
kpt0: left_bottom
kpt1: left_top
kpt2: right_top
kpt3: right_bottom
```

If the model was trained with a different order, the config specifies a remap:

```yaml
postprocess:
  keypoint_remap: [0, 1, 2, 3]  # identity: model kpt0→canonical kpt0, etc.
```

The remap is applied as:
```cpp
for (int j = 0; j < num_keypoints; ++j) {
    int src_idx = config.keypoint_remap[j];
    canonical[j] = model_kpts[src_idx];
}
```

If `keypoint_remap` is `[3, 0, 1, 2]`, then:
- canonical kpt0 = model kpt3
- canonical kpt1 = model kpt0
- canonical kpt2 = model kpt1
- canonical kpt3 = model kpt2

The config should validate that `keypoint_remap` is a permutation of `[0..num_keypoints-1]`.

**Reserved: auto-reorder fallback.** If the model changes and the remap is unknown, a geometric heuristic can reorder: sort by y-coordinate to split top/bottom, then by x to split left/right. This is an optional post-decode validation step gated by a config flag (not implemented in Phase 3):

```yaml
postprocess:
  keypoint_auto_reorder: false   # reserved for future
```

### 3.6 Box Format Handling

The config `box_format` supports `"cxcywh"` (current model) and reserves `"xyxy"`, `"xywh"` for future models. The decode strategy selects the conversion based on this config value rather than hardcoding.

---

## 4. NMS (Non-Maximum Suppression)

### 4.1 Header: `include/armor_detector_nn/geometry/nms.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_NMS_HPP_
#define ARMOR_DETECTOR_NN_NMS_HPP_

#include "armor_detector_nn/core/detection_types.hpp"
#include <vector>

namespace fyt::auto_aim {

// Class-aware NMS: suppress within each class independently.
std::vector<RawDetection> classAwareNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold);

// Class-agnostic NMS: suppress across all classes by confidence only.
std::vector<RawDetection> classAgnosticNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold);

// Convenience dispatcher based on config flag.
inline std::vector<RawDetection> applyNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold,
  bool class_agnostic)
{
  return class_agnostic
    ? classAgnosticNMS(detections, iou_threshold)
    : classAwareNMS(detections, iou_threshold);
}

} // namespace fyt::auto_aim

#endif
```

### 4.2 Algorithm (class-aware)

Standard greedy NMS:

```
1. Sort detections by confidence DESC.
2. For each detection d in sorted order:
   a. If d is suppressed, skip.
   b. Add d to kept list.
   c. For each remaining detection r in same class:
        if IoU(d.bbox, r.bbox) > iou_threshold:
          mark r as suppressed.
3. Return kept list.
```

IoU is computed on the bbox rectangles (axis-aligned, `cv::Rect2f`).

### 4.3 Design Considerations

| Consideration | Decision |
|:--|:--|
| Per-class vs agnostic | Default: class-aware NMS. Can be switched by config if same-location class confusion is observed. |
| Confidence tie-breaking | Higher confidence wins. If equal, keep the one with smaller class_id (deterministic). |
| Keypoint-aware NMS | Reserved for future. If bbox IoU is high but keypoints differ significantly, the detections may be for different armors at similar positions. Not needed for Phase 3. |

---

## 5. LabelMap Integration

### 5.1 Mapping Pipeline

```
RawDetection.class_id → LabelMap::lookup(class_id) → LabelEntry
  → ArmorDetection {
      model_label:    entry.model_label    // "B1", "R2", ...
      publish_number: entry.publish_number // "1", "outpost", ...
      publish_type:   entry.publish_type   // "small", "large"
      color:          entry.color           // RED or BLUE
      confidence:     raw.confidence
      bbox:           raw.bbox
      keypoints:      raw.keypoints
      center:         (bbox.x + bbox.w/2, bbox.y + bbox.h/2)
    }
```

If `lookup()` returns `nullptr` (e.g., unknown class_id from a newer model, or an ignored label), the detection is dropped.

### 5.2 Color/DetectMode Filtering

```cpp
std::vector<ArmorDetection> LabelMap::filterByColor(
  const std::vector<ArmorDetection>& detections,
  EnemyColor target_color)
{
  std::vector<ArmorDetection> result;
  result.reserve(detections.size());
  for (const auto& d : detections) {
    if (d.color == target_color) {
      result.push_back(d);
    }
  }
  return result;
}
```

Called in the node's pipeline:
```cpp
auto mapped = label_map_->map(raw_detections);
auto filtered = label_map_->filterByColor(mapped,
  current_mode_ == DetectMode::RED ? EnemyColor::RED : EnemyColor::BLUE);
```

### 5.3 Validation at Load Time

Before any inference, `LabelMap::validate()` checks:
1. All model class IDs from 0 to max_class_id have an entry.
2. No duplicate class IDs.
3. All `publish_number` values are in `{1,2,3,4,5,outpost,sentry,base,negative}`.
4. All `publish_type` values are in `{small,large,invalid}`.
5. All `color` values are in `{red,blue,unknown}`.
6. All entries in `ignore_labels` exist in the map.

On validation failure, the label map reports the specific error(s) and the node refuses to start detection (logs loudly and returns empty results).

---

## 6. Pipeline Integration

### 6.1 `ArmorDetectorNN` Additions

New members:
```cpp
std::unique_ptr<IDecodeStrategy> decode_strategy_;
std::unique_ptr<LabelMap> label_map_;
```

New in `initialize()`:
```cpp
// After backend + preprocessor setup:
decode_strategy_ = DecodeStrategyFactory::create(config_.postprocess);
if (!decode_strategy_) {
  FYT_ERROR("armor_detector_nn", "Unknown decode strategy: %s",
            config_.postprocess.strategy.c_str());
  return false;
}

label_map_ = std::make_unique<LabelMap>();
label_map_->load(resolveModelPath(config_.label_map.path));
std::string err;
if (!label_map_->validate(err)) {
  FYT_ERROR("armor_detector_nn", "Label map validation failed: %s", err.c_str());
  return false;
}
```

Updated `detectBatch()`:
```cpp
std::vector<FrameDetections> ArmorDetectorNN::detectBatch(
    const std::vector<cv::Mat>& images,
    const std::vector<std_msgs::msg::Header>& headers)
{
  std::vector<FrameDetections> results;
  for (size_t i = 0; i < images.size(); ++i) {
    FrameDetections fd;
    fd.header = headers[i];

    ProfilerEntry entry;  // if profiling

    // preprocess
    auto pre = preprocessor_->process(images[i]);
    entry.preprocess_ms = ...;

    // infer
    auto outputs = backend_->infer(pre.tensor);
    entry.infer_ms = ...;

    // decode
    auto raw_detections = decode_strategy_->decode(
      outputs, pre.image_meta, config_.postprocess);
    entry.raw_candidates = static_cast<int>(raw_detections.size());

    // NMS
    auto nms_result = applyNMS(
      raw_detections,
      config_.postprocess.nms_threshold,
      config_.postprocess.class_agnostic_nms);
    entry.after_nms = static_cast<int>(nms_result.size());

    // label map
    auto mapped = label_map_->map(nms_result);
    auto filtered = (config_.runtime.color_filter_source == ColorFilterSource::MODEL)
      ? label_map_->filterByColor(mapped,
          current_mode_ == DetectMode::RED ? EnemyColor::RED : EnemyColor::BLUE)
      : mapped;

    // clamp
    if (filtered.size() > config_.postprocess.max_detections) {
      std::partial_sort(filtered.begin(),
                        filtered.begin() + config_.postprocess.max_detections,
                        filtered.end(),
                        [](const auto& a, const auto& b) {
                          return a.confidence > b.confidence;
                        });
      filtered.resize(config_.postprocess.max_detections);
    }

    fd.detections = std::move(filtered);
    entry.published = static_cast<int>(fd.detections.size());
    results.push_back(std::move(fd));
  }
  return results;
}
```

**Design note on `current_mode_`:** The filter-by-color step needs to know whether we're in RED/BLUE mode. But `ArmorDetectorNN` (core pipeline) should ideally not know about ROS-level `DetectMode`. In Phase 3, we pass it as a parameter to `detectBatch()` or as a method `setColorFilter(EnemyColor)`. The exact approach:

Add to `ArmorDetectorNN`:
```cpp
void setTargetColor(EnemyColor color) { target_color_ = color; }
```
The node calls this whenever `set_mode` changes.

---

## 7. Debug Drawing

### 7.1 Header: `include/armor_detector_nn/debug/debug_drawer.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_
#define ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_

#include "armor_detector_nn/core/detection_types.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace fyt::auto_aim {

class DebugDrawer {
public:
  DebugDrawer();

  // Draw bbox, keypoints, label, confidence on the image.
  // Modifies the image in-place.
  void drawDetections(
    cv::Mat& image,
    const std::vector<ArmorDetection>& detections,
    bool show_confidence = true);

  // Draw profiler stats overlay (FPS, latency, backend info).
  void drawProfiler(
    cv::Mat& image,
    double fps,
    double latency_ms,
    const std::string& backend_name,
    const std::string& precision);

  // Set color for a specific class label.
  void setClassColor(const std::string& label, const cv::Scalar& color);

private:
  // Predefined color palette for model labels.
  std::unordered_map<std::string, cv::Scalar> class_colors_;

  // Generate a stable color from a label string.
  static cv::Scalar generateColor(const std::string& label);
};

} // namespace fyt::auto_aim

#endif
```

### 7.2 Drawing Conventions

Per detection:
- **Bbox:** Rectangle in the class color, thickness 2.
- **Keypoints:** Filled circles, radius 4, with a small index number drawn next to each point. Color-coding:
  - kpt0 (left_bottom): red
  - kpt1 (left_top): green
  - kpt2 (right_top): blue
  - kpt3 (right_bottom): yellow
- **Label text:** `"{publish_number} {confidence:.2f}"` displayed above the bbox, with a filled background rectangle for readability.

Corner overlay (top-left of image):
- Backend: `"ONNXRuntime"`, Precision: `"FP32"`, FPS: `"120.5"`, Latency: `"8.3ms"`.
- Detection count: `"Armors: 3"`.

The `result_img` topic publishes the drawn image as `sensor_msgs::Image` (bgr8 encoding) only when `debug: true`.

---

## 8. Node Publishing (Phase 3 update)

In `imageCallback`, after detection:

```cpp
// Publish armors (still empty list for now — Phase 4 adds Armor msg fields)
rm_interfaces::msg::Armors armors_msg;
armors_msg.header = img_msg->header;
// number, type, pose are empty — Phase 4 fills them.
armors_pub_->publish(armors_msg);

// Debug image
if (debug_ && result_img_pub_) {
  cv::Mat debug_img = frame.clone();
  debug_drawer_->drawDetections(debug_img, results[0].detections);
  debug_drawer_->drawProfiler(debug_img, fps, latency,
    detector_->backendInfo().backend_name,
    detector_->backendInfo().precision);
  auto msg = cv_bridge::CvImage(img_msg->header, "bgr8", debug_img).toImageMsg();
  result_img_pub_->publish(*msg);
}
```

---

## 9. `DecodeStrategyFactory`

### 9.1 `include/armor_detector_nn/postprocess/decode_strategy_factory.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_

#include "armor_detector_nn/postprocess/decode_strategy.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include <memory>
#include <string>

namespace fyt::auto_aim {

class DecodeStrategyFactory {
public:
  // Create a strategy by name. Returns nullptr if the name is unknown.
  static std::unique_ptr<IDecodeStrategy> create(const PostprocessConfig& config);

  // Register a custom strategy. For plugin architectures or testing.
  // Reserved for future use.
  using FactoryFn = std::unique_ptr<IDecodeStrategy>(*)();
  static void registerStrategy(const std::string& name, FactoryFn fn);

private:
  static std::unordered_map<std::string, FactoryFn>& registry();
};

} // namespace fyt::auto_aim

#endif
```

### 9.2 Implementation

```cpp
std::unique_ptr<IDecodeStrategy> DecodeStrategyFactory::create(
    const PostprocessConfig& config)
{
  auto& reg = registry();
  auto it = reg.find(config.strategy);
  if (it != reg.end()) {
    return it->second();
  }

  // Built-in aliases
  if (config.strategy == "ultralytics_pose") {
    return std::make_unique<UltralyticsPoseDecodeStrategy>();
  }

  return nullptr;
}
```

The registry map allows future strategies (YOLO detect, two-stage, custom) to be registered from other compilation units or plugins. This is an extension point rather than a Phase 3 requirement.

---

## 10. CMakeLists.txt Additions

```cmake
list(APPEND POSTPROCESS_SOURCES
  src/postprocess/ultralytics_pose_decode_strategy.cpp
  src/postprocess/decode_strategy_factory.cpp
)

list(APPEND GEOMETRY_SOURCES
  src/geometry/nms.cpp
)

list(APPEND CORE_SOURCES
  src/core/label_map.cpp
)

list(APPEND DEBUG_SOURCES
  src/debug/debug_drawer.cpp
)
```

---

## 11. Unit Tests

### 11.1 `test_ultralytics_pose_decode.cpp`

**Test: single candidate, high confidence**
- Construct a synthetic output tensor: `[1, 26, 1]`.
- Fill class scores: `class[0] = 0.9`, rest = 0.0.
- Fill bbox: `cx=320, cy=320, w=80, h=40`.
- Fill keypoints: `(100,200), (100,180), (150,180), (150,200)`.
- Set `image_meta` with no padding (scale=1.0, pad=0).
- Verify: 1 detection, class_id=0, confidence=0.9, bbox correct, 4 keypoints correct.

**Test: below confidence threshold**
- Same as above, but `class[0] = 0.2` and `conf_threshold = 0.35`.
- Verify: 0 detections.

**Test: keypoint_remap**
- Remap `[3, 0, 1, 2]`.
- Model outputs kpt0 → canonical[1], kpt1 → canonical[2], kpt2 → canonical[3], kpt3 → canonical[0].
- Verify canonical keypoint positions match the remapped order.

**Test: coordinate restoration with letterbox**
- Original image: `1280x1024`.
- Input size: `640x640`.
- Letterbox: scale=0.5, pad_left=0, pad_top=88.
- Place a detection at model coords `cx=320, cy=320`.
- Verify restored coords are `cx=640, cy=464` in original space.

**Test: max_detections clamp**
- Generate 50 detections above threshold.
- `max_detections = 32`.
- Verify only the top-32 (by confidence) are returned.

### 11.2 `test_nms.cpp`

**Test: class-aware NMS — same class, high IoU**
- Two detections, class=0, boxes overlap 90%, conf=0.9 and 0.8.
- Verify only the 0.9-confidence detection survives.

**Test: class-aware NMS — different class, high IoU**
- Two detections, class=0 and class=1, boxes overlap 90%.
- Verify both survive (class-aware).

**Test: class-agnostic NMS — different class, high IoU**
- Two detections, class=0 and class=1, boxes overlap 90%.
- Verify only the higher-confidence one survives.

**Test: low IoU**
- Two detections, same class, boxes overlap 20%.
- Verify both survive.

### 11.3 `test_label_map.cpp`

See Phase 1 test plan — already implemented.

---

## 12. Profiler Additions

`ProfilerEntry` slots populated in this phase:
- `decode_ms`: time from `decode_strategy_->decode()` start to finish.
- `nms_ms`: time for NMS.
- `raw_candidates`: number of candidates before conf filtering (always ≤ 8400 for this model).
- `after_conf`: number after confidence threshold.
- `after_nms`: number after NMS.
- `published`: number after label-map + color filter + max_detections clamp.

---

## 13. Reserved Interfaces

### 13.1 Future Decode Strategies

The factory's `registerStrategy()` method, plus the full virtual `IDecodeStrategy` interface, enable:

| Strategy | When Needed |
|:--|:--|
| `YoloDetectDecodeStrategy` | Model without keypoint head, bbox + class only |
| `AnchorFreePoseDecodeStrategy` | Center/scale-based keypoint representation |
| `TwoStageArmorDecodeStrategy` | Stage 1 armor detection, stage 2 number classification |
| `CustomTensorDecodeStrategy` | User-defined output layout via config |

All strategies receive `ImageMeta` and `PostprocessConfig`, so adding a new strategy does not require changes to the pipeline or node.

### 13.2 Keypoint Auto-Reorder Validation

Even when `keypoint_auto_reorder: false`, the decode strategy can emit a warning if keypoints appear geometrically unreasonable (e.g., left/right confused, or concave quadrilateral). This is a diagnostic hook:

```cpp
// Reserved in IDecodeStrategy
virtual bool validateKeypoints(const RawDetection& detection,
                                std::string& warning);
```

Phase 3 implements a no-op that always returns true. Future models with different annotation conventions can override.

### 13.3 Multi-Output Models

The `decode()` method receives `std::vector<TensorOutput>` — multiple output tensors. The current model has only `output0`, but a future two-stage model might output `["detections", "classifications"]`. The strategy dispatches by output name.

---

## 14. Acceptance Criteria

1. Synthetic tensor with known bbox/keypoint values decodes correctly.
2. Confidence threshold filters candidates correctly.
3. Keypoint remap reorders as specified in config.
4. Coordinate restoration accounts for letterbox scale and padding.
5. Class-aware NMS suppresses intra-class overlaps; class-agnostic NMS suppresses inter-class overlaps.
6. LabelMap maps class IDs to correct `publish_number`, `publish_type`, `color`.
7. `set_mode RED` filters out blue detections; `set_mode BLUE` filters out red.
8. `max_detections` clamps output count.
9. Debug image shows bbox, keypoints (color-coded), label, confidence.
10. Profiler records decode and NMS timing.
11. Pipeline runs end-to-end on real ONNX model: `image_raw → ArmorDetection list`.
12. No crash when model outputs NaN or Inf (should be filtered by confidence check).
