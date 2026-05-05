# Phase 2: ONNX Runtime CPU Inference

## 1. Overview

**Goal:** Implement the `IInferenceBackend` interface with ONNX Runtime (ORT), add the `Preprocessor` to convert ROS images into model-ready tensors, and wire them into the node pipeline so that `imageCallback` produces raw `output0` tensors. No postprocessing, no PnP — the output of this phase is verifiable tensor data logged via debug/profiler channels.

This phase is a correctness baseline, not the Jetson fastest path. For Jetson deployment, this phase must still reserve interfaces for device-buffer inference to avoid repeated host copies in later TensorRT phases.

**Precondition:** Phase 1 complete (package skeleton, node lifecycle, parameter system).

**Model used:** `model/demo01/best.onnx`

```
Input:  images  [1, 3, 640, 640]  float32
Output: output0 [1, 26, 8400]     float32
```

---

## 2. Architecture Diagram (Phase 2)

```
sensor_msgs::Image
  → cv_bridge → cv::Mat (BGR, original resolution)
  → Preprocessor::process(cv::Mat) → std::vector<float> [1*3*640*640]
  → IInferenceBackend::infer(TensorInput) → std::vector<TensorOutput>
  → log output0.shape + first-N-values (debug)
  → (no publish yet — Phase 3 adds postprocess)
```

---

## 3. OnnxRuntimeBackend

### 3.1 Header: `include/armor_detector_nn/backend/onnxruntime_backend.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_ONNXRUNTIME_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_ONNXRUNTIME_BACKEND_HPP_

#include "armor_detector_nn/backend/inference_backend.hpp"
#include <memory>
#include <vector>
#include <string>

// Forward-declare to avoid leaking ORT headers into all consumers.
namespace Ort {
  class Env;
  class Session;
  class MemoryInfo;
  struct Value;
}

namespace fyt::auto_aim {

class OnnxRuntimeBackend : public IInferenceBackend {
public:
  OnnxRuntimeBackend();
  ~OnnxRuntimeBackend() override;

  void load(const BackendConfig& config) override;
  std::vector<TensorOutput> infer(const TensorInput& input) override;
  void warmup(int iterations) override;
  BackendInfo info() const override;

private:
  // Validate that the loaded ONNX model's I/O signatures match the config.
  void validateModelIO(const BackendConfig& config);

  // Convert ORT tensor back to our platform-agnostic TensorOutput.
  TensorOutput extractOutput(const char* name, const std::vector<int64_t>& shape);

  // Map ORT execution provider string from config.
  static std::string toProviderString(const std::string& device);

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;

  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char*> input_name_ptrs_;
  std::vector<const char*> output_name_ptrs_;

  std::vector<int64_t> input_shape_;
  std::vector<std::vector<int64_t>> output_shapes_;

  BackendInfo info_;
  bool loaded_{false};
};

} // namespace fyt::auto_aim

#endif
```

Design notes:
- ORT headers are NOT included in this public header. They are only included in the `.cpp` file. This prevents every consumer of `IInferenceBackend` from needing to have ORT development headers installed.
- PIMPL idiom via `std::unique_ptr` to opaque ORT types keeps the ABI stable across ORT version changes.

### 3.2 Implementation Outline

**`load(const BackendConfig& config)`:**

1. Initialize `Ort::Env` with `ORT_LOGGING_LEVEL_WARNING` and thread count from `config.num_threads`.
2. Create `Ort::SessionOptions`:
   - Set `graph_optimization_level` to `ORT_ENABLE_ALL`.
   - If `config.device == "cpu"`, no execution-provider registration needed (CPU is default).
   - If `config.device == "cuda"`, attempt `OrtCUDAProviderOptions` — wrap in try/catch and fall back to CPU with a warning.
   - Set `intra_op_num_threads` and `inter_op_num_threads` from config.
3. Load the ONNX model from `config.model_path`:
   - Resolve `package://` URIs by calling `ament_index_cpp::get_package_share_directory()`.
   - Handle the case where the path is a bare filesystem path directly.
4. Query input/output names and shapes from the session metadata:
   - Input count must be exactly 1 (current model).
   - Output count must match `config.output_names.size()`.
   - Input shape must be `[1, 3, 640, 640]` (static, no dynamic dims in this model).
   - Output shapes are captured for later allocation.
5. Call `validateModelIO(config)` to cross-check against config.
6. Populate `info_` (BackendInfo) with `backend_name = "onnxruntime"`, precision from config, `min_batch_size = 1`, `max_batch_size = 1`, `dynamic_batch = false`.
7. Set `loaded_ = true`.

**Warmup (Phase 2 note):** The current ONNX metadata says `dynamic: false` and `batch: 1`. The model does not have dynamic batch dimensions, so the backend reports `max_batch_size = 1`. This will constrain the scheduler in Phase 7 — see section 11 for the interfaces that let the scheduler query this.

**`warmup(int iterations)`:**
- Create a dummy input tensor filled with zeros (shape `[1, 3, 640, 640]`).
- Run inference `iterations` times.
- Discard outputs.
- This ensures GPU kernel compilation and memory allocation happen before the first real frame.

**`infer(const TensorInput& input)`:**
1. Validate `input.info.shape` matches `input_shape_`.
2. Create ORT input tensor from `input.host_data`:
   ```cpp
   auto input_tensor = Ort::Value::CreateTensor<float>(
     *memory_info_,
     const_cast<float*>(input.host_data.data()),
     input.host_data.size(),
     input_shape_.data(),
     input_shape_.size()
   );
   ```
3. Run session:
   ```cpp
   auto outputs = session_->Run(
     Ort::RunOptions{nullptr},
     input_name_ptrs_.data(), &input_tensor, 1,
     output_name_ptrs_.data(), output_name_ptrs_.size()
   );
   ```
4. For each output, extract shape info and copy float data into a `TensorOutput`.

**Error handling:** If ORT throws `Ort::Exception`, catch it and rethrow as a `std::runtime_error` with the model path and error message concatenated. This keeps ORT types out of calling code.

### 3.3 Thread Safety

Phase 2 adopts an explicit serialization policy for `infer()` (single in-flight inference) to keep behavior deterministic across platforms and ORT builds:
```cpp
std::mutex infer_mutex_;
```

In Phase 7 (async scheduling), if true concurrent inference is needed, use a session pool with clear ownership per worker rather than sharing one request path implicitly.

### 3.4 Execution Provider Selection

| `config.device` | ORT Provider | Notes |
|:--|:--|:--|
| `"cpu"` | default CPU | Works everywhere. `num_threads` controls parallelism. |
| `"cuda"` | CUDAExecutionProvider | Requires `onnxruntime-gpu`. Phase 2 includes the try/catch path but the default config uses `cpu`. |

---

## 4. Preprocessor

### 4.1 Header: `include/armor_detector_nn/core/preprocessor.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_PREPROCESSOR_HPP_
#define ARMOR_DETECTOR_NN_PREPROCESSOR_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include <opencv2/core.hpp>

namespace fyt::auto_aim {

// Result of preprocessing — both the tensor data and the metadata needed
// later to restore coordinates to the original image space.
struct PreprocessResult {
  TensorInput tensor;
  ImageMeta image_meta;  // for postprocess coordinate restoration
};

class Preprocessor {
public:
  explicit Preprocessor(const PreprocessConfig& config);

  // Convert a single BGR image (from cv_bridge) to a model-ready tensor.
  PreprocessResult process(const cv::Mat& bgr_image);

  // Batch version — stacks image tensors along batch dim. Reserved for Phase 7.
  // In Phase 2, batch_dim is always 1.
  PreprocessResult processBatch(const std::vector<cv::Mat>& bgr_images);

  const PreprocessConfig& config() const { return config_; }

private:
  // Letterbox resize: fit image into (W,H) preserving aspect ratio, pad to fill.
  // Returns the resized + padded image, and fills scale/pad values in meta.
  cv::Mat letterbox(const cv::Mat& src, ImageMeta& meta);

  // Convert BGR to RGB if config says "rgb".
  cv::Mat convertColor(const cv::Mat& src);

  // Convert HWC to CHW and normalize.
  // Output layout: [C, H, W] with values in [0,1] or mean/std normalized.
  void toTensor(const cv::Mat& src, std::vector<float>& data);

  PreprocessConfig config_;
  cv::Size input_size_;
};

} // namespace fyt::auto_aim

#endif
```

### 4.2 Letterbox Algorithm

Standard YOLO letterbox:
```
1. scale = min(input_width / img_width, input_height / img_height)
2. new_w = img_width  * scale
3. new_h = img_height * scale
4. resize image to (new_w, new_h) with INTER_LINEAR
5. paste into a (input_width, input_height) canvas filled with pad_value
6. record: meta.scale_x = scale, meta.scale_y = scale
7. record: meta.pad_left = (input_width  - new_w) / 2
8. record: meta.pad_top  = (input_height - new_h) / 2
```

### 4.3 Color Conversion

```
if config.input_color == "rgb":
    cv::cvtColor(bgr, result, cv::COLOR_BGR2RGB)
else:
    result = bgr
```

Note: The current ONNX model's OpenVINO IR contains `<reverse_input_channels value="YES"/>`, which means the model itself expects RGB input. We follow `config.input_color` explicitly; the config default is `"rgb"`. The OpenVINO-specific preprocessing (Phase 6) can also use OV's built-in `preprocess_info` instead of manual conversion.

### 4.4 Normalization

```
For each pixel value v at (c, h, w):
    v = v / 255.0           // if normalize == true
    v = (v - mean[c]) / std[c]
```

With the default config `mean: [0,0,0], std: [255,255,255]`, this is equivalent to a simple `/255.0` normalization.

### 4.5 Batch Preprocessing (Phase 2 stub)

```cpp
PreprocessResult Preprocessor::processBatch(const std::vector<cv::Mat>& bgr_images)
{
  if (bgr_images.empty()) {
    throw std::invalid_argument("processBatch: empty image vector");
  }
  // Phase 2: only batch=1 is supported.
  if (bgr_images.size() > 1) {
    throw std::runtime_error("processBatch: batch > 1 not supported in Phase 2");
  }
  return process(bgr_images[0]);
}
```

The stub throws for `size > 1` rather than silently truncating. This makes it a loud failure when later phases attempt batch inference without updating the preprocessor.

---

## 5. ArmorDetectorNN (Core Pipeline, Phase 2)

### 5.1 Header: `include/armor_detector_nn/core/armor_detector_nn.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/preprocessor.hpp"
#include "armor_detector_nn/backend/inference_backend.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <vector>

namespace fyt::auto_aim {

class ArmorDetectorNN {
public:
  explicit ArmorDetectorNN(const DetectorConfig& config);
  ~ArmorDetectorNN();

  // Initialize all sub-components. Must be called before detectBatch().
  // Returns false on failure (missing model, backend unavailable, etc.)
  bool initialize();

  // Synchronous batch detection. Phase 2: batch=1 only.
  std::vector<FrameDetections> detectBatch(
    const std::vector<cv::Mat>& images,
    const std::vector<std_msgs::msg::Header>& headers);

  const DetectorConfig& config() const { return config_; }
  BackendInfo backendInfo() const;

  // Reserved accessors for profiler (Phase 4+).
  const Preprocessor* preprocessor() const { return preprocessor_.get(); }

private:
  DetectorConfig config_;
  std::unique_ptr<Preprocessor> preprocessor_;
  std::unique_ptr<IInferenceBackend> backend_;
  bool initialized_{false};
};

} // namespace fyt::auto_aim

#endif
```

### 5.2 Implementation Outline

**`initialize()`:**
1. Create backend via `InferenceBackendFactory::create(config_.backend)`.
   - If `nullptr`, log error and return false.
2. Call `backend_->load(config_.backend)`.
   - On exception, log error and return false.
3. Create `Preprocessor` with `config_.preprocess`.
4. Call `backend_->warmup(config_.backend.warmup_iterations)`.
5. Set `initialized_ = true`.

**`detectBatch()`:**
1. `Preprocessor::processBatch(images)` → `PreprocessResult`.
2. `backend_->infer(preprocess_result.tensor)` → `std::vector<TensorOutput>`.
3. Construct `FrameDetections` with the raw image header. The `detections` vector is empty (no postprocess yet).
4. The `TensorOutput` is logged at debug level (shape, first few float values).
5. Return the single-element vector.

If postprocess is not yet configured (Phase 2), the raw TensorOutput can optionally be attached to `FrameDetections` via a debug extension field. We add a temporary member:

```cpp
// Phase 2 debug: raw model output for verification.
// Removed in Phase 3 when postprocess is added.
std::vector<TensorOutput> debug_raw_outputs;
```

### 5.3 Jetson-oriented interface reservation

Although Phase 2 uses `std::vector<float>` host tensors, reserve the following extension points now:

- `TensorInput` can carry an optional device-buffer handle (opaque pointer + bytes).
- `IInferenceBackend::infer()` may consume host or device input depending on backend capability.
- `BackendInfo` should expose whether zero-copy/device-input is supported.

Suggested additional fields (to be introduced in TensorRT phase):

```cpp
struct BackendInfo {
  ...
  bool supports_device_input{false};
  bool supports_cuda_graph{false};
};
```

This avoids later API churn when moving from ORT CPU baseline to Jetson TensorRT optimized path.

---

## 6. CMakeLists.txt Additions

Add to the existing Phase 1 `CMakeLists.txt`:

```cmake
# --- ONNX Runtime ---
if(BUILD_ONNX_BACKEND)
  find_package(onnxruntime REQUIRED)

  list(APPEND BACKEND_SOURCES
    src/backend/onnxruntime_backend.cpp
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC
    onnxruntime::onnxruntime
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    ARMOR_DETECTOR_NN_HAS_ONNX=1
  )
endif()

# --- core sources (preprocessor is always built) ---
list(APPEND CORE_SOURCES
  src/core/preprocessor.cpp
  src/core/armor_detector_nn.cpp
)

# --- backend factory ---
list(APPEND BACKEND_SOURCES
  src/backend/inference_backend_factory.cpp
)
```

The `ARMOR_DETECTOR_NN_HAS_ONNX` preprocessor define allows the factory to conditionally compile the ONNX branch.

---

## 7. `package://` URI Resolution

Model paths in config may use the `package://` scheme. We add a utility function:

```cpp
// src/core/path_resolver.cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <regex>

namespace fyt::auto_aim {

std::string resolveModelPath(const std::string& raw_path) {
  static const std::regex pkg_regex("^package://([^/]+)/(.*)$");
  std::smatch match;
  if (std::regex_match(raw_path, match, pkg_regex)) {
    std::string pkg = match[1].str();
    std::string rel = match[2].str();
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  return raw_path;
}

} // namespace fyt::auto_aim
```

This belongs in a `core/path_resolver.hpp` header so all backends can use it.

---

## 8. Profiler (Phase 2 stub)

`include/armor_detector_nn/debug/profiler.hpp`:

```cpp
#ifndef ARMOR_DETECTOR_NN_PROFILER_HPP_
#define ARMOR_DETECTOR_NN_PROFILER_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

namespace fyt::auto_aim {

struct ProfilerEntry {
  double preprocess_ms{0.0};
  double infer_ms{0.0};
  double decode_ms{0.0};    // Phase 3+
  double nms_ms{0.0};       // Phase 3+
  double pose_ms{0.0};      // Phase 4+
  double total_ms{0.0};

  int raw_candidates{0};
  int after_conf{0};
  int after_nms{0};
  int published{0};

  // Reserved for Phase 7+
  int batch_size{1};
  double batch_wait_ms{0.0};
  double observation_age_ms{0.0};
  int queue_size{0};
  int dropped_old{0};
  int dropped_stale{0};
};

// RAII scoped timer for a single profiling slot.
class ScopedTimer {
public:
  explicit ScopedTimer(double& target_ms);
  ~ScopedTimer();
private:
  double& target_;
  std::chrono::steady_clock::time_point start_;
};

// Rolling-window profiler that records the last N frames.
class Profiler {
public:
  explicit Profiler(size_t window_size = 100);

  void record(const ProfilerEntry& entry);

  // Averages over window.
  double avgPreprocessMs() const;
  double avgInferMs() const;
  double avgTotalMs() const;
  double avgFPS() const;

  // Latest entry.
  const ProfilerEntry& latest() const;

  // Reset all accumulated data.
  void reset();

private:
  size_t window_size_;
  std::vector<ProfilerEntry> history_;
  size_t write_index_{0};
  size_t count_{0};
};

} // namespace fyt::auto_aim

#endif
```

In Phase 2, only `preprocess_ms` and `infer_ms` are populated. The remaining slots are reserved and default to 0.

---

## 9. Node Integration (Phase 2 additions to `armor_detector_nn_node`)

### 9.1 New Members

```cpp
std::unique_ptr<ArmorDetectorNN> detector_;
std::unique_ptr<Profiler> profiler_;
```

### 9.2 Initialization

In the constructor, after `validateParameters()`:

```cpp
detector_ = std::make_unique<ArmorDetectorNN>(config_);
if (!detector_->initialize()) {
  FYT_ERROR("armor_detector_nn", "Failed to initialize detector. "
            "Node will start but detection is disabled.");
  // Node stays alive — set_mode can trigger a re-init later.
}

if (config_.runtime.profile) {
  profiler_ = std::make_unique<Profiler>();
}
```

### 9.3 Updated `imageCallback`

```cpp
void ArmorDetectorNNNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  if (current_mode_ == DetectMode::DISABLED || !detector_ || !detector_->isInitialized()) {
    return;
  }

  ProfilerEntry entry;
  auto t_start = std::chrono::steady_clock::now();

  // Convert ROS image to cv::Mat
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  } catch (const cv_bridge::Exception& e) {
    FYT_ERROR("armor_detector_nn", "cv_bridge error: %s", e.what());
    return;
  }

  // Synchronous detection (Phase 2 — single frame, batch=1)
  auto results = detector_->detectBatch({frame}, {img_msg->header});

  entry.total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();

  if (profiler_) {
    profiler_->record(entry);
  }

  // Phase 2: publish empty Armors (no postprocess yet).
  // Maintains topic liveness so downstream nodes can validate connectivity.
  publishEmptyArmors(img_msg->header);

  // Debug: log output tensor shape and first 5 values on first frame.
  static bool first_frame = true;
  if (first_frame && !results.empty() && !results[0].debug_raw_outputs.empty()) {
    const auto& out = results[0].debug_raw_outputs[0];
    FYT_INFO("armor_detector_nn",
             "output0 shape: [%s], first values: [%.4f, %.4f, %.4f, %.4f, %.4f]",
             /* shape fmt */           /* 5 floats */);
    first_frame = false;
  }
}
```

---

## 10. Error Handling & Robustness

### 10.1 Model File Not Found

If `best.onnx` is missing or the resolved path doesn't exist:
- `OnnxRuntimeBackend::load()` logs the resolved path and throws `std::runtime_error("model file not found: ...")`.
- `ArmorDetectorNN::initialize()` catches it, logs, returns false.
- The node continues running without detection capability.
- A subsequent `set_mode` call can trigger a re-initialization attempt if desired (reserved for Phase 7).

### 10.2 I/O Mismatch

If the loaded ONNX model has unexpected input/output counts or shapes:
- `validateModelIO()` compares against config and throws a descriptive error.
- The error message includes expected vs actual shapes, helping field debugging when the wrong model file is deployed.

### 10.3 Inference Exception

If `session_->Run()` throws mid-inference (e.g., OOM on GPU):
- Catch in `infer()`, log the error, and return an empty output vector.
- The caller (`detectBatch`) sees empty outputs and returns empty `FrameDetections`.
- This degrades gracefully: a single bad frame is dropped, the next frame retries.

---

## 11. Interfaces Reserved for Future Phases

### 11.1 BackendInfo for Scheduler (Phase 7)

`BackendInfo` is populated in Phase 2 and read by the scheduler in Phase 7:

```cpp
// Phase 2 populates:
BackendInfo {
  backend_name:   "onnxruntime",
  precision:      "fp32",
  min_batch_size: 1,
  max_batch_size: 1,      // model is static batch=1
  dynamic_batch:  false,
}
```

The scheduler uses `max_batch_size` to clamp `runtime.batch_max_size`. Since this ONNX model has static batch=1, `async_batch` mode will automatically degrade to `batch=1` — equivalent to `async_latest`.

### 11.2 Preprocessor `processBatch` (Phase 7)

The `processBatch` stub already accepts a vector. When Phase 7 enables batch inference:
- ONNX models with dynamic batch will report `max_batch_size > 1` via `BackendInfo`.
- `processBatch` will stack images along the N dimension (NCHW → [N,3,640,640]).
- The current Phase 2 model is static, so this path won't be exercised until we have a dynamic-batch ONNX model or TensorRT engine.

### 11.3 Debug Image Output (Phase 3)

The debug publisher `result_img_pub_` is declared in Phase 1 but not populated until Phase 3 (postprocess draws bbox/keypoints).

### 11.4 Profiler Integration (Phase 3+)

`ProfilerEntry` has slots for `decode_ms`, `nms_ms`, `pose_ms` that are zero-filled in Phase 2. The profiler publishes to `armor_detector/profile` (topic reserved in Phase 1, implemented in Phase 3).

---

## 12. Verification

### 12.1 Sanity Checks

1. Build with `BUILD_ONNX_BACKEND=ON`:
   ```bash
   colcon build --packages-select armor_detector_nn --cmake-args -DBUILD_ONNX_BACKEND=ON
   ```
2. Set `model_path` to `package://armor_detector_nn/model/demo01/best.onnx` in the config YAML.
3. Launch and verify log message: `"OnnxRuntimeBackend: loaded model, input=[1,3,640,640], output=[1,26,8400]"`.
4. Verify that `output0` first values are reasonable (not NaN, not all zero after warmup with real image).
5. Check that `ros2 topic echo /armor_detector/armors` shows empty messages with correct headers.

### 12.2 Performance Baseline (CPU)

On the target NUC or dev machine, record:
- Preprocess time (letterbox + color convert + normalize).
- Inference time (first frame after warmup vs steady-state).
- Total end-to-end latency (image receive → output tensor ready).

This baseline informs whether CPU-only ONNX is viable or whether OpenVINO (Phase 6) / TensorRT (Phase 5) must be prioritized.

---

## 13. Acceptance Criteria

1. ONNX model loads without error from `package://` URI.
2. `infer()` returns a `TensorOutput` with shape `[1, 26, 8400]`.
3. Warmup completes without crashing.
4. Node pipeline: `imageCallback` → `cv_bridge` → `Preprocessor` → `infer` → log output shape (no crash).
5. Missing model file produces clear error, node stays alive.
6. `BackendInfo` correctly reports `max_batch_size = 1, dynamic_batch = false`.
7. `Profiler` records and reports `preprocess_ms` and `infer_ms`.
8. `/armor_detector/armors` publishes empty messages (topic stays alive for downstream testing).
