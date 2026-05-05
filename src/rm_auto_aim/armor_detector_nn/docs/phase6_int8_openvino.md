# Phase 6: INT8 & OpenVINO

## 1. Overview

**Goal:** Implement the `OpenVINOBackend` (OpenVINO 2.0 API / `ov::Core`) for Intel CPU/NUC platforms, with full INT8 quantized model support. Extend the backend factory with fallback logic so the system can degrade gracefully when a preferred backend is unavailable. This phase is the Intel-platform counterpart to Phase 5 (Jetson TensorRT).

**Precondition:** Phases 1–4 complete (full pipeline functional with ONNX Runtime backend).

**Model used:** `model/demo01/best_int8_openvino_model/best.xml` + `best.bin`
- Format: OpenVINO IR v11
- Quantization: NNCF INT8 (`levels=256`, `FakeQuantize` layers in graph)
- Input: `x`, shape `[1, 3, 640, 640]`, precision `FP32` (quantization is internal)
- Output: single `Result` layer (name determined at load time)
- Preprocessing baked into IR metadata: `<reverse_input_channels value="YES"/>`, `<scale_values value="255"/>`, `<resize_type value="fit_to_window_letterbox"/>`, `<pad_value value="114"/>`

---

## 2. The `demo01` INT8 Model

### 2.1 Key Characteristics

From `metadata.yaml` and `best.xml`:

| Property | Value |
|:--|:--|
| Framework | PyTorch → ONNX → OpenVINO IR (via `mo` + NNCF) |
| Task | pose |
| Input | `x`: `[1, 3, 640, 640]`, FP32 |
| Output | `output0`: `[1, 26, 8400]`, FP32 (Result layer resolves to the dequantized output) |
| Quantization | NNCF INT8 with per-channel FakeQuantize nodes |
| Ignored scope | `.*model.32/.*` (detect head kept in FP32 for numerical stability of DFL, sigmoid) |
| Preprocessing | reverse channels, scale 1/255, letterbox, pad 114 (baked into IR `model_info`) |
| Anchor points | Constant tensor `[1, 2, 8400]` embedded in the graph (layer id=1) |

### 2.2 Quantization Architecture

The model uses NNCF's FakeQuantize pattern: each quantized weight is stored as INT8 in a `Const` layer, then converted to FP32 via a `Convert` layer, followed by the actual op. Activations pass through `FakeQuantize` layers (`levels=256`) that simulate INT8 range clipping during inference. On OpenVINO CPU with Intel DL Boost (VNNI), these patterns are recognized and mapped to low-precision ops at runtime.

**Output head preservation:** The `ignored_scope` section in the NNCF config excludes the YOLO detection head (`model.32`) from quantization. This means the final classification, bbox regression, and DFL layers run in FP32. This is intentional — these layers have small weight tensors but are sensitive to quantization error in the final coordinate/logit computation.

The output tensor `output0` is FP32 and has the exact same shape and semantics as the FP32 ONNX model used in Phase 2. **This is critical:** it means the `UltralyticsPoseDecodeStrategy` from Phase 3 works unchanged with INT8 model output.

### 2.3 Baked-In Preprocessing

The IR `model_info` section includes preprocessing directives:

```xml
<model_info>
  <reverse_input_channels value="YES" />
  <scale_values value="255" />
  <resize_type value="fit_to_window_letterbox" />
  <pad_value value="114" />
</model_info>
```

OpenVINO 2.0 provides `ov::preprocess::PrePostProcessor` which can consume this metadata to configure hardware-accelerated preprocessing. However, our Phase 2 `Preprocessor` already handles letterbox + color conversion + normalization in OpenCV on CPU. We have two options:

| Option | Approach | Pros | Cons |
|:--|:--|:--|:--|
| A: Use OV PrePostProcessor | OV handles color, resize, normalize on input tensor | Zero CPU cost for preprocessing; runs on GPU/iGPU if available | Binds preprocessing to OV backend; letterbox logic must match training exactly |
| B: Keep CPU Preprocessor | Unchanged from Phase 2; feed already-preprocessed tensor | Backend-agnostic; proven correct | Duplicates work OV could do for free |

**Phase 6 decision: Option B (cpu preprocessor) by default, with Option A gated by config.** This preserves the backend-agnostic pipeline and avoids coupling preprocessing to backend specifics:

```yaml
backend:
  openvino_use_native_preprocess: false   # true = use OV PrePostProcessor
```

When `true`, the `OpenVINOBackend` accepts raw BGR images (HWC uint8) and configures the OV preprocessing pipeline. When `false` (default), the standard `Preprocessor` runs and feeds `[1,3,640,640]` FP32 tensors just like the ONNX backend.

The native-preprocess path is a forward-looking optimization for Intel NUC with integrated GPU, where OV can offload color conversion and resize. Phase 6 implements the gated path but defaults to the CPU preprocessor for consistency.

---

## 3. OpenVINOBackend

### 3.1 Header: `include/armor_detector_nn/backend/openvino_backend.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_OPENVINO_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_OPENVINO_BACKEND_HPP_

#include "armor_detector_nn/backend/inference_backend.hpp"
#include <memory>
#include <string>
#include <vector>
#include <mutex>

// Forward-declare to avoid leaking OV headers to consumers.
namespace ov {
  class Core;
  class CompiledModel;
  class InferRequest;
  class Tensor;
}

namespace fyt::auto_aim {

class OpenVINOBackend : public IInferenceBackend {
public:
  OpenVINOBackend();
  ~OpenVINOBackend() override;

  void load(const BackendConfig& config) override;
  std::vector<TensorOutput> infer(const TensorInput& input) override;
  void warmup(int iterations) override;
  BackendInfo info() const override;

private:
  // Resolve model path, load IR (xml+bin), compile for device.
  void loadModel(const std::string& xml_path, const std::string& bin_path,
                 const std::string& device, int num_threads);

  // Configure OV PrePostProcessor when native preprocessing is enabled.
  void configurePreprocessing(const BackendConfig& config);

  // Validate model I/O against config.
  void validateModelIO(const BackendConfig& config);

  // Scan quantization info from model metadata (nncf section).
  void detectQuantizationPrecision();

  // Try each available device in order until compilation succeeds.
  std::string selectAvailableDevice(const std::string& preferred_device);

  std::unique_ptr<ov::Core> core_;
  std::unique_ptr<ov::CompiledModel> compiled_model_;
  std::unique_ptr<ov::InferRequest> infer_request_;

  std::string input_name_;
  std::vector<std::string> output_names_;
  std::vector<int64_t> input_shape_;
  std::vector<std::vector<int64_t>> output_shapes_;

  // Native preprocessing toggle.
  bool use_native_preprocess_{false};

  // INT8 detection result.
  bool is_int8_quantized_{false};

  BackendInfo info_;
  bool loaded_{false};

  // Thread safety. OV InferRequest::infer() is not thread-safe on all plugins.
  std::mutex infer_mutex_;
};

} // namespace fyt::auto_aim

#endif
```

### 3.2 Implementation Outline

#### 3.2.1 `load(const BackendConfig& config)`

```
1. Initialize ov::Core with config.num_threads as hint.
   - Optionally set CPU affinity: core_->set_property("CPU",
       ov::intel_cpu::affinity(ov::intel_cpu::Affinity::NUMA));

2. Select device:
   - config.device → "CPU", "GPU" (Intel iGPU), "NPU" (Meteor Lake+).
   - Call selectAvailableDevice() to check actual availability.
   - Query available_devices from ov::Core, match case-insensitively.
   - If preferred device not found and allow_fallback == true:
       Try in order: ["CPU"] (CPU is always available on Intel platforms).
   - If no device available: throw std::runtime_error.

3. Load model:
   - Resolve config.openvino_xml_path (package:// URI).
   - Derive .bin path: replace .xml → .bin, or read from config.openvino_bin_path.
   - core_->read_model(xml_path, bin_path) → std::shared_ptr<ov::Model>.
   - Verify model is not nullptr.

4. Detect quantization:
   - Iterate over model ops searching for FakeQuantize nodes.
   - If found, set is_int8_quantized_ = true.
   - Check NNCF metadata in rt_info section for quantization version.

5. Configure preprocessing (if use_native_preprocess_):
   - ov::preprocess::PrePostProcessor ppp(model).
   - ppp.input().tensor().set_layout("NHWC").set_element_type(ov::element::u8).
   - ppp.input().model().set_layout("NCHW").
   - ppp.input().preprocess().convert_element_type(ov::element::f32).
   - ppp.input().preprocess().scale(1.0f / 255.0f).
   - ppp.input().preprocess().convert_color(ov::preprocess::ColorFormat::BGR).
   - model = ppp.build().

6. Compile model:
   - compiled_model_ = std::make_unique<ov::CompiledModel>(
       core_->compile_model(model, device, compile_config));
   - Compile config: enable performance hints, set num_streams.
     - For CPU: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY).
     - num_streams: typically 1 for real-time (avoid pipeline parallelism latency).
     - If config.num_threads > 0: ov::inference_num_threads(config.num_threads).

7. Create inference request:
   - infer_request_ = std::make_unique<ov::InferRequest>(
       compiled_model_->create_infer_request()).

8. Query I/O:
   - input_name_: from compiled_model_->input().get_any_name().
   - output_names_: from compiled_model_->outputs().
   - input_shape_: from compiled_model_->input().get_shape().
   - output_shapes_: from compiled_model_->outputs() shapes.

9. Validate I/O against config:
   - input_shape_ must be [1, 3, 640, 640].
   - output count must match config.output_names.size().
   - Output shapes are captured for pre-allocation.

10. Populate BackendInfo:
    - backend_name = "openvino"
    - precision = is_int8_quantized_ ? "int8" : config.precision string
    - min_batch_size = input_shape_[0] (1 for static model)
    - max_batch_size = input_shape_[0]
    - dynamic_batch = model has dynamic batch dim?

11. Set loaded_ = true.
```

#### 3.2.2 `detectQuantizationPrecision()`

```cpp
void OpenVINOBackend::detectQuantizationPrecision()
{
  // Iterate the model ops searching for FakeQuantize.
  // If found, this is an INT8-quantized model.
  // Check rt_info for NNCF metadata to confirm.
  is_int8_quantized_ = false;

  for (auto& op : compiled_model_->get_runtime_model()->get_ops()) {
    if (op->get_type_name() == "FakeQuantize") {
      is_int8_quantized_ = true;
      break;
    }
  }

  // Cross-check with NNCF rt_info if available.
  try {
    auto rt_info = compiled_model_->get_runtime_model()->get_rt_info();
    if (rt_info.count("nncf")) {
      auto nncf = rt_info.at("nncf").as<ov::AnyMap>();
      if (nncf.count("version")) {
        auto ver = nncf.at("version").as<std::string>();
        FYT_INFO("openvino", "NNCF quantization version: %s", ver.c_str());
      }
    }
  } catch (...) {
    // rt_info is optional — if missing, rely on FakeQuantize detection.
  }

  // Update info_.
  if (is_int8_quantized_) {
    info_.precision = "int8";
  }
}
```

#### 3.2.3 `infer(const TensorInput& input)`

```cpp
std::vector<TensorOutput> OpenVINOBackend::infer(const TensorInput& input)
{
  std::lock_guard<std::mutex> lock(infer_mutex_);

  // 1. Get input tensor from infer request.
  auto input_tensor = infer_request_->get_tensor(input_name_);

  // 2. Copy host data into OV tensor.
  //    OV tensor expects data in the model's native layout (NCHW, FP32).
  //    The Preprocessor has already prepared this.
  std::memcpy(input_tensor.data<float>(),
              input.host_data.data(),
              input.host_data.size() * sizeof(float));

  // 3. Run inference (synchronous).
  infer_request_->infer();

  // 4. Extract outputs.
  std::vector<TensorOutput> results;
  results.reserve(output_names_.size());

  for (size_t i = 0; i < output_names_.size(); ++i) {
    auto out_tensor = infer_request_->get_tensor(output_names_[i]);
    const auto& shape = output_shapes_[i];

    size_t num_elements = 1;
    for (auto d : shape) num_elements *= d;

    TensorOutput out;
    out.info.name = output_names_[i];
    out.info.shape = shape;
    out.info.dtype = TensorInfo::DType::FLOAT32;
    out.host_data.resize(num_elements);

    std::memcpy(out.host_data.data(),
                out_tensor.data<float>(),
                num_elements * sizeof(float));

    results.push_back(std::move(out));
  }

  return results;
}
```

**Synchronous inference** (`infer_request_->infer()`) is used. OpenVINO also provides asynchronous inference (`start_async()` + `wait()`), which is valuable for overlapping CPU preprocessing with inference on GPU/iGPU. A future optimization (gated by config `openvino_async: true`) would:
1. Call `start_async()` — returns immediately.
2. Do postprocessing on the *previous* frame's output while the current frame infers.
3. Call `wait()` on the previous request to get results.

This async pipeline is documented in section 10 (reserved interfaces) but not implemented in Phase 6.

#### 3.2.4 `warmup(int iterations)`

```cpp
void OpenVINOBackend::warmup(int iterations)
{
  std::vector<float> dummy_data(input_shape_[0] * input_shape_[1] *
                                 input_shape_[2] * input_shape_[3], 0.0f);

  auto input_tensor = infer_request_->get_tensor(input_name_);
  std::memcpy(input_tensor.data<float>(), dummy_data.data(),
              dummy_data.size() * sizeof(float));

  for (int i = 0; i < iterations; ++i) {
    infer_request_->infer();
  }
}
```

#### 3.2.5 `selectAvailableDevice()`

```cpp
std::string OpenVINOBackend::selectAvailableDevice(
    const std::string& preferred_device)
{
  auto available = core_->get_available_devices();

  // Exact match.
  for (const auto& d : available) {
    if (d == preferred_device) return d;
  }

  // Case-insensitive fallback.
  std::string upper_pref = preferred_device;
  std::transform(upper_pref.begin(), upper_pref.end(),
                 upper_pref.begin(), ::toupper);
  for (const auto& d : available) {
    std::string upper = d;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    if (upper == upper_pref) return d;
  }

  // Return empty — caller decides fallback.
  return {};
}
```

---

## 4. Backend Fallback Mechanism

### 4.1 Design

Backend selection follows this priority:

```
1. Try config.backend.type as specified (e.g., "openvino").
2. If that backend fails to load (not compiled, device unavailable, model missing):
   a. If config.backend.allow_fallback == true:
      - Try config.backend.fallback_type.
   b. Else: return nullptr (load failed).
3. If fallback also fails: return nullptr.
```

### 4.2 `InferenceBackendFactory` Implementation

```cpp
// src/backend/inference_backend_factory.cpp

std::unique_ptr<IInferenceBackend> InferenceBackendFactory::create(
    const BackendConfig& config)
{
  auto primary = createOne(config.type, config);
  if (primary) return primary;

  if (!config.allow_fallback) {
    FYT_ERROR("backend", "Primary backend %s failed and fallback is disabled",
              backendTypeToString(config.type).c_str());
    return nullptr;
  }

  FYT_WARN("backend", "Primary backend %s failed, falling back to %s",
           backendTypeToString(config.type).c_str(),
           backendTypeToString(config.fallback_type).c_str());
  return createOne(config.fallback_type, config);
}

std::unique_ptr<IInferenceBackend> InferenceBackendFactory::createOne(
    BackendType type, const BackendConfig& config)
{
  switch (type) {
    case BackendType::ONNX_RUNTIME:
#ifdef ARMOR_DETECTOR_NN_HAS_ONNX
      {
        auto backend = std::make_unique<OnnxRuntimeBackend>();
        backend->load(config);
        return backend;
      }
#else
      FYT_WARN("backend", "ONNX Runtime backend not compiled in");
      return nullptr;
#endif

    case BackendType::OPENVINO:
#ifdef ARMOR_DETECTOR_NN_HAS_OPENVINO
      {
        auto backend = std::make_unique<OpenVINOBackend>();
        backend->load(config);
        return backend;
      }
#else
      FYT_WARN("backend", "OpenVINO backend not compiled in");
      return nullptr;
#endif

    case BackendType::TENSORRT:
#ifdef ARMOR_DETECTOR_NN_HAS_TENSORRT
      {
        auto backend = std::make_unique<TensorRTBackend>();
        backend->load(config);
        return backend;
      }
#else
      FYT_WARN("backend", "TensorRT backend not compiled in");
      return nullptr;
#endif

    default:
      return nullptr;
  }
}
```

### 4.3 Fallback Configuration Examples

**NUC with OpenVINO, fallback to ONNX:**
```yaml
backend:
  type: "openvino"
  device: "cpu"
  precision: "int8"
  openvino_model_xml: "package://armor_detector_nn/model/demo01/best_int8_openvino_model/best.xml"
  openvino_model_bin: "package://armor_detector_nn/model/demo01/best_int8_openvino_model/best.bin"
  model_path: "package://armor_detector_nn/model/demo01/best.onnx"
  allow_fallback: true
  fallback_type: "onnxruntime"
```

**Jetson with TensorRT, fallback to ONNX:**
```yaml
backend:
  type: "tensorrt"
  engine_path: "package://armor_detector_nn/model/best_fp16.engine"
  model_path: "package://armor_detector_nn/model/demo01/best.onnx"
  allow_fallback: true
  fallback_type: "onnxruntime"
```

---

## 5. INT8 Precision Workflow

### 5.1 Deployment Path

```
                    Training
                       │
                 PyTorch weights
                       │
                 Export ONNX (FP32)
                    │        │
                    │    NNCF Quantization
                    │        │
                    │    OpenVINO IR (INT8)
                    │    best.xml + best.bin
                    │
              ONNX Runtime (FP32)
              For dev / fallback
```

### 5.2 Validation Checks at Load Time

When loading an INT8 model, `OpenVINOBackend::load()` performs:

1. **FakeQuantize presence:** Detect FakeQuantize ops in the runtime model graph (confirms INT8).
2. **NNCF version check:** Read `nncf.version` from `rt_info`. If the version is incompatible (future major version change), log a warning — the model may still work but should be re-quantized.
3. **Ignored scope check:** Read `nncf.quantization.ignored_scope` from `rt_info` and log the excluded layer patterns. This helps debugging if detection head accuracy is unexpectedly poor.
4. **Precision metadata:** Cross-check `metadata.yaml` `int8: true` with the IR graph structure.
5. **Output precision verification:** Confirm the output tensor is FP32 (not INT8), ensuring the postprocess pipeline works unchanged.

### 5.3 Performance Expectations

Expected relative performance on Intel NUC (Core i7, 12th gen+ with VNNI):

| Precision | Backend | Relative Throughput | Notes |
|:--|:--|:--|:--|
| FP32 | ONNX Runtime | 1.0× (baseline) | ~8–12 ms infer |
| FP32 | OpenVINO | 1.1–1.4× | OV graph optimizations |
| INT8 | OpenVINO | 2.0–3.0× | VNNI accelerates INT8 ops; head still FP32 |
| FP16 | OpenVINO (GPU) | 1.5–2.5× | Only on Intel Iris Xe / Arc iGPU |

**Note:** INT8 speedup is not 4× because the YOLO detection head (~10–15% of flops) is kept in FP32 by NNCF's ignored scope. This is a deliberate accuracy-vs-speed tradeoff.

---

## 6. CMakeLists.txt Additions

```cmake
# --- OpenVINO ---
if(BUILD_OPENVINO_BACKEND)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)

  list(APPEND BACKEND_SOURCES
    src/backend/openvino_backend.cpp
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC
    openvino::runtime
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    ARMOR_DETECTOR_NN_HAS_OPENVINO=1
  )
endif()

# --- TensorRT (Phase 5) ---
if(BUILD_TENSORRT_BACKEND)
  find_package(TensorRT REQUIRED)
  list(APPEND BACKEND_SOURCES
    src/backend/tensorrt_backend.cpp
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC
    nvinfer nvinfer_plugin
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    ARMOR_DETECTOR_NN_HAS_TENSORRT=1
  )
endif()
```

Both `BUILD_OPENVINO_BACKEND` and `BUILD_TENSORRT_BACKEND` are `OFF` by default. On Jetson, the user sets `-DBUILD_TENSORRT_BACKEND=ON`. On Intel NUC, the user sets `-DBUILD_OPENVINO_BACKEND=ON`. Both can be ON simultaneously if the dev machine has both SDKs installed.

---

## 7. OpenVINO-Specific Optimizations

### 7.1 Performance Hints

```cpp
// In load(), after compiling the model:
auto compile_config = ov::CompilationConfig();

// Latency mode: optimize for single-inference latency, not throughput.
// Critical for real-time aim where each frame must be processed ASAP.
compile_config.set(ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

// Single stream: avoid OpenCL/oneDNN pipeline parallelism which trades
// latency for throughput. For real-time, we process one frame at a time.
compile_config.set(ov::streams::num(1));

// Thread count from config.
if (config.num_threads > 0) {
  compile_config.set(ov::inference_num_threads(config.num_threads));
}

compiled_model_ = std::make_unique<ov::CompiledModel>(
    core_->compile_model(model, device, compile_config));
```

### 7.2 CPU Affinity (NUMA)

If the NUC has NUMA-capable cores (12th-gen P-core + E-core):

```cpp
// Pin inference threads to performance cores.
// Only effective if ov::inference_num_threads is set.
#ifdef __linux__
core_->set_property("CPU", ov::intel_cpu::affinity(ov::intel_cpu::Affinity::HYBRID_AWARE));
#endif
```

This is a config-level toggle:
```yaml
backend:
  openvino_hybrid_affinity: false   # true on hybrid-arch CPUs
```

### 7.3 OpenVINO Model Caching

OpenVINO supports caching compiled models to disk, avoiding recompilation on restart:

```cpp
compile_config.set(ov::cache_dir("/tmp/openvino_cache"));
```

For production, the cache dir should be configurable:
```yaml
backend:
  openvino_cache_dir: ""   # empty = no cache; path = enable
```

---

## 8. INT8 Calibration (Reserved for Custom Models)

The current `demo01` model was quantized via NNCF using a 300-sample calibration subset (`subset_size=300`). If future custom models require re-quantization, the calibration workflow is:

```
1. Collect representative images (rosbag replay, diverse lighting + distances).
2. Use NNCF quantization with preset=mixed:
     nncf.quantize(ov_model, calibration_dataset,
                   preset=nncf.QuantizationPreset.MIXED,
                   ignored_scope=nncf.IgnoredScope(
                     patterns=[".*model.32/.*/Add", ".*model.32/.*/Sub*",
                               ".*model.32/.*/Mul*", ".*model.32/.*/Div*",
                               ".*model.32\\.dfl.*"],
                     types=["Sigmoid"]))
3. Export: ov.serialize(model, "best_int8_openvino_model/best.xml")
```

This is a model-engineering task, not a runtime task. The backend just loads and infers.

---

## 9. Node Integration

### 9.1 Configuration for OpenVINO INT8

For NUC deployment, the launch config uses:

```yaml
backend:
  type: "openvino"
  device: "cpu"
  precision: "int8"
  openvino_model_xml: "package://armor_detector_nn/model/demo01/best_int8_openvino_model/best.xml"
  openvino_model_bin: "package://armor_detector_nn/model/demo01/best_int8_openvino_model/best.bin"
  model_path: "package://armor_detector_nn/model/demo01/best.onnx"
  allow_fallback: true
  fallback_type: "onnxruntime"
```

The node does not require code changes — the backend factory handles the selection transparently.

### 9.2 Profiler Publishing

The profiler's `backend` and `precision` fields now reflect the actual backend used (e.g., `"openvino"`, `"int8"`). This is published to `armor_detector/profile` so operators can verify the correct backend is active.

---

## 10. Reserved Interfaces

### 10.1 Async Inference Pipeline

```cpp
// Reserved in OpenVINOBackend:
class OpenVINOBackend : public IInferenceBackend {
  // Phase 6: sync only.
  // Future: double-buffered async.
  //
  // struct AsyncState {
  //   ov::InferRequest request_a;   // currently inferring
  //   ov::InferRequest request_b;   // ready for postprocessing
  //   int active = 0;               // 0 or 1
  // };
  // std::unique_ptr<AsyncState> async_state_;
  //
  // void startAsync(const TensorInput& input);
  // std::vector<TensorOutput> waitAsync();
};
```

The `armor_detector_nn_design.md` async pipeline (Phase 7) would use async inference to overlap CPU work with GPU/iGPU inference. The stub above reserves space in the backend interface without requiring API changes.

### 10.2 Dynamic Batch Reshape

OpenVINO models can be reshaped at runtime if the input dims have dynamic ranges. If a future model exports with `DynamicShapes`:

```yaml
backend:
  openvino_dynamic_reshape: false    # enable to reshape to batch=N
```

At load time, if enabled:
```cpp
if (config.openvino_dynamic_reshape && model->input().get_partial_shape()[0].is_dynamic()) {
  model->reshape({{input_name_, ov::Shape{static_cast<size_t>(batch_size), 3, 640, 640}}});
}
```

BackendInfo would then report `dynamic_batch: true` and `max_batch_size: <reshaped size>`.

### 10.3 Multiple InferRequest Pool (Throughput Mode)

For batch inference or multi-stream throughput, a pool of `InferRequest` objects can be used. Each request holds its own I/O buffers, enabling concurrent `start_async()` calls. Reserved config:

```yaml
backend:
  openvino_num_requests: 1           # >1 = enable request pool
```

The pool size should be small (2–4) to avoid memory pressure.

### 10.4 Calibration Dataset API

If future models need online calibration (unlikely but reserved):

```cpp
// Reserved in OpenVINOBackend:
// void calibrateInt8(const std::vector<cv::Mat>& calibration_images);
// Requires: ov::pass::QuantizeModel API (NNCF runtime integration).
```

### 10.5 Device-Specific Compilation Options

Reserved for FPGA and NPU targets:

```yaml
backend:
  openvino_device_config: ""   # JSON string for device-specific options
```

This is parsed and forwarded to `core_->compile_model()` as additional properties.

---

## 11. TensorRT INT8 (Phase 5 Connection)

Phase 5 implements `TensorRTBackend` with FP16 engine support. Phase 6 adds INT8 support to that backend:

### 11.1 TensorRT INT8 Loading

```cpp
// In TensorRTBackend::load():
if (config.precision == Precision::INT8) {
  // Load INT8 engine file (.engine).
  // If engine not found and calibration cache exists:
  //   Option A: load FP32 ONNX + calibrate with cache → build INT8 engine.
  //   Option B (recommended): error out, require offline-built engine.
  //
  // Phase 6 follows Option B:
  // - INT8 engines must be pre-built (Phase 5 already established this convention).
  // - The backend checks that the engine's binding DTypes are INT8 or FP32.
}
```

### 11.2 TensorRT INT8 Validation

```cpp
// Validate that the engine uses INT8 precision:
for (int i = 0; i < engine_->getNbBindings(); ++i) {
  auto dtype = engine_->getBindingDataType(i);
  if (dtype == nvinfer1::DataType::kINT8) {
    has_int8_weights = true;
  }
}
if (!has_int8_weights) {
  FYT_WARN("tensorrt", "Engine loaded but no INT8 bindings detected. "
           "Falling back to FP16 if available.");
}
```

### 11.3 Calibration Cache

The `calibration.cache` file is a TensorRT calibration table. Phase 6 only reads it for validation (checking it exists alongside the INT8 engine). The cache is NOT used for runtime re-calibration — that is an offline, developer workflow.

---

## 12. Test Plan

### 12.1 `test_openvino_backend.cpp`

| Test | Verification |
|:--|:--|
| Load INT8 IR model | `loaded_ == true`, `is_int8_quantized_ == true` |
| `BackendInfo` reports int8 precision | `info().precision == "int8"` |
| Infer with dummy FP32 input | Returns output `[1, 26, 8400]`, no exception |
| Output values are finite | No NaN, no Inf in first 100 values |
| Warmup completes | 10 iterations without error |
| Missing .xml file | Throws `std::runtime_error`, caught by factory, returns nullptr |
| Missing .bin file | Throws `std::runtime_error` |
| Device "GPU" not available | Falls back to "CPU" if `allow_fallback` |
| INT8 output shape matches FP32 ONNX output | Same `[1, 26, 8400]` for same input |
| Thread safety | 2 threads calling infer() on same backend — no data corruption |

### 12.2 `test_backend_factory.cpp`

| Test | Verification |
|:--|:--|
| `create(OPENVINO)` succeeds when built | Non-null return |
| `create(OPENVINO)` fails gracefully when not built | nullptr + log warning |
| Fallback from OPENVINO to ONNX | When OV model missing, returns ONNX backend |
| Fallback chain exhausted | nullptr after all options fail |

### 12.3 Regression: INT8 vs FP32 Output Comparison

Using the same input image, compare `output0` between ONNX FP32 and OpenVINO INT8:
- Per-channel mean difference should be < 0.01 (1% of FP32 range).
- Detection results (after postprocessing) should match:
  - Same number of detections (≥ 90% frame agreement).
  - Same class assignments.
  - Bbox center within 2 pixels.
  - Keypoint within 3 pixels.

---

## 13. Acceptance Criteria

1. `OpenVINOBackend` loads `best.xml` + `best.bin` from `demo01/best_int8_openvino_model/`.
2. `BackendInfo.precision == "int8"` when loading the INT8 model.
3. `infer()` returns output with shape `[1, 26, 8400]` in FP32.
4. Full pipeline runs with OpenVINO backend: `image_raw → Armors message`.
5. Fallback from OpenVINO to ONNX Runtime works when OV model is missing.
6. Profiler reports `"openvino"` as backend name.
7. INT8 inference latency is measurably lower than FP32 ONNX on the same Intel CPU.
8. `warmup()` completes successfully with the INT8 model.
9. Node switches between backends via config without recompilation.
10. OpenVINO backend compiles and runs on Ubuntu 22.04 with OpenVINO 2024.x or later.
