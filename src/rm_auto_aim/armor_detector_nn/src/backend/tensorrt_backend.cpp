#include "armor_detector_nn/backend/tensorrt_backend.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>

#include <NvInfer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

namespace {

class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kERROR) {
      FYT_ERROR("armor_detector_nn", "TensorRT: %s", msg);
    } else if (severity == Severity::kWARNING) {
      FYT_WARN("armor_detector_nn", "TensorRT: %s", msg);
    }
  }
};

template <typename T>
struct TrtDeleter {
  void operator()(T* ptr) const {
    if (ptr) {
      delete ptr;
    }
  }
};

std::string resolvePath(const std::string& raw_path) {
  if (raw_path.empty()) return raw_path;
  if (raw_path.compare(0, 10, "package://") == 0) {
    auto slash = raw_path.find('/', 10);
    std::string pkg = raw_path.substr(10, slash - 10);
    std::string rel = raw_path.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  return raw_path;
}

std::vector<int64_t> dimsToShape(const nvinfer1::Dims& dims) {
  std::vector<int64_t> shape;
  shape.reserve(dims.nbDims);
  for (int i = 0; i < dims.nbDims; ++i) {
    shape.push_back(static_cast<int64_t>(dims.d[i]));
  }
  return shape;
}

}  // namespace

TensorRTBackend::TensorRTBackend() {
  info_.backend_name = "tensorrt";
  info_.precision = "fp32";
  info_.min_batch_size = 1;
  info_.max_batch_size = 1;
  info_.dynamic_batch = false;
}

TensorRTBackend::~TensorRTBackend() {
  if (context_ != nullptr) {
    delete context_;
    context_ = nullptr;
  }
  if (engine_ != nullptr) {
    delete engine_;
    engine_ = nullptr;
  }
  if (runtime_ != nullptr) {
    delete runtime_;
    runtime_ = nullptr;
  }

  if (input_device_ != nullptr) {
    cudaFree(input_device_);
    input_device_ = nullptr;
  }
  for (void* p : output_devices_) {
    if (p != nullptr) cudaFree(p);
  }
  output_devices_.clear();

  if (pinned_input_host_ != nullptr) {
    cudaFreeHost(pinned_input_host_);
    pinned_input_host_ = nullptr;
  }
  pinned_input_ = false;

  if (cuda_stream_ != nullptr) {
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
}

void TensorRTBackend::load(const BackendConfig& config) {
  std::string engine_path = resolvePath(config.engine_path);
  if (engine_path.empty()) {
    throw std::runtime_error("TensorRTBackend: engine_path is empty");
  }

  std::ifstream ifs(engine_path, std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("TensorRTBackend: failed to open engine: " + engine_path);
  }

  ifs.seekg(0, std::ios::end);
  const auto size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  if (size <= 0) {
    throw std::runtime_error("TensorRTBackend: empty engine file: " + engine_path);
  }

  std::vector<char> engine_data(static_cast<size_t>(size));
  if (!ifs.read(engine_data.data(), size)) {
    throw std::runtime_error("TensorRTBackend: failed to read engine bytes");
  }

  logger_ = std::make_unique<TrtLogger>();
  runtime_ = nvinfer1::createInferRuntime(*logger_);
  if (!runtime_) {
    throw std::runtime_error("TensorRTBackend: createInferRuntime failed");
  }

  using EngineUPtr = std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter<nvinfer1::ICudaEngine>>;
  using CtxUPtr = std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter<nvinfer1::IExecutionContext>>;

  EngineUPtr engine_guard(
    runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
  if (!engine_guard) {
    throw std::runtime_error("TensorRTBackend: deserializeCudaEngine failed");
  }

  CtxUPtr context_guard(engine_guard->createExecutionContext());
  if (!context_guard) {
    throw std::runtime_error("TensorRTBackend: createExecutionContext failed");
  }

  engine_ = engine_guard.release();
  context_ = context_guard.release();
  precision_ = config.precision;

  if (cudaStreamCreate(&cuda_stream_) != cudaSuccess) {
    throw std::runtime_error("TensorRTBackend: cudaStreamCreate failed");
  }

  const int nb = engine_->getNbIOTensors();
  if (nb < 2) {
    throw std::runtime_error("TensorRTBackend: expected at least 1 input + 1 output binding");
  }

  binding_indices_.clear();
  output_names_.clear();
  output_shapes_.clear();
  output_num_elements_.clear();
  output_devices_.clear();
  input_shape_.clear();
  input_name_.clear();

  bool has_int8_binding = false;
  for (int i = 0; i < nb; ++i) {
    const char* bname = engine_->getIOTensorName(i);
    if (!bname) continue;
    const std::string name = bname;
    binding_indices_[name] = i;
    const auto dtype = engine_->getTensorDataType(bname);
    if (dtype == nvinfer1::DataType::kINT8) has_int8_binding = true;

    const auto dims = context_->getTensorShape(bname);
    auto shape = dimsToShape(dims);

    bool is_input = true;
    // Heuristic: treat first I/O tensor or tensors with "input" in name as input
    if (name.find("input") != std::string::npos || i == 0) {
      is_input = true;
    } else {
      if (i != 0) is_input = false;
    }

    if (is_input) {
      input_name_ = name;
      input_shape_ = shape;
    } else {
      output_names_.push_back(name);
      output_shapes_.push_back(std::move(shape));
    }
  }

  if (input_name_.empty()) {
    throw std::runtime_error("TensorRTBackend: no input binding found");
  }
  if (output_names_.empty()) {
    throw std::runtime_error("TensorRTBackend: no output binding found");
  }

  validateModelIO(config);
  updateBatchCapability();

  if (config.precision == Precision::INT8 && !has_int8_binding) {
    FYT_WARN("armor_detector_nn",
             "TensorRT INT8 requested but engine has no INT8 bindings; using engine dtype");
  }

  const size_t input_bytes = elementCount(input_shape_) * sizeof(float);
  if (cudaMalloc(&input_device_, input_bytes) != cudaSuccess) {
    throw std::runtime_error("TensorRTBackend: cudaMalloc input buffer failed");
  }
  if (config.use_pinned_memory) {
    if (cudaMallocHost(reinterpret_cast<void**>(&pinned_input_host_), input_bytes) == cudaSuccess) {
      pinned_input_ = true;
    } else {
      FYT_WARN("armor_detector_nn", "TensorRT pinned host input alloc failed; fallback to pageable");
    }
  }

  for (const auto& shape : output_shapes_) {
    const size_t elements = elementCount(shape);
    output_num_elements_.push_back(elements);
    void* ptr = nullptr;
    if (cudaMalloc(&ptr, elements * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("TensorRTBackend: cudaMalloc output buffer failed");
    }
    output_devices_.push_back(ptr);
  }

  info_.precision = config.precision == Precision::FP32 ? "fp32" :
                    config.precision == Precision::FP16 ? "fp16" : "int8";
  loaded_ = true;
  FYT_INFO("armor_detector_nn", "TensorRTBackend loaded: {}", engine_path.c_str());
}

std::vector<TensorOutput> TensorRTBackend::infer(const TensorInput& input) {
  if (!loaded_) {
    throw std::runtime_error("TensorRTBackend: not loaded");
  }
  std::lock_guard<std::mutex> lock(infer_mutex_);

  const size_t expected_input = elementCount(input_shape_);
  if (input.host_data.size() != expected_input) {
    throw std::runtime_error(
      "TensorRTBackend::infer input size mismatch. Expected " +
      std::to_string(expected_input) + ", got " +
      std::to_string(input.host_data.size()));
  }

  const size_t input_bytes = expected_input * sizeof(float);
  const float* host_ptr = input.host_data.data();
  if (pinned_input_) {
    std::memcpy(pinned_input_host_, input.host_data.data(), input_bytes);
    host_ptr = pinned_input_host_;
  }

  if (cudaMemcpyAsync(input_device_, host_ptr, input_bytes, cudaMemcpyHostToDevice, cuda_stream_) != cudaSuccess) {
    throw std::runtime_error("TensorRTBackend::infer H2D copy failed");
  }

  nvinfer1::Dims input_dims{};
  if (input_shape_.size() == 4) {
    input_dims.nbDims = 4;
    input_dims.d[0] = static_cast<int>(input_shape_[0]);
    input_dims.d[1] = static_cast<int>(input_shape_[1]);
    input_dims.d[2] = static_cast<int>(input_shape_[2]);
    input_dims.d[3] = static_cast<int>(input_shape_[3]);
  } else {
    input_dims.nbDims = static_cast<int>(input_shape_.size());
    for (size_t i = 0; i < input_shape_.size(); ++i) {
      input_dims.d[i] = static_cast<int>(input_shape_[i]);
    }
  }

  if (!context_->setInputShape(input_name_.c_str(), input_dims)) {
    throw std::runtime_error("TensorRTBackend::infer setInputShape failed");
  }

  if (!context_->setInputTensorAddress(input_name_.c_str(), input_device_)) {
    throw std::runtime_error("TensorRTBackend::infer setInputTensorAddress failed");
  }
  for (size_t i = 0; i < output_names_.size(); ++i) {
    if (!context_->setOutputTensorAddress(output_names_[i].c_str(), output_devices_[i])) {
      throw std::runtime_error("TensorRTBackend::infer setOutputTensorAddress failed");
    }
  }

  if (!context_->enqueueV3(cuda_stream_)) {
    throw std::runtime_error("TensorRTBackend::infer enqueueV3 failed");
  }

  std::vector<TensorOutput> results;
  results.reserve(output_names_.size());
  for (size_t i = 0; i < output_names_.size(); ++i) {
    TensorOutput out;
    out.info.name = output_names_[i];
    out.info.shape = output_shapes_[i];
    out.info.dtype = TensorInfo::DType::FLOAT32;
    out.host_data.resize(output_num_elements_[i], 0.0F);

    const size_t bytes = output_num_elements_[i] * sizeof(float);
    if (cudaMemcpyAsync(out.host_data.data(), output_devices_[i], bytes,
                        cudaMemcpyDeviceToHost, cuda_stream_) != cudaSuccess) {
      throw std::runtime_error("TensorRTBackend::infer D2H copy failed");
    }
    results.push_back(std::move(out));
  }

  if (cudaStreamSynchronize(cuda_stream_) != cudaSuccess) {
    throw std::runtime_error("TensorRTBackend::infer stream sync failed");
  }
  return results;
}

void TensorRTBackend::warmup(int iterations) {
  if (!loaded_ || iterations <= 0) return;

  TensorInput dummy;
  dummy.info.name = input_name_;
  dummy.info.shape = input_shape_;
  dummy.info.dtype = TensorInfo::DType::FLOAT32;
  dummy.host_data.assign(elementCount(input_shape_), 0.0F);

  for (int i = 0; i < iterations; ++i) {
    (void)infer(dummy);
  }
  FYT_INFO("armor_detector_nn", "TensorRTBackend: warmup complete ({} iters)", iterations);
}

BackendInfo TensorRTBackend::info() const {
  return info_;
}

void TensorRTBackend::validateModelIO(const BackendConfig& config) {
  if (input_shape_.size() != 4) {
    throw std::runtime_error("TensorRTBackend: input must be 4D NCHW");
  }
  if (!config.input_name.empty() && config.input_name != input_name_) {
    FYT_WARN("armor_detector_nn",
             "TensorRT input name mismatch. Config: {}, engine: {}",
             config.input_name.c_str(), input_name_.c_str());
  }
  if (!config.output_names.empty()) {
    for (const auto& out : config.output_names) {
      if (binding_indices_.find(out) == binding_indices_.end()) {
        throw std::runtime_error("TensorRTBackend: configured output not found: " + out);
      }
    }
  }
}

void TensorRTBackend::updateBatchCapability() {
  info_.min_batch_size = 1;
  info_.max_batch_size = 1;
  info_.dynamic_batch = false;

  const auto profile_dims_min = engine_->getProfileShape(
    input_name_.c_str(), 0, nvinfer1::OptProfileSelector::kMIN);
  const auto profile_dims_max = engine_->getProfileShape(
    input_name_.c_str(), 0, nvinfer1::OptProfileSelector::kMAX);

  if (profile_dims_min.nbDims > 0 && profile_dims_max.nbDims > 0) {
    info_.min_batch_size = std::max(1, static_cast<int>(profile_dims_min.d[0]));
    info_.max_batch_size = std::max(1, static_cast<int>(profile_dims_max.d[0]));
    info_.dynamic_batch = info_.max_batch_size > info_.min_batch_size;
  }
}

size_t TensorRTBackend::elementCount(const std::vector<int64_t>& shape) const {
  if (shape.empty()) return 0;
  size_t n = 1;
  for (auto d : shape) {
    if (d < 0) {
      throw std::runtime_error("TensorRTBackend: dynamic dim unresolved");
    }
    n *= static_cast<size_t>(d);
  }
  return n;
}

}  // namespace fyt::auto_aim
