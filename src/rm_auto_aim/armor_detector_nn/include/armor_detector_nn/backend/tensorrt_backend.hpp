#ifndef ARMOR_DETECTOR_NN_TENSORRT_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_TENSORRT_BACKEND_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda_runtime_api.h>

#include "armor_detector_nn/backend/inference_backend.hpp"

namespace nvinfer1 {
class ILogger;
class IRuntime;
class ICudaEngine;
class IExecutionContext;
}  // namespace nvinfer1

namespace fyt::auto_aim {

class TensorRTBackend : public IInferenceBackend {
public:
  TensorRTBackend();
  ~TensorRTBackend() override;

  void load(const BackendConfig& config) override;
  std::vector<TensorOutput> infer(const TensorInput& input) override;
  void warmup(int iterations) override;
  BackendInfo info() const override;

private:
  void validateModelIO(const BackendConfig& config);
  void updateBatchCapability();
  size_t elementCount(const std::vector<int64_t>& shape) const;

  std::unique_ptr<nvinfer1::ILogger> logger_;
  nvinfer1::IRuntime* runtime_{nullptr};
  nvinfer1::ICudaEngine* engine_{nullptr};
  nvinfer1::IExecutionContext* context_{nullptr};

  void* input_device_{nullptr};
  std::vector<void*> output_devices_;
  std::vector<size_t> output_num_elements_;

  cudaStream_t cuda_stream_{nullptr};
  bool pinned_input_{false};
  float* pinned_input_host_{nullptr};

  std::string input_name_;
  std::vector<std::string> output_names_;
  std::unordered_map<std::string, int> binding_indices_;
  std::vector<int64_t> input_shape_;
  std::vector<std::vector<int64_t>> output_shapes_;

  Precision precision_{Precision::FP32};
  BackendInfo info_;
  bool loaded_{false};

  std::mutex infer_mutex_;
};

}  // namespace fyt::auto_aim

#endif
