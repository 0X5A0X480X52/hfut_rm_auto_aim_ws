#ifndef ARMOR_DETECTOR_NN_INFERENCE_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_INFERENCE_BACKEND_HPP_

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

class IInferenceBackend {
public:
  virtual ~IInferenceBackend() = default;

  virtual void load(const BackendConfig& config) = 0;
  virtual std::vector<TensorOutput> infer(const TensorInput& input) = 0;
  virtual void warmup(int iterations) = 0;
  virtual BackendInfo info() const = 0;

protected:
  IInferenceBackend() = default;
  IInferenceBackend(const IInferenceBackend&) = delete;
  IInferenceBackend& operator=(const IInferenceBackend&) = delete;
  IInferenceBackend(IInferenceBackend&&) = default;
  IInferenceBackend& operator=(IInferenceBackend&&) = default;
};

}  // namespace fyt::auto_aim

#endif
