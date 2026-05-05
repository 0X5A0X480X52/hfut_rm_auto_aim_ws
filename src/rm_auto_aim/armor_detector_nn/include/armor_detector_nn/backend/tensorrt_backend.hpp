#ifndef ARMOR_DETECTOR_NN_TENSORRT_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_TENSORRT_BACKEND_HPP_

#include "armor_detector_nn/backend/inference_backend.hpp"

namespace fyt::auto_aim {

// TensorRT backend interface skeleton.
// Implementation is intentionally deferred; this class exists to keep
// backend selection and factory paths stable across phases.
class TensorRTBackend : public IInferenceBackend {
public:
  TensorRTBackend();
  ~TensorRTBackend() override;

  void load(const BackendConfig& config) override;
  std::vector<TensorOutput> infer(const TensorInput& input) override;
  void warmup(int iterations) override;
  BackendInfo info() const override;

private:
  BackendInfo info_;
  bool loaded_{false};
};

}  // namespace fyt::auto_aim

#endif
