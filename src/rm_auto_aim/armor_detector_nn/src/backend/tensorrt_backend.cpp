#include "armor_detector_nn/backend/tensorrt_backend.hpp"

#include <stdexcept>

namespace fyt::auto_aim {

TensorRTBackend::TensorRTBackend() {
  info_.backend_name = "tensorrt";
  info_.precision = "unknown";
  info_.min_batch_size = 1;
  info_.max_batch_size = 1;
  info_.dynamic_batch = false;
}

TensorRTBackend::~TensorRTBackend() = default;

void TensorRTBackend::load(const BackendConfig& config) {
  (void)config;
  throw std::runtime_error(
    "TensorRT backend is reserved but not implemented in this phase");
}

std::vector<TensorOutput> TensorRTBackend::infer(const TensorInput& input) {
  (void)input;
  throw std::runtime_error(
    "TensorRT backend infer() called before implementation");
}

void TensorRTBackend::warmup(int iterations) {
  (void)iterations;
}

BackendInfo TensorRTBackend::info() const {
  return info_;
}

}  // namespace fyt::auto_aim
