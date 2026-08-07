#include "armor_detector_nn/backend/inference_backend_factory.hpp"

#include <exception>
#include <memory>

#include "armor_detector_nn/backend/openvino_backend.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

std::unique_ptr<IInferenceBackend> InferenceBackendFactory::create(const BackendConfig &config) {
  try {
    auto backend = std::make_unique<OpenVINOBackend>();
    backend->load(config);
    return backend;
  } catch (const std::exception &e) {
    FYT_ERROR("armor_detector_nn", "OpenVINO backend load failed: {}", e.what());
    return nullptr;
  }
}

}  // namespace fyt::auto_aim
