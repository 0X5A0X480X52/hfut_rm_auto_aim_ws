#ifndef ARMOR_DETECTOR_NN_INFERENCE_BACKEND_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_INFERENCE_BACKEND_FACTORY_HPP_

#include <memory>

#include "armor_detector_nn/backend/inference_backend.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

class InferenceBackendFactory {
public:
  static std::unique_ptr<IInferenceBackend> create(const BackendConfig& config);
  static bool isAvailable(BackendType type);

private:
  static std::unique_ptr<IInferenceBackend> createOne(BackendType type,
                                                       const BackendConfig& config);
};

}  // namespace fyt::auto_aim

#endif
