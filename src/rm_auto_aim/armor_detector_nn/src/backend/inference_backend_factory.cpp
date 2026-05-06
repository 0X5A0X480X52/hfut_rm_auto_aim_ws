#include "armor_detector_nn/backend/inference_backend_factory.hpp"

#include <stdexcept>

#include "rm_utils/logger/log.hpp"

#ifdef ARMOR_DETECTOR_NN_HAS_ONNX
#include "armor_detector_nn/backend/onnxruntime_backend.hpp"
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_OPENVINO
#include "armor_detector_nn/backend/openvino_backend.hpp"
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_TENSORRT
#include "armor_detector_nn/backend/tensorrt_backend.hpp"
#endif

namespace fyt::auto_aim {

std::unique_ptr<IInferenceBackend> InferenceBackendFactory::create(const BackendConfig& config) {
  auto primary = createOne(config.type, config);
  if (primary) return primary;

  if (!config.allow_fallback) {
    FYT_ERROR("armor_detector_nn",
              "Primary backend %s failed and fallback is disabled",
              backendTypeToString(config.type).c_str());
    return nullptr;
  }

  FYT_WARN("armor_detector_nn",
           "Primary backend %s failed, falling back to %s",
           backendTypeToString(config.type).c_str(),
           backendTypeToString(config.fallback_type).c_str());
  return createOne(config.fallback_type, config);
}

bool InferenceBackendFactory::isAvailable(BackendType type) {
  switch (type) {
#ifdef ARMOR_DETECTOR_NN_HAS_ONNX
    case BackendType::ONNX_RUNTIME: return true;
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_OPENVINO
    case BackendType::OPENVINO: return true;
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_TENSORRT
    case BackendType::TENSORRT: return true;
#endif
    default: return false;
  }
}

std::unique_ptr<IInferenceBackend> InferenceBackendFactory::createOne(
    BackendType type, const BackendConfig& config) {
  switch (type) {
#ifdef ARMOR_DETECTOR_NN_HAS_ONNX
    case BackendType::ONNX_RUNTIME: {
      try {
        auto backend = std::make_unique<OnnxRuntimeBackend>();
        backend->load(config);
        return backend;
      } catch (const std::exception& e) {
        FYT_ERROR("armor_detector_nn", "ONNX Runtime backend load failed: %s", e.what());
        return nullptr;
      }
    }
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_OPENVINO
    case BackendType::OPENVINO: {
      try {
        auto backend = std::make_unique<OpenVINOBackend>();
        backend->load(config);
        return backend;
      } catch (const std::exception& e) {
        FYT_ERROR("armor_detector_nn", "OpenVINO backend load failed: %s", e.what());
        return nullptr;
      }
    }
#endif
#ifdef ARMOR_DETECTOR_NN_HAS_TENSORRT
    case BackendType::TENSORRT: {
      try {
        auto backend = std::make_unique<TensorRTBackend>();
        backend->load(config);
        return backend;
      } catch (const std::exception& e) {
        FYT_ERROR("armor_detector_nn", "TensorRT backend load failed: {}", e.what());
        return nullptr;
      }
    }
#endif
    default:
      FYT_WARN("armor_detector_nn",
               "Backend {} not compiled in or unsupported",
               backendTypeToString(type).c_str());
      return nullptr;
  }
}

}  // namespace fyt::auto_aim
