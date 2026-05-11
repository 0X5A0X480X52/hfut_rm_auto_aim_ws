#include "armor_detector_nn/postprocess/decode_strategy_factory.hpp"

#include "armor_detector_nn/postprocess/robotpilots_landmark_decode_strategy.hpp"
#include "armor_detector_nn/postprocess/ultralytics_pose_decode_strategy.hpp"

namespace fyt::auto_aim {

std::unique_ptr<IDecodeStrategy> DecodeStrategyFactory::create(const PostprocessConfig& config) {
  auto& reg = registry();
  auto it = reg.find(config.strategy);
  if (it != reg.end()) {
    return it->second();
  }

  if (config.strategy == "ultralytics_pose") {
    return std::make_unique<UltralyticsPoseDecodeStrategy>();
  }
  if (config.strategy == "robotpilots_landmark") {
    return std::make_unique<RobotPilotsLandmarkDecodeStrategy>();
  }

  return nullptr;
}

void DecodeStrategyFactory::registerStrategy(const std::string& name, FactoryFn fn) {
  registry()[name] = std::move(fn);
}

std::unordered_map<std::string, DecodeStrategyFactory::FactoryFn>&
DecodeStrategyFactory::registry() {
  static std::unordered_map<std::string, FactoryFn> reg;
  return reg;
}

}  // namespace fyt::auto_aim
