#include "armor_detector_nn/postprocess/decode_strategy_factory.hpp"

#include <memory>

#include "armor_detector_nn/postprocess/robotpilots_landmark_decode_strategy.hpp"

namespace fyt::auto_aim {

std::unique_ptr<IDecodeStrategy> DecodeStrategyFactory::create() {
  return std::make_unique<RobotPilotsLandmarkDecodeStrategy>();
}

}  // namespace fyt::auto_aim
