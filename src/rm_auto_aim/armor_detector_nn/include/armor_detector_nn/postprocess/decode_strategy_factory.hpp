#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_

#include <memory>

#include "armor_detector_nn/postprocess/decode_strategy.hpp"

namespace fyt::auto_aim {

class DecodeStrategyFactory {
public:
  static std::unique_ptr<IDecodeStrategy> create();
};

}  // namespace fyt::auto_aim

#endif
