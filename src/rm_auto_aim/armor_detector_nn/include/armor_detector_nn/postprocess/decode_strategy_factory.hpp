#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/postprocess/decode_strategy.hpp"

namespace fyt::auto_aim {

class DecodeStrategyFactory {
public:
  static std::unique_ptr<IDecodeStrategy> create(const PostprocessConfig& config);

  using FactoryFn = std::function<std::unique_ptr<IDecodeStrategy>()>;
  static void registerStrategy(const std::string& name, FactoryFn fn);

private:
  static std::unordered_map<std::string, FactoryFn>& registry();
};

}  // namespace fyt::auto_aim

#endif
