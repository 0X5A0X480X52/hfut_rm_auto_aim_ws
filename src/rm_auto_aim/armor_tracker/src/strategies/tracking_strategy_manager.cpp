// Implementation of TrackingStrategyManager methods
#include "armor_tracker/strategies/tracking_strategy_manager.hpp"
#include "armor_tracker/strategies/detect_source_strategy.hpp"
#include "armor_tracker/strategies/estimate_source_strategy.hpp"
#include "armor_tracker/strategies/predict_source_strategy.hpp"

namespace fyt::auto_aim {

TrackingStrategyManager::TrackingStrategyManager() {
  registerStrategy(std::make_shared<DetectSourceStrategy>());
  registerStrategy(std::make_shared<EstimateSourceStrategy>());
  registerStrategy(std::make_shared<PredictSourceStrategy>());
}

void TrackingStrategyManager::registerStrategy(std::shared_ptr<ITrackingStrategy> strategy) {
  strategies_.push_back(strategy);
}

std::shared_ptr<ITrackingStrategy> TrackingStrategyManager::getStrategy(ArmorSourceType source) const {
  for (const auto& strategy : strategies_) {
    if (strategy->canHandle(source)) {
      return strategy;
    }
  }
  return nullptr;
}

const std::vector<std::shared_ptr<ITrackingStrategy>>& TrackingStrategyManager::getAllStrategies() const {
  return strategies_;
}

void TrackingStrategyManager::clearStrategies() {
  strategies_.clear();
}

}  // namespace fyt::auto_aim
