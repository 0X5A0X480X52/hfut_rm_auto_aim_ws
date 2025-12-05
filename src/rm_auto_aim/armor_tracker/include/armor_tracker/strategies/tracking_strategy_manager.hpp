// Copyright (C) FYT Vision Group. All rights reserved.
//
#ifndef ARMOR_TRACKER__STRATEGIES__TRACKING_STRATEGY_MANAGER_HPP_
#define ARMOR_TRACKER__STRATEGIES__TRACKING_STRATEGY_MANAGER_HPP_

#include <memory>
#include <vector>
#include "armor_tracker/tracking_strategy.hpp"

namespace fyt::auto_aim {

class TrackingStrategyManager {
public:
  TrackingStrategyManager();

  void registerStrategy(std::shared_ptr<ITrackingStrategy> strategy);
  std::shared_ptr<ITrackingStrategy> getStrategy(ArmorSourceType source) const;
  const std::vector<std::shared_ptr<ITrackingStrategy>>& getAllStrategies() const;
  void clearStrategies();

private:
  std::vector<std::shared_ptr<ITrackingStrategy>> strategies_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__STRATEGIES__TRACKING_STRATEGY_MANAGER_HPP_
