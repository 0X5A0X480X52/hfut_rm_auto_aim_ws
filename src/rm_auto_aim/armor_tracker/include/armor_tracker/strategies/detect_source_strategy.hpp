// Copyright (C) FYT Vision Group. All rights reserved.
//
#ifndef ARMOR_TRACKER__STRATEGIES__DETECT_SOURCE_STRATEGY_HPP_
#define ARMOR_TRACKER__STRATEGIES__DETECT_SOURCE_STRATEGY_HPP_

#include <string>
#include "armor_tracker/tracking_strategy.hpp"

namespace fyt::auto_aim {

class DetectSourceStrategy : public ITrackingStrategy {
public:
  explicit DetectSourceStrategy(double confidence_threshold = 0.5);

  std::string getName() const override;
  bool canHandle(ArmorSourceType source) const override;

  double computeWeight(
    const ArmorObservation& observation,
    const TrackedArmorState* current_state = nullptr) const override;

  bool shouldUpdate(
    const ArmorObservation& observation,
    const TrackedArmorState& current_state) const override;

  void postprocess(TrackedArmorState& state) const override;

private:
  double confidence_threshold_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__STRATEGIES__DETECT_SOURCE_STRATEGY_HPP_
