// Copyright (C) FYT Vision Group. All rights reserved.
//
#ifndef ARMOR_TRACKER__STRATEGIES__ESTIMATE_SOURCE_STRATEGY_HPP_
#define ARMOR_TRACKER__STRATEGIES__ESTIMATE_SOURCE_STRATEGY_HPP_

#include <string>
#include "armor_tracker/tracking_strategy.hpp"

namespace fyt::auto_aim {

class EstimateSourceStrategy : public ITrackingStrategy {
public:
  explicit EstimateSourceStrategy(double weight_factor = 0.8);

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
  double weight_factor_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__STRATEGIES__ESTIMATE_SOURCE_STRATEGY_HPP_
