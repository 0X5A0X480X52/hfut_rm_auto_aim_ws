// Implementation of EstimateSourceStrategy
#include "armor_tracker/strategies/estimate_source_strategy.hpp"
#include <algorithm>

namespace fyt::auto_aim {

EstimateSourceStrategy::EstimateSourceStrategy(double weight_factor)
  : weight_factor_(weight_factor) {}

std::string EstimateSourceStrategy::getName() const { return "EstimateSource"; }

bool EstimateSourceStrategy::canHandle(ArmorSourceType source) const {
  return source == ArmorSourceType::ESTIMATE;
}

double EstimateSourceStrategy::computeWeight(
  const ArmorObservation& observation,
  const TrackedArmorState* current_state) const {
  double weight = observation.confidence * weight_factor_;
  return std::max(0.0, std::min(1.0, weight));
}

bool EstimateSourceStrategy::shouldUpdate(
  const ArmorObservation& observation,
  const TrackedArmorState& current_state) const {
  return current_state.tracking_state == TrackingState::TEMP_LOST ||
         current_state.tracking_state == TrackingState::LOST ||
         current_state.time_since_update > 3;
}

void EstimateSourceStrategy::postprocess(TrackedArmorState& state) const {
  state.source_type = ArmorSourceType::ESTIMATE;
}

}  // namespace fyt::auto_aim
