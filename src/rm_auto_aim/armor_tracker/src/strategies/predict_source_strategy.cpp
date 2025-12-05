// Implementation of PredictSourceStrategy
#include "armor_tracker/strategies/predict_source_strategy.hpp"
#include <algorithm>

namespace fyt::auto_aim {

PredictSourceStrategy::PredictSourceStrategy(int max_predict_frames)
  : max_predict_frames_(max_predict_frames) {}

std::string PredictSourceStrategy::getName() const { return "PredictSource"; }

bool PredictSourceStrategy::canHandle(ArmorSourceType source) const {
  return source == ArmorSourceType::PREDICT;
}

double PredictSourceStrategy::computeWeight(
  const ArmorObservation& observation,
  const TrackedArmorState* current_state) const {
  if (current_state) {
    double decay = 1.0 - static_cast<double>(current_state->time_since_update) / max_predict_frames_;
    return std::max(0.0, decay);
  }
  return 0.5;
}

bool PredictSourceStrategy::shouldUpdate(
  const ArmorObservation& observation,
  const TrackedArmorState& current_state) const {
  return current_state.time_since_update < max_predict_frames_;
}

void PredictSourceStrategy::postprocess(TrackedArmorState& state) const {
  state.source_type = ArmorSourceType::PREDICT;
  state.confidence *= 0.95;
}

}  // namespace fyt::auto_aim
