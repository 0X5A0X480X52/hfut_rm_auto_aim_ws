// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Implementation of DetectSourceStrategy
#include "armor_tracker/strategies/detect_source_strategy.hpp"
#include <cmath>
#include <algorithm>

namespace fyt::auto_aim {

DetectSourceStrategy::DetectSourceStrategy(double confidence_threshold)
  : confidence_threshold_(confidence_threshold) {}

std::string DetectSourceStrategy::getName() const { return "DetectSource"; }

bool DetectSourceStrategy::canHandle(ArmorSourceType source) const {
  return source == ArmorSourceType::DETECT;
}

double DetectSourceStrategy::computeWeight(
  const ArmorObservation & observation,
  const TrackedArmorState * current_state) const {
  double weight = observation.confidence;
  if (current_state && current_state->tracking_state == TrackingState::TRACKING) {
    double distance = (observation.position - current_state->position).norm();
    weight *= std::exp(-distance * distance / 0.5);
  }
  return std::max(0.0, std::min(1.0, weight));
}

bool DetectSourceStrategy::shouldUpdate(
  const ArmorObservation & observation,
  const TrackedArmorState & current_state) const {
  return observation.confidence >= confidence_threshold_;
}

void DetectSourceStrategy::postprocess(TrackedArmorState & state) const {
  state.source_type = ArmorSourceType::DETECT;
}

}  // namespace fyt::auto_aim
