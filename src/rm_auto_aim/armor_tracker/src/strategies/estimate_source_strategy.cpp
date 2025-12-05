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

// Implementation of EstimateSourceStrategy
#include "armor_tracker/strategies/estimate_source_strategy.hpp"
#include <algorithm>

namespace fyt::auto_aim
{

EstimateSourceStrategy::EstimateSourceStrategy(double weight_factor)
  : weight_factor_(weight_factor) {}

std::string EstimateSourceStrategy::getName() const {return "EstimateSource";}

bool EstimateSourceStrategy::canHandle(ArmorSourceType source) const
{
  return source == ArmorSourceType::ESTIMATE;
}

double EstimateSourceStrategy::computeWeight(
  const ArmorObservation & observation,
  const TrackedArmorState * current_state) const
{
  double weight = observation.confidence * weight_factor_;
  return std::max(0.0, std::min(1.0, weight));
}

bool EstimateSourceStrategy::shouldUpdate(
  const ArmorObservation & observation,
  const TrackedArmorState & current_state) const
{
  return current_state.tracking_state == TrackingState::TEMP_LOST ||
         current_state.tracking_state == TrackingState::LOST ||
         current_state.time_since_update > 3;
}

void EstimateSourceStrategy::postprocess(TrackedArmorState & state) const
{
  state.source_type = ArmorSourceType::ESTIMATE;
}

}  // namespace fyt::auto_aim
