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

// Implementation of PredictSourceStrategy
#include "armor_tracker/strategies/predict_source_strategy.hpp"
#include <algorithm>

namespace fyt::auto_aim
{

PredictSourceStrategy::PredictSourceStrategy(int max_predict_frames)
  : max_predict_frames_(max_predict_frames) {}

std::string PredictSourceStrategy::getName() const {return "PredictSource";}

bool PredictSourceStrategy::canHandle(ArmorSourceType source) const
{
  return source == ArmorSourceType::PREDICT;
}

double PredictSourceStrategy::computeWeight(
  const ArmorObservation & observation,
  const TrackedArmorState * current_state) const
{
  if (current_state) {
    double decay = 1.0 - static_cast<double>(current_state->time_since_update) /
      max_predict_frames_;
    return std::max(0.0, decay);
  }
  return 0.5;
}

bool PredictSourceStrategy::shouldUpdate(
  const ArmorObservation & observation,
  const TrackedArmorState & current_state) const
{
  return current_state.time_since_update < max_predict_frames_;
}

void PredictSourceStrategy::postprocess(TrackedArmorState & state) const
{
  state.source_type = ArmorSourceType::PREDICT;
  state.confidence *= 0.95;
}

}  // namespace fyt::auto_aim
