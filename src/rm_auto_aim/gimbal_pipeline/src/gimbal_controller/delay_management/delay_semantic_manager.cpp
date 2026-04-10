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

#include "gimbal_controller/delay_management/delay_semantic_manager.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal_controller::delay_management
{

double DelaySemanticManager::clampNonNegative(double value)
{
  return std::max(value, 0.0);
}

double DelaySemanticManager::computeProcessingDelay(
  const DelayRawInputs & raw,
  bool include_processing_delay) const
{
  if (!include_processing_delay) {
    return 0.0;
  }

  const double max_processing = clampNonNegative(raw.max_processing_delay_s);
  const double processing = (raw.current_time - raw.observation_stamp).seconds();
  return std::clamp(processing, 0.0, max_processing);
}

double DelaySemanticManager::computePredictionTime(
  const DelayRawInputs & raw,
  double flight_time_s,
  double max_prediction_time_s) const
{
  const double processing = computeProcessingDelay(raw, true);
  const double flight_time = clampNonNegative(flight_time_s);
  const double prediction_extra = clampNonNegative(raw.prediction_extra_s);

  double total = processing + flight_time + prediction_extra;
  if (max_prediction_time_s > 1e-9) {
    total = std::min(total, max_prediction_time_s);
  }

  return total;
}

MpcDelayResult DelaySemanticManager::computeMpcDelay(
  const DelayRawInputs & raw,
  double dt_s,
  bool include_processing_delay,
  bool use_delayed_b,
  bool allow_fire_control_compensation) const
{
  MpcDelayResult result;

  const double processing = computeProcessingDelay(raw, include_processing_delay);
  const double prediction_extra = clampNonNegative(raw.prediction_extra_s);
  const double control_latency = clampNonNegative(raw.control_latency_s);

  result.processing_delay_s = processing;
  result.base_reference_delay_s = processing + prediction_extra;
  result.control_latency_s = control_latency;
  result.uses_delayed_b = use_delayed_b;

  if (dt_s > 1e-9) {
    result.control_delay_steps = std::max(0, static_cast<int>(std::round(control_latency / dt_s)));
  }

  result.double_compensation_risk =
    use_delayed_b && allow_fire_control_compensation && control_latency > 1e-6;

  result.fire_control_compensation_s =
    (allow_fire_control_compensation && !use_delayed_b) ? control_latency : 0.0;

  return result;
}

}  // namespace gimbal_controller::delay_management
