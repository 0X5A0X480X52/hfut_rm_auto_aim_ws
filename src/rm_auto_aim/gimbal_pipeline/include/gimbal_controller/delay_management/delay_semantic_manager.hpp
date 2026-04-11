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

#ifndef GIMBAL_CONTROLLER__DELAY_MANAGEMENT__DELAY_SEMANTIC_MANAGER_HPP_
#define GIMBAL_CONTROLLER__DELAY_MANAGEMENT__DELAY_SEMANTIC_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace gimbal_controller::delay_management
{

struct DelayRawInputs
{
  rclcpp::Time current_time;
  rclcpp::Time observation_stamp;

  double prediction_extra_s{0.0};
  double control_latency_s{0.0};
  double trigger_to_muzzle_s{0.0};
  double max_processing_delay_s{0.5};
};

struct MpcDelayResult
{
  double processing_delay_s{0.0};
  double base_reference_delay_s{0.0};  // processing_delay + prediction_delay
  double control_latency_s{0.0};
  double fire_control_compensation_s{0.0};  // trigger_to_muzzle

  int control_delay_steps{0};
  bool uses_delayed_b{false};
  bool double_compensation_risk{false};
};

struct FireTimelineResult
{
  double processing_delay_s{0.0};
  double prediction_delay_s{0.0};
  double control_latency_s{0.0};
  double trigger_to_muzzle_s{0.0};

  // 子弹出膛相对当前控制时刻的偏移
  double muzzle_delay_s{0.0};

  // 候选目标预测基准时间: d_proc + d_pred (+d_ctrl 可选) + d_trig
  double target_prediction_base_s{0.0};
};

class DelaySemanticManager
{
public:
  DelaySemanticManager() = default;
  ~DelaySemanticManager() = default;

  double computeProcessingDelay(
    const DelayRawInputs & raw,
    bool include_processing_delay = true) const;

  double computePredictionTime(
    const DelayRawInputs & raw,
    double flight_time_s,
    double max_prediction_time_s) const;

  FireTimelineResult computeFireTimeline(
    const DelayRawInputs & raw,
    bool include_processing_delay = true,
    bool include_control_latency_in_target_prediction = false) const;

  MpcDelayResult computeMpcDelay(
    const DelayRawInputs & raw,
    double dt_s,
    bool include_processing_delay,
    bool use_delayed_b,
    bool allow_muzzle_compensation) const;

private:
  static double clampNonNegative(double value);
};

}  // namespace gimbal_controller::delay_management

#endif  // GIMBAL_CONTROLLER__DELAY_MANAGEMENT__DELAY_SEMANTIC_MANAGER_HPP_
