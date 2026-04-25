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

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROL_ORCHESTRATOR_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROL_ORCHESTRATOR_HPP_

#include <memory>

#include "gimbal_controller/gimbal_control_strategy.hpp"

namespace gimbal_controller
{

class FireAdviceEngine;
class FireAdvisor;

struct FireDecisionConfig
{
  double prediction_delay_s{0.0};
  double control_latency_s{0.0};
  double trigger_to_muzzle_s{0.0};
  double max_processing_delay_s{0.5};
  bool include_processing_delay{true};
  bool include_control_latency_in_target_prediction{false};
};

class GimbalControlOrchestrator
{
public:
  void setFireModules(
    std::shared_ptr<FireAdviceEngine> fire_advice_engine,
    std::shared_ptr<FireAdvisor> fire_advisor);

  void setFireDecisionConfig(const FireDecisionConfig & config)
  {
    fire_cfg_ = config;
  }

  rm_interfaces::msg::GimbalCmd buildIdleCmd(const GimbalControlContext & context) const;

  rm_interfaces::msg::GimbalCmd finalize(
    const GimbalControlContext & context,
    const rm_interfaces::msg::GimbalCmd & control_cmd) const;

private:
  int8_t decideMode(const GimbalControlContext & context) const;

  bool evaluateFireAdvice(
    const GimbalControlContext & context,
    const rm_interfaces::msg::GimbalCmd & cmd,
    double distance) const;

  std::shared_ptr<FireAdviceEngine> fire_advice_engine_;
  std::shared_ptr<FireAdvisor> fire_advisor_;
  FireDecisionConfig fire_cfg_;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROL_ORCHESTRATOR_HPP_
