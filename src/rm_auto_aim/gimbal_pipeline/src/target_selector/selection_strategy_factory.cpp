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

#include <rclcpp/rclcpp.hpp>

#include "target_selector/selection_strategy_factory.hpp"

#include "target_selector/strategies/attack_outpost_first_strategy.hpp"
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"
#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"

namespace fyt::auto_aim {

SelectionStrategyPtr SelectionStrategyFactory::create(
    const std::string& name,
    const std::atomic_bool* attack_outpost_first) {
  if (name == "min_yaw_deviation") {
    return std::make_unique<MinYawDeviationStrategy>();
  }
  if (name == "priority_list") {
    return std::make_unique<PriorityListStrategy>();
  }
  if (name == "sticky_min_yaw_deviation") {
    return std::make_unique<StickyMinYawDeviationStrategy>();
  }
  if (name == "attack_outpost_first") {
    return std::make_unique<AttackOutpostFirstStrategy>(attack_outpost_first);
  }
  RCLCPP_WARN(rclcpp::get_logger("target_selector"),
              "Unknown selector strategy '%s', using min_yaw_deviation",
              name.c_str());
  return std::make_unique<MinYawDeviationStrategy>();
}

}  // namespace fyt::auto_aim
