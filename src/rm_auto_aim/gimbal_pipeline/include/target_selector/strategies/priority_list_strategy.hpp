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

#ifndef TARGET_SELECTOR__STRATEGIES__PRIORITY_LIST_STRATEGY_HPP_
#define TARGET_SELECTOR__STRATEGIES__PRIORITY_LIST_STRATEGY_HPP_

#include "target_selector/selection_strategy.hpp"

namespace fyt::auto_aim {

class PriorityListStrategy : public SelectionStrategy {
 public:
  PriorityListStrategy() = default;
  ~PriorityListStrategy() override = default;

  std::optional<SelectionResult> selectTarget(
      const TrackedRobots& robots,
      const SelectionConfig& config) override;

  std::string getName() const override {
    return "PriorityList";
  }

  std::string getDescription() const override {
    return "Selects target by configured robot-id priority order";
  }

 private:
  const TrackedRobot* findRobotById(
      const std::vector<const TrackedRobot*>& candidates,
      const std::string& robot_id) const;

  const TrackedRobot* findMinYawDeviationRobot(
      const std::vector<const TrackedRobot*>& candidates,
      double reference_yaw) const;
};

}  // namespace fyt::auto_aim

#endif  // TARGET_SELECTOR__STRATEGIES__PRIORITY_LIST_STRATEGY_HPP_
