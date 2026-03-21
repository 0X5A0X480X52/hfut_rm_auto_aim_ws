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

#ifndef TARGET_SELECTOR__STRATEGIES__STICKY_MIN_YAW_DEVIATION_STRATEGY_HPP_
#define TARGET_SELECTOR__STRATEGIES__STICKY_MIN_YAW_DEVIATION_STRATEGY_HPP_

#include <string>

#include "target_selector/selection_strategy.hpp"
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"

namespace fyt::auto_aim {

class StickyMinYawDeviationStrategy : public SelectionStrategy {
 public:
  StickyMinYawDeviationStrategy() = default;
  ~StickyMinYawDeviationStrategy() override = default;

  std::optional<SelectionResult> selectTarget(
      const TrackedRobots& robots,
      const SelectionConfig& config) override;

  std::string getName() const override {
    return "StickyMinYawDeviation";
  }

  std::string getDescription() const override {
    return "Uses min-yaw selection, then promotes stable ID to preferred target";
  }

 private:
  const TrackedRobot* findRobotById(
      const std::vector<const TrackedRobot*>& candidates,
      const std::string& robot_id) const;

  MinYawDeviationStrategy base_strategy_;
  std::string preferred_target_id_;
  std::string lock_candidate_id_;
  int lock_counter_{0};
  int preferred_miss_counter_{0};
};

}  // namespace fyt::auto_aim

#endif  // TARGET_SELECTOR__STRATEGIES__STICKY_MIN_YAW_DEVIATION_STRATEGY_HPP_
