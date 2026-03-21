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

#include "target_selector/strategies/priority_list_strategy.hpp"

#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace fyt::auto_aim {

std::optional<SelectionResult> PriorityListStrategy::selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) {
  auto logger = rclcpp::get_logger("target_selector");

  auto candidates = filterCandidates(robots, config);
  if (candidates.empty()) {
    return std::nullopt;
  }

  for (const auto& robot_id : config.priority_robot_ids) {
    const TrackedRobot* preferred = findRobotById(candidates, robot_id);
    if (preferred != nullptr) {
      return SelectionResult(
          preferred->robot_id,
          preferred->confidence,
          calculateYawDeviation(*preferred, config.reference_yaw),
          calculateDistanceToRobot(*preferred));
    }
  }

  const TrackedRobot* fallback =
      findMinYawDeviationRobot(candidates, config.reference_yaw);
  if (fallback == nullptr) {
    return std::nullopt;
  }

  RCLCPP_DEBUG(logger,
               "Priority list has no match in current candidates, fallback to min yaw: %s",
               fallback->robot_id.c_str());
  return SelectionResult(
      fallback->robot_id,
      fallback->confidence,
      calculateYawDeviation(*fallback, config.reference_yaw),
      calculateDistanceToRobot(*fallback));
}

const PriorityListStrategy::TrackedRobot* PriorityListStrategy::findRobotById(
    const std::vector<const TrackedRobot*>& candidates,
    const std::string& robot_id) const {
  for (const auto* robot : candidates) {
    if (robot->robot_id == robot_id) {
      return robot;
    }
  }
  return nullptr;
}

const PriorityListStrategy::TrackedRobot* PriorityListStrategy::findMinYawDeviationRobot(
    const std::vector<const TrackedRobot*>& candidates,
    double reference_yaw) const {
  const TrackedRobot* best = nullptr;
  double min_deviation = std::numeric_limits<double>::max();

  for (const auto* robot : candidates) {
    const double deviation = calculateYawDeviation(*robot, reference_yaw);
    if (deviation < min_deviation) {
      min_deviation = deviation;
      best = robot;
    }
  }

  return best;
}

}  // namespace fyt::auto_aim
