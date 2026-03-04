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

#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace fyt::auto_aim {

std::optional<SelectionResult> MinYawDeviationStrategy::selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) {
  auto logger = rclcpp::get_logger("target_selector");
  RCLCPP_DEBUG(logger, "MinYawDeviationStrategy::selectTarget called with %zu robots", robots.robots.size());
  
  // Filter candidates based on basic criteria
  auto candidates = filterCandidates(robots, config);
  
  RCLCPP_DEBUG(logger, "After filtering, %zu candidates remain", candidates.size());
  
  if (candidates.empty()) {
    RCLCPP_DEBUG(logger, "No candidates after filtering");
    return std::nullopt;
  }
  
  // Find the robot with minimum yaw deviation
  const TrackedRobot* best_candidate = findMinYawDeviationRobot(
    candidates, config.reference_yaw);
  
  RCLCPP_DEBUG(logger, "Best candidate: %s", best_candidate ? best_candidate->robot_id.c_str() : "nullptr");
  
  if (best_candidate == nullptr) {
    RCLCPP_DEBUG(logger, "No best candidate found");
    return std::nullopt;
  }
  
  // Check hysteresis - whether to keep current target
  if (!config.current_target_id.empty()) {
    if (shouldKeepCurrentTarget(
          candidates,
          config.current_target_id,
          best_candidate,
          config.reference_yaw,
          config.hysteresis_threshold)) {
      // Keep current target
      const TrackedRobot* current = findRobotById(
        candidates, config.current_target_id);
      if (current != nullptr) {
        double yaw_dev = calculateYawDeviation(*current, config.reference_yaw);
        double dist = calculateDistanceToRobot(*current);
        return SelectionResult(
          current->robot_id,
          current->confidence,
          yaw_dev,
          dist
        );
      }
    }
  }
  
  // Return new best candidate
  double yaw_dev = calculateYawDeviation(*best_candidate, config.reference_yaw);
  double dist = calculateDistanceToRobot(*best_candidate);
  
  return SelectionResult(
    best_candidate->robot_id,
    best_candidate->confidence,
    yaw_dev,
    dist
  );
}

const MinYawDeviationStrategy::TrackedRobot* 
MinYawDeviationStrategy::findMinYawDeviationRobot(
    const std::vector<const TrackedRobot*>& candidates,
    double reference_yaw) const {
  
  const TrackedRobot* best = nullptr;
  double min_deviation = std::numeric_limits<double>::max();
  
  for (const auto* robot : candidates) {
    double deviation = calculateYawDeviation(*robot, reference_yaw);
    if (deviation < min_deviation) {
      min_deviation = deviation;
      best = robot;
    }
  }
  
  return best;
}

bool MinYawDeviationStrategy::shouldKeepCurrentTarget(
    const std::vector<const TrackedRobot*>& candidates,
    const std::string& current_target_id,
    const TrackedRobot* best_candidate,
    double reference_yaw,
    double hysteresis_threshold) const {
  
  // Find current target in candidates
  const TrackedRobot* current = findRobotById(candidates, current_target_id);
  
  if (current == nullptr) {
    // Current target is no longer valid
    return false;
  }
  
  if (best_candidate == nullptr) {
    // No better candidate, keep current
    return true;
  }
  
  // Compare yaw deviations with hysteresis
  double current_deviation = calculateYawDeviation(*current, reference_yaw);
  double best_deviation = calculateYawDeviation(*best_candidate, reference_yaw);
  
  // Only switch if new target is significantly better
  // hysteresis_threshold is the ratio of improvement needed
  double threshold = current_deviation * (1.0 - hysteresis_threshold);
  
  return best_deviation > threshold;
}

const MinYawDeviationStrategy::TrackedRobot* 
MinYawDeviationStrategy::findRobotById(
    const std::vector<const TrackedRobot*>& candidates,
    const std::string& robot_id) const {
  
  for (const auto* robot : candidates) {
    if (robot->robot_id == robot_id) {
      return robot;
    }
  }
  return nullptr;
}

}  // namespace fyt::auto_aim
