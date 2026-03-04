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

#include "target_selector/selection_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace fyt::auto_aim {

std::vector<const SelectionStrategy::TrackedRobot*> SelectionStrategy::filterCandidates(
    const TrackedRobots& robots,
    const SelectionConfig& config) const {
  std::vector<const TrackedRobot*> candidates;
  
  auto logger = rclcpp::get_logger("target_selector");
  RCLCPP_DEBUG(logger, "Filtering %zu robots with config: min_conf=%.3f, max_dist=%.2f, max_yaw_dev=%.3f",
              robots.robots.size(), config.min_confidence, config.max_distance, config.max_yaw_deviation);
  
  for (const auto& robot : robots.robots) {
    RCLCPP_DEBUG(logger, "Evaluating robot %s: confidence=%.3f", robot.robot_id.c_str(), robot.confidence);
    
    // Skip robots with low confidence
    if (robot.confidence < config.min_confidence) {
      RCLCPP_DEBUG(logger, "Robot %s rejected: confidence %.3f < %.3f", 
                   robot.robot_id.c_str(), robot.confidence, config.min_confidence);
      continue;
    }
    
    // Calculate distance
    double distance = calculateDistanceToRobot(robot);
    RCLCPP_DEBUG(logger, "Robot %s distance: %.3f", robot.robot_id.c_str(), distance);
    if (distance > config.max_distance) {
      RCLCPP_DEBUG(logger, "Robot %s rejected: distance %.3f > %.3f", 
                   robot.robot_id.c_str(), distance, config.max_distance);
      continue;
    }
    
    // Calculate yaw deviation
    double yaw_deviation = calculateYawDeviation(robot, config.reference_yaw);
    RCLCPP_DEBUG(logger, "Robot %s yaw deviation: %.3f", robot.robot_id.c_str(), yaw_deviation);
    if (yaw_deviation > config.max_yaw_deviation) {
      RCLCPP_DEBUG(logger, "Robot %s rejected: yaw deviation %.3f > %.3f", 
                   robot.robot_id.c_str(), yaw_deviation, config.max_yaw_deviation);
      continue;
    }
    
    RCLCPP_DEBUG(logger, "Robot %s accepted as candidate", robot.robot_id.c_str());
    candidates.push_back(&robot);
  }
  
  RCLCPP_DEBUG(logger, "Filtering complete: %zu candidates selected", candidates.size());
  return candidates;
}

double SelectionStrategy::calculateYawToRobot(const TrackedRobot& robot) const {
  // Calculate yaw angle from origin to robot center in gimbal frame
  // Assuming robot position is already in gimbal coordinate system
  return std::atan2(robot.center_position.y, robot.center_position.x);
}

double SelectionStrategy::calculateDistanceToRobot(const TrackedRobot& robot) const {
  // Calculate Euclidean distance to robot center
  double dx = robot.center_position.x;
  double dy = robot.center_position.y;
  double dz = robot.center_position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double SelectionStrategy::calculateYawDeviation(
    const TrackedRobot& robot,
    double reference_yaw) const {
  double robot_yaw = calculateYawToRobot(robot);
  double deviation = normalizeAngle(robot_yaw - reference_yaw);
  return std::abs(deviation);
}

double SelectionStrategy::normalizeAngle(double angle) const {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace fyt::auto_aim
