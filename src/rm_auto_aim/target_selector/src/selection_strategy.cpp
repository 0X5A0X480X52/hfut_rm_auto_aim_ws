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

namespace fyt::auto_aim {

std::vector<const SelectionStrategy::TrackedRobot*> SelectionStrategy::filterCandidates(
    const TrackedRobots& robots,
    const SelectionConfig& config) const {
  std::vector<const TrackedRobot*> candidates;
  
  for (const auto& robot : robots.robots) {
    // Skip robots with low confidence
    if (robot.confidence < config.min_confidence) {
      continue;
    }
    
    // Calculate distance
    double distance = calculateDistanceToRobot(robot);
    if (distance > config.max_distance) {
      continue;
    }
    
    // Calculate yaw deviation
    double yaw_deviation = calculateYawDeviation(robot, config.reference_yaw);
    if (yaw_deviation > config.max_yaw_deviation) {
      continue;
    }
    
    candidates.push_back(&robot);
  }
  
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
