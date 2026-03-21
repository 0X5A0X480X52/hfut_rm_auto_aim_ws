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

#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"

#include <algorithm>

#include <rclcpp/rclcpp.hpp>

namespace fyt::auto_aim {

std::optional<SelectionResult> StickyMinYawDeviationStrategy::selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) {
  auto logger = rclcpp::get_logger("target_selector");

  auto base_result = base_strategy_.selectTarget(robots, config);
  auto candidates = filterCandidates(robots, config);

  const int lock_frames = std::max(1, config.sticky_lock_frames);
  const int lost_frames = std::max(1, config.sticky_lost_frames);

  if (!preferred_target_id_.empty()) {
    const TrackedRobot* preferred = findRobotById(candidates, preferred_target_id_);
    if (preferred != nullptr) {
      preferred_miss_counter_ = 0;
      return SelectionResult(
          preferred->robot_id,
          preferred->confidence,
          calculateYawDeviation(*preferred, config.reference_yaw),
          calculateDistanceToRobot(*preferred));
    }

    preferred_miss_counter_ += 1;
    if (preferred_miss_counter_ >= lost_frames) {
      RCLCPP_DEBUG(logger, "Clear preferred target id: %s (missed %d frames)",
                   preferred_target_id_.c_str(), preferred_miss_counter_);
      preferred_target_id_.clear();
      preferred_miss_counter_ = 0;
      lock_candidate_id_.clear();
      lock_counter_ = 0;
    }
  }

  if (!base_result.has_value()) {
    lock_candidate_id_.clear();
    lock_counter_ = 0;
    return std::nullopt;
  }

  if (base_result->robot_id == lock_candidate_id_) {
    lock_counter_ += 1;
  } else {
    lock_candidate_id_ = base_result->robot_id;
    lock_counter_ = 1;
  }

  if (lock_counter_ >= lock_frames && preferred_target_id_ != lock_candidate_id_) {
    preferred_target_id_ = lock_candidate_id_;
    preferred_miss_counter_ = 0;
    RCLCPP_DEBUG(logger, "Set preferred target id: %s (locked %d frames)",
                 preferred_target_id_.c_str(), lock_counter_);
  }

  return base_result;
}

const StickyMinYawDeviationStrategy::TrackedRobot* StickyMinYawDeviationStrategy::findRobotById(
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
