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

#include "target_selector/strategies/attack_outpost_first_strategy.hpp"

#include <algorithm>

#include <rclcpp/rclcpp.hpp>

namespace fyt::auto_aim {

AttackOutpostFirstStrategy::AttackOutpostFirstStrategy(
    const std::atomic_bool* attack_outpost_first)
    : attack_outpost_first_(attack_outpost_first) {}

std::optional<SelectionResult> AttackOutpostFirstStrategy::selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) {
  const bool prioritize_ob =
      attack_outpost_first_ != nullptr && attack_outpost_first_->load();

  if (!prioritize_ob) {
    ob_preferred_id_.clear();
    ob_miss_counter_ = 0;
    return base_strategy_.selectTarget(robots, config);
  }

  auto candidates = filterCandidates(robots, config);
  const int lost_frames = std::max(1, config.sticky_lost_frames);

  if (!ob_preferred_id_.empty()) {
    const TrackedRobot* pref = findRobotById(candidates, ob_preferred_id_);
    if (pref != nullptr) {
      ob_miss_counter_ = 0;
      return SelectionResult(
          pref->robot_id, pref->confidence,
          calculateYawDeviation(*pref, config.reference_yaw),
          calculateDistanceToRobot(*pref));
    }
    ob_miss_counter_++;
    if (ob_miss_counter_ >= lost_frames) {
      auto logger = rclcpp::get_logger("target_selector");
      RCLCPP_DEBUG(logger, "Clear outpost/base preferred id: %s (missed %d frames)",
                   ob_preferred_id_.c_str(), ob_miss_counter_);
      ob_preferred_id_.clear();
      ob_miss_counter_ = 0;
    }
  }

  const TrackedRobot* ob = findOutpostOrBase(candidates);
  if (ob != nullptr) {
    ob_preferred_id_ = ob->robot_id;
    ob_miss_counter_ = 0;
    return SelectionResult(
        ob->robot_id, ob->confidence,
        calculateYawDeviation(*ob, config.reference_yaw),
        calculateDistanceToRobot(*ob));
  }

  return base_strategy_.selectTarget(robots, config);
}

const AttackOutpostFirstStrategy::TrackedRobot*
AttackOutpostFirstStrategy::findOutpostOrBase(
    const std::vector<const TrackedRobot*>& candidates) const {
  for (const auto* robot : candidates) {
    if (robot->robot_id == "outpost" || robot->robot_id == "base") {
      return robot;
    }
  }
  return nullptr;
}

}  // namespace fyt::auto_aim
