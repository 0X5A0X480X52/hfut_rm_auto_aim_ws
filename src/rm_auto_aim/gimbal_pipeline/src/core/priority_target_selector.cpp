// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/core/priority_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fyt::auto_aim::pipeline
{
namespace
{
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

bool PriorityTargetSelector::eligible(const RobotTrack & robot) const
{
  if (!robot.is_visible || robot.confidence < config_.min_confidence) {
    return false;
  }
  const double distance = robot.center_position.norm();
  const double yaw = std::atan2(robot.center_position.y(), robot.center_position.x());
  return distance <= config_.max_distance &&
         std::abs(normalizeAngle(yaw - config_.reference_yaw)) <= config_.max_yaw_deviation;
}

SelectedTarget PriorityTargetSelector::describe(const RobotTrack & robot) const
{
  const double yaw = std::atan2(robot.center_position.y(), robot.center_position.x());
  return SelectedTarget{
    robot.robot_id, robot.confidence,
    std::abs(normalizeAngle(yaw - config_.reference_yaw)),
    robot.center_position.norm()};
}

std::optional<SelectedTarget> PriorityTargetSelector::select(const RobotTrackSet & tracks) const
{
  std::vector<const RobotTrack *> candidates;
  for (const auto & robot : tracks.robots) {
    if (eligible(robot)) {
      candidates.push_back(&robot);
    }
  }
  if (candidates.empty()) {
    return std::nullopt;
  }

  for (const auto & preferred_id : config_.priority_robot_ids) {
    const auto it = std::find_if(
      candidates.begin(), candidates.end(),
      [&](const RobotTrack * robot) {return robot->robot_id == preferred_id;});
    if (it != candidates.end()) {
      return describe(**it);
    }
  }

  const auto best = std::min_element(
    candidates.begin(), candidates.end(), [&](const RobotTrack * lhs, const RobotTrack * rhs) {
      return describe(*lhs).yaw_deviation < describe(*rhs).yaw_deviation;
    });
  return describe(**best);
}

}  // namespace fyt::auto_aim::pipeline
