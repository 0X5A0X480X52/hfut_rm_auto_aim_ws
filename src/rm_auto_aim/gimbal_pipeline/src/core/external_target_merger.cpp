// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/core/external_target_merger.hpp"

#include <algorithm>

namespace fyt::auto_aim::pipeline
{

bool ExternalTargetMerger::allowed(const std::string & robot_id, int mode) const
{
  const auto mode_it = config_.allowed_ids_by_mode.find(mode);
  return mode_it != config_.allowed_ids_by_mode.end() &&
         mode_it->second.find(robot_id) != mode_it->second.end();
}

RobotTrackSet ExternalTargetMerger::merge(
  const RobotTrackSet & tracked, const RobotTrackSet & external,
  TimestampNs now_ns, int mode) const
{
  RobotTrackSet result = tracked;
  result.timestamp_ns = std::max(result.timestamp_ns, external.timestamp_ns);
  if (!config_.enabled) {
    return result;
  }

  for (const auto & target : external.robots) {
    const auto age = now_ns - target.timestamp_ns;
    if (age < 0 || age > config_.timeout_ns || !allowed(target.robot_id, mode)) {
      continue;
    }
    const auto it = std::find_if(
      result.robots.begin(), result.robots.end(),
      [&](const RobotTrack & robot) {return robot.robot_id == target.robot_id;});
    if (it == result.robots.end()) {
      result.robots.push_back(target);
    } else {
      *it = target;
    }
  }
  return result;
}

}  // namespace fyt::auto_aim::pipeline
