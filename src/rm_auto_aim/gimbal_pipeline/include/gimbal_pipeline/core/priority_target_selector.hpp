// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__CORE__PRIORITY_TARGET_SELECTOR_HPP_
#define GIMBAL_PIPELINE__CORE__PRIORITY_TARGET_SELECTOR_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gimbal_pipeline/core/pipeline_types.hpp"

namespace fyt::auto_aim::pipeline
{

struct TargetSelectionConfig
{
  double reference_yaw{0.0};
  double max_yaw_deviation{3.14159265358979323846};
  double max_distance{10.0};
  double min_confidence{0.3};
  std::vector<std::string> priority_robot_ids;
};

class PriorityTargetSelector
{
public:
  explicit PriorityTargetSelector(TargetSelectionConfig config)
  : config_(std::move(config)) {}

  std::optional<SelectedTarget> select(const RobotTrackSet & tracks) const;

private:
  bool eligible(const RobotTrack & robot) const;
  SelectedTarget describe(const RobotTrack & robot) const;

  TargetSelectionConfig config_;
};

}  // namespace fyt::auto_aim::pipeline

#endif  // GIMBAL_PIPELINE__CORE__PRIORITY_TARGET_SELECTOR_HPP_
