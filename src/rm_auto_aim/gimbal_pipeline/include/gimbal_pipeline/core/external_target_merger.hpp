// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__CORE__EXTERNAL_TARGET_MERGER_HPP_
#define GIMBAL_PIPELINE__CORE__EXTERNAL_TARGET_MERGER_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "gimbal_pipeline/core/pipeline_types.hpp"

namespace fyt::auto_aim::pipeline
{

struct ExternalTargetConfig
{
  bool enabled{false};
  TimestampNs timeout_ns{300000000};
  std::unordered_map<int, std::unordered_set<std::string>> allowed_ids_by_mode;
};

class ExternalTargetMerger
{
public:
  explicit ExternalTargetMerger(ExternalTargetConfig config)
  : config_(std::move(config)) {}

  RobotTrackSet merge(
    const RobotTrackSet & tracked, const RobotTrackSet & external,
    TimestampNs now_ns, int mode) const;

private:
  bool allowed(const std::string & robot_id, int mode) const;

  ExternalTargetConfig config_;
};

}  // namespace fyt::auto_aim::pipeline

#endif  // GIMBAL_PIPELINE__CORE__EXTERNAL_TARGET_MERGER_HPP_
