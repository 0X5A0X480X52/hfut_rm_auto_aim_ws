// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__CORE__AUTO_AIM_PIPELINE_HPP_
#define GIMBAL_PIPELINE__CORE__AUTO_AIM_PIPELINE_HPP_

#include <functional>

#include "gimbal_pipeline/core/external_target_merger.hpp"
#include "gimbal_pipeline/core/priority_target_selector.hpp"

namespace fyt::auto_aim::pipeline
{

struct ControlRequest
{
  TimestampNs now_ns{0};
  GimbalState gimbal;
  std::optional<RobotTrack> target;
  bool enabled{true};
};

struct PipelineConfig
{
  TargetSelectionConfig target_selection;
  ExternalTargetConfig external_targets;
  TimestampNs tracker_timeout_ns{500000000};
};

// Core module entry: advances tracker lifecycle for exactly one detector frame.
using TrackingStage = std::function<RobotTrackSet(const ObservationFrame &)>;

// Core module entry: runs MPC, armor selection, local ballistics, fire advice and output filtering.
using GimbalCommandStage = std::function<GimbalCommand(const ControlRequest &)>;

class AutoAimPipeline
{
public:
  AutoAimPipeline(
    TrackingStage tracking_stage,
    PriorityTargetSelector selector,
    ExternalTargetMerger external_merger,
    GimbalCommandStage command_stage,
    TimestampNs tracker_timeout_ns);

  AutoAimPipeline(
    TrackingStage tracking_stage,
    PipelineConfig config,
    GimbalCommandStage command_stage);

  // Executes one complete auto-aim business cycle. ROS I/O, parameters and telemetry stay outside.
  PipelineCycleResult runCycle(const PipelineCycleInput & input);

private:
  std::optional<RobotTrack> findSelectedTarget(
    const RobotTrackSet & tracks, const std::optional<SelectedTarget> & selected) const;

  TrackingStage tracking_stage_;
  PriorityTargetSelector selector_;
  ExternalTargetMerger external_merger_;
  GimbalCommandStage command_stage_;
  TimestampNs tracker_timeout_ns_;
  TimestampNs last_frame_timestamp_ns_{0};
  RobotTrackSet latest_tracker_tracks_;
  std::optional<SelectedTarget> selected_target_;
};

}  // namespace fyt::auto_aim::pipeline

#endif  // GIMBAL_PIPELINE__CORE__AUTO_AIM_PIPELINE_HPP_
