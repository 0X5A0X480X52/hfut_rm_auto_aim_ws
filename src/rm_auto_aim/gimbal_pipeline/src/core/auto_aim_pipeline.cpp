// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/core/auto_aim_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fyt::auto_aim::pipeline
{

AutoAimPipeline::AutoAimPipeline(
  TrackingStage tracking_stage,
  PriorityTargetSelector selector,
  ExternalTargetMerger external_merger,
  GimbalCommandStage command_stage,
  TimestampNs tracker_timeout_ns)
: tracking_stage_(std::move(tracking_stage)),
  selector_(std::move(selector)),
  external_merger_(std::move(external_merger)),
  command_stage_(std::move(command_stage)),
  tracker_timeout_ns_(std::max<TimestampNs>(tracker_timeout_ns, 1))
{
}

AutoAimPipeline::AutoAimPipeline(
  TrackingStage tracking_stage,
  PipelineConfig config,
  GimbalCommandStage command_stage)
: AutoAimPipeline(
    std::move(tracking_stage),
    PriorityTargetSelector(std::move(config.target_selection)),
    ExternalTargetMerger(std::move(config.external_targets)),
    std::move(command_stage),
    config.tracker_timeout_ns)
{
}

std::optional<RobotTrack> AutoAimPipeline::findSelectedTarget(
  const RobotTrackSet & tracks, const std::optional<SelectedTarget> & selected) const
{
  if (!selected) {
    return std::nullopt;
  }
  const auto it = std::find_if(
    tracks.robots.begin(), tracks.robots.end(),
    [&](const RobotTrack & robot) {return robot.robot_id == selected->robot_id;});
  return it == tracks.robots.end() ? std::nullopt : std::optional<RobotTrack>(*it);
}

PipelineCycleResult AutoAimPipeline::runCycle(const PipelineCycleInput & input)
{
  PipelineCycleResult result;

  // 1. Tracking entry: consume each queued detector frame once and preserve FIFO ordering.
  // With no new frame, the tracker is intentionally not advanced again in this control cycle.
  for (const auto & frame : input.detection_frames) {
    if (last_frame_timestamp_ns_ != 0 && frame.timestamp_ns < last_frame_timestamp_ns_) {
      result.events.push_back({PipelineEvent::Type::OUT_OF_ORDER_FRAME, {}});
      continue;
    }
    latest_tracker_tracks_ = tracking_stage_(frame);
    last_frame_timestamp_ns_ = frame.timestamp_ns;
    result.tracking_updated = true;
  }

  // 2. External-target entry: merge fresh targets allowed by the current operating mode.
  result.tracks = external_merger_.merge(
    latest_tracker_tracks_, input.external_targets, input.now_ns, input.mode);

  // 3. Selection entry: priority-list selection owns target retention and switching decisions.
  const auto previous_id = selected_target_ ? selected_target_->robot_id : std::string();
  selected_target_ = selector_.select(result.tracks);
  if ((selected_target_ ? selected_target_->robot_id : std::string()) != previous_id) {
    result.events.push_back({PipelineEvent::Type::TARGET_CHANGED, previous_id});
  }

  // Resolve the selected summary back to the full robot state required by the controller.
  // Stale state must not enter prediction, ballistics or fire-decision modules.
  auto target = findSelectedTarget(result.tracks, selected_target_);
  if (target && input.now_ns - target->timestamp_ns > tracker_timeout_ns_) {
    result.events.push_back({PipelineEvent::Type::TARGET_STALE, target->robot_id});
    target.reset();
    selected_target_.reset();
  }

  result.selected_target = selected_target_;
  const bool control_enabled = input.enabled && std::isfinite(input.gimbal.bullet_speed) &&
    input.gimbal.bullet_speed > 0.0;

  // 4. Control entry: MPC -> armor selection -> local ballistics -> fire advice -> output filter.
  result.command = command_stage_(
    ControlRequest{input.now_ns, input.gimbal, target, control_enabled});

  // 5. Final safety gate: invalid input or an absent target can never produce a fire command.
  if (!control_enabled || !target) {
    result.command.fire = false;
  }
  return result;
}

}  // namespace fyt::auto_aim::pipeline
