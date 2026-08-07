// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__CORE__PIPELINE_TYPES_HPP_
#define GIMBAL_PIPELINE__CORE__PIPELINE_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::pipeline
{

using TimestampNs = std::int64_t;

enum class TrackState : std::uint8_t
{
  DETECTING = 0,
  TRACKING = 1,
  TEMP_LOST = 2,
};

enum class RepresentationMode : std::uint8_t
{
  STRUCTURED_ROBOT = 0,
  AMBIGUOUS_SINGLE_ARMOR = 1,
};

struct Pose3
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct ObservationFrame
{
  TimestampNs timestamp_ns{0};
  std::unordered_map<std::string, std::vector<ObservationData>> observations;
};

struct RobotTrack
{
  TimestampNs timestamp_ns{0};
  std::string robot_id;
  std::uint8_t robot_type{255};
  TrackState track_state{TrackState::DETECTING};
  RepresentationMode representation{RepresentationMode::STRUCTURED_ROBOT};

  Eigen::Vector3d center_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_velocity{0.0};
  double yaw_acceleration{0.0};
  double radius{0.0};
  double radius_2{0.0};
  double d_za{0.0};
  double d_zc{0.0};
  std::vector<Pose3> armor_offsets;
  std::vector<double> state_covariance;
  std::uint8_t covariance_dim{0};
  std::vector<std::string> bound_armor_ids;
  double confidence{0.0};
  int num_armors{0};
  bool is_visible{false};
  int visible_armor_count{0};
  std::uint32_t engageable_mask{0};
  int engageable_count{0};
  bool is_maneuvering{false};
};

struct RobotTrackSet
{
  TimestampNs timestamp_ns{0};
  std::vector<RobotTrack> robots;
};

struct SelectedTarget
{
  std::string robot_id;
  double confidence{0.0};
  double yaw_deviation{0.0};
  double distance{0.0};

  bool valid() const noexcept {return !robot_id.empty();}
};

struct GimbalState
{
  double yaw{0.0};
  double pitch{0.0};
  double bullet_speed{20.0};
};

struct GimbalCommand
{
  TimestampNs timestamp_ns{0};
  double yaw{0.0};
  double yaw_diff{0.0};
  double yaw_velocity{0.0};
  double yaw_acceleration{0.0};
  double pitch{0.0};
  double pitch_diff{0.0};
  double pitch_velocity{0.0};
  double pitch_acceleration{0.0};
  double distance{0.0};
  bool fire{false};
  std::string target_id;
  std::int8_t mode{-1};
};

struct PipelineEvent
{
  enum class Type
  {
    TRACKING_UPDATED,
    OUT_OF_ORDER_FRAME,
    TARGET_CHANGED,
    TARGET_STALE,
    CONTROL_FALLBACK,
  };

  Type type{Type::TRACKING_UPDATED};
  std::string detail;
};

struct PipelineCycleInput
{
  TimestampNs now_ns{0};
  std::vector<ObservationFrame> detection_frames;
  RobotTrackSet external_targets;
  GimbalState gimbal;
  int mode{0};
  bool enabled{true};
};

struct PipelineCycleResult
{
  RobotTrackSet tracks;
  std::optional<SelectedTarget> selected_target;
  GimbalCommand command;
  std::vector<PipelineEvent> events;
  bool tracking_updated{false};
};

}  // namespace fyt::auto_aim::pipeline

#endif  // GIMBAL_PIPELINE__CORE__PIPELINE_TYPES_HPP_
