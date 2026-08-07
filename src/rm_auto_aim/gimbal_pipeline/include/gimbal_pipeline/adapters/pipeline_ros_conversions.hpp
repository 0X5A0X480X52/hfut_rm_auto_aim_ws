// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__ADAPTERS__PIPELINE_ROS_CONVERSIONS_HPP_
#define GIMBAL_PIPELINE__ADAPTERS__PIPELINE_ROS_CONVERSIONS_HPP_

#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <rm_interfaces/msg/tracked_robots.hpp>
#include <std_msgs/msg/header.hpp>

#include "gimbal_pipeline/core/pipeline_types.hpp"

namespace fyt::auto_aim::pipeline::ros_adapter
{

TimestampNs toTimestampNs(const builtin_interfaces::msg::Time & stamp);
builtin_interfaces::msg::Time toRosTime(TimestampNs timestamp_ns);

RobotTrack toDomain(const rm_interfaces::msg::TrackedRobot & robot);
RobotTrackSet toDomain(const rm_interfaces::msg::TrackedRobots & robots);
rm_interfaces::msg::TrackedRobot toRos(
  const RobotTrack & robot, const std_msgs::msg::Header & header);
rm_interfaces::msg::TrackedRobots toRos(
  const RobotTrackSet & robots, const std::string & frame_id);

GimbalCommand toDomain(const rm_interfaces::msg::GimbalCmd & command);
rm_interfaces::msg::GimbalCmd toRos(
  const GimbalCommand & command, const std::string & frame_id);

}  // namespace fyt::auto_aim::pipeline::ros_adapter

#endif  // GIMBAL_PIPELINE__ADAPTERS__PIPELINE_ROS_CONVERSIONS_HPP_
