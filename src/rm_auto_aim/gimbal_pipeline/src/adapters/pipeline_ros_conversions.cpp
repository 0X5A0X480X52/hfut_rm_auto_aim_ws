// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/adapters/pipeline_ros_conversions.hpp"

#include <cmath>

namespace fyt::auto_aim::pipeline::ros_adapter
{

TimestampNs toTimestampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<TimestampNs>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time toRosTime(TimestampNs timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000LL);
  return stamp;
}

RobotTrack toDomain(const rm_interfaces::msg::TrackedRobot & robot)
{
  RobotTrack result;
  result.timestamp_ns = toTimestampNs(robot.header.stamp);
  result.robot_id = robot.robot_id;
  result.robot_type = robot.robot_type;
  result.track_state = static_cast<TrackState>(robot.track_state);
  result.representation = static_cast<RepresentationMode>(robot.representation_mode);
  result.center_position = {
    robot.center_position.x, robot.center_position.y, robot.center_position.z};
  result.center_velocity = {
    robot.center_velocity.x, robot.center_velocity.y, robot.center_velocity.z};
  result.center_acceleration = {
    robot.center_acceleration.x, robot.center_acceleration.y, robot.center_acceleration.z};
  result.yaw = robot.yaw;
  result.yaw_velocity = robot.yaw_velocity;
  result.yaw_acceleration = robot.yaw_acceleration;
  result.radius = robot.radius;
  result.radius_2 = robot.radius_2;
  result.d_za = robot.d_za;
  result.d_zc = robot.d_zc;
  result.armor_offsets.reserve(robot.armors_offset.size());
  for (const auto & pose : robot.armors_offset) {
    Pose3 offset;
    offset.position = {pose.position.x, pose.position.y, pose.position.z};
    offset.orientation = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    result.armor_offsets.push_back(offset);
  }
  result.state_covariance = robot.state_covariance;
  result.covariance_dim = robot.covariance_dim;
  result.bound_armor_ids = robot.bound_armor_ids;
  result.confidence = robot.confidence;
  result.num_armors = robot.num_armors;
  result.is_visible = robot.is_visible;
  result.visible_armor_count = robot.visible_armor_count;
  result.engageable_mask = robot.engageable_mask;
  result.engageable_count = robot.engageable_count;
  return result;
}

RobotTrackSet toDomain(const rm_interfaces::msg::TrackedRobots & robots)
{
  RobotTrackSet result;
  result.timestamp_ns = toTimestampNs(robots.header.stamp);
  result.robots.reserve(robots.robots.size());
  for (const auto & robot : robots.robots) {
    result.robots.push_back(toDomain(robot));
  }
  return result;
}

rm_interfaces::msg::TrackedRobot toRos(
  const RobotTrack & robot, const std_msgs::msg::Header & header)
{
  rm_interfaces::msg::TrackedRobot result;
  result.header = header;
  result.header.stamp = toRosTime(robot.timestamp_ns);
  result.robot_id = robot.robot_id;
  result.robot_type = robot.robot_type;
  result.track_state = static_cast<std::uint8_t>(robot.track_state);
  result.representation_mode = static_cast<std::uint8_t>(robot.representation);
  result.center_position.x = robot.center_position.x();
  result.center_position.y = robot.center_position.y();
  result.center_position.z = robot.center_position.z();
  result.center_velocity.x = robot.center_velocity.x();
  result.center_velocity.y = robot.center_velocity.y();
  result.center_velocity.z = robot.center_velocity.z();
  result.center_acceleration.x = robot.center_acceleration.x();
  result.center_acceleration.y = robot.center_acceleration.y();
  result.center_acceleration.z = robot.center_acceleration.z();
  result.yaw = robot.yaw;
  result.yaw_velocity = robot.yaw_velocity;
  result.yaw_acceleration = robot.yaw_acceleration;
  result.radius = robot.radius;
  result.radius_2 = robot.radius_2;
  result.d_za = robot.d_za;
  result.d_zc = robot.d_zc;
  result.armors_offset.reserve(robot.armor_offsets.size());
  for (const auto & offset : robot.armor_offsets) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = offset.position.x();
    pose.position.y = offset.position.y();
    pose.position.z = offset.position.z();
    pose.orientation.w = offset.orientation.w();
    pose.orientation.x = offset.orientation.x();
    pose.orientation.y = offset.orientation.y();
    pose.orientation.z = offset.orientation.z();
    result.armors_offset.push_back(pose);
  }
  result.state_covariance = robot.state_covariance;
  result.covariance_dim = robot.covariance_dim;
  result.bound_armor_ids = robot.bound_armor_ids;
  result.confidence = robot.confidence;
  result.num_armors = robot.num_armors;
  result.is_visible = robot.is_visible;
  result.visible_armor_count = robot.visible_armor_count;
  result.engageable_mask = robot.engageable_mask;
  result.engageable_count = robot.engageable_count;

  result.full_state_valid = true;
  result.center_pose.position = result.center_position;
  result.center_pose.orientation.w = std::cos(0.5 * robot.yaw);
  result.center_pose.orientation.z = std::sin(0.5 * robot.yaw);
  result.center_twist.linear = result.center_velocity;
  result.center_twist.angular.z = robot.yaw_velocity;
  result.center_accel.linear = result.center_acceleration;
  result.center_accel.angular.z = robot.yaw_acceleration;
  return result;
}

rm_interfaces::msg::TrackedRobots toRos(
  const RobotTrackSet & robots, const std::string & frame_id)
{
  rm_interfaces::msg::TrackedRobots result;
  result.header.stamp = toRosTime(robots.timestamp_ns);
  result.header.frame_id = frame_id;
  result.robots.reserve(robots.robots.size());
  for (const auto & robot : robots.robots) {
    result.robots.push_back(toRos(robot, result.header));
  }
  return result;
}

GimbalCommand toDomain(const rm_interfaces::msg::GimbalCmd & command)
{
  GimbalCommand result;
  result.timestamp_ns = toTimestampNs(command.header.stamp);
  result.yaw = command.yaw;
  result.yaw_diff = command.yaw_diff;
  result.yaw_velocity = command.yaw_v;
  result.yaw_acceleration = command.yaw_a;
  result.pitch = command.pitch;
  result.pitch_diff = command.pitch_diff;
  result.pitch_velocity = command.pitch_v;
  result.pitch_acceleration = command.pitch_a;
  result.distance = command.distance;
  result.fire = command.fire_advice;
  result.target_id = command.target_id;
  result.mode = command.mode;
  return result;
}

rm_interfaces::msg::GimbalCmd toRos(
  const GimbalCommand & command, const std::string & frame_id)
{
  rm_interfaces::msg::GimbalCmd result;
  result.header.stamp = toRosTime(command.timestamp_ns);
  result.header.frame_id = frame_id;
  result.yaw = command.yaw;
  result.yaw_diff = command.yaw_diff;
  result.yaw_v = command.yaw_velocity;
  result.yaw_a = command.yaw_acceleration;
  result.pitch = command.pitch;
  result.pitch_diff = command.pitch_diff;
  result.pitch_v = command.pitch_velocity;
  result.pitch_a = command.pitch_acceleration;
  result.distance = command.distance;
  result.fire_advice = command.fire;
  result.target_id = command.target_id;
  result.mode = command.mode;
  return result;
}

}  // namespace fyt::auto_aim::pipeline::ros_adapter
