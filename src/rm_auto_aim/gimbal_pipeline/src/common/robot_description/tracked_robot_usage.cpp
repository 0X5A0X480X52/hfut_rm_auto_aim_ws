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

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim::robot_description
{
namespace
{

double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);

  if (q.length2() < 1e-12) {
    return 0.0;
  }

  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion buildQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

Eigen::Vector3d transformOffsetToWorld(
  const Eigen::Vector3d & center,
  double yaw,
  const Eigen::Vector3d & offset)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  return Eigen::Vector3d(
    center.x() + offset.x() * cos_yaw - offset.y() * sin_yaw,
    center.y() + offset.x() * sin_yaw + offset.y() * cos_yaw,
    center.z() + offset.z());
}

}  // namespace

Eigen::Vector3d TrackedRobotUsage::toEigen(const geometry_msgs::msg::Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d TrackedRobotUsage::toEigen(const geometry_msgs::msg::Vector3 & vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

geometry_msgs::msg::Point TrackedRobotUsage::toPoint(const Eigen::Vector3d & vector)
{
  geometry_msgs::msg::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

geometry_msgs::msg::Vector3 TrackedRobotUsage::toVector3(const Eigen::Vector3d & vector)
{
  geometry_msgs::msg::Vector3 value;
  value.x = vector.x();
  value.y = vector.y();
  value.z = vector.z();
  return value;
}

std::vector<Eigen::Vector3d> TrackedRobotUsage::resolveOffsets(
  const rm_interfaces::msg::TrackedRobot & robot,
  const OffsetFallbackGenerator & fallback_generator)
{
  std::vector<Eigen::Vector3d> offsets;

  if (!robot.armors_offset.empty()) {
    offsets.reserve(robot.armors_offset.size());
    for (const auto & pose : robot.armors_offset) {
      offsets.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    }
    return offsets;
  }

  if (fallback_generator) {
    return fallback_generator(robot);
  }

  return offsets;
}

rm_interfaces::msg::TrackedRobot TrackedRobotUsage::predict(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model)
{
  auto predicted = normalizeState(robot);

  Eigen::Vector3d center = centerPosition(predicted);
  Eigen::Vector3d velocity = linearVelocity(predicted);
  const Eigen::Vector3d acceleration = linearAcceleration(predicted);

  double predicted_yaw = yaw(predicted);
  double predicted_yaw_velocity = yawVelocity(predicted);
  const double predicted_yaw_acceleration = yawAcceleration(predicted);

  center += velocity * dt;
  predicted_yaw += predicted_yaw_velocity * dt;

  if (model == MotionModel::CONSTANT_ACCELERATION) {
    center += 0.5 * acceleration * dt * dt;
    velocity += acceleration * dt;
    predicted_yaw += 0.5 * predicted_yaw_acceleration * dt * dt;
    predicted_yaw_velocity += predicted_yaw_acceleration * dt;
  }

  predicted.center_position = toPoint(center);
  predicted.center_velocity = toVector3(velocity);
  predicted.yaw = predicted_yaw;
  predicted.yaw_velocity = predicted_yaw_velocity;

  syncFullStateFromLegacy(predicted);
  return predicted;
}

Eigen::Vector3d TrackedRobotUsage::predictCenter(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model)
{
  return centerPosition(predict(robot, dt, model));
}

double TrackedRobotUsage::predictYaw(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model)
{
  return yaw(predict(robot, dt, model));
}

std::vector<Eigen::Vector3d> TrackedRobotUsage::calculateArmorWorldPositionsEigen(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model,
  const OffsetFallbackGenerator & fallback_generator)
{
  const auto predicted_robot = predict(robot, dt, model);
  const Eigen::Vector3d center = centerPosition(predicted_robot);
  const double predicted_yaw = yaw(predicted_robot);
  const auto offsets = resolveOffsets(predicted_robot, fallback_generator);

  std::vector<Eigen::Vector3d> world_positions;
  world_positions.reserve(offsets.size());
  for (const auto & offset : offsets) {
    world_positions.push_back(transformOffsetToWorld(center, predicted_yaw, offset));
  }
  return world_positions;
}

std::vector<geometry_msgs::msg::Point> TrackedRobotUsage::calculateArmorWorldPositionsPoints(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model,
  const OffsetFallbackGenerator & fallback_generator)
{
  const auto world_positions = calculateArmorWorldPositionsEigen(robot, dt, model, fallback_generator);

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(world_positions.size());
  for (const auto & pos : world_positions) {
    points.push_back(toPoint(pos));
  }
  return points;
}

void TrackedRobotUsage::syncFullStateFromLegacy(rm_interfaces::msg::TrackedRobot & robot)
{
  robot.center_pose.position = robot.center_position;
  robot.center_pose.orientation = buildQuaternionFromYaw(robot.yaw);

  robot.center_twist.linear = robot.center_velocity;
  robot.center_twist.angular.x = 0.0;
  robot.center_twist.angular.y = 0.0;
  robot.center_twist.angular.z = robot.yaw_velocity;

  robot.center_accel.linear = robot.center_acceleration;
  robot.center_accel.angular.x = 0.0;
  robot.center_accel.angular.y = 0.0;
  robot.center_accel.angular.z = robot.yaw_acceleration;

  robot.full_state_valid = true;
}

void TrackedRobotUsage::syncLegacyStateFromFull(rm_interfaces::msg::TrackedRobot & robot)
{
  if (!robot.full_state_valid) {
    return;
  }

  robot.center_position = robot.center_pose.position;
  robot.center_velocity = robot.center_twist.linear;
  robot.center_acceleration = robot.center_accel.linear;
  robot.yaw = extractYawFromQuaternion(robot.center_pose.orientation);
  robot.yaw_velocity = robot.center_twist.angular.z;
  robot.yaw_acceleration = robot.center_accel.angular.z;
}

rm_interfaces::msg::TrackedRobot TrackedRobotUsage::normalizeState(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  rm_interfaces::msg::TrackedRobot normalized = robot;
  if (normalized.full_state_valid) {
    syncLegacyStateFromFull(normalized);
  } else {
    syncFullStateFromLegacy(normalized);
  }
  return normalized;
}

Eigen::Vector3d TrackedRobotUsage::centerPosition(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return Eigen::Vector3d(
      robot.center_pose.position.x,
      robot.center_pose.position.y,
      robot.center_pose.position.z);
  }

  return Eigen::Vector3d(
    robot.center_position.x,
    robot.center_position.y,
    robot.center_position.z);
}

Eigen::Vector3d TrackedRobotUsage::linearVelocity(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return Eigen::Vector3d(
      robot.center_twist.linear.x,
      robot.center_twist.linear.y,
      robot.center_twist.linear.z);
  }

  return Eigen::Vector3d(
    robot.center_velocity.x,
    robot.center_velocity.y,
    robot.center_velocity.z);
}

Eigen::Vector3d TrackedRobotUsage::linearAcceleration(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return Eigen::Vector3d(
      robot.center_accel.linear.x,
      robot.center_accel.linear.y,
      robot.center_accel.linear.z);
  }

  return Eigen::Vector3d(
    robot.center_acceleration.x,
    robot.center_acceleration.y,
    robot.center_acceleration.z);
}

Eigen::Vector3d TrackedRobotUsage::angularVelocity(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return Eigen::Vector3d(
      robot.center_twist.angular.x,
      robot.center_twist.angular.y,
      robot.center_twist.angular.z);
  }

  return Eigen::Vector3d(0.0, 0.0, robot.yaw_velocity);
}

Eigen::Vector3d TrackedRobotUsage::angularAcceleration(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return Eigen::Vector3d(
      robot.center_accel.angular.x,
      robot.center_accel.angular.y,
      robot.center_accel.angular.z);
  }

  return Eigen::Vector3d(0.0, 0.0, robot.yaw_acceleration);
}

double TrackedRobotUsage::yaw(const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return extractYawFromQuaternion(robot.center_pose.orientation);
  }

  return robot.yaw;
}

double TrackedRobotUsage::yawVelocity(const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return robot.center_twist.angular.z;
  }

  return robot.yaw_velocity;
}

double TrackedRobotUsage::yawAcceleration(const rm_interfaces::msg::TrackedRobot & robot)
{
  if (robot.full_state_valid) {
    return robot.center_accel.angular.z;
  }

  return robot.yaw_acceleration;
}

double TrackedRobotUsage::centerDistance(const rm_interfaces::msg::TrackedRobot & robot)
{
  return centerPosition(robot).norm();
}

std::vector<geometry_msgs::msg::Pose> TrackedRobotUsage::generateArmorsOffsetFromProfile(
  int num_armors,
  double r1,
  double r2,
  double d_za,
  double d_zc)
{
  std::vector<geometry_msgs::msg::Pose> offsets;
  offsets.reserve(static_cast<size_t>(std::max(0, num_armors)));

  bool is_current_pair = true;
  for (int i = 0; i < num_armors; ++i) {
    const double angle = i * (2.0 * M_PI / num_armors);
    double r = r1;
    double dz = d_zc;

    if (num_armors == 4) {
      r = is_current_pair ? r1 : r2;
      dz = d_zc + (is_current_pair ? -d_za : d_za);
      is_current_pair = !is_current_pair;
    } else if (num_armors == 3 && std::abs(d_za) > 1e-6) {
      // Outpost-compatible fallback: high/middle/low tri-layer profile.
      if (i == 0) {
        dz = d_zc + d_za;
      } else if (i == 1) {
        dz = d_zc;
      } else {
        dz = d_zc - d_za;
      }
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = -r * std::cos(angle);
    pose.position.y = -r * std::sin(angle);
    pose.position.z = dz;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle + M_PI);
    pose.orientation = tf2::toMsg(q);

    offsets.push_back(pose);
  }

  return offsets;
}

}  // namespace fyt::auto_aim::robot_description