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
#include <mutex>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim::robot_description
{
namespace
{
struct ProjectionPolicy
{
  fyt::auto_aim::robot_description::TrackedRobotUsage::ProjectionMode default_mode{
    fyt::auto_aim::robot_description::TrackedRobotUsage::ProjectionMode::YAW_PLANE};
  std::unordered_set<std::string> full_se3_ids{"big_buff", "small_buff"};
  std::unordered_set<uint8_t> full_se3_robot_types{};
};

ProjectionPolicy & projectionPolicy()
{
  static ProjectionPolicy policy;
  return policy;
}

std::mutex & projectionPolicyMutex()
{
  static std::mutex mtx;
  return mtx;
}

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

bool isValidQuaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);
  if (q.length2() < 1e-12) {
    return false;
  }
  return std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w());
}

Eigen::Quaterniond normalizedEigenQuat(const geometry_msgs::msg::Quaternion & quat)
{
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  if (q.norm() <= 1e-12 || !std::isfinite(q.norm())) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
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
  ProjectionMode projection_mode,
  const OffsetFallbackGenerator & fallback_generator)
{
  const auto predicted_robot = predict(robot, dt, model);
  const Eigen::Vector3d center = centerPosition(predicted_robot);
  const ProjectionMode resolved_mode =
    projection_mode == ProjectionMode::AUTO ? resolveProjectionMode(predicted_robot) : projection_mode;
  const bool use_full_se3 =
    resolved_mode == ProjectionMode::FULL_SE3 &&
    predicted_robot.full_state_valid &&
    isValidQuaternion(predicted_robot.center_pose.orientation);
  const double predicted_yaw = yaw(predicted_robot);
  const Eigen::Quaterniond q_center_world =
    use_full_se3 ? normalizedEigenQuat(predicted_robot.center_pose.orientation)
                 : Eigen::Quaterniond::Identity();
  const auto offsets = resolveOffsets(predicted_robot, fallback_generator);

  std::vector<Eigen::Vector3d> world_positions;
  world_positions.reserve(offsets.size());
  for (const auto & offset : offsets) {
    if (use_full_se3) {
      world_positions.push_back(center + q_center_world * offset);
    } else {
      world_positions.push_back(transformOffsetToWorld(center, predicted_yaw, offset));
    }
  }
  return world_positions;
}

std::vector<geometry_msgs::msg::Point> TrackedRobotUsage::calculateArmorWorldPositionsPoints(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model,
  ProjectionMode projection_mode,
  const OffsetFallbackGenerator & fallback_generator)
{
  const auto world_positions = calculateArmorWorldPositionsEigen(
    robot, dt, model, projection_mode, fallback_generator);

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(world_positions.size());
  for (const auto & pos : world_positions) {
    points.push_back(toPoint(pos));
  }
  return points;
}

std::vector<Eigen::Vector3d> TrackedRobotUsage::calculateArmorWorldPositionsEigen(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model,
  const OffsetFallbackGenerator & fallback_generator)
{
  return calculateArmorWorldPositionsEigen(
    robot, dt, model, ProjectionMode::AUTO, fallback_generator);
}

std::vector<geometry_msgs::msg::Point> TrackedRobotUsage::calculateArmorWorldPositionsPoints(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt,
  MotionModel model,
  const OffsetFallbackGenerator & fallback_generator)
{
  return calculateArmorWorldPositionsPoints(
    robot, dt, model, ProjectionMode::AUTO, fallback_generator);
}

TrackedRobotUsage::ProjectionMode TrackedRobotUsage::resolveProjectionMode(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  std::scoped_lock lk(projectionPolicyMutex());
  const auto & policy = projectionPolicy();
  if (policy.full_se3_ids.find(robot.robot_id) != policy.full_se3_ids.end()) {
    return ProjectionMode::FULL_SE3;
  }
  if (policy.full_se3_robot_types.find(robot.robot_type) != policy.full_se3_robot_types.end()) {
    return ProjectionMode::FULL_SE3;
  }
  return policy.default_mode;
}

void TrackedRobotUsage::setProjectionModePolicy(
  ProjectionMode default_mode,
  const std::unordered_set<std::string> & full_se3_ids,
  const std::unordered_set<uint8_t> & full_se3_robot_types)
{
  std::scoped_lock lk(projectionPolicyMutex());
  auto & policy = projectionPolicy();
  policy.default_mode = default_mode;
  policy.full_se3_ids = full_se3_ids;
  policy.full_se3_robot_types = full_se3_robot_types;
}

Eigen::Vector3d TrackedRobotUsage::calculateArmorWorldNormal(
  const rm_interfaces::msg::TrackedRobot & robot,
  int armor_index,
  double dt,
  MotionModel model,
  ProjectionMode projection_mode)
{
  const auto predicted_robot = predict(robot, dt, model);
  const auto center = centerPosition(predicted_robot);
  const auto armor_positions = calculateArmorWorldPositionsEigen(
    predicted_robot, 0.0, MotionModel::CONSTANT_VELOCITY, projection_mode,
    [](const rm_interfaces::msg::TrackedRobot &) { return std::vector<Eigen::Vector3d>{}; });
  if (armor_index < 0 || armor_index >= static_cast<int>(armor_positions.size())) {
    return Eigen::Vector3d::UnitX();
  }

  const ProjectionMode resolved_mode =
    projection_mode == ProjectionMode::AUTO ? resolveProjectionMode(predicted_robot) : projection_mode;
  const bool use_full_se3 =
    resolved_mode == ProjectionMode::FULL_SE3 &&
    predicted_robot.full_state_valid &&
    isValidQuaternion(predicted_robot.center_pose.orientation) &&
    armor_index < static_cast<int>(predicted_robot.armors_offset.size()) &&
    isValidQuaternion(predicted_robot.armors_offset[static_cast<size_t>(armor_index)].orientation);

  if (use_full_se3) {
    const auto q_center = normalizedEigenQuat(predicted_robot.center_pose.orientation);
    const auto q_offset = normalizedEigenQuat(
      predicted_robot.armors_offset[static_cast<size_t>(armor_index)].orientation);
    const Eigen::Vector3d n = (q_center * q_offset) * Eigen::Vector3d::UnitX();
    if (n.norm() > 1e-9) {
      return n.normalized();
    }
  }

  Eigen::Vector3d radial = armor_positions[static_cast<size_t>(armor_index)] - center;
  if (radial.norm() <= 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  return radial.normalized();
}

double TrackedRobotUsage::computeFacingCos(
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & armor,
  const Eigen::Vector3d & observer)
{
  Eigen::Vector3d center_to_armor = armor - center;
  Eigen::Vector3d center_to_observer = observer - center;

  Eigen::Vector3d a_xy(center_to_armor.x(), center_to_armor.y(), 0.0);
  Eigen::Vector3d b_xy(center_to_observer.x(), center_to_observer.y(), 0.0);
  const double a_norm = a_xy.norm();
  const double b_norm = b_xy.norm();

  if (a_norm <= 1e-3 || b_norm <= 1e-3) {
    return 1.0;
  }
  const double cos_value = a_xy.dot(b_xy) / (a_norm * b_norm);
  return std::clamp(cos_value, -1.0, 1.0);
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

// ── Representation mode ──

TrackedRobotUsage::RepresentationMode TrackedRobotUsage::inferRepresentationMode(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  // Phase B: explicit field takes priority when set.
  if (robot.representation_mode == rm_interfaces::msg::TrackedRobot::REP_AMBIGUOUS_SINGLE_ARMOR) {
    return RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
  }
  if (robot.representation_mode == rm_interfaces::msg::TrackedRobot::REP_STRUCTURED_ROBOT) {
    return RepresentationMode::STRUCTURED_ROBOT;
  }

  // Phase A convention: num_armors == 1 signals single-armor degraded mode.
  if (robot.num_armors == 1 && robot.armors_offset.size() <= 1) {
    return RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
  }
  if (robot.num_armors >= 3 && robot.armors_offset.size() >= 3) {
    return RepresentationMode::STRUCTURED_ROBOT;
  }
  // Fallback: use robot_type. OUTPOST_3 defaults to single-armor (safer).
  if (robot.robot_type == rm_interfaces::msg::TrackedRobot::OUTPOST_3) {
    return RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
  }
  return RepresentationMode::STRUCTURED_ROBOT;
}

bool TrackedRobotUsage::isSingleArmorRepresentation(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  return inferRepresentationMode(robot) == RepresentationMode::AMBIGUOUS_SINGLE_ARMOR;
}

Eigen::Vector3d TrackedRobotUsage::singleArmorPosition(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  return centerPosition(robot);
}

Eigen::Vector3d TrackedRobotUsage::singleArmorVelocity(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  return linearVelocity(robot);
}

double TrackedRobotUsage::singleArmorYaw(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  return yaw(robot);
}

Eigen::Vector3d TrackedRobotUsage::singleArmorNormal(
    const rm_interfaces::msg::TrackedRobot &robot)
{
  return calculateArmorWorldNormal(
    robot, 0, 0.0, MotionModel::CONSTANT_VELOCITY, ProjectionMode::AUTO);
}

std::vector<geometry_msgs::msg::Pose> TrackedRobotUsage::generateArmorsOffsetFromProfile(
  int num_armors,
  double r1,
  double r2,
  double d_za,
  double d_zc)
{
  constexpr double kDefaultArmorPitchUp = -0.2618;  // +15 deg
  double pitchOffsetSign = 1.0;
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
      pitchOffsetSign = -1.0;  // flip pitch for outpost armor to keep them facing outward
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = -r * std::cos(angle);
    pose.position.y = -r * std::sin(angle);
    pose.position.z = dz;

    tf2::Quaternion q;
    q.setRPY(0.0, kDefaultArmorPitchUp * pitchOffsetSign, angle + M_PI);
    pose.orientation = tf2::toMsg(q);

    offsets.push_back(pose);
  }

  return offsets;
}

}  // namespace fyt::auto_aim::robot_description
