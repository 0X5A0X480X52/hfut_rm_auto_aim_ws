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

#include "gimbal_controller/local_trajectory_compensator.hpp"

namespace gimbal_controller
{

void LocalTrajectoryCompensator::setParameters(
  double bullet_speed,
  double gravity,
  double resistance,
  int iteration_times)
{
  bullet_speed_ = bullet_speed;
  gravity_ = gravity;
  resistance_ = resistance;
  iteration_times_ = iteration_times;
  use_resistance_ = (resistance > 0.0001);
}

TrajectoryCompensationResult LocalTrajectoryCompensator::compensate(
  const Eigen::Vector3d & target_position) const
{
  TrajectoryCompensationResult result;
  result.success = false;

  double distance_xy = std::sqrt(
    target_position.x() * target_position.x() +
    target_position.y() * target_position.y());

  if (distance_xy < 0.001) {
    result.yaw = 0;
    result.pitch = (target_position.z() > 0) ? M_PI / 2 : -M_PI / 2;
    result.flight_time = 0;
    result.success = true;
    return result;
  }

  // yaw 角直接计算
  result.yaw = std::atan2(target_position.y(), target_position.x());

  // pitch 角迭代求解
  double z = target_position.z();
  double pitch = std::atan2(z, distance_xy);  // 初始估计

  for (int i = 0; i < iteration_times_; ++i) {
    double trajectory_z = use_resistance_
                            ? calculateResistanceTrajectory(distance_xy, pitch)
                            : calculateIdealTrajectory(distance_xy, pitch);

    double error = z - trajectory_z;

    if (std::abs(error) < 0.001) {
      result.pitch = pitch;
      result.flight_time = getFlyingTime(target_position);
      result.success = true;
      return result;
    }

    // 调整 pitch 角
    pitch += error / distance_xy;

    // 限制 pitch 角范围
    if (pitch > M_PI / 2) {
      pitch = M_PI / 2;
    } else if (pitch < -M_PI / 2) {
      pitch = -M_PI / 2;
    }
  }

  // 迭代未收敛，返回最后一次结果
  result.pitch = pitch;
  result.flight_time = getFlyingTime(target_position);
  result.success = true;
  return result;
}

double LocalTrajectoryCompensator::getFlyingTime(
  const Eigen::Vector3d & target_position) const
{
  double distance_xy = std::sqrt(
    target_position.x() * target_position.x() +
    target_position.y() * target_position.y());

  if (bullet_speed_ < 0.001) {
    return 0;
  }

  // 简化计算: 使用水平速度分量
  double pitch = std::atan2(target_position.z(), distance_xy);
  double v_horizontal = bullet_speed_ * std::cos(pitch);

  if (v_horizontal < 0.001) {
    return 0;
  }

  return distance_xy / v_horizontal;
}

std::vector<std::pair<double, double>> LocalTrajectoryCompensator::getTrajectory(
  double distance,
  double angle) const
{
  std::vector<std::pair<double, double>> trajectory;

  int num_points = 50;
  double dx = distance / num_points;

  for (int i = 0; i <= num_points; ++i) {
    double x = i * dx;
    double z = use_resistance_
                 ? calculateResistanceTrajectory(x, angle)
                 : calculateIdealTrajectory(x, angle);
    trajectory.emplace_back(x, z);
  }

  return trajectory;
}

double LocalTrajectoryCompensator::calculateIdealTrajectory(
  double x,
  double angle) const
{
  if (bullet_speed_ < 0.001) {
    return 0;
  }

  double v_horizontal = bullet_speed_ * std::cos(angle);
  double v_vertical = bullet_speed_ * std::sin(angle);

  if (v_horizontal < 0.001) {
    return 0;
  }

  double t = x / v_horizontal;
  return v_vertical * t - 0.5 * gravity_ * t * t;
}

double LocalTrajectoryCompensator::calculateResistanceTrajectory(
  double x,
  double angle) const
{
  if (bullet_speed_ < 0.001) {
    return 0;
  }

  // 简化的空气阻力模型
  // 使用欧拉法数值积分
  double vx = bullet_speed_ * std::cos(angle);
  double vz = bullet_speed_ * std::sin(angle);
  double px = 0, pz = 0;

  double dt = 0.001;  // 积分步长
  int max_steps = 10000;

  for (int i = 0; i < max_steps && px < x; ++i) {
    double v = std::sqrt(vx * vx + vz * vz);
    double drag = resistance_ * v * v;

    // 阻力方向与速度方向相反
    double ax = -drag * vx / v;
    double az = -gravity_ - drag * vz / v;

    vx += ax * dt;
    vz += az * dt;
    px += vx * dt;
    pz += vz * dt;
  }

  return pz;
}

}  // namespace gimbal_controller
