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

#include "ballistic_solver/ballistic_solver.hpp"

// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
// Copyright (C) FYT Vision Group. All rights reserved.

#include <cmath>

namespace ballistic_solver {

// ============================================================================
// BallisticSolver 实现
// ============================================================================

BallisticSolver::BallisticSolver(const BallisticConfig &config) : config_(config) {
  compensator_ = CompensatorFactory::createCompensator(config_.compensator_type);

  if (compensator_) {
    compensator_->velocity = config_.bullet_speed;
    compensator_->gravity = config_.gravity;
    compensator_->resistance = config_.air_resistance;
    compensator_->iteration_times = config_.max_iterations;
  }
}

void BallisticSolver::updateConfig(const BallisticConfig &config) {
  config_ = config;

  compensator_ = CompensatorFactory::createCompensator(config_.compensator_type);

  if (compensator_) {
    compensator_->velocity = config_.bullet_speed;
    compensator_->gravity = config_.gravity;
    compensator_->resistance = config_.air_resistance;
    compensator_->iteration_times = config_.max_iterations;
  }
}

void BallisticSolver::setBulletSpeed(double speed) {
  config_.bullet_speed = speed;
  if (compensator_) {
    compensator_->velocity = speed;
  }
}

double BallisticSolver::calculateYaw(const Eigen::Vector3d &target_position) const noexcept {
  return std::atan2(target_position(1), target_position(0));
}

double BallisticSolver::calculateHorizontalDistance(
  const Eigen::Vector3d &position) const noexcept {
  return std::sqrt(position(0) * position(0) + position(1) * position(1));
}

double BallisticSolver::calculateFlightTime(const Eigen::Vector3d &target_position) const noexcept {
  if (!compensator_) {
    // 简化计算: t ≈ d / v
    double distance = calculateHorizontalDistance(target_position);
    return distance / config_.bullet_speed;
  }

  return compensator_->getFlyingTime(target_position);
}

Eigen::Vector3d BallisticSolver::predictTargetPosition(const Eigen::Vector3d &current_position,
                                                       const Eigen::Vector3d &velocity,
                                                       double flight_time) const noexcept {
  return current_position + velocity * flight_time;
}

BallisticResult BallisticSolver::solve(const Eigen::Vector3d &target_position) const {
  BallisticResult result;
  result.success = false;
  result.pitch = 0;
  result.yaw = 0;
  result.flight_time = 0;

  // 检查目标位置有效性
  if (target_position.norm() < 0.01) {
    result.message = "Target position too close or invalid";
    return result;
  }

  // 检查补偿器是否可用
  if (!compensator_) {
    result.message = "Trajectory compensator not initialized";
    return result;
  }

  // 计算 yaw 角
  result.yaw = calculateYaw(target_position);

  // 计算 pitch 角 (通过迭代求解)
  double pitch = 0;
  bool compensate_success = compensator_->compensate(target_position, pitch);

  if (!compensate_success) {
    result.message = "Failed to converge in pitch calculation";
    return result;
  }

  result.pitch = pitch;
  result.flight_time = compensator_->getFlyingTime(target_position);

  // 应用手动补偿
  double dist = calculateHorizontalDistance(target_position);
  double height = target_position(2);
  auto manual_offset = manual_compensator_.angleHardCorrect(dist, height);
  if (manual_offset.size() >= 2) {
    result.pitch += manual_offset[0];
    result.yaw += manual_offset[1];
  }

  result.success = true;
  result.message = "Ballistic solution found successfully";

  return result;
}

BallisticResult BallisticSolver::solveMovingTarget(const Eigen::Vector3d &target_position,
                                                   const Eigen::Vector3d &target_velocity) const {
  return solveMovingTarget(target_position, target_velocity, config_.bullet_speed);
}

BallisticResult BallisticSolver::solveMovingTarget(const Eigen::Vector3d &target_position,
                                                   const Eigen::Vector3d &target_velocity,
                                                   double bullet_speed) const {
  BallisticResult result;
  result.success = false;
  result.pitch = 0;
  result.yaw = 0;
  result.flight_time = 0;

  // 检查输入有效性
  if (target_position.norm() < 0.01) {
    result.message = "Target position too close or invalid";
    return result;
  }

  if (!compensator_) {
    result.message = "Trajectory compensator not initialized";
    return result;
  }

  // 临时修改子弹速度
  double original_velocity = compensator_->velocity;
  compensator_->velocity = bullet_speed;

  // 迭代预测目标位置
  Eigen::Vector3d predicted_position = target_position;
  double flight_time = 0;

  for (int i = 0; i < config_.max_iterations; ++i) {
    // 计算当前预测位置的飞行时间
    double new_flight_time = compensator_->getFlyingTime(predicted_position);

    // 更新预测位置
    predicted_position = predictTargetPosition(target_position, target_velocity, new_flight_time);

    // 检查收敛
    if (std::abs(new_flight_time - flight_time) < config_.convergence_threshold) {
      flight_time = new_flight_time;
      break;
    }

    flight_time = new_flight_time;
  }

  // 对预测位置求解弹道
  result.yaw = calculateYaw(predicted_position);

  double pitch = 0;
  bool compensate_success = compensator_->compensate(predicted_position, pitch);

  // 恢复原始子弹速度
  compensator_->velocity = original_velocity;

  if (!compensate_success) {
    result.message = "Failed to converge in pitch calculation for moving target";
    return result;
  }

  result.pitch = pitch;
  result.flight_time = flight_time;

  // 应用手动补偿
  double dist = calculateHorizontalDistance(predicted_position);
  double height = predicted_position(2);
  auto manual_offset = manual_compensator_.angleHardCorrect(dist, height);
  if (manual_offset.size() >= 2) {
    result.pitch += manual_offset[0];
    result.yaw += manual_offset[1];
  }

  result.success = true;
  result.message = "Ballistic solution for moving target found successfully";

  return result;
}

std::vector<std::pair<double, double>> BallisticSolver::getTrajectory(double distance,
                                                                      double pitch) const noexcept {
  if (!compensator_) {
    return {};
  }

  return compensator_->getTrajectory(distance, pitch);
}

}  // namespace ballistic_solver
