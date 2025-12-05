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

// Created by Amatrix
// Maintained by HFUT RM SKT GROUP
// Copyright (C) HFUT RM SKT GROUP. All rights reserved.

#include "ballistic_solver/compensator/trajectory_compensator.hpp"

#include <cmath>

namespace ballistic_solver {

bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);
  // 迭代高度用于计算每次迭代的发射角度
  double iterative_height = target_height;
  double impact_height = 0;

  // 计算水平距离 (x-y 平面)
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));

  double angle = std::atan2(target_height, distance);
  double dh = 0;

  // 迭代寻找正确的发射角度，使落点高度等于目标高度
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);

    // 角度过大则退出 (超过 72 度)
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }

    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;

    // 收敛判断
    if (std::abs(dh) < 0.01) {
      break;
    }

    iterative_height += dh;
  }

  // 检查是否收敛
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }

  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  // 每 3cm 采样一个点
  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }

  return trajectory;
}

}  // namespace ballistic_solver
