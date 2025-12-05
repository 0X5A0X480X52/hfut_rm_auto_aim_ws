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

#include "ballistic_solver/compensator/resistance_compensator.hpp"

// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
// Copyright (C) FYT Vision Group. All rights reserved.

#include <cmath>

namespace ballistic_solver {

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  // 防止除零
  double r = resistance < 1e-4 ? 1e-4 : resistance;

  // 考虑空气阻力的飞行时间
  // t = (e^(r*x) - 1) / (r * v * cos(θ))
  double t = (std::exp(r * x) - 1) / (r * velocity * std::cos(angle));

  // y = v * sin(θ) * t - 0.5 * g * t^2
  double y = velocity * std::sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  // 防止除零
  double r = resistance < 1e-4 ? 1e-4 : resistance;

  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = std::atan2(target_position(2), distance);

  // t = (e^(r*d) - 1) / (r * v * cos(θ))
  double t = (std::exp(r * distance) - 1) / (r * velocity * std::cos(angle));
  return t;
}

}  // namespace ballistic_solver
