// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
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

#ifndef BALLISTIC_SOLVER__COMPENSATOR__RESISTANCE_COMPENSATOR_HPP_
#define BALLISTIC_SOLVER__COMPENSATOR__RESISTANCE_COMPENSATOR_HPP_

#include "ballistic_solver/compensator/trajectory_compensator.hpp"

namespace ballistic_solver {

/**
 * @brief 考虑空气阻力的弹道补偿器
 * 
 * 使用简化的空气阻力模型:
 * - 水平速度衰减: vx(t) = v0 * cos(θ) * e^(-r*t)
 * - 飞行时间: t = (e^(r*x) - 1) / (r * v * cos(θ))
 * 
 * 其中 r 是空气阻力系数
 * 
 * 适用于中远距离射击，提供更精确的弹道预测
 */
class ResistanceCompensator : public TrajectoryCompensator {
public:
  ResistanceCompensator() = default;
  ~ResistanceCompensator() override = default;

  /**
   * @brief 获取飞行时间 (考虑空气阻力)
   * @param target_position 目标位置
   * @return 飞行时间 (秒)
   */
  double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept override;

protected:
  /**
   * @brief 计算考虑空气阻力的弹道轨迹高度
   * 
   * 飞行时间公式: t = (e^(r*x) - 1) / (r * v * cos(θ))
   * 高度公式: y = v * sin(θ) * t - 0.5 * g * t²
   * 
   * @param x 水平距离
   * @param angle 发射角度 (弧度)
   * @return 落点高度
   */
  double calculateTrajectory(const double x, const double angle) const noexcept override;
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__COMPENSATOR__RESISTANCE_COMPENSATOR_HPP_
