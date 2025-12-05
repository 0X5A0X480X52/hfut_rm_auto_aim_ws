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

#ifndef BALLISTIC_SOLVER__COMPENSATOR__IDEAL_COMPENSATOR_HPP_
#define BALLISTIC_SOLVER__COMPENSATOR__IDEAL_COMPENSATOR_HPP_

#include "ballistic_solver/compensator/trajectory_compensator.hpp"

namespace ballistic_solver {

/**
 * @brief 理想弹道补偿器 (不考虑空气阻力)
 * 
 * 使用简化的抛物线模型:
 * - 水平方向: x = v * cos(θ) * t
 * - 垂直方向: y = v * sin(θ) * t - 0.5 * g * t²
 * 
 * 适用于短距离或低阻力环境
 */
class IdealCompensator : public TrajectoryCompensator {
public:
  IdealCompensator() = default;
  ~IdealCompensator() override = default;

  /**
   * @brief 获取飞行时间
   * @param target_position 目标位置
   * @return 飞行时间 (秒)
   */
  double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept override;

protected:
  /**
   * @brief 计算理想弹道轨迹高度
   * 
   * 公式: y = v * sin(θ) * t - 0.5 * g * t²
   * 其中: t = x / (v * cos(θ))
   * 
   * @param x 水平距离
   * @param angle 发射角度 (弧度)
   * @return 落点高度
   */
  double calculateTrajectory(const double x, const double angle) const noexcept override;
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__COMPENSATOR__IDEAL_COMPENSATOR_HPP_
