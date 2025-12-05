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

#ifndef BALLISTIC_SOLVER__COMPENSATOR__TRAJECTORY_COMPENSATOR_HPP_
#define BALLISTIC_SOLVER__COMPENSATOR__TRAJECTORY_COMPENSATOR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ballistic_solver {

/**
 * @brief 弹道补偿器基类
 * 
 * 提供弹道计算的基本接口，支持不同的弹道模型。
 * 子类需要实现 calculateTrajectory 和 getFlyingTime 方法。
 */
class TrajectoryCompensator {
public:
  TrajectoryCompensator() = default;
  virtual ~TrajectoryCompensator() = default;

  /**
   * @brief 计算击中目标所需的pitch角
   * 
   * 通过迭代方法寻找正确的发射角度，使落点高度等于目标高度
   * 
   * @param target_position 目标位置 (x, y, z)，其中 x-y 平面为水平面，z 为高度
   * @param pitch 输出的pitch角 (弧度)
   * @return 是否成功求解
   */
  bool compensate(const Eigen::Vector3d &target_position, double &pitch) const noexcept;

  /**
   * @brief 获取子弹飞行时间
   * @param target_position 目标位置
   * @return 飞行时间 (秒)
   */
  virtual double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept = 0;

  /**
   * @brief 获取弹道轨迹点
   * @param distance 水平距离
   * @param angle 发射角度
   * @return 轨迹点列表 (距离, 高度)
   */
  std::vector<std::pair<double, double>> getTrajectory(double distance,
                                                       double angle) const noexcept;

  // 弹道参数
  double velocity = 15.0;     ///< 子弹速度 m/s
  int iteration_times = 20;   ///< 迭代次数
  double gravity = 9.8;       ///< 重力加速度 m/s^2
  double resistance = 0.001;  ///< 空气阻力系数

protected:
  /**
   * @brief 计算给定水平距离和发射角度下的落点高度
   * 
   * 子类必须实现此方法，提供具体的弹道计算模型
   * 
   * @param x 水平距离
   * @param angle 发射角度 (弧度)
   * @return 落点高度
   */
  virtual double calculateTrajectory(const double x, const double angle) const noexcept = 0;
};

/**
 * @brief 弹道补偿器工厂类
 * 
 * 用于根据类型字符串创建对应的补偿器实例
 */
class CompensatorFactory {
public:
  /**
   * @brief 创建补偿器实例
   * @param type 补偿器类型: "ideal" 或 "resistance"
   * @return 补偿器智能指针
   */
  static std::unique_ptr<TrajectoryCompensator> createCompensator(const std::string &type);

private:
  CompensatorFactory() = delete;
  ~CompensatorFactory() = delete;
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__COMPENSATOR__TRAJECTORY_COMPENSATOR_HPP_
