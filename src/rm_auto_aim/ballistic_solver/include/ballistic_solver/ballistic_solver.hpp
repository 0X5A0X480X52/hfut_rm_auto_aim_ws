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

#ifndef BALLISTIC_SOLVER__BALLISTIC_SOLVER_HPP_
#define BALLISTIC_SOLVER__BALLISTIC_SOLVER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// 引入补偿器模块
#include "ballistic_solver/compensator/ideal_compensator.hpp"
#include "ballistic_solver/compensator/manual_compensator.hpp"
#include "ballistic_solver/compensator/resistance_compensator.hpp"
#include "ballistic_solver/compensator/trajectory_compensator.hpp"

namespace ballistic_solver {

/**
 * @brief 弹道解算结果结构体
 */
struct BallisticResult {
  double pitch;         ///< 云台pitch角 (弧度)
  double yaw;           ///< 云台yaw角 (弧度)
  double flight_time;   ///< 飞行时间 (秒)
  bool success;         ///< 是否成功求解
  std::string message;  ///< 错误或成功信息
};

/**
 * @brief 弹道解算器配置参数
 */
struct BallisticConfig {
  double bullet_speed = 28.0;                   ///< 子弹速度 m/s
  double gravity = 9.8;                         ///< 重力加速度 m/s^2
  double air_resistance = 0.001;                ///< 空气阻力系数
  int max_iterations = 50;                      ///< 最大迭代次数
  double convergence_threshold = 0.001;         ///< 收敛阈值 (米)
  std::string compensator_type = "resistance";  ///< 补偿器类型: "ideal" 或 "resistance"
};

/**
 * @brief 弹道解算器核心类
 * 
 * 封装弹道解算的完整流程，包括:
 * 1. 计算目标的 yaw 角
 * 2. 迭代求解 pitch 角 (考虑重力和空气阻力)
 * 3. 预测目标位置 (考虑飞行时间)
 * 4. 支持手动补偿微调
 */
class BallisticSolver {
public:
  /**
   * @brief 构造函数
   * @param config 弹道配置参数
   */
  explicit BallisticSolver(const BallisticConfig &config = BallisticConfig());

  /**
   * @brief 析构函数
   */
  ~BallisticSolver() = default;

  /**
   * @brief 计算击中静态目标所需的 pitch 和 yaw
   * @param target_position 目标位置 (相对于发射点)
   * @return 解算结果
   */
  BallisticResult solve(const Eigen::Vector3d &target_position) const;

  /**
   * @brief 计算击中移动目标所需的 pitch 和 yaw
   * 
   * 考虑目标速度，进行位置预测后再计算弹道
   * @param target_position 目标当前位置
   * @param target_velocity 目标速度
   * @return 解算结果
   */
  BallisticResult solveMovingTarget(const Eigen::Vector3d &target_position,
                                    const Eigen::Vector3d &target_velocity) const;

  /**
   * @brief 计算击中移动目标所需的 pitch 和 yaw (指定子弹速度)
   * @param target_position 目标当前位置
   * @param target_velocity 目标速度
   * @param bullet_speed 子弹速度
   * @return 解算结果
   */
  BallisticResult solveMovingTarget(const Eigen::Vector3d &target_position,
                                    const Eigen::Vector3d &target_velocity,
                                    double bullet_speed) const;

  /**
   * @brief 获取弹道轨迹
   * @param distance 水平距离
   * @param pitch 发射pitch角
   * @return 轨迹点列表
   */
  std::vector<std::pair<double, double>> getTrajectory(double distance,
                                                       double pitch) const noexcept;

  /**
   * @brief 更新配置参数
   * @param config 新的配置参数
   */
  void updateConfig(const BallisticConfig &config);

  /**
   * @brief 设置子弹速度
   * @param speed 子弹速度 m/s
   */
  void setBulletSpeed(double speed);

  /**
   * @brief 获取当前子弹速度
   * @return 子弹速度 m/s
   */
  double getBulletSpeed() const { return config_.bullet_speed; }

  /**
   * @brief 获取当前配置
   * @return 当前配置参数
   */
  const BallisticConfig &getConfig() const { return config_; }

  /**
   * @brief 获取手动补偿器引用 (用于配置手动补偿)
   * @return 手动补偿器引用
   */
  ManualCompensator &getManualCompensator() { return manual_compensator_; }

  /**
   * @brief 获取手动补偿器常量引用
   * @return 手动补偿器常量引用
   */
  const ManualCompensator &getManualCompensator() const { return manual_compensator_; }

private:
  /**
   * @brief 计算 yaw 角
   * @param target_position 目标位置
   * @return yaw 角 (弧度)
   */
  double calculateYaw(const Eigen::Vector3d &target_position) const noexcept;

  /**
   * @brief 计算飞行时间
   * @param target_position 目标位置
   * @return 飞行时间 (秒)
   */
  double calculateFlightTime(const Eigen::Vector3d &target_position) const noexcept;

  /**
   * @brief 预测目标位置
   * @param current_position 当前位置
   * @param velocity 目标速度
   * @param flight_time 飞行时间
   * @return 预测位置
   */
  Eigen::Vector3d predictTargetPosition(const Eigen::Vector3d &current_position,
                                        const Eigen::Vector3d &velocity,
                                        double flight_time) const noexcept;

  /**
   * @brief 计算水平距离
   * @param position 目标位置
   * @return 水平距离 (米)
   */
  double calculateHorizontalDistance(const Eigen::Vector3d &position) const noexcept;

  BallisticConfig config_;                              ///< 配置参数
  std::unique_ptr<TrajectoryCompensator> compensator_;  ///< 弹道补偿器
  ManualCompensator manual_compensator_;                ///< 手动补偿器
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__BALLISTIC_SOLVER_HPP_
