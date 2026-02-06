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

#ifndef GIMBAL_CONTROLLER__LOCAL_TRAJECTORY_COMPENSATOR_HPP_
#define GIMBAL_CONTROLLER__LOCAL_TRAJECTORY_COMPENSATOR_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace gimbal_controller
{

/**
 * @brief 本地弹道补偿结果
 */
struct TrajectoryCompensationResult
{
  double pitch;           // 补偿后的pitch角 (弧度)
  double yaw;             // yaw角 (弧度)
  double flight_time;     // 飞行时间 (秒)
  bool success;           // 是否成功
};

/**
 * @brief 本地弹道补偿器
 * 
 * 当 ballistic_solver 服务不可用时，使用本地计算作为 fallback
 */
class LocalTrajectoryCompensator
{
public:
  LocalTrajectoryCompensator() = default;
  ~LocalTrajectoryCompensator() = default;

  /**
   * @brief 设置弹道参数
   * @param bullet_speed 子弹速度 (m/s)
   * @param gravity 重力加速度 (m/s^2)
   * @param resistance 空气阻力系数
   * @param iteration_times 迭代次数
   */
  void setParameters(
    double bullet_speed,
    double gravity = 9.8,
    double resistance = 0.001,
    int iteration_times = 20);

  /**
   * @brief 计算弹道补偿
   * @param target_position 目标位置
   * @return 补偿结果
   */
  TrajectoryCompensationResult compensate(const Eigen::Vector3d & target_position) const;

  /**
   * @brief 获取飞行时间
   * @param target_position 目标位置
   * @return 飞行时间 (秒)
   */
  double getFlyingTime(const Eigen::Vector3d & target_position) const;

  /**
   * @brief 获取弹道轨迹点 (用于可视化)
   * @param distance 水平距离
   * @param angle 初始角度
   * @return 轨迹点列表 (x, z)
   */
  std::vector<std::pair<double, double>> getTrajectory(double distance, double angle) const;

  // 参数访问
  double bulletSpeed() const { return bullet_speed_; }
  void setBulletSpeed(double speed) { bullet_speed_ = speed; }

private:
  /**
   * @brief 计算弹道高度 (理想弹道)
   * @param x 水平距离
   * @param angle 初始角度
   * @return 高度
   */
  double calculateIdealTrajectory(double x, double angle) const;

  /**
   * @brief 计算弹道高度 (考虑空气阻力)
   * @param x 水平距离
   * @param angle 初始角度
   * @return 高度
   */
  double calculateResistanceTrajectory(double x, double angle) const;

  double bullet_speed_{20.0};
  double gravity_{9.8};
  double resistance_{0.001};
  int iteration_times_{20};
  bool use_resistance_{false};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__LOCAL_TRAJECTORY_COMPENSATOR_HPP_
