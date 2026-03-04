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

#ifndef GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_
#define GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <optional>
#include <chrono>

#include "rm_interfaces/srv/solve_ballistic.hpp"

namespace gimbal_controller
{

/**
 * @brief 弹道解算结果
 */
struct BallisticResult
{
  double pitch;           // 云台pitch角 (弧度)
  double yaw;             // 云台yaw角 (弧度)
  double flight_time;     // 飞行时间 (秒)
  bool success;           // 是否成功
  std::string message;    // 消息
};

/**
 * @brief 弹道解算服务客户端
 * 
 * 封装对 ballistic_solver 服务的调用
 */
class BallisticSolverClient
{
public:
  using SolveBallistic = rm_interfaces::srv::SolveBallistic;

  // Accept a raw node pointer to avoid calling shared_from_this in constructors
  explicit BallisticSolverClient(rclcpp::Node * node);
  ~BallisticSolverClient() = default;

  /**
   * @brief 同步调用弹道解算服务
   * @param target_position 目标位置
   * @param target_velocity 目标速度
   * @param bullet_speed 子弹速度
   * @param timeout 超时时间
   * @return 弹道解算结果
   */
  BallisticResult solve(
    const Eigen::Vector3d & target_position,
    const Eigen::Vector3d & target_velocity,
    double bullet_speed,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(50));

  /**
   * @brief 检查服务是否可用
   */
  bool isServiceAvailable() const;

  /**
   * @brief 设置服务名称
   */
  void setServiceName(const std::string & service_name);

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Client<SolveBallistic>::SharedPtr client_;
  std::string service_name_{"/ballistic_solver/solve"};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_
