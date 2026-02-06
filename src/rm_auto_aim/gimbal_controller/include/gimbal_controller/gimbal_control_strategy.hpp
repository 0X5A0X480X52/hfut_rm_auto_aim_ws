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

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROL_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROL_STRATEGY_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/tracked_robot.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"

namespace gimbal_controller
{

// 前向声明组件
class ArmorPositionCalculator;
class ArmorSelector;
class BallisticSolverClient;
class LocalTrajectoryCompensator;
class FireAdvisor;

/**
 * @brief 云台控制上下文
 * 
 * 包含策略执行所需的所有输入信息
 */
struct GimbalControlContext
{
  rm_interfaces::msg::TrackedRobot target_robot;  // 目标机器人状态
  double current_yaw{0.0};                        // 当前云台yaw角 (弧度)
  double current_pitch{0.0};                      // 当前云台pitch角 (弧度)
  double bullet_speed{20.0};                      // 子弹速度 (m/s)
  rclcpp::Time current_time;                      // 当前时间
  rclcpp::Time target_stamp;                      // 目标时间戳
  bool is_tracking{false};                        // 是否正在跟踪
};

/**
 * @brief 云台控制策略抽象基类
 * 
 * 使用组合模式，将可复用的组件组合为完整策略
 */
class GimbalControlStrategy
{
public:
  using SharedPtr = std::shared_ptr<GimbalControlStrategy>;

  GimbalControlStrategy() = default;
  virtual ~GimbalControlStrategy() = default;

  /**
   * @brief 执行策略，计算云台控制命令
   * @param context 控制上下文
   * @return 云台控制命令
   */
  virtual rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) = 0;

  /**
   * @brief 获取策略名称
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 设置组件 (依赖注入)
   */
  void setComponents(
    std::shared_ptr<ArmorPositionCalculator> position_calculator,
    std::shared_ptr<ArmorSelector> armor_selector,
    std::shared_ptr<BallisticSolverClient> ballistic_client,
    std::shared_ptr<LocalTrajectoryCompensator> local_compensator,
    std::shared_ptr<FireAdvisor> fire_advisor);

protected:
  std::shared_ptr<ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<ArmorSelector> armor_selector_;
  std::shared_ptr<BallisticSolverClient> ballistic_client_;
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<FireAdvisor> fire_advisor_;

  /**
   * @brief 创建空闲状态的控制命令
   */
  rm_interfaces::msg::GimbalCmd createIdleCmd() const;

  /**
   * @brief 计算弹道补偿
   * @param target_position 目标位置
   * @param target_velocity 目标速度
   * @param bullet_speed 子弹速度
   * @param[out] pitch 补偿后的pitch角
   * @param[out] yaw yaw角
   * @param[out] flight_time 飞行时间
   * @return 是否成功
   */
  bool computeBallistic(
    const Eigen::Vector3d & target_position,
    const Eigen::Vector3d & target_velocity,
    double bullet_speed,
    double & pitch,
    double & yaw,
    double & flight_time) const;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROL_STRATEGY_HPP_
