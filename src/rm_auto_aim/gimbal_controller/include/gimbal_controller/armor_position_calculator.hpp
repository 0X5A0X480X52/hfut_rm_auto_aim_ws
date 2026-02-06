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

#ifndef GIMBAL_CONTROLLER__ARMOR_POSITION_CALCULATOR_HPP_
#define GIMBAL_CONTROLLER__ARMOR_POSITION_CALCULATOR_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "rm_interfaces/msg/tracked_robot.hpp"

namespace gimbal_controller
{

/**
 * @brief 装甲板位置计算器
 * 
 * 根据 TrackedRobot 消息中的机器人状态和装甲板几何偏移，
 * 计算各装甲板在世界坐标系中的位置
 */
class ArmorPositionCalculator
{
public:
  ArmorPositionCalculator() = default;
  ~ArmorPositionCalculator() = default;

  /**
   * @brief 计算所有装甲板在世界坐标系中的位置
   * @param robot 被跟踪机器人的状态信息
   * @return 各装甲板的世界坐标位置
   */
  std::vector<Eigen::Vector3d> calculate(
    const rm_interfaces::msg::TrackedRobot & robot) const;

  /**
   * @brief 计算预测位置的装甲板坐标 (基于速度预测)
   * @param robot 被跟踪机器人的状态信息
   * @param dt 预测时间 (秒)
   * @return 预测时刻各装甲板的世界坐标位置
   */
  std::vector<Eigen::Vector3d> calculatePredicted(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt) const;

  /**
   * @brief 根据机器人类型生成默认的装甲板几何偏移
   * @param robot_type 机器人类型
   * @param num_armors 装甲板数量
   * @param radius 主半径
   * @param radius_2 副半径 (4装甲板时使用)
   * @param d_za 装甲板高度差
   * @param d_zc 中心高度偏移
   * @return 装甲板在机器人坐标系中的几何偏移
   */
  static std::vector<Eigen::Vector3d> generateDefaultOffsets(
    uint8_t robot_type,
    int num_armors,
    double radius,
    double radius_2,
    double d_za,
    double d_zc);

private:
  /**
   * @brief 计算单个装甲板的世界坐标
   * @param center 机器人中心位置
   * @param yaw 机器人yaw角
   * @param offset 装甲板在机器人坐标系中的偏移
   * @return 装甲板世界坐标
   */
  Eigen::Vector3d transformToWorld(
    const Eigen::Vector3d & center,
    double yaw,
    const Eigen::Vector3d & offset) const;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__ARMOR_POSITION_CALCULATOR_HPP_
