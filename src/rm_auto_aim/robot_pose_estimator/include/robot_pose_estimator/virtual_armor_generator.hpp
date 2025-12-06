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

#ifndef ROBOT_POSE_ESTIMATOR__VIRTUAL_ARMOR_GENERATOR_HPP_
#define ROBOT_POSE_ESTIMATOR__VIRTUAL_ARMOR_GENERATOR_HPP_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "robot_pose_estimator/robot_types.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"

namespace fyt::auto_aim {

/**
 * @brief 虚拟装甲板生成器
 * 
 * 基于机器人姿态估计结果，生成所有装甲板（包括被遮挡的）的位置，
 * 用于反馈给跟踪器进行状态更新
 */
class VirtualArmorGenerator {
public:
  /**
   * @brief 默认构造函数
   */
  VirtualArmorGenerator() = default;
  
  /**
   * @brief 析构函数
   */
  ~VirtualArmorGenerator() = default;

  /**
   * @brief 从机器人状态生成所有装甲板
   * @param robot_state 机器人状态
   * @return 所有装甲板的消息
   */
  std::vector<rm_interfaces::msg::Armor> generateArmors(const RobotState& robot_state) const;

  /**
   * @brief 计算所有装甲板位置
   * @param center 机器人中心位置
   * @param yaw 机器人yaw角
   * @param r1 主半径
   * @param r2 副半径 (4装甲板时)
   * @param d_zc 中心z偏移
   * @param d_za 装甲板z偏移差
   * @param armors_num 装甲板数量
   * @return 所有装甲板位置
   */
  std::vector<Eigen::Vector3d> calculateArmorPositions(
    const Eigen::Vector3d& center,
    double yaw,
    double r1,
    double r2,
    double d_zc,
    double d_za,
    int armors_num) const;

  /**
   * @brief 计算装甲板的yaw角
   * @param base_yaw 机器人yaw角
   * @param armor_index 装甲板索引
   * @param armors_num 装甲板数量
   * @return 装甲板yaw角
   */
  double calculateArmorYaw(double base_yaw, int armor_index, int armors_num) const;

  /**
   * @brief 创建 Armor 消息
   * @param position 装甲板位置
   * @param yaw 装甲板yaw角
   * @param armor_id 装甲板ID (也用于判断pitch角)
   * @param armor_type 装甲板类型
   * @return Armor 消息
   */
  rm_interfaces::msg::Armor createArmorMsg(
    const Eigen::Vector3d& position,
    double yaw,
    const std::string& armor_id,
    const std::string& armor_type) const;

private:
  // 装甲板倾斜角度常量 (约15度)
  static constexpr double ARMOR_PITCH_ANGLE = 0.2618;  // rad
};

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__VIRTUAL_ARMOR_GENERATOR_HPP_
