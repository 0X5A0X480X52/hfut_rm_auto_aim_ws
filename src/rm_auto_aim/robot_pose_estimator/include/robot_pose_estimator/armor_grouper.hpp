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

#ifndef ROBOT_POSE_ESTIMATOR__ARMOR_GROUPER_HPP_
#define ROBOT_POSE_ESTIMATOR__ARMOR_GROUPER_HPP_

#include <map>
#include <vector>
#include <string>

#include "robot_pose_estimator/robot_types.hpp"
#include "rm_interfaces/msg/tracked_armor.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"

namespace fyt::auto_aim {

/**
 * @brief 装甲板分组器
 * 
 * 将检测到的装甲板按机器人ID分组，
 * 并过滤掉状态不满足要求的装甲板
 */
class ArmorGrouper {
public:
  /**
   * @brief 默认构造函数
   */
  ArmorGrouper() = default;
  
  /**
   * @brief 析构函数
   */
  ~ArmorGrouper() = default;

  /**
   * @brief 将跟踪的装甲板分组到各个机器人
   * @param tracked_armors 跟踪的装甲板消息
   * @param valid_states 有效的跟踪状态列表
   * @return 按机器人ID分组的装甲板状态
   */
  std::map<std::string, std::vector<ArmorState>> groupArmors(
    const rm_interfaces::msg::TrackedArmors& tracked_armors,
    const std::vector<uint8_t>& valid_states);

  /**
   * @brief 将检测到的装甲板分组到各个机器人
   * @param armors 检测的装甲板消息（来自armor_detector）
   * @return 按机器人ID分组的装甲板状态
   */
  std::map<std::string, std::vector<ArmorState>> groupArmors(
    const rm_interfaces::msg::Armors& armors);

  /**
   * @brief 将 TrackedArmor 转换为 ArmorState
   * @param msg 跟踪装甲板消息
   * @return 装甲板状态
   */
  ArmorState trackedArmorToState(const rm_interfaces::msg::TrackedArmor& msg);

  /**
   * @brief 将 Armor 转换为 ArmorState
   * @param msg 检测装甲板消息
   * @return 装甲板状态
   */
  ArmorState armorToState(const rm_interfaces::msg::Armor& msg);

  /**
   * @brief 检查跟踪状态是否有效
   * @param state 跟踪状态
   * @param valid_states 有效状态列表
   * @return 是否有效
   */
  bool isValidState(uint8_t state, const std::vector<uint8_t>& valid_states);

  /**
   * @brief 根据装甲板ID获取机器人ID
   * 
   * 装甲板ID规则:
   * - "1"~"5": 对应机器人1~5
   * - "outpost": 前哨站
   * - "base": 基地
   * 
   * @param armor_id 装甲板ID
   * @return 机器人ID
   */
  std::string getRobotIdFromArmorId(const std::string& armor_id);
};

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__ARMOR_GROUPER_HPP_
