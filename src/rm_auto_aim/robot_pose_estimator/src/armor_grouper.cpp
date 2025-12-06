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

#include "robot_pose_estimator/armor_grouper.hpp"

#include <algorithm>

namespace fyt::auto_aim {

std::map<std::string, std::vector<ArmorState>> ArmorGrouper::groupArmors(
    const rm_interfaces::msg::TrackedArmors& tracked_armors,
    const std::vector<uint8_t>& valid_states) {
  
  std::map<std::string, std::vector<ArmorState>> groups;
  
  for (const auto& tracked_armor : tracked_armors.armors) {
    // 检查跟踪状态是否有效
    if (!isValidState(tracked_armor.tracking_state, valid_states)) {
      continue;
    }
    
    // 转换为 ArmorState
    ArmorState state = trackedArmorToState(tracked_armor);
    
    // 获取机器人ID
    std::string robot_id = getRobotIdFromArmorId(state.armor_id);
    
    // 添加到对应组
    groups[robot_id].push_back(state);
  }
  
  return groups;
}

std::map<std::string, std::vector<ArmorState>> ArmorGrouper::groupArmors(
    const rm_interfaces::msg::Armors& armors) {
  
  std::map<std::string, std::vector<ArmorState>> groups;
  
  for (const auto& armor : armors.armors) {
    // 转换为 ArmorState
    ArmorState state = armorToState(armor);
    
    // 获取机器人ID
    std::string robot_id = getRobotIdFromArmorId(state.armor_id);
    
    // 添加到对应组
    groups[robot_id].push_back(state);
  }
  
  return groups;
}

ArmorState ArmorGrouper::trackedArmorToState(const rm_interfaces::msg::TrackedArmor& msg) {
  ArmorState state;
  
  state.armor_id = msg.armor_id;
  state.armor_type = msg.armor_type;
  
  state.position = Eigen::Vector3d(
    msg.position.x,
    msg.position.y,
    msg.position.z
  );
  
  state.velocity = Eigen::Vector3d(
    msg.velocity.x,
    msg.velocity.y,
    msg.velocity.z
  );
  
  state.yaw = msg.yaw;
  state.yaw_velocity = msg.yaw_velocity;
  state.confidence = msg.confidence;
  state.is_observed = (msg.source_type == rm_interfaces::msg::TrackedArmor::SOURCE_DETECT);
  state.track_id = msg.track_id;
  
  return state;
}

ArmorState ArmorGrouper::armorToState(const rm_interfaces::msg::Armor& msg) {
  ArmorState state;
  
  state.armor_id = msg.number;
  state.armor_type = msg.type;
  
  state.position = Eigen::Vector3d(
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z
  );
  
  // 检测结果没有速度信息，设为0
  state.velocity = Eigen::Vector3d::Zero();
  
  // 从四元数计算 yaw
  double qw = msg.pose.orientation.w;
  double qx = msg.pose.orientation.x;
  double qy = msg.pose.orientation.y;
  double qz = msg.pose.orientation.z;
  state.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  
  state.yaw_velocity = 0.0;  // 检测结果没有角速度信息
  state.confidence = 1.0;     // 检测结果没有confidence字段，默认设为1.0
  state.is_observed = true;   // 检测结果总是观测
  state.track_id = -1;        // 检测结果没有track_id
  
  return state;
}

bool ArmorGrouper::isValidState(uint8_t state, const std::vector<uint8_t>& valid_states) {
  return std::find(valid_states.begin(), valid_states.end(), state) != valid_states.end();
}

std::string ArmorGrouper::getRobotIdFromArmorId(const std::string& armor_id) {
  // 装甲板ID规则:
  // - "1"~"5": 对应机器人1~5
  // - "outpost": 前哨站
  // - "base": 基地
  // 
  // 直接返回 armor_id 作为 robot_id
  return armor_id;
}

}  // namespace fyt::auto_aim
