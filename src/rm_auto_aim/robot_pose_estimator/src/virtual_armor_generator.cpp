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

#include "robot_pose_estimator/virtual_armor_generator.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim {

std::vector<rm_interfaces::msg::Armor> VirtualArmorGenerator::generateArmors(
    const RobotState& robot_state) const {
  
  std::vector<rm_interfaces::msg::Armor> armors;
  
  if (!robot_state.is_tracking) {
    return armors;
  }
  
  // 计算所有装甲板位置
  auto positions = calculateArmorPositions(
    robot_state.center_position,
    robot_state.yaw,
    robot_state.radius_1,
    robot_state.radius_2,
    robot_state.d_zc,
    robot_state.d_za,
    robot_state.num_armors
  );
  
  // 确定装甲板类型
  std::string armor_type = "small";
  if (robot_state.robot_type == RobotType::BALANCE_2 ||
      robot_state.robot_type == RobotType::HERO_4 ||
      robot_state.robot_type == RobotType::OUTPOST_3 ||
      robot_state.robot_type == RobotType::BASE) {
    armor_type = "large";
  }
  
  // 生成所有装甲板消息
  for (int i = 0; i < robot_state.num_armors; ++i) {
    double yaw = calculateArmorYaw(robot_state.yaw, i, robot_state.num_armors);
    auto armor_msg = createArmorMsg(positions[i], yaw, robot_state.robot_id, armor_type);
    armors.push_back(armor_msg);
  }
  
  return armors;
}

std::vector<Eigen::Vector3d> VirtualArmorGenerator::calculateArmorPositions(
    const Eigen::Vector3d& center,
    double yaw,
    double r1,
    double r2,
    double d_zc,
    double d_za,
    int armors_num) const {
  
  std::vector<Eigen::Vector3d> positions(armors_num, Eigen::Vector3d::Zero());
  
  bool is_current_pair = true;
  double r = 0.0, target_dz = 0.0;
  
  for (int i = 0; i < armors_num; ++i) {
    double temp_yaw = yaw + i * (2.0 * M_PI / armors_num);
    
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    
    positions[i] = center + Eigen::Vector3d(
      -r * std::cos(temp_yaw),
      -r * std::sin(temp_yaw),
      target_dz
    );
  }
  
  return positions;
}

double VirtualArmorGenerator::calculateArmorYaw(double base_yaw, int armor_index, int armors_num) const {
  return base_yaw + armor_index * (2.0 * M_PI / armors_num);
}

rm_interfaces::msg::Armor VirtualArmorGenerator::createArmorMsg(
    const Eigen::Vector3d& position,
    double yaw,
    const std::string& armor_id,
    const std::string& armor_type) const {
  
  rm_interfaces::msg::Armor armor;
  
  armor.number = armor_id;
  armor.type = armor_type;
  armor.distance_to_image_center = 0.0;  // 虚拟装甲板没有图像信息
  
  // 设置位置
  armor.pose.position.x = position.x();
  armor.pose.position.y = position.y();
  armor.pose.position.z = position.z();
  
  // 从 yaw 和 pitch 创建四元数
  // outpost 的装甲板向下倾斜，其他机器人的装甲板向上倾斜
  double pitch = (armor_id == "outpost") ? -ARMOR_PITCH_ANGLE : ARMOR_PITCH_ANGLE;
  tf2::Quaternion q;
  q.setRPY(0, pitch, yaw);
  armor.pose.orientation = tf2::toMsg(q);
  
  return armor;
}

}  // namespace fyt::auto_aim
