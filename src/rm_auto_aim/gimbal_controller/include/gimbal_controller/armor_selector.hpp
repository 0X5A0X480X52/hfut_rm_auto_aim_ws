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

#ifndef GIMBAL_CONTROLLER__ARMOR_SELECTOR_HPP_
#define GIMBAL_CONTROLLER__ARMOR_SELECTOR_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

namespace gimbal_controller
{

/**
 * @brief 装甲板选择结果
 */
struct ArmorSelectionResult
{
  int selected_index;           // 选中的装甲板索引
  Eigen::Vector3d position;     // 选中装甲板的位置
  double gimbal_movement;       // 云台移动量 (yaw^2 + pitch^2)
  double distance;              // 目标距离
};

/**
 * @brief 装甲板选择器
 * 
 * 根据当前云台姿态和装甲板位置，选择最优的打击目标
 */
class ArmorSelector
{
public:
  ArmorSelector() = default;
  ~ArmorSelector() = default;

  /**
   * @brief 设置选择参数
   * @param side_angle 侧向角度阈值 (度)
   * @param min_switching_v_yaw 最小切换角速度阈值
   */
  void setParameters(double side_angle, double min_switching_v_yaw);

  /**
   * @brief 选择最佳装甲板 (基于云台移动最小)
   * @param armor_positions 各装甲板的世界坐标位置
   * @param current_yaw 当前云台yaw角 (弧度)
   * @param current_pitch 当前云台pitch角 (弧度)
   * @return 选择结果
   */
  ArmorSelectionResult selectByMinMovement(
    const std::vector<Eigen::Vector3d> & armor_positions,
    double current_yaw,
    double current_pitch) const;

  /**
   * @brief 选择最佳装甲板 (基于传统决策角)
   * @param armor_positions 各装甲板的世界坐标位置
   * @param target_center 目标中心位置
   * @param target_yaw 目标yaw角
   * @param target_v_yaw 目标yaw角速度
   * @return 选择结果索引
   */
  int selectByDecisionAngle(
    const std::vector<Eigen::Vector3d> & armor_positions,
    const Eigen::Vector3d & target_center,
    double target_yaw,
    double target_v_yaw) const;

  /**
   * @brief 过滤掉距离最远的装甲板
   * @param armor_positions 装甲板位置列表
   * @return 过滤后的索引列表
   */
  std::vector<int> filterByDistance(
    const std::vector<Eigen::Vector3d> & armor_positions) const;

  /**
   * @brief 计算从当前云台位置到目标位置的yaw和pitch角
   * @param target_position 目标位置
   * @param current_yaw 当前云台yaw角 (用于参考)
   * @param[out] yaw 目标yaw角
   * @param[out] pitch 目标pitch角
   */
  static void calculateYawPitch(
    const Eigen::Vector3d & target_position,
    double current_yaw,
    double & yaw,
    double & pitch);

private:
  double side_angle_{15.0};           // 侧向角度阈值 (度)
  double min_switching_v_yaw_{1.0};   // 最小切换角速度阈值
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__ARMOR_SELECTOR_HPP_
