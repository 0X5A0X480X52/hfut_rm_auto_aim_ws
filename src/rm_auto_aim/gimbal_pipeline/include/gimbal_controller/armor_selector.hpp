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
  int selected_index{-1};       // 选中的装甲板索引
  Eigen::Vector3d position;     // 选中装甲板的位置
  double gimbal_movement{0.0};  // 云台移动量 (yaw^2 + pitch^2)
  double distance{0.0};         // 目标距离
  double facing_angle{0.0};     // 选中装甲板的朝向角 (弧度, 0=正面)
  bool is_center_fallback{false}; // 是否因全部过滤而 fallback 到车体中心
};

/**
 * @brief 装甲板选择器
 * 
 * 根据当前云台姿态和装甲板位置，选择最优的打击目标。
 * 支持:
 *  - 基础最小运动选板 (selectByMinMovement)
 *  - 带正面朝向过滤+hysteresis 的选板 (selectByMinMovementWithFacing)
 *  - 基于决策角的选板 (selectByDecisionAngle)
 */
class ArmorSelector
{
public:
  /**
   * @brief 选板策略枚举
   *  - MIN_MOVEMENT_WITH_FACING : 最小运动量 + 正面朝向 Hysteresis 过滤 (默认)
   *  - MIN_MOVEMENT             : 最小运动量，无朝向过滤
   *  - DECISION_ANGLE           : 传统决策角算法 (与 armor_solver 原版一致)
   */
  enum class SelectionMethod
  {
    MIN_MOVEMENT_WITH_FACING = 0,
    MIN_MOVEMENT             = 1,
    DECISION_ANGLE           = 2,
  };

  ArmorSelector() = default;
  ~ArmorSelector() = default;

  /**
   * @brief 设置选板策略
   * @param method 选板方法枚举
   */
  void setSelectionMethod(SelectionMethod method);

  /**
   * @brief 统一选板入口，根据 setSelectionMethod 配置路由到对应算法
   * 
   * @param armor_positions 各装甲板的世界坐标位置
   * @param target_center   目标机器人中心位置
   * @param target_yaw      目标机器人 yaw 角 (弧度)
   * @param num_armors      装甲板总数
   * @param target_v_yaw    目标 yaw 角速度 (仅 DECISION_ANGLE 模式使用)
   * @param current_yaw     当前云台 yaw 角 (弧度)
   * @param current_pitch   当前云台 pitch 角 (弧度)
   * @return 选择结果
   */
  ArmorSelectionResult selectBest(
    const std::vector<Eigen::Vector3d> & armor_positions,
    const Eigen::Vector3d & target_center,
    double target_yaw,
    int num_armors,
    double target_v_yaw,
    double current_yaw,
    double current_pitch);

  /**
   * @brief 设置基础选择参数
   * @param side_angle 侧向角度阈值 (度)
   * @param min_switching_v_yaw 最小切换角速度阈值
   */
  void setParameters(double side_angle, double min_switching_v_yaw);

  /**
   * @brief 设置 Facing 过滤参数 (hysteresis 双阈值)
   * @param enter_angle 进入阈值 (度): 未锁定时，facing angle ≤ enter 才允许选中
   * @param exit_angle 退出阈值 (度): 已锁定时，facing angle > exit 才释放
   */
  void setFacingParameters(double enter_angle, double exit_angle);

  /**
   * @brief 重置内部记忆状态 (目标丢失时调用)
   */
  void resetState();

  /**
   * @brief 选择最佳装甲板 (基于云台移动最小, 无 facing 过滤)
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
   * @brief 选择最佳装甲板 (基于云台移动最小 + Facing 过滤 + Hysteresis)
   * 
   * 对每个装甲板计算其法向量与云台视线方向的夹角 (facing angle),
   * 使用双阈值 (enter/exit) 进行 hysteresis 过滤,
   * 然后在通过过滤的装甲板中选择云台移动最小的目标。
   * 如果所有装甲板都被过滤掉, 则 fallback 到目标中心。
   * 
   * @param armor_positions 各装甲板的世界坐标位置
   * @param target_center 目标机器人中心位置
   * @param target_yaw 目标机器人 yaw 角 (弧度)
   * @param num_armors 装甲板总数
   * @param current_yaw 当前云台 yaw 角 (弧度)
   * @param current_pitch 当前云台 pitch 角 (弧度)
   * @return 选择结果 (注意检查 is_center_fallback)
   */
  ArmorSelectionResult selectByMinMovementWithFacing(
    const std::vector<Eigen::Vector3d> & armor_positions,
    const Eigen::Vector3d & target_center,
    double target_yaw,
    int num_armors,
    double current_yaw,
    double current_pitch);

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

  /**
   * @brief 计算每个装甲板的 facing angle (法向量与视线夹角)
   * @param armor_positions 各装甲板位置
   * @param target_yaw 目标 yaw 角 (弧度)
   * @param num_armors 装甲板总数
   * @return 每个装甲板的 facing angle (弧度, 0=正面, π/2=侧面)
   */
  static std::vector<double> computeFacingAngles(
    const std::vector<Eigen::Vector3d> & armor_positions,
    double target_yaw,
    int num_armors);

private:
  double side_angle_{15.0};           // 侧向角度阈值 (度)
  double min_switching_v_yaw_{1.0};   // 最小切换角速度阈值

  // Facing hysteresis 参数
  double facing_enter_angle_{40.0};   // 进入阈值 (度)
  double facing_exit_angle_{55.0};    // 退出阈值 (度)

  // 选板策略
  SelectionMethod selection_method_{SelectionMethod::MIN_MOVEMENT_WITH_FACING};

  // 记忆上次选择 (用于 hysteresis)
  mutable int last_selected_index_{-1};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__ARMOR_SELECTOR_HPP_
