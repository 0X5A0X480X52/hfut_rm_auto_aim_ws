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

#ifndef GIMBAL_CONTROLLER__MPC__MPC_REFERENCE_GENERATOR_HPP_
#define GIMBAL_CONTROLLER__MPC__MPC_REFERENCE_GENERATOR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_controller/mpc/gimbal_dynamics_model.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"

namespace gimbal_controller
{
namespace mpc
{

/**
 * @brief MPC 参考轨迹生成器
 *
 * 对未来 N 步逐步执行:
 *   propagate TrackedRobot → calculatePredicted → selectBest → compensate
 * 生成 yaw/pitch 参考序列 X_ref[1..N]
 *
 * 复用已有组件:
 *   - ArmorPositionCalculator: 预测装甲板位置
 *   - ArmorSelector: 选板 (SelectionMethod 可配置)
 *   - LocalTrajectoryCompensator: 弹道解算
 */
/**
 * @brief 延时补偿配置
 */
struct DelayCompConfig
{
  double base_delay_s{0.0};     // processing_delay + controller_delay + prediction_delay
  double ctrl_delay_s{0.0};     // 控制延迟 (秒), 用于延时感知选板和开火判断
  int flight_time_iters{2};     // 飞行时间迭代次数
};

class MpcReferenceGenerator
{
public:
  MpcReferenceGenerator() = default;
  ~MpcReferenceGenerator() = default;

  /**
   * @brief 注入组件依赖
   */
  void setComponents(
    std::shared_ptr<ArmorPositionCalculator> position_calculator,
    std::shared_ptr<ArmorSelector> armor_selector,
    std::shared_ptr<LocalTrajectoryCompensator> local_compensator)
  {
    position_calculator_ = position_calculator;
    armor_selector_ = armor_selector;
    local_compensator_ = local_compensator;
  }

  /**
   * @brief 生成 N 步参考轨迹
   *
   * @param target_robot 当前被跟踪目标状态
   * @param current_yaw 当前云台 yaw (弧度)
   * @param current_pitch 当前云台 pitch (弧度)
   * @param N 预测步数
   * @param dt 时间步长 (秒)
   * @return X_ref (4N × 1) flatten 的参考状态序列
   *         每 4 个元素为 [yaw_ref, pitch_ref, yaw_dot_ref, pitch_dot_ref]
   */
  Eigen::VectorXd generate(
    const rm_interfaces::msg::TrackedRobot & target_robot,
    double current_yaw,
    double current_pitch,
    int N,
    double dt) const;

  /**
   * @brief 生成带延时补偿的 N 步参考轨迹
   *
   * 在 generate() 基础上增加:
   *   1. base_delay 偏移: 参考轨迹起点前移 base_delay 秒
   *   2. 子弹飞行时间迭代补偿
   *   3. 延时感知选板: 以延迟后的云台姿态作为选板参考
   *
   * @param target_robot 当前被跟踪目标状态
   * @param current_yaw 当前云台 yaw (弧度)
   * @param current_pitch 当前云台 pitch (弧度)
   * @param N 预测步数
   * @param dt 时间步长 (秒)
   * @param delay_config 延时补偿配置
   * @return X_ref (4N × 1) flatten 的参考状态序列
   */
  Eigen::VectorXd generateWithDelay(
    const rm_interfaces::msg::TrackedRobot & target_robot,
    double current_yaw,
    double current_pitch,
    int N,
    double dt,
    const DelayCompConfig & delay_config) const;

private:
  /**
   * @brief 将 TrackedRobot 状态向前传播 dt 秒
   * 用于在预测窗口内推演目标未来状态
   */
  static rm_interfaces::msg::TrackedRobot propagateRobot(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt);

  std::shared_ptr<ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<ArmorSelector> armor_selector_;
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator_;
};

}  // namespace mpc
}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__MPC__MPC_REFERENCE_GENERATOR_HPP_
