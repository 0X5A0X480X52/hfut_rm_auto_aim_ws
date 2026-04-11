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

#ifndef GIMBAL_CONTROLLER__STRATEGIES__STATE_MACHINE_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__STRATEGIES__STATE_MACHINE_STRATEGY_HPP_

#include "gimbal_controller/delay_management/delay_semantic_manager.hpp"
#include "gimbal_controller/gimbal_control_strategy.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace gimbal_controller
{

/**
 * @brief 状态机式装甲选择策略
 *
 * 将装甲板选择逻辑结构化为四种离散状态:
 *  - LOST:   无目标或目标丢失, 返回空闲命令
 *  - CENTER: 瞄准目标中心 (所有装甲板 facing 不满足条件时)
 *  - SINGLE: 单装甲板锁定 (低速旋转, 有正面装甲板时)
 *  - SPIN:   高速自旋模式, 使用 decision_angle 预测装甲板轮换
 *
 * 统一考虑 "朝向 + 最小运动 + 自旋 + hysteresis + 目标中心", 
 * 提高对高速自旋目标的稳定性和鲁棒性.
 */
class StateMachineStrategy : public GimbalControlStrategy
{
public:
  /**
   * @brief 装甲选择状态
   */
  enum class State
  {
    LOST,     ///< 无目标或目标丢失
    CENTER,   ///< 瞄准目标中心
    SINGLE,   ///< 单装甲板锁定 (低速旋转)
    SPIN      ///< 高速旋转, decision_angle 选板
  };

  StateMachineStrategy() = default;
  ~StateMachineStrategy() override = default;

  /**
   * @brief 执行策略
   * @param context 控制上下文
   * @return 云台控制命令
   */
  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) override;

  /**
   * @brief 获取策略名称
   */
  std::string getName() const override { return "StateMachineStrategy"; }

  /**
   * @brief 设置 Facing 过滤参数 (hysteresis 双阈值)
   * @param enter_angle 进入阈值 (度)
   * @param exit_angle 退出阈值 (度)
   */
  void setFacingParameters(double enter_angle, double exit_angle);

  /**
   * @brief 设置自旋检测参数
   * @param spin_thresh 进入 SPIN 模式的角速度阈值 (rad/s)
   * @param calm_thresh 退出 SPIN 模式的角速度阈值 (rad/s)
   * @param enter_count 进入 SPIN 所需连续超阈值帧数
   * @param exit_count 退出 SPIN 所需连续低于阈值帧数
   * @param side_angle SPIN 模式的跳板角度 (度)
   */
  void setSpinParameters(
    double spin_thresh,
    double calm_thresh,
    int enter_count,
    int exit_count,
    double side_angle);

  /**
   * @brief 设置预测参数
   * @param prediction_delay 额外预测延迟 (秒)
   * @param max_prediction_time 最大预测时间 (秒)
   */
  void setPredictionParameters(double prediction_delay, double max_prediction_time = 0.5);

  /**
   * @brief 设置 processing_delay 上限
   * @param max_processing_delay 最大处理延迟上限 (秒)
   */
  void setMaxProcessingDelay(double max_processing_delay);

  /**
   * @brief 设置触发到出膛延迟
   * @param trigger_to_muzzle_s 出膛延迟 (秒)
   */
  void setTriggerToMuzzleDelay(double trigger_to_muzzle_s);

  /**
   * @brief 设置手动补偿参数
   * @param pitch_offset pitch补偿 (度)
   * @param yaw_offset yaw补偿 (度)
   */
  void setManualOffset(double pitch_offset, double yaw_offset);

  /**
   * @brief 获取当前状态 (用于调试/可视化)
   */
  State getState() const { return state_; }

  /**
   * @brief 获取状态名称字符串
   */
  static std::string stateToString(State s);

private:
  // ======================== 状态处理 ========================

  /**
   * @brief LOST 状态处理: 无目标, 返回空闲
   */
  rm_interfaces::msg::GimbalCmd handleLost(const GimbalControlContext & context);

  /**
   * @brief CENTER 状态处理: 瞄准目标中心
   */
  rm_interfaces::msg::GimbalCmd handleCenter(const GimbalControlContext & context);

  /**
   * @brief SINGLE 状态处理: 锁定单个装甲板
   */
  rm_interfaces::msg::GimbalCmd handleSingle(const GimbalControlContext & context);

  /**
   * @brief SPIN 状态处理: decision_angle 选板
   */
  rm_interfaces::msg::GimbalCmd handleSpin(const GimbalControlContext & context);

  // ======================== 辅助方法 ========================

  /**
   * @brief 计算预测时间
   */
  double computePredictionTime(
    const GimbalControlContext & context,
    const Eigen::Vector3d & current_center);

  /**
   * @brief 在正面装甲板中选择最优板 (最小运动)
   * @param armor_positions 装甲板位置列表
   * @param facing_angles 各板 facing angle
   * @param threshold_deg 朝向阈值 (度)
   * @param current_yaw 当前云台 yaw
   * @param current_pitch 当前云台 pitch
   * @return 选中索引, -1 表示没有满足条件的板
   */
  int selectBestFacingArmor(
    const std::vector<Eigen::Vector3d> & armor_positions,
    const std::vector<double> & facing_angles,
    double threshold_deg,
    double current_yaw,
    double current_pitch) const;

  /**
   * @brief 构建云台控制命令
   */
  rm_interfaces::msg::GimbalCmd buildCommand(
    const GimbalControlContext & context,
    const Eigen::Vector3d & control_target,
    const Eigen::Vector3d & fire_target,
    double fire_distance,
    bool force_fire = false);

  /**
   * @brief 判断两个装甲板索引是否相邻 (考虑首尾相连)
   */
  static bool isAdjacentArmor(int idx_a, int idx_b, int num_armors);

  /**
   * @brief 重置状态机
   */
  void resetStateMachine();

  // ======================== 状态 ========================
  State state_{State::LOST};

  int locked_armor_index_{-1};      ///< SINGLE 模式锁定的装甲板索引
  int spin_decision_index_{-1};     ///< SPIN 模式当前 decision_angle 选中的索引
  int spin_count_{0};               ///< 自旋检测计数器
  int calm_count_{0};               ///< 减速检测计数器
  Eigen::Vector3d last_target_position_{Eigen::Vector3d::Zero()}; ///< 上次目标位置

  // ======================== 参数 ========================

  // Facing hysteresis
  double facing_enter_angle_{40.0};   ///< 进入阈值 (度)
  double facing_exit_angle_{55.0};    ///< 退出阈值 (度)

  // 自旋检测
  double spin_v_yaw_thresh_{4.0};     ///< 进入 SPIN 的角速度阈值 (rad/s)
  double calm_v_yaw_thresh_{2.0};     ///< 退出 SPIN 的角速度阈值 (rad/s)
  int spin_enter_count_{5};           ///< 进入 SPIN 所需计数
  int spin_exit_count_{5};            ///< 退出 SPIN 所需计数
  double side_angle_{15.0};           ///< SPIN 模式跳板角度 (度)

  // 预测参数
  double prediction_delay_{0.0};      ///< 额外预测延迟 (秒)
  double max_prediction_time_{0.5};   ///< 最大预测时间 (秒)
  double max_processing_delay_s_{0.5};  ///< processing_delay 上限 (秒)
  delay_management::DelaySemanticManager delay_manager_;
  double last_processing_delay_s_{0.0};
  double last_flight_time_s_{0.0};
  double last_total_prediction_time_s_{0.0};
  double trigger_to_muzzle_s_{0.0};

  // 手动补偿
  double pitch_offset_{0.0};          ///< pitch 手动补偿 (度)
  double yaw_offset_{0.0};            ///< yaw 手动补偿 (度)
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__STRATEGIES__STATE_MACHINE_STRATEGY_HPP_
