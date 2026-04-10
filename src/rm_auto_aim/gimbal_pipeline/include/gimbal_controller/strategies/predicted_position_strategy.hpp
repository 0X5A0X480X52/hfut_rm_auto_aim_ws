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

#ifndef GIMBAL_CONTROLLER__STRATEGIES__PREDICTED_POSITION_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__STRATEGIES__PREDICTED_POSITION_STRATEGY_HPP_

#include "gimbal_controller/delay_management/delay_semantic_manager.hpp"
#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/adaptive_delay_controller.hpp"

namespace gimbal_controller
{

/**
 * @brief 预测位置选板策略
 * 
 * 基于飞行时间预测目标位置后进行选板和云台控制。
 * 适用于云台跟踪和高精度打击场景。
 */
class PredictedPositionStrategy : public GimbalControlStrategy
{
public:
  PredictedPositionStrategy() = default;
  ~PredictedPositionStrategy() override = default;

  /**
   * @brief 执行策略
   * @param context 控制上下文
   * @return 云台控制命令
   */
  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) override;

  /**
   * @brief 获取策略名称
   */
  std::string getName() const override { return "PredictedPositionStrategy"; }

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
   * @brief 设置手动补偿参数
   * @param pitch_offset pitch补偿 (度)
   * @param yaw_offset yaw补偿 (度)
   */
  void setManualOffset(double pitch_offset, double yaw_offset);

  /**
   * @brief 设置高转速跟踪中心模式参数
   * @param max_tracking_v_yaw 触发跟踪中心的角速度阈值
   * @param transfer_thresh 状态切换阈值
   */
  void setTrackingCenterParams(double max_tracking_v_yaw, int transfer_thresh);

  /**
   * @brief 设置 controller_delay 参数
   * 
   * 与 armor_solver 原版一致：在 TRACKING_ARMOR 状态下，若 controller_delay > 0，
   * 则在 total_prediction_time 基础上额外叠加 controller_delay 秒作为云台控制目标；
   * 开火判断仍基于当前位置，不受影响。
   * @param controller_delay 额外云台前馈延迟 (秒, 0 表示禁用)
   */
  void setControllerDelay(double controller_delay);

  /**
   * @brief 配置自适应 delay AIMD 参数
   * @param enable              是否启用自适应模式（false 时退化为静态 controller_delay）
   * @param initial_delay       初始 delay，即 controller.solver.controller_delay 的值 (秒)
   * @param min_delay           delay 下限 (秒)
   * @param max_delay           delay 上限 (秒)
   * @param add_step            开火成功时每帧减小量 (秒)
   * @param mul_factor          未开火时乘性增幅因子 (>1.0)
   * @param fire_wait_threshold 连续未开火超过此帧数后才开始增大
   * @param max_linear_speed    线速度归一化参考值 (m/s)
   * @param max_angular_speed   角速度归一化参考值 (rad/s)
   */
  void setAdaptiveDelayParams(
    bool enable,
    double initial_delay,
    double min_delay,
    double max_delay,
    double add_step,
    double mul_factor,
    int    fire_wait_threshold,
    double max_linear_speed,
    double max_angular_speed);

private:
  double prediction_delay_{0.0};      // 额外预测延迟 (秒)
  double max_prediction_time_{0.5};   // 最大预测时间 (秒)
  double max_processing_delay_s_{0.5};  // processing_delay 上限 (秒)
  double controller_delay_{0.0};      // 云台前馈延迟 (秒, 0=禁用)
  double pitch_offset_{0.0};          // pitch手动补偿 (度)
  double yaw_offset_{0.0};            // yaw手动补偿 (度)
  double max_tracking_v_yaw_{6.0};    // 触发跟踪中心的角速度阈值
  int transfer_thresh_{5};            // 状态切换阈值
  int overflow_count_{0};             // 溢出计数

  // 自适应 delay AIMD
  bool adaptive_delay_enabled_{false};
  AdaptiveDelayController adaptive_ctrl_;
  delay_management::DelaySemanticManager delay_manager_;

  enum TrackingState { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 };
  TrackingState state_{TRACKING_ARMOR};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__STRATEGIES__PREDICTED_POSITION_STRATEGY_HPP_
