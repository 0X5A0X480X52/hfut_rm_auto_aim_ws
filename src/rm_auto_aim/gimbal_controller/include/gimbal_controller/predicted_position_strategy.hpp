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

#ifndef GIMBAL_CONTROLLER__PREDICTED_POSITION_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__PREDICTED_POSITION_STRATEGY_HPP_

#include "gimbal_controller/gimbal_control_strategy.hpp"

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

private:
  double prediction_delay_{0.0};      // 额外预测延迟 (秒)
  double max_prediction_time_{0.5};   // 最大预测时间 (秒)
  double pitch_offset_{0.0};          // pitch手动补偿 (度)
  double yaw_offset_{0.0};            // yaw手动补偿 (度)
  double max_tracking_v_yaw_{6.0};    // 触发跟踪中心的角速度阈值
  int transfer_thresh_{5};            // 状态切换阈值
  int overflow_count_{0};             // 溢出计数

  enum TrackingState { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 };
  TrackingState state_{TRACKING_ARMOR};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__PREDICTED_POSITION_STRATEGY_HPP_
