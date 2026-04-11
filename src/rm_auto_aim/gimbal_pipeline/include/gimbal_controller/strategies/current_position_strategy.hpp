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

#ifndef GIMBAL_CONTROLLER__STRATEGIES__CURRENT_POSITION_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__STRATEGIES__CURRENT_POSITION_STRATEGY_HPP_

#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/adaptive_delay_controller.hpp"
#include "gimbal_controller/delay_management/delay_semantic_manager.hpp"

namespace gimbal_controller
{

/**
 * @brief 当前位置选板策略
 * 
 * 使用装甲板的当前位置进行选板和云台控制。
 * 适用于开火判断和低延迟场景。
 */
class CurrentPositionStrategy : public GimbalControlStrategy
{
public:
  CurrentPositionStrategy() = default;
  ~CurrentPositionStrategy() override = default;

  /**
   * @brief 执行策略
   * @param context 控制上下文
   * @return 云台控制命令
   */
  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) override;

  /**
   * @brief 获取策略名称
   */
  std::string getName() const override { return "CurrentPositionStrategy"; }

  /**
   * @brief 设置手动补偿参数
   * @param pitch_offset pitch补偿 (度)
   * @param yaw_offset yaw补偿 (度)
   */
  void setManualOffset(double pitch_offset, double yaw_offset);

  /**
   * @brief 设置 controller_delay 参数
   * 
   * 当 controller_delay > 0 时，用 calculatePredicted(dt=controller_delay) 计算
   * 云台控制目标（超前击打），开火判断仍使用当前位置。
   * @param controller_delay 前馈延迟 (秒, 0 表示禁用)
   */
  void setControllerDelay(double controller_delay);

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
   * @brief 配置自适应 delay AIMD 参数（与 PredictedPositionStrategy 同名接口）
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
  double pitch_offset_{0.0};       // pitch手动补偿 (度)
  double yaw_offset_{0.0};         // yaw手动补偿 (度)
  double controller_delay_{0.0};   // 云台前馈延迟 (秒, 0=禁用)
  double trigger_to_muzzle_s_{0.0};
  double max_processing_delay_s_{0.5};

  // 自适应 delay AIMD
  bool adaptive_delay_enabled_{false};
  AdaptiveDelayController adaptive_ctrl_;
  delay_management::DelaySemanticManager delay_manager_;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__STRATEGIES__CURRENT_POSITION_STRATEGY_HPP_
