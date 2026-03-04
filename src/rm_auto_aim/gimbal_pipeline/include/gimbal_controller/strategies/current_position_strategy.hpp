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

private:
  double pitch_offset_{0.0};  // pitch手动补偿 (度)
  double yaw_offset_{0.0};    // yaw手动补偿 (度)
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__STRATEGIES__CURRENT_POSITION_STRATEGY_HPP_
