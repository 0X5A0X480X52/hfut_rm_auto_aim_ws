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

#ifndef GIMBAL_CONTROLLER__MPC_CONTROL_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__MPC_CONTROL_STRATEGY_HPP_

#include "gimbal_controller/gimbal_control_strategy.hpp"

namespace gimbal_controller
{

/**
 * @brief MPC 控制策略 (预留接口)
 * 
 * 基于模型预测控制的云台控制策略。
 * 当前为预留接口，暂不实现具体逻辑。
 */
class MpcControlStrategy : public GimbalControlStrategy
{
public:
  MpcControlStrategy() = default;
  ~MpcControlStrategy() override = default;

  /**
   * @brief 执行策略
   * @param context 控制上下文
   * @return 云台控制命令
   */
  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) override;

  /**
   * @brief 获取策略名称
   */
  std::string getName() const override { return "MpcControlStrategy"; }

  /**
   * @brief 设置 MPC 参数 (预留)
   * @param prediction_horizon 预测时域
   * @param control_horizon 控制时域
   * @param dt 时间步长
   */
  void setMpcParameters(int prediction_horizon, int control_horizon, double dt);

private:
  int prediction_horizon_{18};    // 预测时域
  int control_horizon_{10};       // 控制时域
  double dt_{0.01};               // 时间步长
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__MPC_CONTROL_STRATEGY_HPP_
