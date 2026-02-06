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

#include "gimbal_controller/mpc_control_strategy.hpp"

namespace gimbal_controller
{

rm_interfaces::msg::GimbalCmd MpcControlStrategy::solve(
  const GimbalControlContext & context)
{
  // MPC 策略预留接口，暂时返回空闲命令
  // TODO: 实现基于 MPC 的云台控制逻辑
  // 可参考 trajectory_planner 中的 Python 实现

  (void)context;  // 避免未使用参数警告

  return createIdleCmd();
}

void MpcControlStrategy::setMpcParameters(
  int prediction_horizon,
  int control_horizon,
  double dt)
{
  prediction_horizon_ = prediction_horizon;
  control_horizon_ = control_horizon;
  dt_ = dt;
}

}  // namespace gimbal_controller
