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

#ifndef GIMBAL_CONTROLLER__ADAPTIVE_DELAY_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER__ADAPTIVE_DELAY_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace gimbal_controller
{

/**
 * @brief 自适应 controller_delay AIMD 控制器
 *
 * 根据目标线速度、角速度以及连续未开火帧数动态调整 controller_delay：
 *   - 开火成功（fire_advice == true） → 线性减小 delay（Additive Decrease）
 *   - 连续未开火超过阈值             → 加权乘性增大 delay（Multiplicative Increase）
 *
 * 加权因子取 max(w_linear, w_angular)：
 *   w_linear  = min(|v_xyz| / max_linear_speed,  1.0)
 *   w_angular = min(|v_yaw| / max_angular_speed, 1.0)
 *
 * 使用示例（在策略 solve() 末尾）：
 * @code
 *   double v_lin = center_velocity.norm();
 *   double v_ang = std::abs(target_robot.yaw_velocity);
 *   adaptive_ctrl_.update(fire_advice, v_lin, v_ang);
 *   // 之后通过 getDelay() 获取本帧使用的延迟
 * @endcode
 */
class AdaptiveDelayController
{
public:
  AdaptiveDelayController() = default;

  /**
   * @brief 初始化参数
   * @param initial_delay   delay 初始值，也是 reset() 后的恢复值 (秒)
   * @param min_delay       delay 下限 (秒)
   * @param max_delay       delay 上限 (秒)
   * @param add_step        开火成功时每帧减小量 (秒)
   * @param mul_factor      未开火时乘性增幅因子 (>1.0)
   * @param fire_wait_threshold   连续未开火超过此帧数后才开始增大 delay
   * @param max_linear_speed  线速度归一化参考值 (m/s)
   * @param max_angular_speed 角速度归一化参考值 (rad/s)
   */
  void init(
    double initial_delay,
    double min_delay,
    double max_delay,
    double add_step,
    double mul_factor,
    int    fire_wait_threshold,
    double max_linear_speed,
    double max_angular_speed)
  {
    initial_delay_       = initial_delay;
    min_delay_           = min_delay;
    max_delay_           = max_delay;
    add_step_            = add_step;
    mul_factor_          = mul_factor;
    fire_wait_threshold_ = fire_wait_threshold;
    max_linear_speed_    = max_linear_speed;
    max_angular_speed_   = max_angular_speed;

    adaptive_delay_ = initial_delay_;
    no_fire_count_  = 0;
  }

  /**
   * @brief 重置为初始状态（跟丢目标时调用）
   */
  void reset()
  {
    adaptive_delay_ = initial_delay_;
    no_fire_count_  = 0;
  }

  /**
   * @brief 根据本帧 fire_advice 和目标速度更新 delay
   * @param fire_advice   本帧是否建议开火
   * @param v_linear      目标线速度大小 (m/s)，通常为 center_velocity.norm()
   * @param v_angular     目标角速度绝对值 (rad/s)，通常为 |yaw_velocity|
   */
  void update(bool fire_advice, double v_linear, double v_angular)
  {
    if (fire_advice) {
      // Additive Decrease：线性减小并重置计数
      adaptive_delay_ -= add_step_;
      adaptive_delay_  = std::max(adaptive_delay_, min_delay_);
      no_fire_count_   = 0;
    } else {
      // 累计未开火帧数
      no_fire_count_++;
      if (no_fire_count_ > fire_wait_threshold_) {
        // 归一化速度因子
        double w_linear  = std::min(v_linear  / std::max(max_linear_speed_,  1e-6), 1.0);
        double w_angular = std::min(v_angular / std::max(max_angular_speed_, 1e-6), 1.0);
        double speed_factor = std::max(w_linear, w_angular);

        // Multiplicative Increase：加权乘性增大
        adaptive_delay_ *= 1.0 + (mul_factor_ - 1.0) * speed_factor;
        adaptive_delay_  = std::min(adaptive_delay_, max_delay_);
      }
    }
  }

  /**
   * @brief 获取当前自适应 delay 值 (秒)
   */
  double getDelay() const { return adaptive_delay_; }

  /**
   * @brief 获取当前连续未开火帧数（调试用）
   */
  int getNoFireCount() const { return no_fire_count_; }

private:
  double initial_delay_{0.0};
  double min_delay_{0.0};
  double max_delay_{0.1};
  double add_step_{0.005};
  double mul_factor_{1.2};
  int    fire_wait_threshold_{10};
  double max_linear_speed_{3.0};
  double max_angular_speed_{10.0};

  double adaptive_delay_{0.0};
  int    no_fire_count_{0};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__ADAPTIVE_DELAY_CONTROLLER_HPP_
