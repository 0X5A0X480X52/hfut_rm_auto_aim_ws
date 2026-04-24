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

#ifndef GIMBAL_CONTROLLER__FIRE_ADVISOR_HPP_
#define GIMBAL_CONTROLLER__FIRE_ADVISOR_HPP_

#include <cmath>
#include <memory>

namespace gimbal_controller
{

struct FireAdviceInput
{
  double current_yaw{0.0};
  double current_pitch{0.0};
  double target_yaw{0.0};
  double target_pitch{0.0};
  double distance{0.0};

  double muzzle_delay_s{0.0};
  double yaw_rate{0.0};
  double pitch_rate{0.0};
  double yaw_accel{0.0};
  double pitch_accel{0.0};
};

struct FireAdviceResult
{
  bool fire{false};
  double predicted_yaw_at_muzzle{0.0};
  double predicted_pitch_at_muzzle{0.0};
  double yaw_diff{0.0};
  double pitch_diff{0.0};
  double yaw_threshold{0.0};
  double pitch_threshold{0.0};
  double confidence{0.0};
};

class FireDecisionPolicy
{
public:
  virtual ~FireDecisionPolicy() = default;

  virtual bool decide(
    double yaw_diff,
    double pitch_diff,
    double yaw_threshold,
    double pitch_threshold) const = 0;
};

class AxisThresholdFireDecisionPolicy final : public FireDecisionPolicy
{
public:
  bool decide(
    double yaw_diff,
    double pitch_diff,
    double yaw_threshold,
    double pitch_threshold) const override;
};

class EllipseFireDecisionPolicy final : public FireDecisionPolicy
{
public:
  bool decide(
    double yaw_diff,
    double pitch_diff,
    double yaw_threshold,
    double pitch_threshold) const override;
};

/**
 * @brief 开火建议器
 * 
 * 根据当前云台姿态和目标位置判断是否应该开火
 */
class FireAdvisor
{
public:
  FireAdvisor();
  ~FireAdvisor() = default;

  /**
   * @brief 设置开火参数
   * @param shooting_range_w 射击范围宽度 (米)
   * @param shooting_range_h 射击范围高度 (米)
   * @param min_shooting_angle 最小射击角度 (度)
   */
  void setParameters(
    double shooting_range_w,
    double shooting_range_h,
    double min_shooting_angle = 1.0);

  /**
   * @brief 判断是否应该开火
   * @param current_yaw 当前云台yaw角 (弧度)
   * @param current_pitch 当前云台pitch角 (弧度)
   * @param target_yaw 目标yaw角 (弧度)
   * @param target_pitch 目标pitch角 (弧度)
   * @param distance 目标距离 (米)
   * @return 是否应该开火
   */
  bool shouldFire(
    double current_yaw,
    double current_pitch,
    double target_yaw,
    double target_pitch,
    double distance) const;

  /**
   * @brief 延时感知开火判断
   * @param muzzle_delay_s 从当前时刻到子弹出膛的延时 (秒)
   * @param yaw_rate 云台yaw角速度 (弧度/秒)
   * @param pitch_rate 云台pitch角速度 (弧度/秒)
   * @param yaw_accel 云台yaw角加速度 (弧度/秒^2)
   * @param pitch_accel 云台pitch角加速度 (弧度/秒^2)
   */
  bool shouldFireWithDelay(
    double current_yaw,
    double current_pitch,
    double target_yaw,
    double target_pitch,
    double distance,
    double muzzle_delay_s,
    double yaw_rate = 0.0,
    double pitch_rate = 0.0,
    double yaw_accel = 0.0,
    double pitch_accel = 0.0) const;

  /**
   * @brief 统一 fire_advice 评估入口
   */
  FireAdviceResult evaluate(const FireAdviceInput & input) const;

  /**
   * @brief 设置判定策略（策略模式）
   */
  void setDecisionPolicy(std::shared_ptr<FireDecisionPolicy> decision_policy);

  /**
   * @brief 计算开火置信度
   * @param yaw_diff yaw偏差 (弧度)
   * @param pitch_diff pitch偏差 (弧度)
   * @param distance 目标距离 (米)
   * @return 开火置信度 [0, 1]
   */
  double getFireConfidence(
    double yaw_diff,
    double pitch_diff,
    double distance) const;

private:
  double shooting_range_w_{0.135};    // 射击范围宽度 (米)
  double shooting_range_h_{0.135};    // 射击范围高度 (米)
  double min_shooting_angle_{1.0};    // 最小射击角度 (度)

  std::shared_ptr<FireDecisionPolicy> decision_policy_;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__FIRE_ADVISOR_HPP_
