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

namespace gimbal_controller
{

/**
 * @brief 开火建议器
 * 
 * 根据当前云台姿态和目标位置判断是否应该开火
 */
class FireAdvisor
{
public:
  FireAdvisor() = default;
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
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__FIRE_ADVISOR_HPP_
