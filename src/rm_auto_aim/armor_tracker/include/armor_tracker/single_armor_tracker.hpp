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

#ifndef ARMOR_TRACKER__SINGLE_ARMOR_TRACKER_HPP_
#define ARMOR_TRACKER__SINGLE_ARMOR_TRACKER_HPP_

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/tracked_armor.hpp"
#include "armor_tracker/ekf.hpp"

namespace fyt::auto_aim {

/**
 * @brief 单个装甲板跟踪器
 * 使用扩展卡尔曼滤波器(EKF)跟踪单个装甲板的状态
 */
class SingleArmorTracker {
public:
  using Armor = rm_interfaces::msg::Armor;
  using TrackedArmor = rm_interfaces::msg::TrackedArmor;

  enum class State {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2,
    TEMP_LOST = 3
  };

  /**
   * @brief 构造函数
   * @param armor_id 装甲板ID
   * @param armor_type 装甲板类型 ("small" 或 "large")
   * @param max_match_distance 最大匹配距离
   * @param max_match_yaw_diff 最大匹配yaw角度差
   * @param tracking_threshold 进入TRACKING状态的阈值
   * @param lost_threshold 进入LOST状态的阈值
   */
  SingleArmorTracker(
    const std::string& armor_id,
    const std::string& armor_type,
    double max_match_distance,
    double max_match_yaw_diff,
    int tracking_threshold,
    int lost_threshold
  );

  /**
   * @brief 初始化EKF
   * @param armor 检测到的装甲板
   */
  void initEKF(const Armor& armor);

  /**
   * @brief 更新跟踪器
   * @param armor 检测到的装甲板 (可选,如果未检测到则传nullptr)
   * @param dt 时间间隔
   */
  void update(const Armor* armor, double dt);

  /**
   * @brief 获取跟踪状态
   */
  TrackedArmor getTrackedArmor(const builtin_interfaces::msg::Time& timestamp) const;

  /**
   * @brief 判断是否匹配
   * @param armor 检测到的装甲板
   * @return 是否匹配
   */
  bool isMatched(const Armor& armor) const;

  /**
   * @brief 获取当前状态
   */
  State getState() const { return state_; }

  /**
   * @brief 获取装甲板ID
   */
  std::string getArmorId() const { return armor_id_; }

  /**
   * @brief 是否应该被删除
   */
  bool shouldBeRemoved() const { return state_ == State::LOST; }

private:
  std::string armor_id_;
  std::string armor_type_;
  
  State state_;
  
  std::unique_ptr<ArmorEKF> ekf_;
  
  double max_match_distance_;
  double max_match_yaw_diff_;
  
  int tracking_threshold_;
  int lost_threshold_;
  
  int detect_count_;
  int lost_count_;
  
  double last_yaw_;
  
  builtin_interfaces::msg::Time last_detected_time_;

  /**
   * @brief 从四元数获取yaw角
   */
  double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const;

  /**
   * @brief 处理装甲板跳变
   */
  void handleArmorJump(const Armor& armor);
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__SINGLE_ARMOR_TRACKER_HPP_
