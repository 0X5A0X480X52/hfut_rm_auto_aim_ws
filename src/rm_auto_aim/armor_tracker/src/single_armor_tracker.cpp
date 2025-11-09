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

#include "armor_tracker/single_armor_tracker.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace fyt::auto_aim {

SingleArmorTracker::SingleArmorTracker(
  const std::string& armor_id,
  const std::string& armor_type,
  double max_match_distance,
  double max_match_yaw_diff,
  int tracking_threshold,
  int lost_threshold)
: armor_id_(armor_id)
, armor_type_(armor_type)
, state_(State::DETECTING)
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, tracking_threshold_(tracking_threshold)
, lost_threshold_(lost_threshold)
, detect_count_(0)
, lost_count_(0)
, last_yaw_(0.0)
{
}

void SingleArmorTracker::initEKF(const Armor& armor) {
  // 构建过程噪声协方差矩阵
  ArmorEKF::StateMatrix Q = ArmorEKF::StateMatrix::Identity();
  Q.diagonal() << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 1.0, 1.0;
  
  // 构建测量噪声协方差矩阵
  ArmorEKF::MeasurementMatrix R = ArmorEKF::MeasurementMatrix::Identity();
  R.diagonal() << 0.05, 0.05, 0.05, 0.02;
  
  ekf_ = std::make_unique<ArmorEKF>(Q, R);
  
  // 初始化状态
  ArmorEKF::StateVector initial_state;
  initial_state << 
    armor.pose.position.x, 0.0,  // x, vx
    armor.pose.position.y, 0.0,  // y, vy
    armor.pose.position.z, 0.0,  // z, vz
    quaternionToYaw(armor.pose.orientation), 0.0;  // yaw, vyaw
  
  ekf_->init(initial_state);
  last_yaw_ = quaternionToYaw(armor.pose.orientation);
}

void SingleArmorTracker::update(const Armor* armor, double dt) {
  if (!ekf_) {
    if (armor) {
      initEKF(*armor);
      detect_count_ = 1;
      lost_count_ = 0;
      state_ = State::DETECTING;
    }
    return;
  }

  // 预测步骤
  ekf_->predict(dt);

  if (armor != nullptr) {
    // 有检测结果,进行更新
    double current_yaw = quaternionToYaw(armor->pose.orientation);
    
    // 检查是否发生装甲板跳变
    double yaw_diff = angles::shortest_angular_distance(last_yaw_, current_yaw);
    if (std::abs(yaw_diff) > max_match_yaw_diff_) {
      handleArmorJump(*armor);
    }
    
    // 更新EKF
    ArmorEKF::MeasurementVector measurement;
    measurement << 
      armor->pose.position.x,
      armor->pose.position.y,
      armor->pose.position.z,
      current_yaw;
    
    ekf_->update(measurement);
    
    last_yaw_ = current_yaw;
    last_detected_time_ = rclcpp::Time(0);  // 需要从外部传入真实时间戳
    
    // 更新状态机
    detect_count_++;
    lost_count_ = 0;
    
    if (state_ == State::DETECTING && detect_count_ >= tracking_threshold_) {
      state_ = State::TRACKING;
    } else if (state_ == State::TEMP_LOST) {
      state_ = State::TRACKING;
    }
  } else {
    // 没有检测结果
    lost_count_++;
    detect_count_ = 0;
    
    if (lost_count_ >= lost_threshold_) {
      state_ = State::LOST;
    } else if (state_ == State::TRACKING) {
      state_ = State::TEMP_LOST;
    }
  }
}

rm_interfaces::msg::TrackedArmor SingleArmorTracker::getTrackedArmor(
  const builtin_interfaces::msg::Time& timestamp) const {
  rm_interfaces::msg::TrackedArmor tracked;
  
  tracked.header.stamp = timestamp;
  tracked.armor_id = armor_id_;
  tracked.armor_type = armor_type_;
  
  if (ekf_) {
    auto pos = ekf_->getPosition();
    auto vel = ekf_->getVelocity();
    
    tracked.position.x = pos.x();
    tracked.position.y = pos.y();
    tracked.position.z = pos.z();
    
    tracked.velocity.x = vel.x();
    tracked.velocity.y = vel.y();
    tracked.velocity.z = vel.z();
    
    tracked.yaw = ekf_->getYaw();
    tracked.yaw_velocity = ekf_->getYawVelocity();
  }
  
  tracked.tracking_state = static_cast<uint8_t>(state_);
  tracked.tracking_count = detect_count_;
  tracked.lost_count = lost_count_;
  tracked.last_detected_time = last_detected_time_;
  
  // 计算置信度
  if (state_ == State::TRACKING) {
    tracked.confidence = 1.0;
  } else if (state_ == State::DETECTING) {
    tracked.confidence = static_cast<double>(detect_count_) / tracking_threshold_;
  } else if (state_ == State::TEMP_LOST) {
    tracked.confidence = 1.0 - static_cast<double>(lost_count_) / lost_threshold_;
  } else {
    tracked.confidence = 0.0;
  }
  
  return tracked;
}

bool SingleArmorTracker::isMatched(const Armor& armor) const {
  if (!ekf_) return false;
  
  // 检查ID是否匹配
  if (armor.number != armor_id_) return false;
  
  // 检查距离
  auto predicted_pos = ekf_->getPosition();
  Eigen::Vector3d detected_pos(
    armor.pose.position.x,
    armor.pose.position.y,
    armor.pose.position.z
  );
  
  double distance = (predicted_pos - detected_pos).norm();
  if (distance > max_match_distance_) return false;
  
  // 检查yaw角差
  double detected_yaw = quaternionToYaw(armor.pose.orientation);
  double yaw_diff = std::abs(angles::shortest_angular_distance(
    ekf_->getYaw(), detected_yaw));
  
  if (yaw_diff > max_match_yaw_diff_) return false;
  
  return true;
}

double SingleArmorTracker::quaternionToYaw(
  const geometry_msgs::msg::Quaternion& q) const {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void SingleArmorTracker::handleArmorJump(const Armor& armor) {
  // 装甲板发生跳变,调整状态
  // 这里可以根据机器人模型来调整位置估计
  // 简单起见,我们重新初始化yaw
  if (ekf_) {
    auto state = ekf_->getState();
    state(6) = quaternionToYaw(armor.pose.orientation);  // 更新yaw
    state(7) = 0.0;  // 重置yaw速度
    ekf_->setState(state);
  }
}

}  // namespace fyt::auto_aim
