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

#include "robot_pose_estimator/robot_tracker.hpp"

#include <cmath>
#include <cfloat>
#include <numeric>

#include <angles/angles.h>
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

RobotTracker::RobotTracker(const PoseEstimatorConfig& config)
  : config_(config)
  , target_state_(Eigen::VectorXd::Zero(ROBOT_STATE_DIM))
  , measurement_(Eigen::VectorXd::Zero(ROBOT_MEAS_DIM)) {
}

void RobotTracker::init(const ArmorState& armor) {
  robot_id_ = armor.armor_id;
  determineRobotType(armor);
  initEKF(armor);
  
  state_ = State::DETECTING;
  detect_count_ = 0;
  lost_count_ = 0;
  last_armor_ = armor;
  // 初始化置信度窗口
  confidence_window_.clear();
  confidence_window_.push_back(armor.confidence > 0.0 ? armor.confidence : 1.0);
  
  FYT_INFO("robot_pose_estimator", "Init tracker for robot {}, type: {}", 
           robot_id_, getRobotTypeName(robot_type_));
}

void RobotTracker::initEKF(const ArmorState& armor) {
  // 从装甲板位置估计机器人中心
  double xa = armor.position.x();
  double ya = armor.position.y();
  double za = armor.position.z();
  double yaw = armor.yaw;
  last_yaw_ = 0.0;
  
  // 设置初始半径
  double r = config_.robot_params.standard_radius;
  switch (robot_type_) {
    case RobotType::BALANCE_2:
      r = config_.robot_params.balance_radius;
      break;
    case RobotType::HERO_4:
      r = config_.robot_params.hero_radius;
      break;
    case RobotType::OUTPOST_3:
      r = config_.robot_params.outpost_radius;
      break;
    default:
      r = config_.robot_params.standard_radius;
      break;
  }
  
  // 从装甲板位置估计中心位置
  // 装甲板在中心后方 r 距离处
  double xc = xa + r * std::cos(yaw);
  double yc = ya + r * std::sin(yaw);
  double zc = za;
  
  // 初始化状态向量
  d_za_ = 0.0;
  d_zc_ = 0.0;
  another_r_ = r;
  
  target_state_ = Eigen::VectorXd::Zero(ROBOT_STATE_DIM);
  // [xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc]
  target_state_ << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc_;
  
  // 初始化协方差矩阵
  P_post_ = Eigen::MatrixXd::Identity(ROBOT_STATE_DIM, ROBOT_STATE_DIM);
  P_post_.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0, 0.1, 0.1;
}

void RobotTracker::determineRobotType(const ArmorState& armor) {
  robot_type_ = getRobotTypeFromArmor(armor.armor_id, armor.armor_type);
  num_armors_ = getArmorsNumFromType(robot_type_);
}

bool RobotTracker::update(const std::vector<ArmorState>& armors, double dt) {
  if (armors.empty()) {
    // 无观测，执行预测
    predict(dt);
    return false;
  }
  
  // 执行预测步骤
  predictStep(dt);
  
  bool matched = false;
  
  // 找到最佳匹配的装甲板
  int best_idx = findBestMatch(armors);
  
  if (best_idx >= 0) {
    const ArmorState& matched_armor = armors[best_idx];
    
    // 检查是否发生装甲板跳变
    double yaw_diff = std::abs(matched_armor.yaw - target_state_(6));
    if (yaw_diff > config_.max_match_yaw_diff) {
      // 可能是装甲板跳变
      handleArmorJump(matched_armor);
    }
    
    // 执行更新步骤
    measurement_ << matched_armor.position.x(),
                    matched_armor.position.y(),
                    matched_armor.position.z(),
                    matched_armor.yaw;
    
    updateStep(measurement_);
    matched = true;
    last_armor_ = matched_armor;
    // 记录观测置信度到滑动窗口（使用检测提供的置信度，若无则视为1.0）
    double obs_conf = matched_armor.confidence > 0.0 ? matched_armor.confidence : 1.0;
    confidence_window_.push_back(obs_conf);
    // 裁剪窗口大小
    while (confidence_window_.size() > confidence_window_size_) {
      confidence_window_.pop_front();
    }
  }
  
  // 限制半径范围
  if (target_state_(8) < 0.12) {
    target_state_(8) = 0.12;
  } else if (target_state_(8) > 0.4) {
    target_state_(8) = 0.4;
  }
  
  // 状态机更新
  updateStateMachine(matched);
  
  return matched;
}

void RobotTracker::predict(double dt) {
  predictStep(dt);
  
  // 状态机更新（无匹配）
  updateStateMachine(false);
}

void RobotTracker::predictStep(double dt) {
  // 设置过程噪声
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(ROBOT_STATE_DIM, ROBOT_STATE_DIM);
  Q.diagonal() << config_.sigma2_q_xyz, config_.sigma2_q_xyz * 10,  // x, vx
                  config_.sigma2_q_xyz, config_.sigma2_q_xyz * 10,  // y, vy
                  config_.sigma2_q_xyz, config_.sigma2_q_xyz * 10,  // z, vz
                  config_.sigma2_q_yaw, config_.sigma2_q_yaw * 10,  // yaw, v_yaw
                  config_.sigma2_q_r, config_.sigma2_q_r;           // r, d_zc
  
  // 预测状态
  Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(ROBOT_STATE_DIM);
  
  // 简单的匀速+匀转模型预测
  x_pred(0) = target_state_(0) + target_state_(1) * dt;  // xc
  x_pred(1) = target_state_(1);                           // vxc
  x_pred(2) = target_state_(2) + target_state_(3) * dt;  // yc
  x_pred(3) = target_state_(3);                           // vyc
  x_pred(4) = target_state_(4) + target_state_(5) * dt;  // zc
  x_pred(5) = target_state_(5);                           // vzc
  x_pred(6) = target_state_(6) + target_state_(7) * dt;  // yaw
  x_pred(7) = target_state_(7);                           // v_yaw
  x_pred(8) = target_state_(8);                           // r
  x_pred(9) = target_state_(9);                           // d_zc
  
  // 计算状态转移矩阵 F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(ROBOT_STATE_DIM, ROBOT_STATE_DIM);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;
  
  // 更新协方差
  P_pri_ = F * P_post_ * F.transpose() + Q;
  
  target_state_ = x_pred;
}

void RobotTracker::updateStep(const Eigen::VectorXd& z) {
  // 设置观测噪声
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(ROBOT_MEAS_DIM, ROBOT_MEAS_DIM);
  R.diagonal() << config_.r_xyz, config_.r_xyz, config_.r_xyz, config_.r_yaw;
  
  // 计算预测观测值
  // z = [xa, ya, za, yaw_a]
  // xa = xc - r * cos(yaw)
  // ya = yc - r * sin(yaw)
  // za = zc + d_zc
  // yaw_a = yaw
  double xc = target_state_(0);
  double yc = target_state_(2);
  double zc = target_state_(4);
  double yaw = target_state_(6);
  double r = target_state_(8);
  double d_zc = target_state_(9);
  
  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(ROBOT_MEAS_DIM);
  z_pred(0) = xc - r * std::cos(yaw);
  z_pred(1) = yc - r * std::sin(yaw);
  z_pred(2) = zc + d_zc;
  z_pred(3) = yaw;
  
  // 计算观测矩阵 H (雅可比矩阵)
  // H = dh/dx
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(ROBOT_MEAS_DIM, ROBOT_STATE_DIM);
  H(0, 0) = 1;                          // dxa/dxc
  H(0, 6) = r * std::sin(yaw);          // dxa/dyaw
  H(0, 8) = -std::cos(yaw);             // dxa/dr
  H(1, 2) = 1;                          // dya/dyc
  H(1, 6) = -r * std::cos(yaw);         // dya/dyaw
  H(1, 8) = -std::sin(yaw);             // dya/dr
  H(2, 4) = 1;                          // dza/dzc
  H(2, 9) = 1;                          // dza/d_zc
  H(3, 6) = 1;                          // dyaw_a/dyaw
  
  // 计算卡尔曼增益
  Eigen::MatrixXd S = H * P_pri_ * H.transpose() + R;
  Eigen::MatrixXd K = P_pri_ * H.transpose() * S.inverse();
  
  // 更新状态
  target_state_ = target_state_ + K * (z - z_pred);
  
  // 更新协方差
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ROBOT_STATE_DIM, ROBOT_STATE_DIM);
  P_post_ = (I - K * H) * P_pri_;
}

void RobotTracker::updateStateMachine(bool matched) {
  switch (state_) {
    case State::DETECTING:
      if (matched) {
        detect_count_++;
        if (detect_count_ > config_.tracking_threshold) {
          // 保持 detect_count_ 的值，不重置，用于置信度计算
          state_ = State::TRACKING;
          FYT_DEBUG("robot_pose_estimator", "Robot {} state: TRACKING", robot_id_);
        }
      } else {
        detect_count_ = 0;
        state_ = State::LOST;
        FYT_DEBUG("robot_pose_estimator", "Robot {} state: LOST", robot_id_);
      }
      break;
      
    case State::TRACKING:
      if (matched) {
        // 在 TRACKING 状态下继续增加 detect_count_，但设置上限
        detect_count_ = std::min(detect_count_ + 1, config_.tracking_threshold * 3);
      }
      if (!matched) {
        state_ = State::TEMP_LOST;
        lost_count_++;
        FYT_DEBUG("robot_pose_estimator", "Robot {} state: TEMP_LOST", robot_id_);
      }
      break;
      
    case State::TEMP_LOST:
      if (!matched) {
        lost_count_++;
        // 每丢失一帧，降低 detect_count_
        detect_count_ = std::max(0, detect_count_ - 1);
        if (lost_count_ > config_.lost_threshold) {
          lost_count_ = 0;
          detect_count_ = 0;
          state_ = State::LOST;
          FYT_DEBUG("robot_pose_estimator", "Robot {} state: LOST", robot_id_);
        }
      } else {
        state_ = State::TRACKING;
        lost_count_ = 0;
        // 恢复跟踪时，增加 detect_count_
        detect_count_ = std::min(detect_count_ + 1, config_.tracking_threshold * 3);
        FYT_DEBUG("robot_pose_estimator", "Robot {} state: TRACKING", robot_id_);
      }
      break;
      
    case State::LOST:
    default:
      break;
  }
}

int RobotTracker::findBestMatch(const std::vector<ArmorState>& armors) {
  auto predicted_pos = getArmorPositionFromState(target_state_);
  double min_dist = DBL_MAX;
  int best_idx = -1;
  
  for (size_t i = 0; i < armors.size(); ++i) {
    double dist = (armors[i].position - predicted_pos).norm();
    if (dist < min_dist && dist < config_.max_match_distance) {
      min_dist = dist;
      best_idx = static_cast<int>(i);
    }
  }
  
  return best_idx;
}

void RobotTracker::handleArmorJump(const ArmorState& current_armor) {
  double last_yaw = target_state_(6);
  double yaw = current_armor.yaw;
  
  if (std::abs(yaw - last_yaw) > 0.4) {
    // 装甲板角度跳变，认为是目标旋转
    target_state_(6) = yaw;
    
    // 4装甲板机器人有两个不同的半径和高度
    if (num_armors_ == 4) {
      d_za_ = target_state_(4) + target_state_(9) - current_armor.position.z();
      std::swap(target_state_(8), another_r_);
      d_zc_ = d_zc_ == 0 ? -d_za_ : 0;
      target_state_(9) = d_zc_;
    }
    FYT_DEBUG("robot_pose_estimator", "Robot {} armor jump!", robot_id_);
  }
  
  Eigen::Vector3d current_p = current_armor.position;
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state_);
  
  if ((current_p - infer_p).norm() > config_.max_match_distance) {
    // 距离过大，重置状态
    d_zc_ = 0;
    double r = target_state_(8);
    target_state_(0) = current_p.x() + r * std::cos(yaw);  // xc
    target_state_(1) = 0;                                    // vxc
    target_state_(2) = current_p.y() + r * std::sin(yaw);  // yc
    target_state_(3) = 0;                                    // vyc
    target_state_(4) = current_p.z();                       // zc
    target_state_(5) = 0;                                    // vzc
    target_state_(9) = d_zc_;                                // d_zc
    FYT_WARN("robot_pose_estimator", "Robot {} state reset!", robot_id_);
  }
}

Eigen::Vector3d RobotTracker::getArmorPositionFromState(const Eigen::VectorXd& state) const {
  double xc = state(0);
  double yc = state(2);
  double za = state(4) + state(9);
  double yaw = state(6);
  double r = state(8);
  
  double xa = xc - r * std::cos(yaw);
  double ya = yc - r * std::sin(yaw);
  
  return Eigen::Vector3d(xa, ya, za);
}

RobotState RobotTracker::getRobotState() const {
  RobotState state;
  state.robot_id = robot_id_;
  state.robot_type = robot_type_;
  state.num_armors = num_armors_;
  
  // 中心位置和速度
  state.center_position = Eigen::Vector3d(target_state_(0), target_state_(2), target_state_(4));
  state.center_velocity = Eigen::Vector3d(target_state_(1), target_state_(3), target_state_(5));
  
  // 姿态
  state.yaw = target_state_(6);
  state.yaw_velocity = target_state_(7);
  
  // 几何参数
  state.radius_1 = target_state_(8);
  state.radius_2 = another_r_;
  state.d_zc = target_state_(9);
  state.d_za = d_za_;
  
  // 状态信息
  state.is_tracking = (state_ == State::TRACKING || state_ == State::TEMP_LOST);
  state.confidence = getConfidence();
  
  // 绑定的装甲板ID
  state.bound_armor_ids = bound_armor_ids_;
  
  return state;
}

std::vector<Eigen::Vector3d> RobotTracker::getArmorPositions() const {
  std::vector<Eigen::Vector3d> positions(num_armors_, Eigen::Vector3d::Zero());
  
  Eigen::Vector3d center(target_state_(0), target_state_(2), target_state_(4));
  double yaw = target_state_(6);
  double r1 = target_state_(8);
  double r2 = another_r_;
  double d_zc = target_state_(9);
  
  bool is_current_pair = true;
  for (int i = 0; i < num_armors_; ++i) {
    double temp_yaw = yaw + i * (2.0 * M_PI / num_armors_);
    double r, target_dz;
    
    if (num_armors_ == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za_);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    
    positions[i] = center + Eigen::Vector3d(
      -r * std::cos(temp_yaw),
      -r * std::sin(temp_yaw),
      target_dz
    );
  }
  
  return positions;
}

bool RobotTracker::shouldRemove() const {
  return state_ == State::LOST;
}

void RobotTracker::setBoundArmorIds(const std::vector<std::string>& armor_ids) {
  bound_armor_ids_ = armor_ids;
}

void RobotTracker::setBoundTrackCount(int count) {
  bound_track_count_ = count;
}

double RobotTracker::getConfidence() const {
  // 基于状态的基线置信度
  double base = 0.0;
  switch (state_) {
    case State::TRACKING:
      base = 0.8;
      break;
    case State::TEMP_LOST:
      // 丢失时从 0.5 逐渐衰减到 0
      base = 0.5 * (1.0 - static_cast<double>(lost_count_) / std::max(1, config_.lost_threshold));
      break;
    case State::DETECTING:
      base = 0.4;
      break;
    case State::LOST:
    default:
      base = 0.0;
      break;
  }

  // 检测器置信度因子
  double armor_conf = last_armor_.confidence;
  if (armor_conf <= 0.0) {
    armor_conf = 1.0;  // 无观测时默认为 1.0
  }

  // 稳定性因子：基于连续检测次数
  double stability_factor = 1.0;
  if (state_ == State::TRACKING || state_ == State::DETECTING) {
    int max_count = std::max(1, config_.tracking_threshold * 2);
    // detect_count_ 达到 threshold 时 factor = 1.0
    stability_factor = std::min(1.0, static_cast<double>(detect_count_) / max_count);
    // 确保最低为 0.5，避免置信度过低
    stability_factor = std::max(0.5, stability_factor);
  } else if (state_ == State::TEMP_LOST) {
    // 临时丢失时，稳定性逐渐降低
    int max_count = std::max(1, config_.tracking_threshold * 2);
    stability_factor = std::min(1.0, static_cast<double>(detect_count_) / max_count);
    stability_factor = std::max(0.3, stability_factor);
  }

  // 绑定因子：基于绑定的 track 数量
  // 即使无绑定也给 0.7 的基础值，有绑定时增加
  double binding_factor = 0.7;
  if (bound_track_count_ >= 1) {
    // 1个绑定 -> 0.85, 2个及以上 -> 1.0
    binding_factor = std::min(1.0, 0.7 + 0.15 * bound_track_count_);
  }

  // 当前帧置信度
  double current_conf = base * armor_conf * stability_factor * binding_factor;

  // 滑动窗口平滑
  double final_conf = current_conf;
  if (!confidence_window_.empty()) {
    double sum = std::accumulate(confidence_window_.begin(), confidence_window_.end(), 0.0);
    double window_mean = sum / static_cast<double>(confidence_window_.size());
    // 当前帧与窗口平均的加权融合（当前帧权重 0.3，历史权重 0.7）
    final_conf = 0.3 * current_conf + 0.7 * window_mean;
  }

  // 限制范围
  final_conf = std::clamp(final_conf, 0.0, 1.0);
  return final_conf;
}

}  // namespace fyt::auto_aim
