// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"
// std
#include <Eigen/SVD>
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if (!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();

  _armorPredictedSecquence = std::make_shared<std::vector<std::vector<Eigen::Vector3d>>>();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Use flying time to approximately predict the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  double dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  // Choose the best armor to shoot
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(target_position,
                                                                   target_yaw,
                                                                   target.radius_1,
                                                                   target.radius_2,
                                                                   target.d_zc,
                                                                   target.d_za,
                                                                   target.armors_num);
  _armorPositionSets = armor_positions;

  int idx =
    selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);

  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch, distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // Initialize gimbal_cmd
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
      if (controller_delay_ != 0) {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.d_zc,
                                            target.d_za,
                                            target.armors_num);
        chosen_armor_position = armor_positions.at(idx);
        gimbal_cmd.distance = chosen_armor_position.norm();
        if (chosen_armor_position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      gimbal_cmd.fire_advice = true;
      calcYawAndPitch(target_position, rpy_, yaw, pitch);
      break;
    }
  }

  // Compensate angle by angle_offset_map
  auto angle_offset =
    manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

rm_interfaces::msg::GimbalCmd Solver::solve_withArmorFliter(
  const rm_interfaces::msg::Target &target,
  const rclcpp::Time &current_time,
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Use flying time to approximately predict the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  // double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  // double dt =
  //   (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  // target_position.x() += dt * target.velocity.x;
  // target_position.y() += dt * target.velocity.y;
  // target_position.z() += dt * target.velocity.z;
  // target_yaw += dt * target.v_yaw;

  // 基于 armors_num 生成装甲板当前位置，并通过 ArmorFliter 进行一定程度上的最优估计
  std::vector<Eigen::Vector3d> armor_positions =
    getArmorPositions_withArmorFliter(target_position,
                                      target_yaw,
                                      target.radius_1,
                                      target.radius_2,
                                      target.d_zc,
                                      target.d_za,
                                      target.id,
                                      target.armors_num);

  _armorPositionSets = armor_positions;

  // 通过 ArmorFliter 生成一系列装甲板的位置预测值，在当前值与预测值之间选择云台移动最小的作为打击目标
  std::pair<Eigen::Vector3d, Eigen::Vector3d> selected_pair = selectBestArmor_withArmorFliter(
    target, armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);

  Eigen::Vector3d chosen_armor_position_current = selected_pair.first;     // 选中装甲板的当前位置
  Eigen::Vector3d chosen_armor_position_predicted = selected_pair.second;  // 选中装甲板的预测位置

  // 以下给出开火建议时，应使用选中装甲板的当前位置进行解算
  if (chosen_armor_position_current.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch, distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position_current, rpy_, yaw, pitch);
  double distance = chosen_armor_position_current.norm();

  // Initialize gimbal_cmd
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
      if (controller_delay_ != 0) {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.d_zc,
                                            target.d_za,
                                            target.armors_num);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> selected_pair = selectBestArmor_withArmorFliter(
          target, armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);

        Eigen::Vector3d chosen_armor_position_current = selected_pair.first;

        gimbal_cmd.distance = chosen_armor_position_current.norm();
        if (chosen_armor_position_current.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        calcYawAndPitch(chosen_armor_position_current, rpy_, yaw, pitch);
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      gimbal_cmd.fire_advice = true;
      calcYawAndPitch(target_position, rpy_, yaw, pitch);
      break;
    }
  }

  // 以下进行云台运行移动时，应使用选中装甲板的预测位置进行解算
  double yaw_control, pitch_control;
  calcYawAndPitch(chosen_armor_position_predicted, rpy_, yaw_control, pitch_control);

  // Compensate angle by angle_offset_map
  auto angle_offset =
    manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch_control + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw_control + yaw_offset);

  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(
  const Eigen::Vector3d &target_center,
  const double target_yaw,
  const double r1,
  const double r2,
  const double d_zc,
  const double d_za,
  const std::size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (std::size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

// 在原版 getArmorPositions 的基础上在最后添加 ArmorFliter 进行修正
std::vector<Eigen::Vector3d> Solver::getArmorPositions_withArmorFliter(
  const Eigen::Vector3d &target_center,
  const double target_yaw,
  const double r1,
  const double r2,
  const double d_zc,
  const double d_za,
  const std::string id,
  const std::size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());

  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (std::size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }

  // Update the position of each armor by measurement
  std::vector<Eigen::Vector3d> armor_positions_predicted_by_measurement =
    armorFliter->update(armor_positions, id, armors_num);

  return armor_positions_predicted_by_measurement;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const std::size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
Eigen::Matrix2d R_odom2center;
Eigen::Matrix2d R_odom2armor;
R_odom2center << std::cos(alpha), std::sin(alpha), 
-std::sin(alpha), std::cos(alpha);
R_odom2armor << std::cos(beta), std::sin(beta), 
-std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

#ifdef Flag_SelectBestArmor_v1
// 使用 ArmorFliter 对装甲板可能运行轨迹进行预测，并在预测位置及当前位置中选择云台移动最小的作为最终选板
std::pair<Eigen::Vector3d, Eigen::Vector3d> Solver::selectBestArmor_withArmorFliter(
  const rm_interfaces::msg::Target &target,
  const std::vector<Eigen::Vector3d> &armor_positions,
  const Eigen::Vector3d &target_center,
  const double target_yaw,
  const double target_v_yaw,
  const std::size_t armors_num) const noexcept {
  /* 使用原版的选板方案作为最后的异常处理方案 */

  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
Eigen::Matrix2d R_odom2center;
Eigen::Matrix2d R_odom2armor;
R_odom2center << std::cos(alpha), std::sin(alpha), 
-std::sin(alpha), std::cos(alpha);
R_odom2armor << std::cos(beta), std::sin(beta), 
-std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));

  // Compensate the selected armor
  int selected_id_compensated = selected_id;

  /* armor_positions 是装甲板的当前位置，去除其中距离最远的一个，在其中选择云台移动最小的作为候选方案之一 */

  std::vector<Eigen::Vector3d> armor_current_positions =
    armorFliter->predict(armor_positions, armor_current_positions_predicted_iter);

  // 基于实际位置选板
  double maxDist = 0;
  std::size_t maxDist_id = 1000;
  for (std::size_t i = 0; i < armors_num; ++i) {
    // double distance = armor_positions[i].head(2).norm();
    double distance = armor_positions[i].head(2).norm();
    if (distance > maxDist) {
      maxDist = distance;
      maxDist_id = i;
    }
  }

  double min_GimbalCmd_diff = 1000;
  double min_GimbalCmd_diff_distance = 10000;

  FYT_DEBUG("armor_solver", "current yaw: {}, pitch: {}", rpy_[2], rpy_[1]);

  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);

  for (std::size_t i = 0; i < armors_num && i != maxDist_id; ++i) {
    double yaw, pitch;
    // calcYawAndPitch(armor_positions[i], rpy_, yaw, pitch);
    calcYawAndPitch(armor_positions[i], rpy_, yaw, pitch);

    auto angle_offset =
      manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;
    double cmd_pitch = pitch + pitch_offset;
    double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

    double yaw_diff = std::abs(cmd_yaw - -rpy_[2]) * std::abs(cmd_yaw - -rpy_[2]);
    double pitch_diff = std::abs(cmd_pitch - rpy_[1]) * std::abs(cmd_pitch - rpy_[1]);

    double current_distance = armor_positions[i].head(2).norm();
    if (std::abs(yaw_diff + pitch_diff - min_GimbalCmd_diff) < diff_threshold_to_use_minDist &&
        current_distance < min_GimbalCmd_diff_distance) {
      // 如果两个偏差不大（用阈值衡量），优先选择距离最近的
      min_GimbalCmd_diff = yaw_diff + pitch_diff;
      selected_id_compensated = i;
      min_GimbalCmd_diff_distance = current_distance;
      FYT_DEBUG("armor_solver",
                "Selected current armor index: {}, min_current_diff: {}",
                selected_id_compensated,
                min_GimbalCmd_diff);
    } else if (yaw_diff + pitch_diff < min_GimbalCmd_diff) {
      // 选择云台移动小的目标
      min_GimbalCmd_diff = yaw_diff + pitch_diff;
      selected_id_compensated = i;
      min_GimbalCmd_diff_distance = current_distance;
      FYT_DEBUG("armor_solver",
                "Selected current armor index: {}, min_current_diff: {}",
                selected_id_compensated,
                min_GimbalCmd_diff);
    }
  }

  double min_GimbalCmd_diff_current = min_GimbalCmd_diff;
  int selected_id_compensated_current = selected_id_compensated;

  /* 基于预测选板，先通过 ArmorFliter 对装甲板的可能预测位置进行预测，去除每一时刻预测中距离最远的一个，在其中选择云台移动最小的作为候选方案之一 */

  // 基于 armorFliter 生成装甲板的可能预测位置
  std::vector<Eigen::Vector3d> armor_predicted_positions;

  std::size_t total_predict = 0;
  for (int iter : armor_predicted_iter_list) {
    FYT_DEBUG("armor_solver", "Current iter: {}", iter);

    std::vector<Eigen::Vector3d> armor_predicted_positions_oneiter =
      armorFliter->predict(armor_positions, iter);
    maxDist = 0;
    maxDist_id = 1000;
    for (std::size_t i = 0; i < armors_num; ++i) {
      double distance = armor_predicted_positions_oneiter[i].head(2).norm();
      if (distance > maxDist) {
        maxDist = distance;
        maxDist_id = i;
      }
    }

    // 除最远的装甲板，其余放入待选装甲板
    for (std::size_t i = 0; i < armors_num && maxDist_id != i; ++i) {
      armor_predicted_positions.push_back(armor_predicted_positions_oneiter[i]);
      total_predict++;
    }
  }

  min_GimbalCmd_diff = 1000;

  FYT_DEBUG("armor_solver", "current yaw: {}, pitch: {}", rpy_[2], rpy_[1]);

  // 在所有装甲板的预测位置中选择云台移动最小的作为候选目标
  for (std::size_t i = 0; i < total_predict; ++i) {
    double yaw, pitch;
    calcYawAndPitch(armor_predicted_positions[i], rpy_, yaw, pitch);

    auto angle_offset =
      manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;
    double cmd_pitch = pitch + pitch_offset;
    double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

    double yaw_diff = std::abs(cmd_yaw - -rpy_[2]) * std::abs(cmd_yaw - -rpy_[2]);
    double pitch_diff = std::abs(cmd_pitch - rpy_[1]) * std::abs(cmd_pitch - rpy_[1]);

    double current_distance = armor_predicted_positions[i].head(2).norm();
    if (std::abs(yaw_diff + pitch_diff - min_GimbalCmd_diff) < diff_threshold_to_use_minDist &&
        current_distance < min_GimbalCmd_diff_distance) {
      // 如果两个偏差不大（用阈值衡量），优先选择距离最近的
      min_GimbalCmd_diff = yaw_diff + pitch_diff;
      selected_id_compensated = i;
      min_GimbalCmd_diff_distance = current_distance;
      FYT_DEBUG("armor_solver",
                "Selected current armor index: {}, min_current_diff: {}",
                selected_id_compensated,
                min_GimbalCmd_diff);
    } else if (yaw_diff + pitch_diff < min_GimbalCmd_diff) {
      // 默认选择移动最小的
      min_GimbalCmd_diff = yaw_diff + pitch_diff;
      selected_id_compensated = i;
      FYT_DEBUG("armor_solver",
                "Selected armor index: {}, min_dif: {}",
                selected_id_compensated,
                min_GimbalCmd_diff);
    }
  }

  double min_GimbalCmd_diff_predict = min_GimbalCmd_diff;
  int selected_id_compensated_predict = selected_id_compensated;

  /* 基于规则选择最终候选目标 */
  // 阈值：selectBestArmor_useDefault_threshold 若

  // 初始化当前位置和目标控制位置
  Eigen::Vector3d current_position = armor_positions[selected_id_compensated];
  Eigen::Vector3d control_position = armor_predicted_positions[selected_id_compensated];

  if (min_GimbalCmd_diff_current > selectBestArmor_useDefault_threshold ||
      min_GimbalCmd_diff_predict > selectBestArmor_useDefault_threshold) {
    // 如果预测和差距都太大，仍以原先计算的 select_id 为打击目标
    current_position = armor_positions[selected_id];
    control_position = armor_positions[selected_id];
  } else if (min_GimbalCmd_diff_current < selectBestArmor_useCurrent_threshold ||
             min_GimbalCmd_diff_predict >= min_GimbalCmd_diff_current) {
    // 如果实际位置移动较小，或预测的最小差距比实际的大，控制位置使用实际位置
    current_position = armor_positions[selected_id_compensated_current];
    control_position = armor_positions[selected_id_compensated_current];
    FYT_DEBUG("armor_solver",
              "Use current {}, current_diff: {}",
              selected_id_compensated_current,
              min_GimbalCmd_diff_current);
  } else {
    // 如果预测的最小差距比实际的小，控制位置使用预测位置
    current_position = armor_positions[selected_id_compensated_current];
    control_position = armor_positions[selected_id_compensated_predict];
    FYT_DEBUG("armor_solver",
              "Use predict {}, predict_diff: {}",
              selected_id_compensated_predict,
              min_GimbalCmd_diff_predict);
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> output_pair =
    std::make_pair(current_position, control_position);

  return output_pair;
}

#endif  // Flag_SelectBestArmor_v1

#ifdef Flag_SelectBestArmor_v2
// 在原版 getArmorPositions 的基础上在最后添加 ArmorFliter 进行修正
std::pair<Eigen::Vector3d, Eigen::Vector3d> Solver::selectBestArmor_withArmorFliter(
  const rm_interfaces::msg::Target &target,
  const std::vector<Eigen::Vector3d> &armor_positions,
  const Eigen::Vector3d &target_center,
  const double target_yaw,
  const double target_v_yaw,
  const std::size_t armors_num) const noexcept {
  _armorPredictedSecquence->clear();

  // 原始选板逻辑
  const auto [selected_id, decision_angle] =
    calculateBaseSelection(target_center, target_yaw, target_v_yaw, armors_num);

  // 处理当前装甲板
  auto [current_diff, current_id, current_distance] = processArmorCandidates(
    armor_positions,
    armorFliter->predict(armor_positions, armor_current_positions_predicted_iter),
    armors_num);

  // 处理预测装甲板
  auto [predicted_diff, predicted_id, predicted_distance] =
    processPredictedArmors(armor_positions, armors_num);

  // 最终决策逻辑
  return makeFinalDecision(
    armor_positions, selected_id, current_id, current_diff, predicted_id, predicted_diff);
}

// 基于原始方案的选板逻辑
std::pair<int, double> Solver::calculateBaseSelection(const Eigen::Vector3d &target_center,
                                                      double target_yaw,
                                                      double target_v_yaw,
                                                      std::size_t armors_num) const {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
  -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return {selected_id, decision_angle};
}

// 对于装甲板当前位置选择云台移动最小的作为候选方案之一
Solver::SelectionResult Solver::processArmorCandidates(const std::vector<Eigen::Vector3d> &original,
                                                       const std::vector<Eigen::Vector3d> &filtered,
                                                       std::size_t armors_num) const {
  auto valid_positions = filterArmor(filtered, armors_num);
  _armorPredictedSecquence->push_back(valid_positions);

  return evaluateCandidates(original, valid_positions);
}

// 对于预测装甲板位置选择云台移动最小的作为候选方案之一
Solver::SelectionResult Solver::processPredictedArmors(const std::vector<Eigen::Vector3d> &original,
                                                       std::size_t armors_num) const {
  std::vector<Eigen::Vector3d> all_predicted;
  std::vector<Eigen::Vector3d> all_predicted_flitered;
  for (int iter : armor_predicted_iter_list) {
    auto predicted = armorFliter->predict(original, iter);
    auto valid_positions = filterArmor(predicted, armors_num);
    all_predicted.insert(all_predicted.end(), valid_positions.begin(), valid_positions.end());

    _armorPredictedSecquence->push_back(valid_positions);
  }

  return evaluateCandidates(original, all_predicted);
}

// 在原始装甲板位置中找到与预测位置最接近的装甲板索引
int Solver::findOriginalIndex(const std::vector<Eigen::Vector3d> &original,
                              const Eigen::Vector3d &predicted,
                              double position_tolerance) const {
  for (size_t i = 0; i < original.size(); ++i) {
    if ((original[i] - predicted).norm() < position_tolerance) {
      return static_cast<int>(i);
    }
  }

  // 容错机制：若未找到则返回第一个
  FYT_WARN("armor_solver", "Failed to map predicted position to original index");
  return 0;
}

#endif  // Flag_SelectBestArmor_v2

// 对得到的装甲板位置进行过滤
// 输入应为某一时刻的一组装甲板位置
#if defined(Flag_ARMOR_FLITER_v1) && defined(Flag_SelectBestArmor_v2)
std::vector<Eigen::Vector3d> Solver::filterArmor(const std::vector<Eigen::Vector3d> &positions,
                                                 std::size_t armors_num) const {
  int maxDist_id = -1;
  double maxDist = 0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (double dist = positions[i].head(2).norm(); dist > maxDist) {
      maxDist = dist;
      maxDist_id = i;
    }
  }

  std::vector<Eigen::Vector3d> filtered;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (i != static_cast<std::size_t>(maxDist_id)) {
      filtered.push_back(positions[i]);
    }
  }
  return filtered;
}
#endif  // Flag_ARMOR_FLITER_v1

#if defined(Flag_ARMOR_FLITER_v2) && defined(Flag_SelectBestArmor_v2)
std::vector<Eigen::Vector3d> Solver::filterArmor(const std::vector<Eigen::Vector3d> &positions,
                                                 std::size_t armors_num) const {
  // 1. 计算旋转中心 ----------------------------------------------------------
  Eigen::Vector3d O = Eigen::Vector3d::Zero();
  for (const auto &pos : positions) {
    O += pos;
  }
  O /= positions.size();

  // 2. 观测参数设置 ----------------------------------------------------------
  const Eigen::Vector3d A = Eigen::Vector3d::Zero();  // 观测者位置（原点）
  const Eigen::Vector3d OA = O - A;                   // 观测方向向量
  const double angle_threshold = 45.0;                // 示例阈值45度（可配置为类成员）

  // 3. 夹角计算与过滤 --------------------------------------------------------
  double max_violation_angle = 0.0;  // 最大违规角度
  std::vector<Eigen::Vector3d> filtered;

  for (size_t i = 0; i < positions.size(); ++i) {
    const Eigen::Vector3d Z = positions[i];
    const Eigen::Vector3d OZ = O - Z;

    // 处理零向量特殊情况
    if (OA.norm() < 1e-6 || OZ.norm() < 1e-6) {
      filtered.push_back(Z);
      continue;
    }

    // 计算锐角夹角（0~90度）
    const double cos_theta = OA.normalized().dot(OZ.normalized());
    const double theta = std::acos(cos_theta) * 180.0 / M_PI;

    // 决策逻辑
    if (theta <= angle_threshold) {
      filtered.push_back(Z);
    } else if (theta > max_violation_angle) {
      max_violation_angle = theta;
    }
  }

  // 4. 返回过滤结果 ----------------------------------------------------------
  return filtered;
}
#endif  // Flag_ARMOR_FLITER_v2

#if defined(Flag_ARMOR_FLITER_v3) && defined(Flag_SelectBestArmor_v2)
std::vector<Eigen::Vector3d> Solver::filterArmor(const std::vector<Eigen::Vector3d> &positions,
                                                 std::size_t armors_num) const {
  // ================== 1. 椭圆拟合旋转中心 ==================
  Eigen::Vector3d O = Eigen::Vector3d::Zero();

  // 仅在有效点数足够时进行椭圆拟合
  if (positions.size() >= 5) {  // 椭圆拟合至少需要5个点
    // 构造系数矩阵
    Eigen::MatrixXd D(positions.size(), 6);

    // 提取XY坐标（假设水平面旋转）
    std::vector<Eigen::Vector2d> points;
    for (const auto &p : positions) {
      points.emplace_back(p.x(), p.y());
    }

    // 构建椭圆方程 Ax² + Bxy + Cy² + Dx + Ey + F = 0 的系数矩阵
    for (size_t i = 0; i < points.size(); ++i) {
      const double x = points[i].x();
      const double y = points[i].y();
      D.row(i) << x * x, x * y, y * y, x, y, 1;
    }

    // 使用SVD求解最小二乘解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeFullV);
    Eigen::VectorXd coeffs = svd.matrixV().rightCols<1>();

    // 解析椭圆参数
    const double A = coeffs[0], B = coeffs[1], C = coeffs[2];
    const double Dc = coeffs[3], Ec = coeffs[4];
    // const double F = coeffs[5]; // F不需要用于计算中心

    // 计算椭圆中心 (h,k)
    const double denominator = B * B - 4 * A * C;
    if (std::abs(denominator) > 1e-6) {  // 确保非退化椭圆
      const double h = (2 * C * Dc - B * Ec) / denominator;
      const double k = (2 * A * Ec - B * Dc) / denominator;
      O << h, k, 0.0;  // Z坐标为0（水平面假设）
    } else {
      // 退化为圆的情况，使用几何中心
      O = positions[0];
      for (size_t i = 1; i < positions.size(); ++i) O += positions[i];
      O /= positions.size();
    }
  } else {  // 点数不足时使用传统方法
    for (const auto &p : positions) O += p;
    O /= positions.size();
  }

  // ================== 2. 观测参数设置 ==================
  // 2. 观测参数设置 ----------------------------------------------------------
  const Eigen::Vector3d A = Eigen::Vector3d::Zero();  // 观测者位置（原点）
  const Eigen::Vector3d OA = O - A;                   // 观测方向向量
  const double angle_threshold = 45.0;                // 示例阈值45度（可配置为类成员）

  // 3. 夹角计算与过滤 --------------------------------------------------------
  double max_violation_angle = 0.0;  // 最大违规角度
  std::vector<Eigen::Vector3d> filtered;

  for (size_t i = 0; i < positions.size(); ++i) {
    const Eigen::Vector3d Z = positions[i];
    const Eigen::Vector3d OZ = Z - O;

    // 处理零向量特殊情况
    if (OA.norm() < 1e-6 || OZ.norm() < 1e-6) {
      filtered.push_back(Z);
      continue;
    }

    // 计算锐角夹角（0~90度）
    const double cos_theta = OA.normalized().dot(OZ.normalized());
    const double theta = std::acos(std::abs(cos_theta)) * 180.0 / M_PI;

    // 决策逻辑
    if (theta <= angle_threshold) {
      filtered.push_back(Z);
    } else if (theta > max_violation_angle) {
      max_violation_angle = theta;
    }
  }
  // 4. 返回过滤结果 ----------------------------------------------------------

  return filtered;
}
#endif  // Flag_ARMOR_FLITER_v3

#ifdef Flag_SelectBestArmor_v2

// 评估候选装甲板
Solver::SelectionResult Solver::evaluateCandidates(
  const std::vector<Eigen::Vector3d> &original,
  const std::vector<Eigen::Vector3d> &candidates) const {
  double min_diff = 1000;
  double min_distance = 10000;
  int best_index = 0;

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    auto [diff, distance] = calculateMovementDiff(candidates[i]);

    if (shouldUpdateSelection(diff, min_diff, distance, min_distance)) {
      min_diff = diff;
      min_distance = distance;
      best_index = findOriginalIndex(original, candidates[i], position_tolerance);
    }
  }
  return {min_diff, best_index, min_distance};
}

// 计算云台移动代价
std::pair<double, double> Solver::calculateMovementDiff(const Eigen::Vector3d &position) const {
  // 1. 计算基础角度
  double yaw, pitch;
  calcYawAndPitch(position, rpy_, yaw, pitch);

  // 2. 获取手动补偿量（假设返回值为角度值）
  const auto angle_offset =
    manual_compensator_->angleHardCorrect(position.head(2).norm(),  // 水平距离
                                          position.z()              // 垂直高度
    );

  // 3. 转换为弧度补偿值
  const double pitch_offset = angle_offset[0] * M_PI / 180.0;
  const double yaw_offset = angle_offset[1] * M_PI / 180.0;

  // 4. 计算补偿后指令角度
  const double cmd_pitch = pitch + pitch_offset;
  const double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

  // 5. 计算与当前云台角度的差值
  const double yaw_diff = std::abs(cmd_yaw - (-rpy_[2]));  // 注意符号处理
  const double pitch_diff = std::abs(cmd_pitch - rpy_[1]);

  // 6. 计算综合移动代价（平方和为非线性权重）
  const double movement_cost = yaw_diff * yaw_diff + pitch_diff * pitch_diff;

  // 7. 返回移动代价和装甲板距离
  return {movement_cost, position.head(2).norm()};
  return {yaw_diff + pitch_diff, position.head(2).norm()};
}

// 判断是否需要更新选板
bool Solver::shouldUpdateSelection(double current_diff,
  double current_min_diff,
  double current_distance,
  double min_distance) const {
// 情况1：差异在阈值范围内且距离更近
if (std::abs(current_diff - current_min_diff) < diff_threshold_to_use_minDist) {
return current_distance < min_distance;
}
// 情况2：差异明显更小
else {
return current_diff < current_min_diff;
}
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Solver::makeFinalDecision(
  const std::vector<Eigen::Vector3d> &armors,
  int base_id,
  int current_id,
  double current_diff,
  int predicted_id,
  double predicted_diff) const {
  // 默认使用基础选择结果
  Eigen::Vector3d current_pos = armors[base_id];
  Eigen::Vector3d control_pos = armors[base_id];

  // 决策树逻辑
  if (current_diff > selectBestArmor_useDefault_threshold ||
      predicted_diff > selectBestArmor_useDefault_threshold) {
    // 情况1：任一方案差异过大，使用基础算法结果
    FYT_DEBUG("armor_solver", "Fallback to base selection id: {}", base_id);
  } else if (current_diff < selectBestArmor_useCurrent_threshold ||
             predicted_diff >= current_diff) {
    // 情况2：当前方案足够好或预测不优于当前
    current_pos = armors[current_id];
    control_pos = armors[current_id];
    FYT_DEBUG("armor_solver", "Select current id: {}", current_id);
  } else {
    // 情况3：预测方案更优
    current_pos = armors[current_id];    // 当前位置仍来自当前帧
    control_pos = armors[predicted_id];  // 控制位置使用预测结果
    FYT_DEBUG("armor_solver", "Select predicted id: {}", predicted_id);
  }

  // 安全校验
  const auto validate_position = [&](const Eigen::Vector3d &pos) {
    return pos.allFinite() && (pos.norm() < 20.0);  // 假设有效距离小于20米
  };

  if (!validate_position(current_pos) || !validate_position(control_pos)) {
    FYT_ERROR("armor_solver", "Invalid position detected!");
    return {armors[base_id], armors[base_id]};  // 完全回退到基础方案
  }

  return {current_pos, control_pos};
}

#endif  // Flag_SelectBestArmor_v2

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  double temp_pitch = pitch;
  if (trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace fyt::auto_aim
