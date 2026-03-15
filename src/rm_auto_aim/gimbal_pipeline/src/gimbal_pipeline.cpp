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

#include "gimbal_pipeline/gimbal_pipeline.hpp"

// std
#include <cmath>
#include <stdexcept>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

GimbalPipeline::GimbalPipeline(std::weak_ptr<rclcpp::Node> node) : node_(node) {
  auto n = node_.lock();

  shooting_range_w_    = n->declare_parameter("pipeline.shooting_range_width",  0.135);
  shooting_range_h_    = n->declare_parameter("pipeline.shooting_range_height", 0.135);
  max_tracking_v_yaw_  = n->declare_parameter("pipeline.max_tracking_v_yaw",    6.0);
  prediction_delay_    = n->declare_parameter("pipeline.prediction_delay",       0.0);
  controller_delay_    = n->declare_parameter("pipeline.controller_delay",       0.0);
  side_angle_          = n->declare_parameter("pipeline.side_angle",             15.0);
  min_switching_v_yaw_ = n->declare_parameter("pipeline.min_switching_v_yaw",   1.0);

  std::string compensator_type = n->declare_parameter("pipeline.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compensator_type);
  trajectory_compensator_->iteration_times =
    n->declare_parameter("pipeline.iteration_times", 20);
  trajectory_compensator_->velocity = n->declare_parameter("pipeline.bullet_speed", 20.0);
  trajectory_compensator_->gravity  = n->declare_parameter("pipeline.gravity", 9.8);
  trajectory_compensator_->resistance = n->declare_parameter("pipeline.resistance", 0.001);

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset =
    n->declare_parameter("pipeline.angle_offset", std::vector<std::string>{});
  if (!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("gimbal_pipeline", "Manual compensator update failed!");
  }

  n.reset();
}

// ---------------------------------------------------------------------------
// buildContext
// ---------------------------------------------------------------------------
GimbalControlContext GimbalPipeline::buildContext(
  const rm_interfaces::msg::Target &target,
  bool is_temp_lost,
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer) {
  GimbalControlContext context;
  context.target_robot = target;
  context.is_temp_lost = is_temp_lost;

  // Obtain the current gimbal orientation from TF.
  //
  //   rpy[0] = roll   (not used for aiming)
  //   rpy[1] = pitch  (raw: positive = barrel down due to URDF convention)
  //   rpy[2] = yaw    (positive = counter-clockwise from X-axis)
  //
  // The pitch is negated so that a positive current_pitch means the barrel
  // is pointing upward, which is consistent with how cmd_pitch is signed.
  try {
    auto gimbal_tf =
      tf2_buffer->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);

    tf2::Quaternion tf_q;
    tf2::fromMsg(gimbal_tf.transform.rotation, tf_q);

    double roll{}, raw_pitch{}, yaw{};
    tf2::Matrix3x3(tf_q).getRPY(roll, raw_pitch, yaw);

    // current_yaw   = rpy[2]   (no sign change)
    context.current_yaw   = yaw;
    // current_pitch = -rpy[1]  (negate so positive = barrel up)
    context.current_pitch = -raw_pitch;
  } catch (const tf2::TransformException &ex) {
    FYT_ERROR("gimbal_pipeline", "TF lookup failed: {}", ex.what());
    throw;
  }

  return context;
}

// ---------------------------------------------------------------------------
// solve
// ---------------------------------------------------------------------------
rm_interfaces::msg::GimbalCmd GimbalPipeline::solve(const GimbalControlContext &context,
                                                     const rclcpp::Time &current_time) {
  // Re-read dynamic parameters
  try {
    auto n = node_.lock();
    max_tracking_v_yaw_  = n->get_parameter("pipeline.max_tracking_v_yaw").as_double();
    prediction_delay_    = n->get_parameter("pipeline.prediction_delay").as_double();
    controller_delay_    = n->get_parameter("pipeline.controller_delay").as_double();
    side_angle_          = n->get_parameter("pipeline.side_angle").as_double();
    min_switching_v_yaw_ = n->get_parameter("pipeline.min_switching_v_yaw").as_double();
    n.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("gimbal_pipeline", "{}", e.what());
  }

  const auto &target = context.target_robot;

  // Predict target position forward by flight time + controller delay.
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  double dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw          += dt * target.v_yaw;

  // Build all armor plate positions for the predicted robot pose.
  auto armor_positions = getArmorPositions(target_position,
                                           target_yaw,
                                           target.radius_1,
                                           target.radius_2,
                                           target.d_zc,
                                           target.d_za,
                                           target.armors_num);

  int idx = selectBestArmor(
    armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);

  Eigen::Vector3d chosen_position = armor_positions.at(static_cast<std::size_t>(idx));
  if (chosen_position.norm() < 0.1) {
    throw std::runtime_error("GimbalPipeline: no valid armor position");
  }

  // Compute aim angles to the chosen armor (current position, used for fire
  // advice and for the absolute yaw/pitch output fields).
  double yaw{}, pitch{};
  calcYawAndPitch(chosen_position, yaw, pitch);
  double distance = chosen_position.norm();

  // Fire advice: is the gimbal already pointing at the target?
  bool fire_advice = isOnTarget(
    context.current_yaw, context.current_pitch, yaw, pitch, distance);

  // Optionally shift the aim point forward by controller_delay to compensate
  // for closed-loop latency (makes the gimbal proactively lead the target).
  if (controller_delay_ != 0.0) {
    Eigen::Vector3d delayed_pos = target_position;
    double delayed_yaw          = target_yaw;
    delayed_pos.x() += controller_delay_ * target.velocity.x;
    delayed_pos.y() += controller_delay_ * target.velocity.y;
    delayed_pos.z() += controller_delay_ * target.velocity.z;
    delayed_yaw     += controller_delay_ * target.v_yaw;

    auto delayed_armors = getArmorPositions(delayed_pos,
                                            delayed_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.d_zc,
                                            target.d_za,
                                            target.armors_num);
    chosen_position = delayed_armors.at(static_cast<std::size_t>(idx));
    if (chosen_position.norm() < 0.1) {
      throw std::runtime_error("GimbalPipeline: no valid armor position (controller delay)");
    }
    calcYawAndPitch(chosen_position, yaw, pitch);
    distance = chosen_position.norm();
  }

  // Apply manual (table-based) angle offsets.
  auto angle_offset =
    manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180.0;
  double yaw_offset   = angle_offset[1] * M_PI / 180.0;

  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw   = angles::normalize_angle(yaw + yaw_offset);

  // Gimbal movement deltas:
  //   yaw_diff   = cmd_yaw   - current_yaw   [rad -> deg]
  //   pitch_diff = cmd_pitch - current_pitch [rad -> deg]
  double yaw_diff   = cmd_yaw   - context.current_yaw;
  double pitch_diff = cmd_pitch - context.current_pitch;

  // Build the output command.
  // cmd.yaw / cmd.pitch are *absolute* world-frame angles [deg] so that the
  // receiver can reconstruct the full aim pose.
  // cmd.yaw_diff / cmd.pitch_diff are the *relative* deltas [deg] that the
  // gimbal servo controller should add to the current joint angles.
  // TEMP_LOST 时无 detector 实际观测，distance 输出 -1 以示无有效测量
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header     = context.target_robot.header;
  cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
  cmd.pitch      = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff   = yaw_diff   * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  // TEMP_LOST 时无 detector 实际观测，distance 输出 -1 以示无有效测量
  cmd.distance   = context.is_temp_lost ? -1.0 : distance;
  cmd.fire_advice = fire_advice;
  return cmd;
}

// ---------------------------------------------------------------------------
// calcYawAndPitch
// ---------------------------------------------------------------------------
void GimbalPipeline::calcYawAndPitch(const Eigen::Vector3d &p,
                                     double &yaw,
                                     double &pitch) const noexcept {
  // Horizontal bearing to the target
  yaw = atan2(p.y(), p.x());

  // Geometric elevation angle
  pitch = atan2(p.z(), p.head(2).norm());

  // Apply trajectory (gravity / air-resistance) compensation to pitch
  double temp_pitch = pitch;
  if (trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

// ---------------------------------------------------------------------------
// isOnTarget
// ---------------------------------------------------------------------------
bool GimbalPipeline::isOnTarget(double cur_yaw,
                                double cur_pitch,
                                double target_yaw,
                                double target_pitch,
                                double distance) const noexcept {
  double range_yaw =
    std::max(std::abs(atan2(shooting_range_w_ / 2.0, distance)), 1.0 * M_PI / 180.0);
  double range_pitch =
    std::max(std::abs(atan2(shooting_range_h_ / 2.0, distance)), 1.0 * M_PI / 180.0);

  return std::abs(cur_yaw - target_yaw) < range_yaw &&
         std::abs(cur_pitch - target_pitch) < range_pitch;
}

// ---------------------------------------------------------------------------
// getArmorPositions
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector3d> GimbalPipeline::getArmorPositions(const Eigen::Vector3d &center,
                                                                double yaw,
                                                                double r1,
                                                                double r2,
                                                                double d_zc,
                                                                double d_za,
                                                                std::size_t armors_num) const
  noexcept {
  std::vector<Eigen::Vector3d> positions(armors_num, Eigen::Vector3d::Zero());
  bool is_current_pair = true;
  double r = 0.0, target_dz = 0.0;
  for (std::size_t i = 0; i < armors_num; ++i) {
    double temp_yaw = yaw + i * (2.0 * M_PI / static_cast<double>(armors_num));
    if (armors_num == 4) {
      r         = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0.0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r         = r1;
      target_dz = d_zc;
    }
    positions[i] = center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return positions;
}

// ---------------------------------------------------------------------------
// selectBestArmor
// ---------------------------------------------------------------------------
int GimbalPipeline::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                                    const Eigen::Vector3d &target_center,
                                    double target_yaw,
                                    double target_v_yaw,
                                    std::size_t armors_num) const noexcept {
  double alpha = std::atan2(target_center.y(), target_center.x());
  double beta  = target_yaw;

  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  // clang-format off
  R_odom2center <<  std::cos(alpha),  std::sin(alpha),
                   -std::sin(alpha),  std::cos(alpha);
  R_odom2armor  <<  std::cos(beta),   std::sin(beta),
                   -std::sin(beta),   std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;
  double decision_angle = -std::asin(R_center2armor(0, 1));

  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0.0;
  }

  double temp_angle = decision_angle + M_PI / static_cast<double>(armors_num) - theta;
  if (temp_angle < 0.0) {
    temp_angle += 2.0 * M_PI;
  }

  return static_cast<int>(temp_angle / (2.0 * M_PI / static_cast<double>(armors_num)));
}

}  // namespace fyt::auto_aim
