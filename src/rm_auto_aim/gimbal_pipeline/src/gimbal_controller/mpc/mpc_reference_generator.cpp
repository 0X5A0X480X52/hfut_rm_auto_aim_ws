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

#include "gimbal_controller/mpc/mpc_reference_generator.hpp"

#include <angles/angles.h>
#include <cmath>
#include <iostream>

namespace gimbal_controller
{
namespace mpc
{

Eigen::VectorXd MpcReferenceGenerator::generate(
  const rm_interfaces::msg::TrackedRobot & target_robot,
  double current_yaw,
  double current_pitch,
  int N,
  double dt) const
{
  const int nx = GimbalDynamicsModel::STATE_DIM;
  Eigen::VectorXd X_ref(nx * N);
  X_ref.setZero();

  // 轨迹生成前对速度和 yaw_velocity 执行 clamp
  const auto clamped_robot = applyVelocityClamp(target_robot);

  if (!position_calculator_ || !armor_selector_ || !local_compensator_) {
    // 组件未初始化，返回当前位置作为参考
    for (int k = 0; k < N; ++k) {
      X_ref.segment(k * nx, nx) << current_yaw, current_pitch, 0.0, 0.0;
    }
    return X_ref;
  }

  double prev_yaw_ref = current_yaw;
  double prev_pitch_ref = current_pitch;

  for (int k = 0; k < N; ++k) {
    double t_ahead = (k + 1) * dt;

    // 1. 传播目标状态到未来 t_ahead 秒
    auto future_robot = propagateRobot(clamped_robot, t_ahead);

    // 2. 在预测位置计算装甲板坐标
    auto armor_positions = position_calculator_->calculatePredicted(future_robot, 0.0);
    // 注: calculatePredicted(future_robot, 0.0) 因为已经 propagate 过了

    if (armor_positions.empty()) {
      // 无装甲板，保持上一步参考
      X_ref.segment(k * nx, nx) << prev_yaw_ref, prev_pitch_ref, 0.0, 0.0;
      continue;
    }

    // 3. 预测时刻的目标中心
    Eigen::Vector3d target_center(
      future_robot.center_position.x,
      future_robot.center_position.y,
      future_robot.center_position.z);

    // 4. 选板
    auto selection = armor_selector_->selectBest(
      armor_positions,
      target_center,
      future_robot.yaw,
      future_robot.num_armors,
      future_robot.yaw_velocity,
      current_yaw,
      current_pitch);

    Eigen::Vector3d target_position = selection.is_center_fallback
      ? target_center : selection.position;

    // 5. 弹道解算 → 得到期望 yaw/pitch
    auto ballistic = local_compensator_->compensate(target_position);

    double yaw_ref, pitch_ref;
    if (ballistic.success) {
      yaw_ref = ballistic.yaw;
      pitch_ref = ballistic.pitch;
    } else {
      // fallback: 直接几何计算
      double dist_xy = std::sqrt(
        target_position.x() * target_position.x() +
        target_position.y() * target_position.y());
      yaw_ref = std::atan2(target_position.y(), target_position.x());
      pitch_ref = std::atan2(target_position.z(), dist_xy);

      // std::cout << "Ballistic compensation failed at step " << k
      //           << ", using geometric fallback. Target position: "
      //           << target_position.transpose() << std::endl;
    }

    // 6. 估计参考角速度 (数值微分)
    double yaw_dot_ref = (yaw_ref - prev_yaw_ref) / dt;
    double pitch_dot_ref = (pitch_ref - prev_pitch_ref) / dt;

    X_ref.segment(k * nx, nx) << yaw_ref, pitch_ref, yaw_dot_ref, pitch_dot_ref;

    prev_yaw_ref = yaw_ref;
    prev_pitch_ref = pitch_ref;

    // Debug 输出
    // std::cout << "Step " << k << ": t_ahead=" << t_ahead
    //           << "s, target_pos=" << target_position.transpose()
    //           << ", yaw_ref=" << yaw_ref << ", pitch_ref=" << pitch_ref
    //           << ", yaw_dot_ref=" << yaw_dot_ref << ", pitch_dot_ref=" << pitch_dot_ref
    //           << (selection.is_center_fallback ? " (center fallback)" : "")
    //           << std::endl;
  }

  return X_ref;
}

rm_interfaces::msg::TrackedRobot MpcReferenceGenerator::propagateRobot(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt)
{
  auto future = robot;

  // 匀速/匀加速传播中心位置
  future.center_position.x += robot.center_velocity.x * dt
    + 0.5 * robot.center_acceleration.x * dt * dt;
  future.center_position.y += robot.center_velocity.y * dt
    + 0.5 * robot.center_acceleration.y * dt * dt;
  future.center_position.z += robot.center_velocity.z * dt
    + 0.5 * robot.center_acceleration.z * dt * dt;

  // 传播速度
  future.center_velocity.x += robot.center_acceleration.x * dt;
  future.center_velocity.y += robot.center_acceleration.y * dt;
  future.center_velocity.z += robot.center_acceleration.z * dt;

  // 传播 yaw (匀加速旋转)
  future.yaw += robot.yaw_velocity * dt
    + 0.5 * robot.yaw_acceleration * dt * dt;
  future.yaw_velocity += robot.yaw_acceleration * dt;

  return future;
}

Eigen::VectorXd MpcReferenceGenerator::generateWithDelay(
  const rm_interfaces::msg::TrackedRobot & target_robot,
  double current_yaw,
  double current_pitch,
  int N,
  double dt,
  const DelayCompConfig & delay_config) const
{
  const int nx = GimbalDynamicsModel::STATE_DIM;
  Eigen::VectorXd X_ref(nx * N);
  X_ref.setZero();

  // 轨迹生成前对速度和 yaw_velocity 执行 clamp
  const auto clamped_robot = applyVelocityClamp(target_robot);

  if (!position_calculator_ || !armor_selector_ || !local_compensator_) {
    for (int k = 0; k < N; ++k) {
      X_ref.segment(k * nx, nx) << current_yaw, current_pitch, 0.0, 0.0;
    }
    return X_ref;
  }

  double prev_yaw_ref = current_yaw;
  double prev_pitch_ref = current_pitch;

  for (int k = 0; k < N; ++k) {
    double t_ahead = (k + 1) * dt;

    // 1. 计算参考时间: base_delay (processing + controller + prediction) + t_ahead
    double t_predict = delay_config.base_delay_s + t_ahead;

    // 2. 传播目标状态到未来 t_predict 秒
    auto future_robot = propagateRobot(clamped_robot, t_predict);

    // 3. 子弹飞行时间迭代补偿
    Eigen::Vector3d pred_center(
      future_robot.center_position.x,
      future_robot.center_position.y,
      future_robot.center_position.z);

    double t_flight = 0.0;
    for (int iter = 0; iter < delay_config.flight_time_iters; ++iter) {
      t_flight = local_compensator_->getFlyingTime(pred_center);
      auto flight_robot = propagateRobot(clamped_robot, t_predict + t_flight);
      pred_center = Eigen::Vector3d(
        flight_robot.center_position.x,
        flight_robot.center_position.y,
        flight_robot.center_position.z);
    }

    // 用飞行时间补偿后的时刻重新传播整个机器人状态 (含 yaw)
    auto compensated_robot = propagateRobot(clamped_robot, t_predict + t_flight);

    // 4. 在补偿后位置计算装甲板坐标
    auto armor_positions = position_calculator_->calculatePredicted(compensated_robot, 0.0);

    if (armor_positions.empty()) {
      X_ref.segment(k * nx, nx) << prev_yaw_ref, prev_pitch_ref, 0.0, 0.0;
      continue;
    }

    // 5. 补偿后的目标中心
    Eigen::Vector3d target_center(
      compensated_robot.center_position.x,
      compensated_robot.center_position.y,
      compensated_robot.center_position.z);

    // 6. 选板逻辑: 使用最小移动加 facing 策略
    auto selection = armor_selector_->selectByMinMovementWithFacing(
      armor_positions,
      target_center,
      compensated_robot.yaw,
      compensated_robot.num_armors,
      current_yaw,
      current_pitch);

    Eigen::Vector3d target_position = selection.is_center_fallback
      ? target_center : selection.position;

    // 7. 弹道解算
    auto ballistic = local_compensator_->compensate(target_position);

    double yaw_ref, pitch_ref;
    if (ballistic.success) {
      yaw_ref = ballistic.yaw;
      pitch_ref = ballistic.pitch;
    } else {
      // fallback: 几何计算
      double dist_xy = std::sqrt(
        target_position.x() * target_position.x() +
        target_position.y() * target_position.y());
      yaw_ref = std::atan2(target_position.y(), target_position.x());
      pitch_ref = std::atan2(target_position.z(), dist_xy);
    }

    // 8. 估计参考角速度
    double yaw_dot_ref = (yaw_ref - prev_yaw_ref) / dt;
    double pitch_dot_ref = (pitch_ref - prev_pitch_ref) / dt;

    // 9. 填充参考轨迹
    X_ref.segment(k * nx, nx) << yaw_ref, pitch_ref, yaw_dot_ref, pitch_dot_ref;

    prev_yaw_ref = yaw_ref;
    prev_pitch_ref = pitch_ref;
  }

  return X_ref;
}

rm_interfaces::msg::TrackedRobot MpcReferenceGenerator::applyVelocityClamp(
  const rm_interfaces::msg::TrackedRobot & robot) const
{
  if (!vel_clamp_config_.enable) {
    return robot;
  }

  auto result = robot;

  // 线速度: 保持方向不变，对标量限幅
  const double vx = robot.center_velocity.x;
  const double vy = robot.center_velocity.y;
  const double vz = robot.center_velocity.z;
  const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (speed > vel_clamp_config_.max_linear_speed && speed > 1e-9) {
    const double scale = vel_clamp_config_.max_linear_speed / speed;
    result.center_velocity.x = vx * scale;
    result.center_velocity.y = vy * scale;
    result.center_velocity.z = vz * scale;
  }

  // yaw 角速度: 直接限幅
  result.yaw_velocity = std::clamp(
    robot.yaw_velocity,
    -vel_clamp_config_.max_v_yaw,
    vel_clamp_config_.max_v_yaw);

  return result;
}

}  // namespace mpc
}  // namespace gimbal_controller
