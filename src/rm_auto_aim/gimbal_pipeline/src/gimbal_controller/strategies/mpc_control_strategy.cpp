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

#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

#include <angles/angles.h>
#include <algorithm>
#include <iostream>

#include "gimbal_controller/fire_advisor.hpp"

namespace gimbal_controller
{

MpcControlStrategy::MpcControlStrategy()
: dynamics_model_(0.01)
{
}

void MpcControlStrategy::setMpcParameters(
  int N, double dt, double control_delay_s, double max_accel,
  double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel,
  double r_yaw, double r_pitch, double s_yaw, double s_pitch)
{
  N_ = N;
  dt_ = dt;
  control_delay_s_ = control_delay_s;
  max_accel_ = max_accel;
  q_yaw_ = q_yaw;
  q_pitch_ = q_pitch;
  q_yaw_vel_ = q_yaw_vel;
  q_pitch_vel_ = q_pitch_vel;
  r_yaw_ = r_yaw;
  r_pitch_ = r_pitch;
  s_yaw_ = s_yaw;
  s_pitch_ = s_pitch;

  dynamics_model_.setDt(dt_);
  matrices_dirty_ = true;
}

void MpcControlStrategy::initReferenceGenerator()
{
  ref_generator_.setComponents(position_calculator_, armor_selector_, local_compensator_);
}

void MpcControlStrategy::setDelayCompensation(
  bool enable, double prediction_delay_s, int flight_time_iters,
  double max_processing_delay_s)
{
  enable_delay_compensation_ = enable;
  prediction_delay_s_ = prediction_delay_s;
  flight_time_iters_ = flight_time_iters;
  max_processing_delay_s_ = max_processing_delay_s;
}

void MpcControlStrategy::setManeuverAdaptParameters(
  bool enable, double a_max, double eta, double tau, double r_scale)
{
  enable_maneuver_adapt_ = enable;
  a_max_ = a_max;
  eta_ = eta;
  tau_ = tau;
  r_scale_maneuver_ = r_scale;
  // 切换启用状态时重置 EMA 状态
  alpha_ema_ = 0.0;
  has_prev_velocity_ = false;
}

void MpcControlStrategy::rebuildMatrices()
{
  dynamics_model_.buildPredictionMatrices(N_, A_pred_, B_ctrl_);

  if (control_delay_s_ > 1e-6) {
    B_ctrl_ = dynamics_model_.buildDelayedB(N_, control_delay_s_);
  }

  D_ = mpc::GimbalDynamicsModel::buildDifferenceMatrix(N_);
  Q_blk_ = mpc::GimbalDynamicsModel::buildWeightQ(N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_);
  R_blk_ = mpc::GimbalDynamicsModel::buildWeightR(N_, r_yaw_, r_pitch_);
  S_blk_ = mpc::GimbalDynamicsModel::buildWeightS(N_, s_yaw_, s_pitch_);

  matrices_dirty_ = false;
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::solve(
  const GimbalControlContext & context)
{

  // std::cout << "MPC Control Strategy: Solving for target robot at position ("
  //           << context.target_robot.center_position.x << ", "
  //           << context.target_robot.center_position.y << ", "
  //           << context.target_robot.center_position.z << ") with velocity ("
  //           << context.target_robot.center_velocity.x << ", "
  //           << context.target_robot.center_velocity.y << ", "
  //           << context.target_robot.center_velocity.z << ") and yaw "
  //           << context.target_robot.yaw << " rad." 
  //           << context.target_robot.yaw_velocity << " rad/s." << std::endl;

  if (!context.is_tracking && !context.is_temp_lost) {
    std::cout << "Target not in tracking/temp_lost state, skipping MPC control.  " << std::endl;
    has_prev_state_ = false;
    U_prev_.resize(0);
    // 机动自适应状态重置：防止旧跟踪历史污染新跟踪
    alpha_ema_ = 0.0;
    has_prev_velocity_ = false;
    return createIdleCmd();
  }

  // 1) 差分估计角速度
  double yaw_dot = 0.0;
  double pitch_dot = 0.0;
  if (has_prev_state_) {
    yaw_dot = angles::normalize_angle(context.current_yaw - prev_yaw_) / dt_;
    pitch_dot = (context.current_pitch - prev_pitch_) / dt_;
  }
  prev_yaw_ = context.current_yaw;
  prev_pitch_ = context.current_pitch;
  has_prev_state_ = true;

  // 2) 组装当前状态
  mpc::GimbalDynamicsModel::StateVector x0;
  x0 << context.current_yaw, context.current_pitch, yaw_dot, pitch_dot;

  // 3) 构建/缓存 QP 结构矩阵
  if (matrices_dirty_) {
    rebuildMatrices();
  }

  // 4) 生成参考轨迹
  Eigen::VectorXd X_ref;
  if (enable_delay_compensation_) {
    // 计算处理延迟: 当前时间与 tracker 状态时间戳的差
    double processing_delay = (context.current_time - context.target_stamp).seconds();
    processing_delay = std::clamp(processing_delay, 0.0, max_processing_delay_s_);

    mpc::DelayCompConfig delay_cfg;
    delay_cfg.base_delay_s = processing_delay + control_delay_s_ + prediction_delay_s_;
    delay_cfg.ctrl_delay_s = control_delay_s_;
    delay_cfg.flight_time_iters = flight_time_iters_;

    X_ref = ref_generator_.generateWithDelay(
      context.target_robot, context.current_yaw, context.current_pitch,
      N_, dt_, delay_cfg);
  } else {
    X_ref = ref_generator_.generate(
      context.target_robot, context.current_yaw, context.current_pitch, N_, dt_);
  }

  // 5) 构造 QP
  //    机动自适应模式: 通过对 UKF center_velocity 做时间戳感知差分计算机动因子 alpha,
  //    用 alpha 衰减远期 Q 权重并放大 R 正则项。
  //    如果禁用 (enable_maneuver_adapt_==false) 或目标未机动, 则与原实现完全一致。
  if (enable_maneuver_adapt_ && context.is_maneuvering) {
    // 仅当 target_stamp 发生变化时才更新 alpha（tracker 20-30 Hz 更新，控制环 250 Hz）
    const rclcpp::Time & cur_stamp = context.target_stamp;
    if (has_prev_velocity_ && cur_stamp != prev_target_stamp_) {
      double delta_t = (cur_stamp - prev_target_stamp_).seconds();
      if (delta_t > 1e-6) {
        Eigen::Vector3d vel_now(
          context.target_robot.center_velocity.x,
          context.target_robot.center_velocity.y,
          context.target_robot.center_velocity.z);
        double accel_est = (vel_now - prev_target_velocity_).norm() / delta_t;
        double alpha_raw = std::clamp(accel_est / a_max_, 0.0, 1.0);
        alpha_ema_ = eta_ * alpha_raw + (1.0 - eta_) * alpha_ema_;
      }
    }
    // 更新历史状态（仅 stamp 变化时）
    if (!has_prev_velocity_ || cur_stamp != prev_target_stamp_) {
      prev_target_velocity_ = Eigen::Vector3d(
        context.target_robot.center_velocity.x,
        context.target_robot.center_velocity.y,
        context.target_robot.center_velocity.z);
      prev_target_stamp_ = cur_stamp;
      has_prev_velocity_ = true;
    }

    // 构建自适应权重矩阵
    Eigen::MatrixXd Q_eff = mpc::GimbalDynamicsModel::buildAdaptiveWeightQ(
      N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_, alpha_ema_, tau_);
    Eigen::MatrixXd R_eff = mpc::GimbalDynamicsModel::buildAdaptiveWeightR(
      N_, r_yaw_, r_pitch_, alpha_ema_, r_scale_maneuver_);

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    mpc::GimbalDynamicsModel::buildQP(A_pred_, B_ctrl_, D_, Q_eff, R_eff, S_blk_, x0, X_ref, H, f);

    // 6) 框约束
    int n_vars = 2 * N_;
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_accel_);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_accel_);

    // 7) 求解 QP
    auto result = qp_solver_.solve(H, f, lb, ub);
    if (!result.success) {
      std::cout << "MPC QP solve failed (maneuver-adapt), fallback to direct aim.  " << std::endl;
      return fallbackDirectAim(context, X_ref);
    }

    U_prev_ = result.U;

    // 8) 提取首步控制量
    mpc::GimbalDynamicsModel::ControlVector u_opt(result.U(0), result.U(1));
    auto x_next = dynamics_model_.predict(x0, u_opt);
    double cmd_yaw   = x_next(0);
    double cmd_pitch = x_next(1);

    double yaw_diff   = angles::normalize_angle(cmd_yaw   - context.current_yaw);
    double pitch_diff = cmd_pitch - context.current_pitch;

    double ref_yaw   = X_ref(0);
    double ref_pitch = X_ref(1);
    double distance  = std::sqrt(
      context.target_robot.center_position.x * context.target_robot.center_position.x +
      context.target_robot.center_position.y * context.target_robot.center_position.y +
      context.target_robot.center_position.z * context.target_robot.center_position.z);

    bool fire_advice = false;
    if (fire_advisor_) {
      if (enable_delay_compensation_ && control_delay_s_ > 1e-6) {
        double ref_yaw_dot   = X_ref(2);
        double ref_pitch_dot = X_ref(3);
        double fire_yaw   = context.current_yaw   + ref_yaw_dot   * control_delay_s_;
        double fire_pitch = context.current_pitch + ref_pitch_dot * control_delay_s_;
        fire_advice = fire_advisor_->shouldFire(fire_yaw, fire_pitch, ref_yaw, ref_pitch, distance);
      } else {
        fire_advice = fire_advisor_->shouldFire(
          context.current_yaw, context.current_pitch, ref_yaw, ref_pitch, distance);
      }
    }

    rm_interfaces::msg::GimbalCmd cmd;
    cmd.header     = context.target_robot.header;
    cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
    cmd.pitch      = cmd_pitch * 180.0 / M_PI;
    cmd.yaw_diff   = yaw_diff   * 180.0 / M_PI;
    cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
    cmd.distance   = distance;
    cmd.fire_advice = fire_advice;
    return cmd;
  }

  // 禁用机动自适应或目标未机动时: 使用原有缓存的 Q_blk_, R_blk_
  // 同时将 alpha_ema_ 归零，避免机动结束后残留高权重污染下一次跟踪
  alpha_ema_ = 0.0;
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  mpc::GimbalDynamicsModel::buildQP(A_pred_, B_ctrl_, D_, Q_blk_, R_blk_, S_blk_, x0, X_ref, H, f);

  // 6) 框约束
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_accel_);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_accel_);

  // 7) 求解 QP
  auto result = qp_solver_.solve(H, f, lb, ub);
  if (!result.success) {
    // QP 求解失败: 回退到弹道直瞄
    std::cout << "MPC QP solve failed, fallback to direct aim.  " << std::endl;
    return fallbackDirectAim(context, X_ref);
  }

  // 存储 warmstart
  U_prev_ = result.U;

  // 8) 提取首步控制量, 推算期望 yaw/pitch
  mpc::GimbalDynamicsModel::ControlVector u_opt(result.U(0), result.U(1));
  auto x_next = dynamics_model_.predict(x0, u_opt);

  double cmd_yaw = x_next(0);
  double cmd_pitch = x_next(1);

  // std::cout << "MPC optimal control: yaw_accel=" << u_opt(0) << " rad/s^2, pitch_accel=" << u_opt(1)
  //           << " rad/s^2. Predicted next state: yaw=" << cmd_yaw << " rad, pitch=" << cmd_pitch
  //           << " rad." << std::endl;

  // 9) 计算与当前的差值
  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  // std::cout << "Current state: yaw=" << context.current_yaw << " rad, pitch=" << context.current_pitch
  //           << " rad. Command diff: yaw_diff=" << yaw_diff << " rad, pitch_diff=" << pitch_diff
  //           << " rad." << std::endl;

  // 10) 开火判断: 使用参考轨迹第一步的 yaw/pitch 作为开火目标
  double ref_yaw = X_ref(0);
  double ref_pitch = X_ref(1);
  double distance = std::sqrt(
    context.target_robot.center_position.x * context.target_robot.center_position.x +
    context.target_robot.center_position.y * context.target_robot.center_position.y +
    context.target_robot.center_position.z * context.target_robot.center_position.z);

  bool fire_advice = false;
  if (fire_advisor_) {
    if (enable_delay_compensation_ && control_delay_s_ > 1e-6) {
      // 延时补偿开火判断: 预测控制延迟后的云台姿态
      double ref_yaw_dot = X_ref(2);
      double ref_pitch_dot = X_ref(3);
      double fire_yaw = context.current_yaw + ref_yaw_dot * control_delay_s_;
      double fire_pitch = context.current_pitch + ref_pitch_dot * control_delay_s_;
      fire_advice = fire_advisor_->shouldFire(
        fire_yaw, fire_pitch, ref_yaw, ref_pitch, distance);
    } else {
      fire_advice = fire_advisor_->shouldFire(
        context.current_yaw, context.current_pitch,
        ref_yaw, ref_pitch, distance);
    }
  }

  // 11) 填充 GimbalCmd (角度以度为单位)
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = context.target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::fallbackDirectAim(
  const GimbalControlContext & context,
  const Eigen::VectorXd & X_ref)
{
  // QP 失败时回退: 直接用参考轨迹首步 yaw/pitch 作为目标
  double ref_yaw = X_ref(0);
  double ref_pitch = X_ref(1);

  double yaw_diff = angles::normalize_angle(ref_yaw - context.current_yaw);
  double pitch_diff = ref_pitch - context.current_pitch;

  double distance = std::sqrt(
    context.target_robot.center_position.x * context.target_robot.center_position.x +
    context.target_robot.center_position.y * context.target_robot.center_position.y +
    context.target_robot.center_position.z * context.target_robot.center_position.z);

  bool fire_advice = false;
  if (fire_advisor_) {
    fire_advice = fire_advisor_->shouldFire(
      context.current_yaw, context.current_pitch,
      ref_yaw, ref_pitch, distance);
  }

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = context.target_robot.header;
  cmd.yaw = ref_yaw * 180.0 / M_PI;
  cmd.pitch = ref_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

}  // namespace gimbal_controller
