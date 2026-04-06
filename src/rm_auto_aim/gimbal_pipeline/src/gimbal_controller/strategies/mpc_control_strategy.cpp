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
#include <cmath>
#include <iostream>

#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

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

void MpcControlStrategy::setYawFeedforward(double yaw_feedforward_k_s)
{
  yaw_feedforward_k_s_ = yaw_feedforward_k_s;
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

void MpcControlStrategy::setWeightingParameters(
  bool enable, double alpha, double k_omega,
  double sigma_min, double sigma_max, double sigma_sys,
  double target_size, double delay_s, double max_w,
  double smooth_alpha, double min_distance, double bullet_speed,
  double sigma_beta, double gamma)
{
  enable_weighting_ = enable;
  weighting_alpha_ = alpha;
  weighting_k_omega_ = k_omega;
  weighting_sigma_min_ = sigma_min;
  weighting_sigma_max_ = sigma_max;
  weighting_sigma_sys_ = sigma_sys;
  weighting_target_size_ = target_size;
  weighting_delay_s_ = delay_s;
  weighting_max_w_ = max_w;
  weighting_smooth_alpha_ = smooth_alpha;
  weighting_min_distance_ = min_distance;
  weighting_bullet_speed_ = bullet_speed;
  weighting_sigma_beta_ = sigma_beta;
  weighting_gamma_ = gamma;
  has_prev_w_steps_ = false;
  prev_w_steps_.resize(0);
}

void MpcControlStrategy::setFovConstraintParameters(
  bool enable, double margin, double slack_weight, int constraint_steps,
  bool dynamic_margin_enable, double margin_vel_scale,
  double fallback_fov_yaw, double fallback_fov_pitch)
{
  enable_fov_constraint_ = enable;
  fov_margin_ = margin;
  fov_slack_weight_ = slack_weight;
  fov_constraint_steps_ = constraint_steps;
  enable_dynamic_margin_ = dynamic_margin_enable;
  margin_vel_scale_ = margin_vel_scale;
  fallback_fov_yaw_ = fallback_fov_yaw;
  fallback_fov_pitch_ = fallback_fov_pitch;
  // 初始化 FOV 为 fallback 值，收到 camera_info 后会被覆盖
  if (!camera_info_received_) {
    fov_half_yaw_ = fallback_fov_yaw;
    fov_half_pitch_ = fallback_fov_pitch;
  }
}

void MpcControlStrategy::updateFov(double fov_half_yaw, double fov_half_pitch)
{
  fov_half_yaw_ = fov_half_yaw;
  fov_half_pitch_ = fov_half_pitch;
  camera_info_received_ = true;
}

Eigen::VectorXd MpcControlStrategy::buildWeightingVector(
  const GimbalControlContext & context,
  const Eigen::VectorXd & X_ref)
{
  Eigen::VectorXd w = Eigen::VectorXd::Ones(N_);
  if (!enable_weighting_) {
    return w;
  }

  const auto robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(robot);
  const auto linear_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(robot);
  const auto linear_acceleration =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearAcceleration(robot);
  const double yaw =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yaw(robot);
  const double yaw_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(robot);
  const double yaw_acceleration =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawAcceleration(robot);
  const double bullet_speed = std::max(weighting_bullet_speed_, 1e-3);
  const double alpha = std::max(weighting_alpha_, 0.0);
  const double smooth_alpha = std::clamp(weighting_smooth_alpha_, 0.0, 1.0);
  const double sigma_beta = std::max(weighting_sigma_beta_, 0.0);
  const double gamma = std::max(weighting_gamma_, 1e-3);
  const int nx = mpc::GimbalDynamicsModel::STATE_DIM;

  for (int k = 0; k < N_; ++k) {
    double t = (k + 1) * dt_;

    // 预测时刻目标位置：考虑匀加速运动模型的三阶预测，适用于快速机动的目标
    double px = center_position.x() + linear_velocity.x() * t +
      0.5 * linear_acceleration.x() * t * t;
    double py = center_position.y() + linear_velocity.y() * t +
      0.5 * linear_acceleration.y() * t * t;
    double pz = center_position.z() + linear_velocity.z() * t +
      0.5 * linear_acceleration.z() * t * t;
    double distance = std::sqrt(px * px + py * py + pz * pz);
    distance = std::max(distance, weighting_min_distance_);

    double yaw_k = yaw + yaw_velocity * t +
      0.5 * yaw_acceleration * t * t;
    double omega_k = yaw_velocity + yaw_acceleration * t;

    double t_bullet = distance / bullet_speed;
    // 命中时刻角度：考虑子弹飞行时间的目标朝向
    double theta_hit = yaw_k + omega_k * t_bullet;
    // 参考轨迹给出的“理想击打角”
    double theta_target = X_ref(k * nx);
    // 注意：避免重复提前量，这里不再叠加 delay
    double dtheta = angles::normalize_angle(theta_hit - theta_target);

    // 角度不确定性模型：考虑目标尺寸、系统误差和动态误差
    double sigma_theta = weighting_target_size_ / distance; // 目标尺寸引起的角度不确定性
    double sigma_eff = std::sqrt(sigma_theta * sigma_theta +
      weighting_sigma_sys_ * weighting_sigma_sys_);         // 系统误差引起的角度不确定性
    double sigma_dynamic = sigma_eff / (1.0 + sigma_beta * std::abs(omega_k)); // 动态误差引起的角度不确定性，快速转动时不确定性增大
    double sigma = std::clamp(sigma_dynamic, weighting_sigma_min_, weighting_sigma_max_);

    // 权重计算：不确定性越大权重越小；快速转动时权重降低；最终通过 gamma 调整权重衰减的激烈程度
    double p_angle = std::exp(-0.5 * (dtheta * dtheta) / (sigma * sigma));
    double p_omega = std::exp(-weighting_k_omega_ * std::abs(omega_k));
    double r_k = p_angle * p_omega;
    r_k = std::pow(std::clamp(r_k, 0.0, 1.0), gamma);

    double w_k = 1.0 + alpha * r_k;
    w(k) = std::clamp(w_k, 1.0, weighting_max_w_);
  }

  // 权重平滑：与前一次计算的权重进行指数移动平均，避免权重突变导致控制输入抖动
  if (smooth_alpha > 1e-6 && has_prev_w_steps_ && prev_w_steps_.size() == N_) {
    w = smooth_alpha * prev_w_steps_ + (1.0 - smooth_alpha) * w;
  }

  prev_w_steps_ = w;
  has_prev_w_steps_ = true;
  return w;
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
  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const Eigen::Vector3d target_linear_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(target_robot);
  const double target_distance =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerDistance(target_robot);

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
    has_prev_w_steps_ = false;
    prev_w_steps_.resize(0);
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
  const double yaw_feedforward_s =
    std::clamp(yaw_feedforward_k_s_, 0.0, max_yaw_feedforward_s_);
  const bool use_delayed_reference =
    enable_delay_compensation_ || (yaw_feedforward_s > 1e-6);

  Eigen::VectorXd X_ref;
  if (use_delayed_reference) {
    // 仅在启用 delay compensation 时使用 processing_delay。
    // 当仅启用 yaw 前馈时保持 processing_delay=0，避免改变原有处理延迟语义。
    double processing_delay = 0.0;
    if (enable_delay_compensation_) {
      processing_delay = (context.current_time - context.target_stamp).seconds();
      processing_delay = std::clamp(processing_delay, 0.0, max_processing_delay_s_);
    }

    mpc::DelayCompConfig delay_cfg;
    delay_cfg.base_delay_s = processing_delay + prediction_delay_s_;
    delay_cfg.ctrl_delay_s = control_delay_s_;
    delay_cfg.flight_time_iters = flight_time_iters_;

    X_ref = ref_generator_.generateWithDelay(
      target_robot, context.current_yaw, context.current_pitch,
      N_, dt_, delay_cfg);
  } else {
    X_ref = ref_generator_.generate(
      target_robot, context.current_yaw, context.current_pitch, N_, dt_);
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
        Eigen::Vector3d vel_now = target_linear_velocity;
        double accel_est = (vel_now - prev_target_velocity_).norm() / delta_t;
        double alpha_raw = std::clamp(accel_est / a_max_, 0.0, 1.0);
        alpha_ema_ = eta_ * alpha_raw + (1.0 - eta_) * alpha_ema_;
      }
    }
    // 更新历史状态（仅 stamp 变化时）
    if (!has_prev_velocity_ || cur_stamp != prev_target_stamp_) {
      prev_target_velocity_ = target_linear_velocity;
      prev_target_stamp_ = cur_stamp;
      has_prev_velocity_ = true;
    }

    // 构建自适应权重矩阵
    Eigen::MatrixXd Q_eff = mpc::GimbalDynamicsModel::buildAdaptiveWeightQ(
      N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_, alpha_ema_, tau_);
    if (enable_weighting_) {
      Eigen::VectorXd w_steps = buildWeightingVector(context, X_ref);
      mpc::GimbalDynamicsModel::scaleBlockDiagonalQ(Q_eff, w_steps);
    }
    Eigen::MatrixXd R_eff = mpc::GimbalDynamicsModel::buildAdaptiveWeightR(
      N_, r_yaw_, r_pitch_, alpha_ema_, r_scale_maneuver_);

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    mpc::GimbalDynamicsModel::buildQP(A_pred_, B_ctrl_, D_, Q_eff, R_eff, S_blk_, x0, X_ref, H, f);

    // 6) 框约束 + 可选 FOV 软约束
    int n_vars = 2 * N_;
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_accel_);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_accel_);

    // 7) 求解 QP
    mpc::QPResult result;
    if (enable_fov_constraint_) {
      result = solveFovConstrainedQP(H, f, lb, ub, x0, X_ref, context);
    } else {
      result = qp_solver_.solve(H, f, lb, ub);
    }
    if (!result.success) {
      std::cout << "MPC QP solve failed (maneuver-adapt), fallback to direct aim.  " << std::endl;
      return fallbackDirectAim(context, X_ref);
    }

    U_prev_ = result.U;

    // 8) 提取首步控制量
    mpc::GimbalDynamicsModel::ControlVector u_opt(result.U(0), result.U(1));
    auto x_next = dynamics_model_.predict(x0, u_opt);
    double cmd_yaw   = angles::normalize_angle(x_next(0));
    double cmd_pitch = x_next(1);

    double yaw_diff   = angles::normalize_angle(cmd_yaw   - context.current_yaw);
    double pitch_diff = cmd_pitch - context.current_pitch;

    double ref_yaw   = X_ref(0);
    double ref_pitch = X_ref(1);
    double distance  = target_distance;

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
    cmd.header     = target_robot.header;
    cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
    cmd.pitch      = cmd_pitch * 180.0 / M_PI;
    cmd.yaw_diff   = yaw_diff   * 180.0 / M_PI;
    cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
    cmd.yaw_v      = x_next(2) * 180.0 / M_PI;
    cmd.pitch_v    = x_next(3) * 180.0 / M_PI;
    // TEMP_LOST 时无 detector 实际观测，distance 输出 -1 以示无有效测量
    cmd.distance   = context.is_temp_lost ? -1.0 : distance;
    cmd.fire_advice = fire_advice;
    return cmd;
  }

  // 禁用机动自适应或目标未机动时: 使用原有缓存的 Q_blk_, R_blk_
  // 同时将 alpha_ema_ 归零，避免机动结束后残留高权重污染下一次跟踪
  alpha_ema_ = 0.0;
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  Eigen::MatrixXd Q_eff = Q_blk_;
  if (enable_weighting_) {
    Eigen::VectorXd w_steps = buildWeightingVector(context, X_ref);
    mpc::GimbalDynamicsModel::scaleBlockDiagonalQ(Q_eff, w_steps);
  }
  mpc::GimbalDynamicsModel::buildQP(A_pred_, B_ctrl_, D_, Q_eff, R_blk_, S_blk_, x0, X_ref, H, f);

  // 6) 框约束 + 可选 FOV 软约束
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_accel_);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_accel_);

  // 7) 求解 QP
  mpc::QPResult result;
  if (enable_fov_constraint_) {
    result = solveFovConstrainedQP(H, f, lb, ub, x0, X_ref, context);
  } else {
    result = qp_solver_.solve(H, f, lb, ub);
  }
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

  double cmd_yaw = angles::normalize_angle(x_next(0));
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
  double distance = target_distance;

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
  cmd.header = target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.yaw_v = x_next(2) * 180.0 / M_PI;
  cmd.pitch_v = x_next(3) * 180.0 / M_PI;
  // TEMP_LOST 时无 detector 实际观测，distance 输出 -1 以示无有效测量
  cmd.distance = context.is_temp_lost ? -1.0 : distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

mpc::QPResult MpcControlStrategy::solveFovConstrainedQP(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  const mpc::GimbalDynamicsModel::StateVector & x0,
  const Eigen::VectorXd & X_ref,
  const GimbalControlContext & context)
{
  const int n_u = 2 * N_;

  // 确定约束步数 K
  int K = (fov_constraint_steps_ > 0 && fov_constraint_steps_ < N_)
    ? fov_constraint_steps_ : N_;

  // 计算有效 margin（可选动态调整）
  double margin_eff = fov_margin_;
  if (enable_dynamic_margin_) {
    const auto target_robot =
      fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
    const Eigen::Vector3d target_velocity =
      fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(target_robot);
    double v_target = target_velocity.norm();
    margin_eff += margin_vel_scale_ * v_target;
  }

  // 构建 FOV 软约束扩展 QP
  Eigen::MatrixXd H_ext, A_con;
  Eigen::VectorXd f_ext, lbA, ubA;

  mpc::GimbalDynamicsModel::buildFovSoftConstraintQP(
    H, f, A_pred_, B_ctrl_, x0, X_ref,
    N_, K,
    fov_half_yaw_, fov_half_pitch_, margin_eff, fov_slack_weight_,
    H_ext, f_ext, A_con, lbA, ubA);

  // 扩展 box 约束: [U bounds; slack >= 0]
  const int n_s = 2 * K;
  const int n_z = n_u + n_s;
  Eigen::VectorXd lb_ext(n_z), ub_ext(n_z);
  lb_ext.head(n_u) = lb;
  lb_ext.tail(n_s) = Eigen::VectorXd::Zero(n_s);          // slack >= 0
  ub_ext.head(n_u) = ub;
  ub_ext.tail(n_s) = Eigen::VectorXd::Constant(n_s, 1e6); // slack 上界

  // 求解扩展 QP（使用独立求解器实例，因为维度不同于原始 QP）
  auto result_ext = qp_solver_fov_.solve(H_ext, f_ext, lb_ext, ub_ext, A_con, lbA, ubA);

  // 提取原始控制变量部分
  mpc::QPResult result;
  result.success = result_ext.success;
  result.num_iterations = result_ext.num_iterations;
  result.cost = result_ext.cost;
  if (result_ext.success) {
    result.U = result_ext.U.head(n_u);
  }
  return result;
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::fallbackDirectAim(
  const GimbalControlContext & context,
  const Eigen::VectorXd & X_ref)
{
  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);

  // QP 失败时回退: 直接用参考轨迹首步 yaw/pitch 作为目标
  double ref_yaw = X_ref(0);
  double ref_pitch = X_ref(1);

  double yaw_diff = angles::normalize_angle(ref_yaw - context.current_yaw);
  double pitch_diff = ref_pitch - context.current_pitch;

  double distance =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerDistance(target_robot);

  bool fire_advice = false;
  if (fire_advisor_) {
    fire_advice = fire_advisor_->shouldFire(
      context.current_yaw, context.current_pitch,
      ref_yaw, ref_pitch, distance);
  }

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = target_robot.header;
  cmd.yaw = ref_yaw * 180.0 / M_PI;
  cmd.pitch = ref_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.yaw_v = 0.0;
  cmd.pitch_v = 0.0;
  // TEMP_LOST 时无 detector 实际观测，distance 输出 -1 以示无有效测量
  cmd.distance = context.is_temp_lost ? -1.0 : distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

}  // namespace gimbal_controller
