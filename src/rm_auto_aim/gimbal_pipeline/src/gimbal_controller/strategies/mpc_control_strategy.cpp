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

#include <Eigen/Eigenvalues>
#include <angles/angles.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace gimbal_controller
{

namespace
{

double safeRatio(double num, double den, double eps = 1e-9)
{
  return num / std::max(std::abs(den), eps);
}

int countActiveBounds(
  const Eigen::VectorXd & x,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  double tol)
{
  const int n = static_cast<int>(x.size());
  int count = 0;
  for (int i = 0; i < n; ++i) {
    if (std::abs(x(i) - lb(i)) <= tol || std::abs(ub(i) - x(i)) <= tol) {
      ++count;
    }
  }
  return count;
}

int countActiveLinear(
  const Eigen::VectorXd & Ax,
  const Eigen::VectorXd & lbA,
  const Eigen::VectorXd & ubA,
  double tol)
{
  const int m = static_cast<int>(Ax.size());
  int count = 0;
  for (int i = 0; i < m; ++i) {
    const bool has_lb = lbA(i) > -1e19;
    const bool has_ub = ubA(i) < 1e19;
    if ((has_lb && std::abs(Ax(i) - lbA(i)) <= tol) ||
      (has_ub && std::abs(ubA(i) - Ax(i)) <= tol))
    {
      ++count;
    }
  }
  return count;
}

}  // namespace

MpcControlStrategy::MpcControlStrategy()
: dynamics_model_(0.01)
{
  configureRmsWindows();
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
  bool enable, double prediction_delay_s, double trigger_to_muzzle_s, int flight_time_iters,
  double max_processing_delay_s)
{
  enable_delay_compensation_ = enable;
  prediction_delay_s_ = prediction_delay_s;
  trigger_to_muzzle_s_ = trigger_to_muzzle_s;
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

void MpcControlStrategy::setNumericalNormalizationParameters(
  bool enable, int window_size, int min_samples, double rms_epsilon,
  const std::string & mode,
  const Eigen::Vector4d & state_typical,
  const Eigen::Vector2d & control_typical,
  const Eigen::Vector2d & delta_control_typical)
{
  enable_normalization_ = enable;
  rms_window_size_ = std::max(1, window_size);
  rms_min_samples_ = std::max(1, min_samples);
  rms_epsilon_ = std::max(rms_epsilon, 1e-12);

  std::string mode_lower = mode;
  std::transform(
    mode_lower.begin(), mode_lower.end(), mode_lower.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if (mode_lower == "typical") {
    normalization_mode_ = NormalizationMode::TYPICAL;
  } else {
    normalization_mode_ = NormalizationMode::RMS;
    if (mode_lower != "rms") {
      RCLCPP_WARN(
        rclcpp::get_logger("MpcControlStrategy"),
        "Unknown normalization mode '%s', fallback to 'rms'.", mode.c_str());
    }
  }

  state_typical_ = state_typical.cwiseAbs().cwiseMax(Eigen::Vector4d::Constant(1e-12));
  control_typical_ = control_typical.cwiseAbs().cwiseMax(Eigen::Vector2d::Constant(1e-12));
  delta_control_typical_ =
    delta_control_typical.cwiseAbs().cwiseMax(Eigen::Vector2d::Constant(1e-12));

  configureRmsWindows();
  resetNumericalStates();
}

void MpcControlStrategy::setHessianRegularizationParameters(
  bool enable, double epsilon_abs, double epsilon_rel, double epsilon_max,
  bool retry_on_fail, double retry_scale)
{
  enable_hessian_regularization_ = enable;
  hessian_reg_eps_abs_ = std::max(epsilon_abs, 1e-12);
  hessian_reg_eps_rel_ = std::max(epsilon_rel, 0.0);
  hessian_reg_eps_max_ = std::max(epsilon_max, hessian_reg_eps_abs_);
  hessian_reg_retry_on_fail_ = retry_on_fail;
  hessian_reg_retry_scale_ = std::max(retry_scale, 1.0);
}

void MpcControlStrategy::setDiagnosticsParameters(
  bool enable,
  bool low_cost_always,
  bool high_cost_enable,
  int high_cost_sample_every,
  int log_every,
  bool log_on_failure,
  double active_tol,
  double rank_tol_rel)
{
  enable_diagnostics_ = enable;
  diagnostics_low_cost_always_ = low_cost_always;
  diagnostics_high_cost_enable_ = high_cost_enable;
  diagnostics_high_cost_sample_every_ = std::max(1, high_cost_sample_every);
  diagnostics_log_every_ = std::max(1, log_every);
  diagnostics_log_on_failure_ = log_on_failure;
  diagnostics_active_tol_ = std::max(active_tol, 1e-9);
  diagnostics_rank_tol_rel_ = std::max(rank_tol_rel, 1e-12);
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

void MpcControlStrategy::configureRmsWindows()
{
  for (auto & tracker : state_rms_trackers_) {
    tracker.setWindowSize(rms_window_size_);
  }
  for (auto & tracker : control_rms_trackers_) {
    tracker.setWindowSize(rms_window_size_);
  }
  for (auto & tracker : delta_control_rms_trackers_) {
    tracker.setWindowSize(rms_window_size_);
  }
}

void MpcControlStrategy::resetNumericalStates()
{
  for (auto & tracker : state_rms_trackers_) {
    tracker.reset();
  }
  for (auto & tracker : control_rms_trackers_) {
    tracker.reset();
  }
  for (auto & tracker : delta_control_rms_trackers_) {
    tracker.reset();
  }
  prev_applied_u_.setZero();
  has_prev_applied_u_ = false;
  diagnostics_cycle_ = 0;
  last_diagnostics_ = DiagnosticsSnapshot{};
}

Eigen::Vector4d MpcControlStrategy::updateAndGetStateRms(const Eigen::VectorXd & free_error)
{
  const int nx = mpc::GimbalDynamicsModel::STATE_DIM;
  Eigen::Vector4d result = Eigen::Vector4d::Ones();

  for (int d = 0; d < nx; ++d) {
    double mean_sq = 0.0;
    for (int k = 0; k < N_; ++k) {
      const double v = free_error(k * nx + d);
      mean_sq += v * v;
    }
    mean_sq /= std::max(N_, 1);
    state_rms_trackers_[d].addSample(std::sqrt(std::max(mean_sq, 0.0)));

    if (state_rms_trackers_[d].size() >= rms_min_samples_) {
      result(d) = state_rms_trackers_[d].rms(rms_epsilon_, 1.0);
    }
  }

  return result;
}

Eigen::Vector2d MpcControlStrategy::getControlRms() const
{
  Eigen::Vector2d result = Eigen::Vector2d::Ones();
  for (int i = 0; i < mpc::GimbalDynamicsModel::CONTROL_DIM; ++i) {
    if (control_rms_trackers_[i].size() >= rms_min_samples_) {
      result(i) = control_rms_trackers_[i].rms(rms_epsilon_, 1.0);
    }
  }
  return result;
}

Eigen::Vector2d MpcControlStrategy::getDeltaControlRms() const
{
  Eigen::Vector2d result = Eigen::Vector2d::Ones();
  for (int i = 0; i < mpc::GimbalDynamicsModel::CONTROL_DIM; ++i) {
    if (delta_control_rms_trackers_[i].size() >= rms_min_samples_) {
      result(i) = delta_control_rms_trackers_[i].rms(rms_epsilon_, 1.0);
    }
  }
  return result;
}

void MpcControlStrategy::updateControlHistory(const mpc::GimbalDynamicsModel::ControlVector & u_opt)
{
  for (int i = 0; i < mpc::GimbalDynamicsModel::CONTROL_DIM; ++i) {
    control_rms_trackers_[i].addSample(u_opt(i));
  }

  if (has_prev_applied_u_) {
    const auto du = u_opt - prev_applied_u_;
    for (int i = 0; i < mpc::GimbalDynamicsModel::CONTROL_DIM; ++i) {
      delta_control_rms_trackers_[i].addSample(du(i));
    }
  } else {
    for (int i = 0; i < mpc::GimbalDynamicsModel::CONTROL_DIM; ++i) {
      delta_control_rms_trackers_[i].addSample(u_opt(i));
    }
  }

  prev_applied_u_ = u_opt;
  has_prev_applied_u_ = true;
}

double MpcControlStrategy::computeRegularizationEpsilon(const Eigen::MatrixXd & H) const
{
  const double mean_diag = H.diagonal().cwiseAbs().mean();
  const double eps_rel = hessian_reg_eps_rel_ * mean_diag;
  const double eps = std::max(hessian_reg_eps_abs_, eps_rel);
  return std::clamp(eps, hessian_reg_eps_abs_, hessian_reg_eps_max_);
}

void MpcControlStrategy::applyHessianRegularization(Eigen::MatrixXd & H, double epsilon) const
{
  H.diagonal().array() += epsilon;
  H = 0.5 * (H + H.transpose());
}

void MpcControlStrategy::fillAndLogDiagnostics(
  bool maneuver_path,
  const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & Q_eff,
  const Eigen::MatrixXd & R_eff,
  const Eigen::MatrixXd & S_eff,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  const mpc::QPResult & result,
  double applied_regularization)
{
  ++diagnostics_cycle_;
  if (!enable_diagnostics_) {
    return;
  }

  last_diagnostics_ = DiagnosticsSnapshot{};
  last_diagnostics_.cycle = diagnostics_cycle_;
  last_diagnostics_.maneuver_path = maneuver_path;
  last_diagnostics_.qp_success = result.success;
  last_diagnostics_.qp_iterations = result.num_iterations;
  last_diagnostics_.active_bound_size = result.active_bound_size;
  last_diagnostics_.active_linear_size = result.active_linear_size;
  last_diagnostics_.active_set_size = result.active_set_size;
  last_diagnostics_.qp_cost = result.cost;
  last_diagnostics_.regularization_eps = applied_regularization;

  const bool should_compute_low = diagnostics_low_cost_always_ || !result.success;
  if (should_compute_low) {
    last_diagnostics_.trace_q = Q_eff.diagonal().sum();
    last_diagnostics_.trace_r = R_eff.diagonal().sum();
    last_diagnostics_.trace_s = S_eff.diagonal().sum();
    last_diagnostics_.trace_q_over_r = safeRatio(last_diagnostics_.trace_q, last_diagnostics_.trace_r);
    last_diagnostics_.trace_s_over_r = safeRatio(last_diagnostics_.trace_s, last_diagnostics_.trace_r);

    if (result.success && result.U.size() == lb.size() && result.U.size() == ub.size()) {
      const double u_norm = result.U.norm();
      const double bu_norm = (B_ctrl_ * result.U).norm();
      last_diagnostics_.bu_over_u = safeRatio(bu_norm, u_norm);
      if (last_diagnostics_.active_bound_size == 0) {
        last_diagnostics_.active_bound_size =
          countActiveBounds(result.U, lb, ub, diagnostics_active_tol_);
        last_diagnostics_.active_set_size =
          last_diagnostics_.active_bound_size + last_diagnostics_.active_linear_size;
      }
    }
  }

  const bool high_cost_by_sample =
    diagnostics_high_cost_enable_ &&
    (diagnostics_cycle_ % static_cast<uint64_t>(diagnostics_high_cost_sample_every_) == 0);
  const bool should_compute_high = high_cost_by_sample || (diagnostics_log_on_failure_ && !result.success);
  if (should_compute_high) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_solver(H);
    if (eig_solver.info() == Eigen::Success) {
      const Eigen::VectorXd eig_vals = eig_solver.eigenvalues();
      const double lambda_max = eig_vals.maxCoeff();
      const double lambda_min = eig_vals.minCoeff();
      const double rank_tol = std::max(std::abs(lambda_max) * diagnostics_rank_tol_rel_, 1e-12);
      int rank = 0;
      for (int i = 0; i < eig_vals.size(); ++i) {
        if (std::abs(eig_vals(i)) > rank_tol) {
          ++rank;
        }
      }

      last_diagnostics_.high_cost_valid = true;
      last_diagnostics_.lambda_min_h = lambda_min;
      last_diagnostics_.cond_h = safeRatio(lambda_max, lambda_min, 1e-12);
      last_diagnostics_.rank_h = rank;
    }
  }

  const bool should_log =
    (diagnostics_cycle_ % static_cast<uint64_t>(diagnostics_log_every_) == 0) ||
    (diagnostics_log_on_failure_ && !result.success);
  if (!should_log) {
    return;
  }

  auto logger = rclcpp::get_logger("MpcControlStrategy");
  if (last_diagnostics_.high_cost_valid) {
    RCLCPP_INFO(
      logger,
      "[MPC-NUM] cyc=%llu path=%s ok=%d it=%d act=%d(cost=%.3e) trQ/R=%.3e trS/R=%.3e BU/U=%.3e reg=%.3e cond=%.3e lmin=%.3e rank=%d",
      static_cast<unsigned long long>(last_diagnostics_.cycle),
      last_diagnostics_.maneuver_path ? "maneuver" : "normal",
      last_diagnostics_.qp_success ? 1 : 0,
      last_diagnostics_.qp_iterations,
      last_diagnostics_.active_set_size,
      last_diagnostics_.qp_cost,
      last_diagnostics_.trace_q_over_r,
      last_diagnostics_.trace_s_over_r,
      last_diagnostics_.bu_over_u,
      last_diagnostics_.regularization_eps,
      last_diagnostics_.cond_h,
      last_diagnostics_.lambda_min_h,
      last_diagnostics_.rank_h);
  } else {
    RCLCPP_INFO(
      logger,
      "[MPC-NUM] cyc=%llu path=%s ok=%d it=%d act=%d cost=%.3e trQ/R=%.3e trS/R=%.3e BU/U=%.3e reg=%.3e",
      static_cast<unsigned long long>(last_diagnostics_.cycle),
      last_diagnostics_.maneuver_path ? "maneuver" : "normal",
      last_diagnostics_.qp_success ? 1 : 0,
      last_diagnostics_.qp_iterations,
      last_diagnostics_.active_set_size,
      last_diagnostics_.qp_cost,
      last_diagnostics_.trace_q_over_r,
      last_diagnostics_.trace_s_over_r,
      last_diagnostics_.bu_over_u,
      last_diagnostics_.regularization_eps);
  }
}

void MpcControlStrategy::rebuildMatrices()
{
  dynamics_model_.buildPredictionMatrices(N_, A_pred_, B_ctrl_);

  uses_delayed_b_model_ = (control_delay_s_ > 1e-6);
  if (uses_delayed_b_model_) {
    B_ctrl_ = dynamics_model_.buildDelayedB(N_, control_delay_s_);
  }

  D_ = mpc::GimbalDynamicsModel::buildDifferenceMatrix(N_);
  Q_blk_ = mpc::GimbalDynamicsModel::buildWeightQ(N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_);
  R_blk_ = mpc::GimbalDynamicsModel::buildWeightR(N_, r_yaw_, r_pitch_);
  S_blk_ = mpc::GimbalDynamicsModel::buildWeightS(N_, s_yaw_, s_pitch_);

  matrices_dirty_ = false;
}

// ============================================================================
// solve() 辅助方法
// ============================================================================

void MpcControlStrategy::resetTrackingState()
{
  has_prev_state_ = false;
  alpha_ema_ = 0.0;
  has_prev_velocity_ = false;
  has_prev_w_steps_ = false;
  prev_w_steps_.resize(0);
  resetNumericalStates();
}

void MpcControlStrategy::updateManeuverAlpha(
  const Eigen::Vector3d & target_linear_velocity,
  const rclcpp::Time & target_stamp)
{
  if (has_prev_velocity_ && target_stamp != prev_target_stamp_) {
    double delta_t = (target_stamp - prev_target_stamp_).seconds();
    if (delta_t > 1e-6) {
      double accel_est = (target_linear_velocity - prev_target_velocity_).norm() / delta_t;
      double alpha_raw = std::clamp(accel_est / a_max_, 0.0, 1.0);
      alpha_ema_ = eta_ * alpha_raw + (1.0 - eta_) * alpha_ema_;
    }
  }
  if (!has_prev_velocity_ || target_stamp != prev_target_stamp_) {
    prev_target_velocity_ = target_linear_velocity;
    prev_target_stamp_ = target_stamp;
    has_prev_velocity_ = true;
  }
}

bool MpcControlStrategy::solveQpWithRetry(
  Eigen::MatrixXd & H, const Eigen::VectorXd & f,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub,
  const Eigen::Vector4d & x0, const Eigen::VectorXd & X_ref,
  const GimbalControlContext & context,
  double & applied_regularization,
  mpc::QPResult & result)
{
  auto solveOnce = [&](const Eigen::MatrixXd & H_mat) {
    if (enable_fov_constraint_) {
      result = solveFovConstrainedQP(H_mat, f, lb, ub, x0, X_ref, context);
    } else {
      result = qp_solver_.solve(H_mat, f, lb, ub);
      if (result.success) {
        result.active_bound_size = countActiveBounds(result.U, lb, ub, diagnostics_active_tol_);
        result.active_set_size = result.active_bound_size;
      }
    }
  };

  solveOnce(H);
  if (!result.success && enable_hessian_regularization_ && hessian_reg_retry_on_fail_) {
    const double retry_eps = std::clamp(
      applied_regularization * hessian_reg_retry_scale_,
      hessian_reg_eps_abs_, hessian_reg_eps_max_);
    Eigen::MatrixXd H_retry = H;
    applyHessianRegularization(H_retry, retry_eps);

    solveOnce(H_retry);
    if (result.success) {
      H = std::move(H_retry);
      applied_regularization = retry_eps;
    }
  }

  return result.success;
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::buildGimbalCmd(
  const Eigen::Vector4d & x0, const Eigen::Vector2d & u_opt,
  const GimbalControlContext & context, double distance)
{
  auto x_next = dynamics_model_.predict(x0, u_opt);

  double cmd_yaw = angles::normalize_angle(x_next(0));
  double cmd_pitch = x_next(1);
  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.yaw_v = x_next(2) * 180.0 / M_PI;
  cmd.pitch_v = x_next(3) * 180.0 / M_PI;
  cmd.yaw_a = u_opt(0) * 180.0 / M_PI;
  cmd.pitch_a = u_opt(1) * 180.0 / M_PI;
  cmd.distance = std::max(distance, 0.0);
  return cmd;
}

void MpcControlStrategy::fillDelayAudit(
  const delay_management::MpcDelayResult & mpc_delay,
  const GimbalControlContext & context)
{
  DelayAuditSnapshot audit;
  audit.strategy_name = getName();
  audit.tracking = context.is_tracking;
  audit.processing_delay_s = mpc_delay.processing_delay_s;
  audit.prediction_extra_s = std::max(prediction_delay_s_, 0.0);
  audit.flight_time_s = 0.0;
  audit.total_prediction_time_s = mpc_delay.base_reference_delay_s;
  audit.control_latency_s = mpc_delay.control_latency_s;
  audit.fire_control_compensation_s = mpc_delay.fire_control_compensation_s;
  audit.control_delay_steps = mpc_delay.control_delay_steps;
  audit.uses_delayed_b = mpc_delay.uses_delayed_b;
  audit.double_compensation_risk = mpc_delay.double_compensation_risk;
  markDelayAuditValid(audit);
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::solve(
  const GimbalControlContext & context)
{
  // 预处理目标状态
  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const Eigen::Vector3d target_linear_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(target_robot);
  const double target_distance =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerDistance(target_robot);

  // 未跟踪时返回空闲指令
  if (!context.is_tracking && !context.is_temp_lost) {
    markDelayAuditInvalid(getName(), false);
    resetTrackingState();
    return createIdleCmd();
  }

  // 差分估计角速度
  double yaw_dot = 0.0, pitch_dot = 0.0;
  if (has_prev_state_) {
    yaw_dot = angles::normalize_angle(context.current_yaw - prev_yaw_) / dt_;
    pitch_dot = (context.current_pitch - prev_pitch_) / dt_;
  }
  prev_yaw_ = context.current_yaw;
  prev_pitch_ = context.current_pitch;
  has_prev_state_ = true;

  // 构建初始状态
  Eigen::Vector4d x0;
  x0 << context.current_yaw, context.current_pitch, yaw_dot, pitch_dot;

  // 参数变更时重建 QP 结构矩阵
  if (matrices_dirty_) rebuildMatrices();

  // 延迟语义计算
  const double yaw_feedforward_s =
    std::clamp(yaw_feedforward_k_s_, 0.0, max_yaw_feedforward_s_);
  const bool use_delayed_reference =
    enable_delay_compensation_ || (yaw_feedforward_s > 1e-6);

  delay_management::DelayRawInputs delay_raw;
  delay_raw.current_time = context.current_time;
  delay_raw.observation_stamp = context.target_stamp;
  delay_raw.prediction_extra_s = prediction_delay_s_;
  delay_raw.control_latency_s = control_delay_s_;
  delay_raw.trigger_to_muzzle_s = trigger_to_muzzle_s_;
  delay_raw.max_processing_delay_s = max_processing_delay_s_;

  const bool allow_muzzle_compensation =
    enable_delay_compensation_ && trigger_to_muzzle_s_ > 1e-6;
  const auto mpc_delay = delay_manager_.computeMpcDelay(
    delay_raw, dt_, enable_delay_compensation_,
    uses_delayed_b_model_, allow_muzzle_compensation);

  if (mpc_delay.double_compensation_risk && !warned_double_compensation_) {
    RCLCPP_WARN(rclcpp::get_logger("MpcControlStrategy"),
      "检测到延迟双重补偿风险: delayed B 已激活, fire control 延迟补偿已被禁用.");
    warned_double_compensation_ = true;
  }

  fillDelayAudit(mpc_delay, context);

  // 生成参考轨迹
  Eigen::VectorXd X_ref;
  if (use_delayed_reference) {
    mpc::DelayCompConfig delay_cfg;
    delay_cfg.base_delay_s = mpc_delay.base_reference_delay_s;
    delay_cfg.ctrl_delay_s = mpc_delay.control_latency_s;
    delay_cfg.flight_time_iters = flight_time_iters_;
    X_ref = ref_generator_.generateWithDelay(
      target_robot, context.current_yaw, context.current_pitch, N_, dt_, delay_cfg);
  } else {
    X_ref = ref_generator_.generate(
      target_robot, context.current_yaw, context.current_pitch, N_, dt_);
  }

  // 数值归一化尺度
  const bool use_normalization = enable_normalization_;
  const bool use_rms_normalization =
    use_normalization && normalization_mode_ == NormalizationMode::RMS;

  Eigen::Vector4d state_scale = state_typical_;
  Eigen::Vector2d control_scale = control_typical_;
  Eigen::Vector2d delta_control_scale = delta_control_typical_;

  if (use_rms_normalization) {
    const Eigen::VectorXd free_error = A_pred_ * x0 - X_ref;
    state_scale = updateAndGetStateRms(free_error);
    control_scale = getControlRms();
    delta_control_scale = getDeltaControlRms();
  }

  // 构建 QP 权重矩阵
  Eigen::MatrixXd Q_eff, R_eff, S_eff;
  bool is_maneuver_path = false;

  if (enable_maneuver_adapt_ && context.is_maneuvering) {
    is_maneuver_path = true;
    updateManeuverAlpha(target_linear_velocity, context.target_stamp);

    Q_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildAdaptiveWeightQ(
          N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_,
          alpha_ema_, tau_, state_scale, rms_epsilon_)
      : mpc::GimbalDynamicsModel::buildAdaptiveWeightQ(
          N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_, alpha_ema_, tau_);

    R_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildAdaptiveWeightR(
          N_, r_yaw_, r_pitch_, alpha_ema_, r_scale_maneuver_, control_scale, rms_epsilon_)
      : mpc::GimbalDynamicsModel::buildAdaptiveWeightR(
          N_, r_yaw_, r_pitch_, alpha_ema_, r_scale_maneuver_);

    S_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildWeightS(
          N_, s_yaw_, s_pitch_, delta_control_scale, rms_epsilon_)
      : S_blk_;
  } else {
    alpha_ema_ = 0.0;

    Q_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildWeightQ(
          N_, q_yaw_, q_pitch_, q_yaw_vel_, q_pitch_vel_, state_scale, rms_epsilon_)
      : Q_blk_;

    R_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildWeightR(
          N_, r_yaw_, r_pitch_, control_scale, rms_epsilon_)
      : R_blk_;

    S_eff = use_normalization
      ? mpc::GimbalDynamicsModel::buildWeightS(
          N_, s_yaw_, s_pitch_, delta_control_scale, rms_epsilon_)
      : S_blk_;
  }

  if (enable_weighting_) {
    mpc::GimbalDynamicsModel::scaleBlockDiagonalQ(
        Q_eff, buildWeightingVector(context, X_ref));
  }

  // 组装 QP 并求解
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  mpc::GimbalDynamicsModel::buildQP(A_pred_, B_ctrl_, D_, Q_eff, R_eff, S_eff, x0, X_ref, H, f);

  double applied_regularization = 0.0;
  if (enable_hessian_regularization_) {
    applied_regularization = computeRegularizationEpsilon(H);
    applyHessianRegularization(H, applied_regularization);
  }

  const int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_accel_);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_accel_);

  mpc::QPResult result;
  if (!solveQpWithRetry(H, f, lb, ub, x0, X_ref, context, applied_regularization, result)) {
    std::cout << "MPC QP 求解失败"
              << (is_maneuver_path ? " (机动自适应)" : "")
              << ", 回退到直瞄." << std::endl;
    return fallbackDirectAim(context, X_ref);
  }

  fillAndLogDiagnostics(
    is_maneuver_path, H, Q_eff, R_eff, S_eff, lb, ub, result, applied_regularization);

  // 提取控制量, 构建云台指令
  Eigen::Vector2d u_opt(result.U(0), result.U(1));
  if (use_rms_normalization) updateControlHistory(u_opt);
  return buildGimbalCmd(x0, u_opt, context, target_distance);
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
    const Eigen::VectorXd A_times_z = A_con * result_ext.U;
    result.active_bound_size = countActiveBounds(
      result_ext.U, lb_ext, ub_ext, diagnostics_active_tol_);
    result.active_linear_size = countActiveLinear(
      A_times_z, lbA, ubA, diagnostics_active_tol_);
    result.active_set_size = result.active_bound_size + result.active_linear_size;
  }
  return result;
}

rm_interfaces::msg::GimbalCmd MpcControlStrategy::fallbackDirectAim(
  const GimbalControlContext & context,
  const Eigen::VectorXd & X_ref)
{
  std::cout << "Falling back to direct aim with reference yaw=" << X_ref(0) << " rad, pitch=" << X_ref(1)
            << " rad." << std::endl;
  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);

  // QP 失败时回退: 直接用参考轨迹首步 yaw/pitch 作为目标
  double ref_yaw = X_ref(0);
  double ref_pitch = X_ref(1);

  double yaw_diff = angles::normalize_angle(ref_yaw - context.current_yaw);
  double pitch_diff = ref_pitch - context.current_pitch;

  double distance =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerDistance(target_robot);

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = ref_yaw * 180.0 / M_PI;
  cmd.pitch = ref_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.yaw_v = 0.0;
  cmd.pitch_v = 0.0;
  cmd.distance = std::max(distance, 0.0);

  return cmd;
}

}  // namespace gimbal_controller
