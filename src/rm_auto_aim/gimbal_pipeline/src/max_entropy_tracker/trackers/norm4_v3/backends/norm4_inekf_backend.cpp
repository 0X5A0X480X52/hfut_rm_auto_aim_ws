// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_inekf_backend.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim::norm4_v3 {

InvariantPoseBackend::InvariantPoseBackend(
    std::unique_ptr<IMotionModelBundle> motion,
    std::unique_ptr<IMeasurementNoiseModel> noise,
    std::unique_ptr<IStructureProvider> structure,
    const Norm4V3UkfConfig &ukf_config,
    const UnifiedConfig &config, double dt)
    : config_(config),
      ukf_config_(ukf_config),
      dt_(dt),
      motion_(std::move(motion)),
      noise_(std::move(noise)),
      structure_(std::move(structure)) {
  int n = motion_->state_dim();
  x_ = Eigen::VectorXd::Zero(n);
  P_ = Eigen::MatrixXd::Identity(n, n) * 100.0;
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

void InvariantPoseBackend::reset(const ObservationData &obs, int panel_id,
                                  double r1, double r2, double dza) {
  current_panel_id_ = ((panel_id % 4) + 4) % 4;
  phase_index_ = current_panel_id_;

  x_ = initialize_invariant_state(obs, current_panel_id_, r1, r2, dza);
  P_ = motion_->initial_covariance();

  auto idx = motion_->state_idx();
  const auto pp = get_panel_profile(current_panel_id_);
  double panel_angle = pp.phase_offset;
  double center_yaw = normalize_angle(obs.yaw - panel_angle);
  k_ = phase_index_;
  last_k_ = phase_index_;
  x_(idx.DELTA()) = center_yaw;

  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;

  initialized_ = true;
}

void InvariantPoseBackend::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  int n = motion_->state_dim();
  auto idx = motion_->state_idx();

  // ── Mean propagation on SE(2.5)-style nominal state ──
  // R_{k+1} = R_k * Exp(e_z * beta * dt)  <=> yaw += yaw_rate * dt
  // p_{k+1} = p_k + v_k * dt (+ 0.5 * a * dt^2 for CA/Singer-style layout)
  // v → v
  // beta → beta
  // theta → theta
  Eigen::VectorXd x_pred = x_;
  const bool ca_xyz = idx.has("AX") && idx.has("AY") && idx.has("AZ");
  if (ca_xyz) {
    x_pred(idx.X()) += x_(idx.VX()) * dt + 0.5 * x_(idx.AX()) * dt * dt;
    x_pred(idx.Y()) += x_(idx.VY()) * dt + 0.5 * x_(idx.AY()) * dt * dt;
    x_pred(idx.Z()) += x_(idx.VZ()) * dt + 0.5 * x_(idx.AZ()) * dt * dt;
    x_pred(idx.VX()) += x_(idx.AX()) * dt;
    x_pred(idx.VY()) += x_(idx.AY()) * dt;
    x_pred(idx.VZ()) += x_(idx.AZ()) * dt;
  } else {
    x_pred(idx.X()) += x_(idx.VX()) * dt;
    x_pred(idx.Y()) += x_(idx.VY()) * dt;
    x_pred(idx.Z()) += x_(idx.VZ()) * dt;
  }

  if (idx.has("DELTA_ACC")) {
    x_pred(idx.DELTA()) = normalize_angle(
        x_(idx.DELTA()) + x_(idx.DELTA_RATE()) * dt +
        0.5 * x_(idx.get("DELTA_ACC")) * dt * dt);
    x_pred(idx.DELTA_RATE()) += x_(idx.get("DELTA_ACC")) * dt;
  } else {
    x_pred(idx.DELTA()) =
        normalize_angle(x_(idx.DELTA()) + x_(idx.DELTA_RATE()) * dt);
  }
  // velocity, yaw-rate, structure: unchanged (CV model)

  // ── Error-state transition matrix F (n × n) ──
  // Build as identity + dt contributions
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
  F(idx.X(), idx.VX()) = dt;
  F(idx.Y(), idx.VY()) = dt;
  F(idx.Z(), idx.VZ()) = dt;
  F(idx.DELTA(), idx.DELTA_RATE()) = dt;

  if (ca_xyz) {
    F(idx.X(), idx.AX()) = 0.5 * dt * dt;
    F(idx.Y(), idx.AY()) = 0.5 * dt * dt;
    F(idx.Z(), idx.AZ()) = 0.5 * dt * dt;
    F(idx.VX(), idx.AX()) = dt;
    F(idx.VY(), idx.AY()) = dt;
    F(idx.VZ(), idx.AZ()) = dt;
  }

  if (idx.has("DELTA_ACC")) {
    const int dacc = idx.get("DELTA_ACC");
    F(idx.DELTA(), dacc) = 0.5 * dt * dt;
    F(idx.DELTA_RATE(), dacc) = dt;
  }

  // ── Covariance propagation ──
  Eigen::MatrixXd Q = build_invariant_Q(dt);
  P_ = F * P_ * F.transpose() + Q;

  x_ = x_pred;
  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

PredictContext InvariantPoseBackend::buildPredictContext() const {
  PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  ctx.hybrid_prior.panel_id = current_panel_id_;
  ctx.hybrid_prior.phase_index = phase_index_;
  return ctx;
}

Eigen::Vector4d InvariantPoseBackend::obs_model_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  (void)k;
  auto idx = motion_->state_idx();
  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double z_mean = x(idx.Z());
  double d_za = x(idx.DZA());

  const auto pp = get_panel_profile(panel_id);
  double radius = pp.use_r2 ? x(idx.R2()) : x(idx.R1());
  double center_yaw = normalize_angle(x(idx.DELTA()));
  // Group action viewpoint:
  // p_armor = p_center + R(center_yaw) * Exp(panel_phase) * [radius, 0]^T.
  const double armor_yaw = normalize_angle(center_yaw + pp.phase_offset);
  const Eigen::Vector2d radial_vec(radius * std::cos(armor_yaw),
                                   radius * std::sin(armor_yaw));
  double x_obs = x_c + radial_vec.x();
  double y_obs = y_c + radial_vec.y();
  double z_offset = pp.z_sign * d_za;
  double z_obs = z_mean + z_offset;

  Eigen::Vector4d z;
  z << x_obs, y_obs, z_obs, center_yaw;
  return z;
}

Eigen::MatrixXd InvariantPoseBackend::obs_jacobian_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  (void)k;
  int n = motion_->state_dim();
  auto idx = motion_->state_idx();
  const int p = ((panel_id % 4) + 4) % 4;

  const auto pp = get_panel_profile(p);
  double radius = pp.use_r2 ? x(idx.R2()) : x(idx.R1());
  double center_yaw = normalize_angle(x(idx.DELTA()));
  double armor_yaw = normalize_angle(center_yaw + pp.phase_offset);

  double cos_a = std::cos(armor_yaw);
  double sin_a = std::sin(armor_yaw);

  // H: 4 rows × n cols, initialized to zero
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, n);

  // Row 0: ∂x_obs/∂state
  H(0, idx.X()) = 1.0;
  H(0, idx.Y()) = 0.0;
  H(0, idx.Z()) = 0.0;
  H(0, idx.DELTA()) = -radius * sin_a;
  if (!pp.use_r2) {
    H(0, idx.R1()) = cos_a;
  } else {
    H(0, idx.R2()) = cos_a;
  }

  // Row 1: ∂y_obs/∂state
  H(1, idx.X()) = 0.0;
  H(1, idx.Y()) = 1.0;
  H(1, idx.Z()) = 0.0;
  H(1, idx.DELTA()) = radius * cos_a;
  if (!pp.use_r2) {
    H(1, idx.R1()) = sin_a;
  } else {
    H(1, idx.R2()) = sin_a;
  }

  // Row 2: ∂z_obs/∂state
  H(2, idx.Z()) = 1.0;
  H(2, idx.DZA()) = pp.z_sign;

  // Row 3: ∂yaw_obs/∂state
  H(3, idx.DELTA()) = 1.0;

  return H;
}

MeasurementEval InvariantPoseBackend::evaluateSingle(
    const PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  const int p = ((panel_id % 4) + 4) % 4;

  double center_yaw_obs = normalize_angle(obs.yaw - p * (M_PI / 2.0));
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  Eigen::Vector4d z_pred =
      obs_model_single(ctx.x_prior, ctx.k_prior, p);
  Eigen::MatrixXd H =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p);

  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));

  Eigen::Matrix4d R = noise_->build_single_R(obs);

  Eigen::Matrix4d S = H * ctx.P_prior * H.transpose() + R;

  MeasurementEval eval;
  Eigen::LLT<Eigen::Matrix4d> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }
  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 4; ++i) logdet += std::log(L(i, i));
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 4.0 * std::log(2.0 * M_PI));

  double chi2_yaw = (innov(3) * innov(3)) / S(3, 3);
  Eigen::Vector3d innov_pos = innov.head<3>();
  Eigen::Matrix3d S_pos = S.topLeftCorner<3, 3>();
  double chi2_pos =
      innov_pos.transpose() * S_pos.inverse() * innov_pos;

  const auto &gt = ukf_config_.gate;
  bool gate_pass = true;
  if (nis > gt.single_total_nis) gate_pass = false;
  if (chi2_pos > gt.single_pos_chi2) gate_pass = false;
  if (chi2_yaw > gt.single_yaw_chi2) gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = chi2_yaw;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos
        << ",chi2_yaw=" << chi2_yaw;
    eval.reject_reason = oss.str();
  }
  return eval;
}

MeasurementEval InvariantPoseBackend::evaluateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double cy0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double cy1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, cy0, obs1.x, obs1.y, obs1.z, cy1;

  Eigen::Vector4d zp0 =
      obs_model_single(ctx.x_prior, ctx.k_prior, p0);
  Eigen::Vector4d zp1 =
      obs_model_single(ctx.x_prior, ctx.k_prior, p1);
  Eigen::Matrix<double, 8, 1> z_pred;
  z_pred << zp0, zp1;

  Eigen::MatrixXd H0 =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p0);
  Eigen::MatrixXd H1 =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p1);
  int n = motion_->state_dim();
  Eigen::MatrixXd H(8, n);
  H << H0, H1;

  Eigen::Matrix<double, 8, 1> innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));
  innov(7) = angle_difference(z_obs(7), z_pred(7));

  Eigen::Matrix<double, 8, 8> R = noise_->build_dual_R(obs0, obs1);
  Eigen::Matrix<double, 8, 8> S = H * ctx.P_prior * H.transpose() + R;

  MeasurementEval eval;
  Eigen::LLT<Eigen::Matrix<double, 8, 8>> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }
  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 8; ++i) logdet += std::log(L(i, i));
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 8.0 * std::log(2.0 * M_PI));

  double chi2_yaw0 = (innov(3) * innov(3)) / S(3, 3);
  double chi2_yaw1 = (innov(7) * innov(7)) / S(7, 7);
  double chi2_yaw = std::max(chi2_yaw0, chi2_yaw1);

  Eigen::Vector3d ip0 = innov.segment<3>(0);
  Eigen::Vector3d ip1 = innov.segment<3>(4);
  Eigen::Matrix3d Sp0 = S.block<3, 3>(0, 0);
  Eigen::Matrix3d Sp1 = S.block<3, 3>(4, 4);
  double chi2_pos0 = ip0.transpose() * Sp0.inverse() * ip0;
  double chi2_pos1 = ip1.transpose() * Sp1.inverse() * ip1;
  double chi2_pos = std::max(chi2_pos0, chi2_pos1);

  const auto &gt = ukf_config_.gate;
  bool gate_pass = true;
  if (nis > gt.dual_total_nis) gate_pass = false;
  if (chi2_pos0 > gt.dual_each_pos_chi2 ||
      chi2_pos1 > gt.dual_each_pos_chi2)
    gate_pass = false;
  if (chi2_yaw0 > gt.dual_each_yaw_chi2 ||
      chi2_yaw1 > gt.dual_each_yaw_chi2)
    gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = chi2_yaw;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos
        << ",chi2_yaw=" << chi2_yaw;
    eval.reject_reason = oss.str();
  }
  return eval;
}

UkfTrial InvariantPoseBackend::tryUpdateSingle(
    const PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  UkfTrial trial;
  const int p = ((panel_id % 4) + 4) % 4;

  trial.hypothesis.kind = HypothesisKind::Single;
  trial.hypothesis.assignments[0] = {0, p};
  trial.hypothesis.assignment_count = 1;

  MeasurementEval eval = evaluateSingle(ctx, obs, p);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  int n = motion_->state_dim();
  auto idx = motion_->state_idx();

  Eigen::MatrixXd H =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p);

  Eigen::Vector4d innov = eval.innovation;
  Eigen::Matrix4d R = noise_->build_single_R(obs);
  Eigen::Matrix4d S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (n × 4)
  Eigen::MatrixXd K = ctx.P_prior * H.transpose() * S.inverse();

  // Structural slow gain
  const auto &su = ukf_config_.single_update;
  K.row(idx.R1()) *= su.structural_gain_r;
  K.row(idx.R2()) *= su.structural_gain_r;
  K.row(idx.DZA()) *= su.structural_gain_dza;

  // Error-state correction on algebra, then retract to nominal SE(2.5)-style state.
  Eigen::VectorXd dx = K * innov;
  Eigen::VectorXd x_post = retract_se25_state(ctx.x_prior, dx);

  // Joseph form covariance update
  Eigen::MatrixXd I_KH =
      Eigen::MatrixXd::Identity(n, n) - K * H;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() + K * R * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = p;
  trial.last_k_post = ctx.k_prior;
  trial.hybrid_post.panel_id = p;
  trial.hybrid_post.phase_index = p;

  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, trial.k_post, obs, p);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  // Clamp structure params
  trial.x_post(idx.R1()) =
      std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) =
      std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) =
      std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

UkfTrial InvariantPoseBackend::tryUpdateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  UkfTrial trial;
  trial.hypothesis.kind = HypothesisKind::Dual;
  trial.hypothesis.assignments[0] = {0, panel_id_0};
  trial.hypothesis.assignments[1] = {1, panel_id_1};
  trial.hypothesis.assignment_count = 2;

  MeasurementEval eval =
      evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;
  int n = motion_->state_dim();
  auto idx = motion_->state_idx();

  Eigen::MatrixXd H0 =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p0);
  Eigen::MatrixXd H1 =
      obs_jacobian_single(ctx.x_prior, ctx.k_prior, p1);
  Eigen::MatrixXd H(8, n);
  H << H0, H1;

  Eigen::Matrix<double, 8, 1> innov = eval.innovation;
  Eigen::Matrix<double, 8, 8> R = noise_->build_dual_R(obs0, obs1);
  Eigen::Matrix<double, 8, 8> S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (n × 8)
  Eigen::MatrixXd K = ctx.P_prior * H.transpose() * S.inverse();

  // Structural slow gain (dual: more permissive)
  const auto &du = ukf_config_.dual_update;
  K.row(idx.R1()) *= du.structural_gain_r;
  K.row(idx.R2()) *= du.structural_gain_r;
  K.row(idx.DZA()) *= du.structural_gain_dza;

  Eigen::VectorXd dx = K * innov;
  Eigen::VectorXd x_post = retract_se25_state(ctx.x_prior, dx);

  // Joseph form
  Eigen::MatrixXd I_KH =
      Eigen::MatrixXd::Identity(n, n) - K * H;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() + K * R * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = p0;
  trial.last_k_post = ctx.k_prior;
  trial.hybrid_post.panel_id = p0;
  trial.hybrid_post.phase_index = p0;

  double recon0 =
      compute_reconstruction_error(x_post, trial.k_post, obs0, p0);
  double recon1 =
      compute_reconstruction_error(x_post, trial.k_post, obs1, p1);
  trial.reconstruction_pos_error = std::max(recon0, recon1);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  trial.x_post(idx.R1()) =
      std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) =
      std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) =
      std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

void InvariantPoseBackend::commit(const UkfTrial &trial) {
  if (!trial.success) return;

  x_ = trial.x_post;
  P_ = trial.P_post;
  k_ = trial.k_post;
  last_k_ = trial.last_k_post;
  phase_index_ = trial.hybrid_post.phase_index;

  if (trial.hypothesis.kind == HypothesisKind::Single) {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  } else {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  }

  last_nis_ = trial.eval.nis;
  if (trial.eval.innovation.size() >= 3) {
    last_innov_xyz_ = trial.eval.innovation.head<3>();
  }
  if (trial.eval.innovation.size() >= 4) {
    last_innov_yaw_ = trial.eval.innovation(3);
  }
  last_update_type_ =
      (trial.hypothesis.kind == HypothesisKind::Single) ? 1 : 2;

  // Update slow structure estimator with is_dual flag
  if (structure_) {
    bool is_dual = (trial.hypothesis.kind == HypothesisKind::Dual);
    structure_->update(x_, P_, k_, dt_, is_dual);
  }

  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

BackendSnapshot InvariantPoseBackend::snapshot() const {
  BackendSnapshot snap;
  snap.x = x_;
  snap.P = P_;
  snap.k = k_;
  snap.last_k = last_k_;
  snap.current_panel_id = current_panel_id_;
  snap.hybrid.panel_id = current_panel_id_;
  snap.hybrid.phase_index = phase_index_;
  snap.last_nis = last_nis_;
  snap.last_innov_xyz = last_innov_xyz_;
  snap.last_innov_yaw = last_innov_yaw_;
  snap.last_update_type = last_update_type_;
  return snap;
}

Eigen::Vector3d InvariantPoseBackend::get_center_position() const {
  auto idx = motion_->state_idx();
  return Eigen::Vector3d(x_(idx.X()), x_(idx.Y()), x_(idx.Z()));
}

std::pair<double, double> InvariantPoseBackend::get_radii() const {
  // Read from structure provider if available, fallback to state
  if (structure_ && structure_->converged()) {
    Eigen::Vector3d s = structure_->get_structure();
    return {s(0), s(1)};
  }
  auto idx = motion_->state_idx();
  return {x_(idx.R1()), x_(idx.R2())};
}

double InvariantPoseBackend::get_dza() const {
  if (structure_ && structure_->converged()) {
    return structure_->get_structure()(2);
  }
  return x_(motion_->state_idx().DZA());
}

double InvariantPoseBackend::get_yaw() const {
  return normalize_angle(x_(motion_->state_idx().DELTA()));
}

double InvariantPoseBackend::get_delta() const {
  return x_(motion_->state_idx().DELTA());
}

bool InvariantPoseBackend::check_posterior_sanity(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &x_post,
    const Eigen::MatrixXd &P_post) const {
  auto idx = motion_->state_idx();
  const auto &ps = ukf_config_.posterior_sanity;

  Eigen::Vector3d prior_center(x_prior(idx.X()), x_prior(idx.Y()),
                                x_prior(idx.Z()));
  Eigen::Vector3d post_center(x_post(idx.X()), x_post(idx.Y()),
                               x_post(idx.Z()));
  if ((post_center - prior_center).norm() > ps.max_center_jump) return false;

  double delta_jump = std::abs(
      angle_difference(x_post(idx.DELTA()), x_prior(idx.DELTA())));
  if (delta_jump > ps.max_yaw_jump) return false;

  double r1 = x_post(idx.R1()), r2 = x_post(idx.R2());
  if (r1 < ps.min_r || r1 > ps.max_r || r2 < ps.min_r || r2 > ps.max_r)
    return false;
  if (std::abs(r1 - x_prior(idx.R1())) > ps.max_r_jump) return false;
  if (std::abs(r2 - x_prior(idx.R2())) > ps.max_r_jump) return false;

  double dza = x_post(idx.DZA());
  if (dza < ps.min_dza || dza > ps.max_dza) return false;
  if (std::abs(dza - x_prior(idx.DZA())) > ps.max_dza_jump) return false;

  if (!P_post.allFinite()) return false;
  return true;
}

double InvariantPoseBackend::compute_reconstruction_error(
    const Eigen::VectorXd &x_post, int k, const ObservationData &obs,
    int panel_id) const {
  Eigen::Vector4d z_rebuild = obs_model_single(x_post, k, panel_id);
  Eigen::Vector3d pos_rebuild = z_rebuild.head<3>();
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - pos_rebuild).norm();
}

Eigen::VectorXd InvariantPoseBackend::retract_se25_state(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &dx) const {
  const auto idx = motion_->state_idx();
  Eigen::VectorXd x_post = x_prior;

  // Group part: (R_yaw, p_xy, z) uses Lie retraction; in yaw-only form this is
  // equivalent to additive update + angle wrap on yaw.
  x_post(idx.X()) += dx(idx.X());
  x_post(idx.Y()) += dx(idx.Y());
  x_post(idx.Z()) += dx(idx.Z());
  x_post(idx.DELTA()) = normalize_angle(x_prior(idx.DELTA()) + dx(idx.DELTA()));

  // Algebra / Euclidean part.
  if (idx.has("VX")) x_post(idx.VX()) += dx(idx.VX());
  if (idx.has("VY")) x_post(idx.VY()) += dx(idx.VY());
  if (idx.has("VZ")) x_post(idx.VZ()) += dx(idx.VZ());
  if (idx.has("DELTA_RATE")) x_post(idx.DELTA_RATE()) += dx(idx.DELTA_RATE());
  if (idx.has("AX")) x_post(idx.AX()) += dx(idx.AX());
  if (idx.has("AY")) x_post(idx.AY()) += dx(idx.AY());
  if (idx.has("AZ")) x_post(idx.AZ()) += dx(idx.AZ());
  if (idx.has("DELTA_ACC")) x_post(idx.get("DELTA_ACC")) += dx(idx.get("DELTA_ACC"));

  // Slow structural state (theta) remains Euclidean.
  x_post(idx.R1()) += dx(idx.R1());
  x_post(idx.R2()) += dx(idx.R2());
  x_post(idx.DZA()) += dx(idx.DZA());
  return x_post;
}

Eigen::VectorXd InvariantPoseBackend::initialize_invariant_state(
    const ObservationData &obs, int panel_id, double r1, double r2,
    double dza) const {
  const auto idx = motion_->state_idx();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(motion_->state_dim());
  const auto pp = get_panel_profile(panel_id);
  const double use_r = pp.use_r2 ? r2 : r1;
  const double center_yaw = normalize_angle(obs.yaw - pp.phase_offset);

  x0(idx.X()) = obs.x - use_r * std::cos(obs.yaw);
  x0(idx.Y()) = obs.y - use_r * std::sin(obs.yaw);
  x0(idx.Z()) = obs.z - pp.z_sign * dza;
  x0(idx.DELTA()) = center_yaw;
  x0(idx.R1()) = r1;
  x0(idx.R2()) = r2;
  x0(idx.DZA()) = dza;

  if (idx.has("VX")) x0(idx.VX()) = 0.0;
  if (idx.has("VY")) x0(idx.VY()) = 0.0;
  if (idx.has("VZ")) x0(idx.VZ()) = 0.0;
  if (idx.has("DELTA_RATE")) x0(idx.DELTA_RATE()) = 0.0;
  if (idx.has("AX")) x0(idx.AX()) = 0.0;
  if (idx.has("AY")) x0(idx.AY()) = 0.0;
  if (idx.has("AZ")) x0(idx.AZ()) = 0.0;
  if (idx.has("DELTA_ACC")) x0(idx.get("DELTA_ACC")) = 0.0;
  return x0;
}

Eigen::MatrixXd InvariantPoseBackend::build_invariant_Q(double dt) const {
  const auto idx = motion_->state_idx();
  const int n = motion_->state_dim();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
  const double dt2 = dt * dt;

  const bool ca_xyz = idx.has("AX") && idx.has("AY") && idx.has("AZ");
  const bool ca_yaw = idx.has("DELTA_ACC");
  const double q_v_cv = config_.motion.cv_process_noise_vel *
                        config_.motion.cv_process_noise_vel;
  const double q_v_ca = config_.motion.ca_process_noise_acc *
                        config_.motion.ca_process_noise_acc;
  const double q_w_cv = config_.spin.spin_process_noise_delta_rate *
                        config_.spin.spin_process_noise_delta_rate;
  const double q_w_ca = config_.spin.spin_process_noise_delta_acc *
                        config_.spin.spin_process_noise_delta_acc;
  const double q_r = config_.motion.process_noise_r *
                     config_.motion.process_noise_r;
  const double q_dza = config_.motion.process_noise_dz *
                       config_.motion.process_noise_dz;

  if (ca_xyz) {
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;
    auto fill_ca_block = [&](int p, int v, int a) {
      Q(p, p) = q_v_ca * dt5 / 20.0;
      Q(p, v) = q_v_ca * dt4 / 8.0;
      Q(v, p) = Q(p, v);
      Q(p, a) = q_v_ca * dt3 / 6.0;
      Q(a, p) = Q(p, a);
      Q(v, v) = q_v_ca * dt3 / 3.0;
      Q(v, a) = q_v_ca * dt2 / 2.0;
      Q(a, v) = Q(v, a);
      Q(a, a) = q_v_ca * dt;
    };
    fill_ca_block(idx.X(), idx.VX(), idx.AX());
    fill_ca_block(idx.Y(), idx.VY(), idx.AY());
    fill_ca_block(idx.Z(), idx.VZ(), idx.AZ());
  } else {
    Q(idx.X(), idx.X()) = std::max(1e-8, q_v_cv * dt2);
    Q(idx.Y(), idx.Y()) = std::max(1e-8, q_v_cv * dt2);
    Q(idx.Z(), idx.Z()) = std::max(1e-8, q_v_cv * dt2);
    Q(idx.VX(), idx.VX()) = std::max(1e-8, q_v_cv * dt);
    Q(idx.VY(), idx.VY()) = std::max(1e-8, q_v_cv * dt);
    Q(idx.VZ(), idx.VZ()) = std::max(1e-8, q_v_cv * dt);
  }

  if (ca_yaw) {
    const int dacc = idx.get("DELTA_ACC");
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;
    Q(idx.DELTA(), idx.DELTA()) = q_w_ca * dt5 / 20.0;
    Q(idx.DELTA(), idx.DELTA_RATE()) = q_w_ca * dt4 / 8.0;
    Q(idx.DELTA_RATE(), idx.DELTA()) = Q(idx.DELTA(), idx.DELTA_RATE());
    Q(idx.DELTA(), dacc) = q_w_ca * dt3 / 6.0;
    Q(dacc, idx.DELTA()) = Q(idx.DELTA(), dacc);
    Q(idx.DELTA_RATE(), idx.DELTA_RATE()) = q_w_ca * dt3 / 3.0;
    Q(idx.DELTA_RATE(), dacc) = q_w_ca * dt2 / 2.0;
    Q(dacc, idx.DELTA_RATE()) = Q(idx.DELTA_RATE(), dacc);
    Q(dacc, dacc) = q_w_ca * dt;
  } else {
    Q(idx.DELTA(), idx.DELTA()) = std::max(1e-8, q_w_cv * dt2);
    Q(idx.DELTA_RATE(), idx.DELTA_RATE()) = std::max(1e-8, q_w_cv * dt);
  }
  Q(idx.R1(), idx.R1()) = std::max(1e-10, q_r * dt);
  Q(idx.R2(), idx.R2()) = std::max(1e-10, q_r * dt);
  Q(idx.DZA(), idx.DZA()) = std::max(1e-10, q_dza * dt);

  return Q;
}

void InvariantPoseBackend::apply_state_constraints() {
  auto idx = motion_->state_idx();
  x_ = fyt::auto_aim::apply_state_constraints(
      x_, idx.R1(), idx.R2(), idx.DZA(), config_.constraints.min_radius,
      config_.constraints.max_radius, 0.0, config_.constraints.max_dz);
  x_(idx.DELTA()) = normalize_angle(x_(idx.DELTA()));
}

}  // namespace fyt::auto_aim::norm4_v3
