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

  x_ = motion_->initial_state(obs, current_panel_id_, r1, r2, dza);
  P_ = motion_->initial_covariance();

  auto idx = motion_->state_idx();
  double panel_angle = current_panel_id_ * (M_PI / 2.0);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);
  auto [k_val, delta_val] = decompose_yaw(center_yaw);
  k_ = k_val;
  last_k_ = k_;
  x_(idx.DELTA()) = delta_val;

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

  // ── Mean propagation (nominal dynamics) ──
  // R → R * Exp(e_z * β * dt)  ⟺  delta → delta + delta_rate * dt
  // p → p + v * dt
  // v → v
  // beta → beta
  // theta → theta
  Eigen::VectorXd x_pred = x_;
  x_pred(idx.X()) += x_(idx.VX()) * dt;
  x_pred(idx.Y()) += x_(idx.VY()) * dt;
  x_pred(idx.Z()) += x_(idx.VZ()) * dt;
  x_pred(idx.DELTA()) += x_(idx.DELTA_RATE()) * dt;
  // velocity, yaw-rate, structure: unchanged (CV model)

  // ── Error-state transition matrix F (n × n) ──
  // Build as identity + dt contributions
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
  F(idx.X(), idx.VX()) = dt;
  F(idx.Y(), idx.VY()) = dt;
  F(idx.Z(), idx.VZ()) = dt;
  F(idx.DELTA(), idx.DELTA_RATE()) = dt;

  // Optional CA states: AX/AY/AZ decay
  if (idx.has("AX")) {
    F(idx.AX(), idx.AX()) = 0.0;  // no acceleration integration in CV
  }
  if (idx.has("AY")) {
    F(idx.AY(), idx.AY()) = 0.0;
  }
  if (idx.has("AZ")) {
    F(idx.AZ(), idx.AZ()) = 0.0;
  }

  // ── Covariance propagation ──
  Eigen::MatrixXd Q = motion_->build_Q(dt);
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
  return ctx;
}

Eigen::Vector4d InvariantPoseBackend::obs_model_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  auto idx = motion_->state_idx();
  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double z_mean = x(idx.Z());
  double delta = x(idx.DELTA());
  double d_za = x(idx.DZA());

  double radius = (panel_id % 2 == 0) ? x(idx.R1()) : x(idx.R2());
  double center_yaw = compose_yaw(k, delta);
  double panel_angle = panel_id * (M_PI / 2.0);
  double armor_yaw = normalize_angle(center_yaw + panel_angle);

  double x_obs = x_c + radius * std::cos(armor_yaw);
  double y_obs = y_c + radius * std::sin(armor_yaw);
  double z_offset = (panel_id % 2 == 0) ? -d_za : d_za;
  double z_obs = z_mean + z_offset;

  Eigen::Vector4d z;
  z << x_obs, y_obs, z_obs, center_yaw;
  return z;
}

Eigen::MatrixXd InvariantPoseBackend::obs_jacobian_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  int n = motion_->state_dim();
  auto idx = motion_->state_idx();
  const int p = ((panel_id % 4) + 4) % 4;

  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double delta = x(idx.DELTA());
  double d_za = x(idx.DZA());
  double radius = (p % 2 == 0) ? x(idx.R1()) : x(idx.R2());
  double center_yaw = compose_yaw(k, delta);
  double panel_angle = p * (M_PI / 2.0);
  double armor_yaw = normalize_angle(center_yaw + panel_angle);

  double cos_a = std::cos(armor_yaw);
  double sin_a = std::sin(armor_yaw);

  // H: 4 rows × n cols, initialized to zero
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, n);

  // Row 0: ∂x_obs/∂state
  H(0, idx.X()) = 1.0;
  H(0, idx.Y()) = 0.0;
  H(0, idx.Z()) = 0.0;
  H(0, idx.DELTA()) = -radius * sin_a;
  if (p % 2 == 0) {
    H(0, idx.R1()) = cos_a;
  } else {
    H(0, idx.R2()) = cos_a;
  }

  // Row 1: ∂y_obs/∂state
  H(1, idx.X()) = 0.0;
  H(1, idx.Y()) = 1.0;
  H(1, idx.Z()) = 0.0;
  H(1, idx.DELTA()) = radius * cos_a;
  if (p % 2 == 0) {
    H(1, idx.R1()) = sin_a;
  } else {
    H(1, idx.R2()) = sin_a;
  }

  // Row 2: ∂z_obs/∂state
  H(2, idx.Z()) = 1.0;
  H(2, idx.DZA()) = (p % 2 == 0) ? -1.0 : 1.0;

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
  innov(3) = normalize_angle(innov(3));

  Eigen::Matrix4d R = noise_->build_R(UpdateKind::Single);

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
  innov(3) = normalize_angle(innov(3));
  innov(7) = normalize_angle(innov(7));

  Eigen::Matrix<double, 8, 8> R = noise_->build_R(UpdateKind::Dual);
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

  double center_yaw_obs = normalize_angle(obs.yaw - p * (M_PI / 2.0));
  Eigen::Vector4d innov = eval.innovation;
  Eigen::Matrix4d R = noise_->build_R(UpdateKind::Single);
  Eigen::Matrix4d S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (n × 4)
  Eigen::MatrixXd K = ctx.P_prior * H.transpose() * S.inverse();

  // Structural slow gain
  const auto &su = ukf_config_.single_update;
  K.row(idx.R1()) *= su.structural_gain_r;
  K.row(idx.R2()) *= su.structural_gain_r;
  K.row(idx.DZA()) *= su.structural_gain_dza;

  // Error-state correction
  Eigen::VectorXd dx = K * innov;

  // Retract: x⁺ = x + dx (Euclidean for all states in our layout)
  // For yaw: handled via wrap in the innovation, retraction is additive on delta
  Eigen::VectorXd x_post = ctx.x_prior + dx;

  // Joseph form covariance update
  Eigen::MatrixXd I_KH =
      Eigen::MatrixXd::Identity(n, n) - K * H;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() + K * R * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  // k selection
  double post_delta = x_post(idx.DELTA());
  int best_k =
      select_best_k_from_center_yaw(post_delta, center_yaw_obs, ctx.k_prior);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = best_k;
  trial.last_k_post = ctx.k_prior;

  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, best_k, obs, p);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  // Handle mode switch (delta > π/2 or < -π/2)
  if (post_delta > M_PI / 2.0) {
    trial.x_post(idx.DELTA()) = post_delta - M_PI;
    trial.k_post = 1 - best_k;
  } else if (post_delta < -M_PI / 2.0) {
    trial.x_post(idx.DELTA()) = post_delta + M_PI;
    trial.k_post = 1 - best_k;
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
  Eigen::Matrix<double, 8, 8> R = noise_->build_R(UpdateKind::Dual);
  Eigen::Matrix<double, 8, 8> S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (n × 8)
  Eigen::MatrixXd K = ctx.P_prior * H.transpose() * S.inverse();

  // Structural slow gain (dual: more permissive)
  const auto &du = ukf_config_.dual_update;
  K.row(idx.R1()) *= du.structural_gain_r;
  K.row(idx.R2()) *= du.structural_gain_r;
  K.row(idx.DZA()) *= du.structural_gain_dza;

  Eigen::VectorXd dx = K * innov;
  Eigen::VectorXd x_post = ctx.x_prior + dx;

  // Joseph form
  Eigen::MatrixXd I_KH =
      Eigen::MatrixXd::Identity(n, n) - K * H;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() + K * R * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  double cy0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double cy1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  double avg_cy = normalize_angle(0.5 * (cy0 + cy1));
  double post_delta = x_post(idx.DELTA());
  int best_k =
      select_best_k_from_center_yaw(post_delta, avg_cy, ctx.k_prior);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = best_k;
  trial.last_k_post = ctx.k_prior;

  double recon0 =
      compute_reconstruction_error(x_post, best_k, obs0, p0);
  double recon1 =
      compute_reconstruction_error(x_post, best_k, obs1, p1);
  trial.reconstruction_pos_error = std::max(recon0, recon1);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  if (post_delta > M_PI / 2.0) {
    trial.x_post(idx.DELTA()) = post_delta - M_PI;
    trial.k_post = 1 - best_k;
  } else if (post_delta < -M_PI / 2.0) {
    trial.x_post(idx.DELTA()) = post_delta + M_PI;
    trial.k_post = 1 - best_k;
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
  return compose_yaw(k_, x_(motion_->state_idx().DELTA()));
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
      delta_angle_diff(x_post(idx.DELTA()), x_prior(idx.DELTA())));
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

void InvariantPoseBackend::apply_state_constraints() {
  auto idx = motion_->state_idx();
  x_ = fyt::auto_aim::apply_state_constraints(
      x_, idx.R1(), idx.R2(), idx.DZA(), config_.constraints.min_radius,
      config_.constraints.max_radius, 0.0, config_.constraints.max_dz);
}

}  // namespace fyt::auto_aim::norm4_v3
