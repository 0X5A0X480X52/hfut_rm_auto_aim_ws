// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v3/outpost_inekf_backend.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim::outpost_v3 {

using Idx = OutpostStateIndex;

// ═══════════════════════════════════════════════════════════════════
// Static state layout
// ═══════════════════════════════════════════════════════════════════

StateLayout OutpostInEKFBackend::s_layout_{};
bool OutpostInEKFBackend::s_layout_initialized_ = false;

StateLayout OutpostStateIndex::build_layout() {
  StateLayout layout;
  layout.register_state("X", X);
  layout.register_state("Y", Y);
  layout.register_state("Z", Z);
  layout.register_state("VX", VX);
  layout.register_state("VY", VY);
  layout.register_state("VZ", VZ);
  layout.register_state("AX", AX);
  layout.register_state("AY", AY);
  layout.register_state("AZ", AZ);
  layout.register_state("DELTA", YAW);
  layout.register_state("DELTA_RATE", YAW_RATE);
  layout.register_state("DELTA_ACC", YAW_ACC);
  // Provide R1/R2/DZA stubs so DynamicStateIndex accessors don't throw
  layout.register_state("R1", kDim);
  layout.register_state("R2", kDim);
  layout.register_state("DZA", kDim);
  layout.freeze();
  return layout;
}

DynamicStateIndex OutpostStateIndex::make_idx() {
  static StateLayout layout = build_layout();
  return DynamicStateIndex(layout);
}

// ═══════════════════════════════════════════════════════════════════
// Lie group operations (SO(2))
// ═══════════════════════════════════════════════════════════════════

double OutpostInEKFBackend::left_jacobian_SO2(double dpsi) {
  if (std::abs(dpsi) < 1e-8) return 1.0;
  return std::sin(dpsi) / dpsi;
}

double OutpostInEKFBackend::right_jacobian_SO2(double dpsi) {
  return left_jacobian_SO2(dpsi);  // SO(2) is abelian
}

// ═══════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════

OutpostInEKFBackend::OutpostInEKFBackend(const OutpostV3Config &cfg,
                                         double dt)
    : cfg_(cfg), dt_(dt), geom_(cfg.geometry), state_idx_(s_layout_) {
  if (!s_layout_initialized_) {
    s_layout_ = OutpostStateIndex::build_layout();
    s_layout_initialized_ = true;
  }
  x_ = Eigen::VectorXd::Zero(Idx::kDim);
  const auto &ip = cfg_.initial_P;
  P_ = Eigen::MatrixXd::Identity(Idx::kDim, Idx::kDim);
  P_(Idx::X, Idx::X) = ip.pos;
  P_(Idx::Y, Idx::Y) = ip.pos;
  P_(Idx::Z, Idx::Z) = ip.pos;
  P_(Idx::VX, Idx::VX) = ip.vel;
  P_(Idx::VY, Idx::VY) = ip.vel;
  P_(Idx::VZ, Idx::VZ) = ip.vel;
  P_(Idx::AX, Idx::AX) = ip.acc;
  P_(Idx::AY, Idx::AY) = ip.acc;
  P_(Idx::AZ, Idx::AZ) = ip.acc;
  P_(Idx::YAW, Idx::YAW) = ip.yaw;
  P_(Idx::YAW_RATE, Idx::YAW_RATE) = ip.yaw_rate;
  P_(Idx::YAW_ACC, Idx::YAW_ACC) = ip.yaw_acc;
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

// ═══════════════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::reset(const ObservationData &obs, int panel_id,
                                double /*r1*/, double /*r2*/,
                                double /*dza*/) {
  // r1/r2/dza are ignored — structure parameters are known constants.
  current_panel_id_ = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  x_ = initialize_state(obs, current_panel_id_);
  k_ = current_panel_id_;
  last_k_ = current_panel_id_;
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;
  initialized_ = true;
}

// ═══════════════════════════════════════════════════════════════════
// Predict
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  const double dt2 = dt * dt;

  // ── Mean propagation (nominal dynamics) ──
  Eigen::VectorXd x_pred = x_;
  // p += v·dt + 0.5·a·dt²
  x_pred(Idx::X) += x_(Idx::VX) * dt + 0.5 * x_(Idx::AX) * dt2;
  x_pred(Idx::Y) += x_(Idx::VY) * dt + 0.5 * x_(Idx::AY) * dt2;
  x_pred(Idx::Z) += x_(Idx::VZ) * dt + 0.5 * x_(Idx::AZ) * dt2;
  // v += a·dt
  x_pred(Idx::VX) += x_(Idx::AX) * dt;
  x_pred(Idx::VY) += x_(Idx::AY) * dt;
  x_pred(Idx::VZ) += x_(Idx::AZ) * dt;
  // a unchanged (random walk with process noise)
  // yaw += yaw_rate·dt + 0.5·yaw_acc·dt²
  x_pred(Idx::YAW) = normalize_angle(
      x_(Idx::YAW) + x_(Idx::YAW_RATE) * dt +
      0.5 * x_(Idx::YAW_ACC) * dt2);
  // yaw_rate += yaw_acc·dt
  x_pred(Idx::YAW_RATE) += x_(Idx::YAW_ACC) * dt;
  // yaw_acc unchanged

  // ── Error-state transition F ──
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> F =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Identity();
  // Position ← velocity
  F(Idx::X, Idx::VX) = dt;
  F(Idx::Y, Idx::VY) = dt;
  F(Idx::Z, Idx::VZ) = dt;
  // Position ← acceleration
  F(Idx::X, Idx::AX) = 0.5 * dt2;
  F(Idx::Y, Idx::AY) = 0.5 * dt2;
  F(Idx::Z, Idx::AZ) = 0.5 * dt2;
  // Velocity ← acceleration
  F(Idx::VX, Idx::AX) = dt;
  F(Idx::VY, Idx::AY) = dt;
  F(Idx::VZ, Idx::AZ) = dt;
  // Yaw ← yaw_rate
  F(Idx::YAW, Idx::YAW_RATE) = dt;
  // Yaw ← yaw_acc
  F(Idx::YAW, Idx::YAW_ACC) = 0.5 * dt2;
  // Yaw_rate ← yaw_acc
  F(Idx::YAW_RATE, Idx::YAW_ACC) = dt;

  // ── Covariance propagation: P = F·P·Fᵀ + Q ──
  auto Q = build_process_Q(dt);
  P_ = F * P_ * F.transpose() + Q;

  x_ = x_pred;
  P_ = ensure_positive_definite(P_);
}

// ═══════════════════════════════════════════════════════════════════
// PredictContext
// ═══════════════════════════════════════════════════════════════════

norm4_v3::PredictContext OutpostInEKFBackend::buildPredictContext() const {
  norm4_v3::PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  ctx.hybrid_prior.panel_id = current_panel_id_;
  ctx.hybrid_prior.phase_index = current_panel_id_;
  return ctx;
}

// ═══════════════════════════════════════════════════════════════════
// Observation model:  z_pred = p + Rz(yaw) · r_i
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::obs_model(
    const Eigen::VectorXd &x, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double yaw = x(Idx::YAW);
  const double armor_yaw = yaw + geom_.panel_angles[pid];
  const double cos_a = std::cos(armor_yaw);
  const double sin_a = std::sin(armor_yaw);

  Eigen::Vector3d z;
  z(0) = x(Idx::X) + geom_.radius * cos_a;
  z(1) = x(Idx::Y) + geom_.radius * sin_a;
  z(2) = x(Idx::Z) + geom_.z_offsets[pid];
  return z;
}

// ═══════════════════════════════════════════════════════════════════
// World-frame Jacobian:  H_world = ∂z_pred/∂x  (3 × 12)
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix<double, 3, Idx::kDim>
OutpostInEKFBackend::obs_jacobian_world(
    const Eigen::VectorXd &x, int panel_id) const {
  (void)x;
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double yaw = x(Idx::YAW);
  const double armor_yaw = yaw + geom_.panel_angles[pid];
  const double cos_a = std::cos(armor_yaw);
  const double sin_a = std::sin(armor_yaw);

  Eigen::Matrix<double, 3, Idx::kDim> H =
      Eigen::Matrix<double, 3, Idx::kDim>::Zero();

  // Row 0: ∂x_obs/∂state
  H(0, Idx::X) = 1.0;
  H(0, Idx::YAW) = -geom_.radius * sin_a;

  // Row 1: ∂y_obs/∂state
  H(1, Idx::Y) = 1.0;
  H(1, Idx::YAW) = geom_.radius * cos_a;

  // Row 2: ∂z_obs/∂state
  H(2, Idx::Z) = 1.0;

  return H;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame innovation:  ν = Rz(-yaw) · (z_obs − z_pred)
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::compute_body_frame_innovation(
    const Eigen::Vector3d &z_obs, const Eigen::Vector3d &z_pred,
    double center_yaw) const {
  const double dx = z_obs(0) - z_pred(0);
  const double dy = z_obs(1) - z_pred(1);
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);

  Eigen::Vector3d innov;
  innov(0) =  cos_cy * dx + sin_cy * dy;
  innov(1) = -sin_cy * dx + cos_cy * dy;
  innov(2) = z_obs(2) - z_pred(2);
  return innov;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame H:  H_body = Ad_Rz(-yaw) · H_world
// Key simplification: body-frame YAW derivatives depend only on
// panel phase φᵢ, not center_yaw.
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix<double, 3, Idx::kDim>
OutpostInEKFBackend::compute_body_frame_H(
    const Eigen::Matrix<double, 3, Idx::kDim> &H_world,
    double center_yaw, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);
  const double phi = geom_.panel_angles[pid];
  const double cos_phi = std::cos(phi);
  const double sin_phi = std::sin(phi);

  // Start from world-frame H, rotate position rows (0,1)
  Eigen::Matrix<double, 3, Idx::kDim> H_body = H_world;

  // Row 0: cos(cy)·H(0,:) + sin(cy)·H(1,:)
  H_body.row(0) = cos_cy * H_world.row(0) + sin_cy * H_world.row(1);
  // Row 1: -sin(cy)·H(0,:) + cos(cy)·H(1,:)
  H_body.row(1) = -sin_cy * H_world.row(0) + cos_cy * H_world.row(1);

  // Replace YAW derivatives with simplified body-frame form
  // that depends only on panel phase φ, not center_yaw.
  H_body(0, Idx::YAW) = -geom_.radius * sin_phi;
  H_body(1, Idx::YAW) =  geom_.radius * cos_phi;

  return H_body;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame R:  R_body = Ad_Rz(-yaw) · R_world · Ad_Rz(-yaw)ᵀ
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix3d OutpostInEKFBackend::rotate_R_to_body_frame(
    const Eigen::Matrix3d &R_world, double center_yaw) const {
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);

  // Rz^T = [cos,  sin]   on the [x,y] subspace
  //        [-sin, cos]
  Eigen::Matrix3d R_body = R_world;

  // Rotate rows 0,1 by Rz^T
  for (int col = 0; col < 3; ++col) {
    const double vx = R_world(0, col);
    const double vy = R_world(1, col);
    R_body(0, col) =  cos_cy * vx + sin_cy * vy;
    R_body(1, col) = -sin_cy * vx + cos_cy * vy;
  }
  // Rotate cols 0,1 by Rz
  for (int row = 0; row < 3; ++row) {
    const double vx = R_body(row, 0);
    const double vy = R_body(row, 1);
    R_body(row, 0) = cos_cy * vx + sin_cy * vy;
    R_body(row, 1) = -sin_cy * vx + cos_cy * vy;
  }

  return R_body;
}

// ═══════════════════════════════════════════════════════════════════
// Build observation noise R
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix3d OutpostInEKFBackend::build_observation_R() const {
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  const double sxy2 = cfg_.observation_noise.sigma_pos_xy *
                      cfg_.observation_noise.sigma_pos_xy;
  const double sz2 = cfg_.observation_noise.sigma_pos_z *
                     cfg_.observation_noise.sigma_pos_z;
  R(0, 0) = sxy2;
  R(1, 1) = sxy2;
  R(2, 2) = sz2;
  return R;
}

// ═══════════════════════════════════════════════════════════════════
// evaluateSingle  —  body-frame invariant innovation, 3D obs
// ═══════════════════════════════════════════════════════════════════

norm4_v3::MeasurementEval OutpostInEKFBackend::evaluateSingle(
    const norm4_v3::PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;

  // Build 3D observation vector
  Eigen::Vector3d z_obs(obs.x, obs.y, obs.z);

  // World-frame prediction
  Eigen::Vector3d z_pred = obs_model(ctx.x_prior, pid);

  // World-frame Jacobian
  auto H_world = obs_jacobian_world(ctx.x_prior, pid);

  // Body-frame innovation
  const double center_yaw = ctx.x_prior(Idx::YAW);
  Eigen::Vector3d innov = compute_body_frame_innovation(
      z_obs, z_pred, center_yaw);

  // Body-frame H
  auto H_body = compute_body_frame_H(H_world, center_yaw, pid);

  // Body-frame R
  Eigen::Matrix3d R_world = build_observation_R();
  Eigen::Matrix3d R_body = rotate_R_to_body_frame(R_world, center_yaw);

  // Innovation covariance: S = H·P·Hᵀ + R  (all body-frame)
  Eigen::Matrix3d S = H_body * ctx.P_prior * H_body.transpose() + R_body;

  norm4_v3::MeasurementEval eval;
  Eigen::LLT<Eigen::Matrix3d> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }

  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 3; ++i) logdet += std::log(L(i, i));
  logdet *= 2.0;
  constexpr int m = 3;  // obs dim
  double log_likelihood =
      -0.5 * (nis + logdet + m * std::log(2.0 * M_PI));

  // Per-component chi2
  double chi2_pos = 0.0;
  for (int i = 0; i < 3; ++i) {
    chi2_pos += (innov(i) * innov(i)) / std::max(S(i, i), 1e-10);
  }

  const auto &gt = cfg_.gate;
  bool gate_pass = true;
  if (nis > gt.single_total_nis) gate_pass = false;
  if (chi2_pos > gt.single_pos_chi2) gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = 0.0;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos;
    eval.reject_reason = oss.str();
  }
  return eval;
}

// ═══════════════════════════════════════════════════════════════════
// evaluateDual  —  stub, Phase 1 does not implement dual
// ═══════════════════════════════════════════════════════════════════

norm4_v3::MeasurementEval OutpostInEKFBackend::evaluateDual(
    const norm4_v3::PredictContext & /*ctx*/,
    const ObservationData & /*obs0*/,
    const ObservationData & /*obs1*/,
    int /*panel_id_0*/, int /*panel_id_1*/) const {
  norm4_v3::MeasurementEval eval;
  eval.valid = false;
  eval.reject_reason = "dual_not_implemented";
  return eval;
}

// ═══════════════════════════════════════════════════════════════════
// tryUpdateSingle
// ═══════════════════════════════════════════════════════════════════

norm4_v3::UkfTrial OutpostInEKFBackend::tryUpdateSingle(
    const norm4_v3::PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  norm4_v3::UkfTrial trial;
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;

  trial.hypothesis.kind = norm4_v3::HypothesisKind::Single;
  trial.hypothesis.assignments[0] = {0, pid};
  trial.hypothesis.assignment_count = 1;

  norm4_v3::MeasurementEval eval = evaluateSingle(ctx, obs, pid);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  // Recompute body-frame quantities
  const double center_yaw = ctx.x_prior(Idx::YAW);
  auto H_world = obs_jacobian_world(ctx.x_prior, pid);
  auto H_body = compute_body_frame_H(H_world, center_yaw, pid);

  Eigen::Vector3d innov = eval.innovation;
  Eigen::Matrix3d R_world = build_observation_R();
  Eigen::Matrix3d R_body = rotate_R_to_body_frame(R_world, center_yaw);
  Eigen::Matrix3d S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (12 × 3)
  Eigen::Matrix<double, Idx::kDim, 3> K =
      ctx.P_prior * H_body.transpose() * S.inverse();

  // Error-state correction, then retract to nominal state
  Eigen::VectorXd dx = K * innov;
  Eigen::VectorXd x_post = retract_left_invariant(ctx.x_prior, dx);

  // Joseph form covariance
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> I_KH =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Identity() -
      K * H_body;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() +
      K * R_body * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = pid;
  trial.last_k_post = ctx.k_prior;
  trial.hybrid_post.panel_id = pid;
  trial.hybrid_post.phase_index = pid;

  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, obs, pid);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  return trial;
}

// ═══════════════════════════════════════════════════════════════════
// tryUpdateDual  —  stub
// ═══════════════════════════════════════════════════════════════════

norm4_v3::UkfTrial OutpostInEKFBackend::tryUpdateDual(
    const norm4_v3::PredictContext & /*ctx*/,
    const ObservationData & /*obs0*/,
    const ObservationData & /*obs1*/,
    int /*panel_id_0*/, int /*panel_id_1*/) const {
  norm4_v3::UkfTrial trial;
  trial.reject_reason = "dual_not_implemented";
  return trial;
}

// ═══════════════════════════════════════════════════════════════════
// commit
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::commit(const norm4_v3::UkfTrial &trial) {
  if (!trial.success) return;

  x_ = trial.x_post;
  P_ = trial.P_post;
  k_ = trial.k_post;
  last_k_ = trial.last_k_post;

  if (trial.hypothesis.kind == norm4_v3::HypothesisKind::Single) {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  }

  last_nis_ = trial.eval.nis;
  if (trial.eval.innovation.size() >= 3) {
    last_innov_xyz_ = trial.eval.innovation.head<3>();
  }
  last_innov_yaw_ = 0.0;
  last_update_type_ =
      (trial.hypothesis.kind == norm4_v3::HypothesisKind::Single) ? 1 : 2;

  P_ = ensure_positive_definite(P_);
}

// ═══════════════════════════════════════════════════════════════════
// snapshot
// ═══════════════════════════════════════════════════════════════════

norm4_v3::BackendSnapshot OutpostInEKFBackend::snapshot() const {
  norm4_v3::BackendSnapshot snap;
  snap.x = x_;
  snap.P = P_;
  snap.k = k_;
  snap.last_k = last_k_;
  snap.current_panel_id = current_panel_id_;
  snap.hybrid.panel_id = current_panel_id_;
  snap.hybrid.phase_index = current_panel_id_;
  snap.last_nis = last_nis_;
  snap.last_innov_xyz = last_innov_xyz_;
  snap.last_innov_yaw = last_innov_yaw_;
  snap.last_update_type = last_update_type_;
  return snap;
}

// ═══════════════════════════════════════════════════════════════════
// SpinFilterInterface accessors
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::get_center_position() const {
  return Eigen::Vector3d(x_(Idx::X), x_(Idx::Y), x_(Idx::Z));
}

std::pair<double, double> OutpostInEKFBackend::get_radii() const {
  return {geom_.radius, geom_.radius};
}

double OutpostInEKFBackend::get_yaw() const {
  return normalize_angle(x_(Idx::YAW));
}

double OutpostInEKFBackend::get_delta() const { return x_(Idx::YAW); }

// ═══════════════════════════════════════════════════════════════════
// Posterior sanity checks
// ═══════════════════════════════════════════════════════════════════

bool OutpostInEKFBackend::check_posterior_sanity(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &x_post,
    const Eigen::MatrixXd &P_post) const {
  const auto &ps = cfg_.posterior_sanity;

  // Center jump
  Eigen::Vector3d prior_center(x_prior(Idx::X), x_prior(Idx::Y),
                                x_prior(Idx::Z));
  Eigen::Vector3d post_center(x_post(Idx::X), x_post(Idx::Y),
                               x_post(Idx::Z));
  if ((post_center - prior_center).norm() > ps.max_center_jump)
    return false;

  // Yaw jump
  double yaw_jump =
      std::abs(angle_difference(x_post(Idx::YAW), x_prior(Idx::YAW)));
  if (yaw_jump > ps.max_yaw_jump) return false;

  // Yaw rate / yaw acc physical limits
  if (std::abs(x_post(Idx::YAW_RATE)) > ps.max_yaw_rate) return false;
  if (std::abs(x_post(Idx::YAW_ACC)) > ps.max_yaw_acc) return false;

  // Covariance sanity
  if (!P_post.allFinite()) return false;
  for (int i = 0; i < Idx::kDim; ++i) {
    if (P_post(i, i) <= 0.0) return false;
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════
// Reconstruction error
// ═══════════════════════════════════════════════════════════════════

double OutpostInEKFBackend::compute_reconstruction_error(
    const Eigen::VectorXd &x_post, const ObservationData &obs,
    int panel_id) const {
  Eigen::Vector3d z_rebuild = obs_model(x_post, panel_id);
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - z_rebuild).norm();
}

// ═══════════════════════════════════════════════════════════════════
// Left-invariant retraction:  X̂⁺ = Exp(δξ̂) ∘ X̂
// ═══════════════════════════════════════════════════════════════════

Eigen::VectorXd OutpostInEKFBackend::retract_left_invariant(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &dx) const {
  Eigen::VectorXd x_post = x_prior;

  // Position (additive, left-invariant for ℝⁿ)
  x_post(Idx::X) += dx(Idx::X);
  x_post(Idx::Y) += dx(Idx::Y);
  x_post(Idx::Z) += dx(Idx::Z);

  // Velocity
  x_post(Idx::VX) += dx(Idx::VX);
  x_post(Idx::VY) += dx(Idx::VY);
  x_post(Idx::VZ) += dx(Idx::VZ);

  // Acceleration
  x_post(Idx::AX) += dx(Idx::AX);
  x_post(Idx::AY) += dx(Idx::AY);
  x_post(Idx::AZ) += dx(Idx::AZ);

  // Rotation: Exp(δψ) · R̂ ⇔ yaw⁺ = normalize(yaw + δψ)
  x_post(Idx::YAW) = normalize_angle(x_prior(Idx::YAW) + dx(Idx::YAW));

  // Yaw rate / yaw acc
  x_post(Idx::YAW_RATE) += dx(Idx::YAW_RATE);
  x_post(Idx::YAW_ACC) += dx(Idx::YAW_ACC);

  return x_post;
}

// ═══════════════════════════════════════════════════════════════════
// State initialization from observation
// ═══════════════════════════════════════════════════════════════════

Eigen::VectorXd OutpostInEKFBackend::initialize_state(
    const ObservationData &obs, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(Idx::kDim);

  // Observation yaw is the armor face direction. The structure phase uses the
  // radial direction from center to armor, which is opposite by pi.
  const double armor_yaw = normalize_angle(obs.yaw - M_PI);
  const double center_yaw =
      normalize_angle(armor_yaw - geom_.panel_angles[pid]);

  // Back-project from armor position to center position
  x0(Idx::X) = obs.x - geom_.radius * std::cos(armor_yaw);
  x0(Idx::Y) = obs.y - geom_.radius * std::sin(armor_yaw);
  x0(Idx::Z) = obs.z - geom_.z_offsets[pid];
  x0(Idx::YAW) = center_yaw;

  return x0;
}

// ═══════════════════════════════════════════════════════════════════
// Process noise Q (discrete white-noise jerk / CA model)
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix<double, Idx::kDim, Idx::kDim>
OutpostInEKFBackend::build_process_Q(double dt) const {
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> Q =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Zero();

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;

  const double q_a = cfg_.process_noise.acc * cfg_.process_noise.acc;
  const double q_alpha =
      cfg_.process_noise.yaw_acc * cfg_.process_noise.yaw_acc;

  // CA block: [p, v, a] for each axis (X, Y, Z)
  auto fill_ca_block = [&](int p_idx, int v_idx, int a_idx, double q) {
    Q(p_idx, p_idx) = q * dt5 / 20.0;
    Q(p_idx, v_idx) = q * dt4 / 8.0;
    Q(v_idx, p_idx) = Q(p_idx, v_idx);
    Q(p_idx, a_idx) = q * dt3 / 6.0;
    Q(a_idx, p_idx) = Q(p_idx, a_idx);
    Q(v_idx, v_idx) = q * dt3 / 3.0;
    Q(v_idx, a_idx) = q * dt2 / 2.0;
    Q(a_idx, v_idx) = Q(v_idx, a_idx);
    Q(a_idx, a_idx) = q * dt;
  };

  fill_ca_block(Idx::X, Idx::VX, Idx::AX, q_a);
  fill_ca_block(Idx::Y, Idx::VY, Idx::AY, q_a);
  fill_ca_block(Idx::Z, Idx::VZ, Idx::AZ, q_a);

  // Yaw CA block: [yaw, yaw_rate, yaw_acc]
  fill_ca_block(Idx::YAW, Idx::YAW_RATE, Idx::YAW_ACC, q_alpha);

  return Q;
}

}  // namespace fyt::auto_aim::outpost_v3
