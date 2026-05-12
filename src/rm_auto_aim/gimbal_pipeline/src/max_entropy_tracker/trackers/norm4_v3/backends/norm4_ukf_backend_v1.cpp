// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v1.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"
#include "max_entropy_tracker/utils/sigma_points.hpp"

namespace fyt::auto_aim::norm4_v3 {

namespace {

std::shared_ptr<CompositeProcessModel> create_v1_process_model(
    const UnifiedConfig &cfg) {
  TranslationConfig tc;
  tc.cv_process_noise_vel = cfg.motion.cv_process_noise_vel;
  tc.ca_process_noise_acc = cfg.motion.ca_process_noise_acc;
  tc.singer_alpha = cfg.motion.singer_alpha;
  tc.singer_sigma = cfg.motion.singer_sigma;

  RotationConfig rc;
  rc.cv_process_noise_rate = cfg.spin.spin_process_noise_delta_rate;
  rc.ca_process_noise_acc = cfg.spin.spin_process_noise_delta_acc;

  StructuralConfig sc;
  sc.process_noise_r = cfg.motion.process_noise_r;
  sc.process_noise_dz = cfg.motion.process_noise_dz;

  return create_default_process_model(cfg.motion.translation_model,
                                      RotationModel::CV, tc, rc, sc, 3);
}

double weighted_angle_mean(const Eigen::MatrixXd &samples,
                           const Eigen::VectorXd &weights, int col) {
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (int i = 0; i < samples.rows(); ++i) {
    sin_sum += weights(i) * std::sin(samples(i, col));
    cos_sum += weights(i) * std::cos(samples(i, col));
  }
  return std::atan2(sin_sum, cos_sum);
}

}  // namespace

Norm4UkfBackendV1::Norm4UkfBackendV1(const UnifiedConfig &config, double dt)
    : config_(config),
      dt_(dt),
      process_model_(create_v1_process_model(config)),
      state_idx_(process_model_->layout()) {
  int n = process_model_->state_dim();
  x_ = Eigen::VectorXd::Zero(n);
  P_ = Eigen::MatrixXd::Identity(n, n) * 100.0;
  Q_ = process_model_->build_Q(dt_);
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

void Norm4UkfBackendV1::reset(const ObservationData &obs, int panel_id,
                                double r1, double r2, double dza) {
  current_panel_id_ = ((panel_id % 4) + 4) % 4;
  auto idx = state_idx_;

  double use_r = (current_panel_id_ % 2 == 0) ? r1 : r2;
  double panel_angle = current_panel_id_ * (M_PI / 2.0);
  double center_x = obs.x - use_r * std::cos(obs.yaw);
  double center_y = obs.y - use_r * std::sin(obs.yaw);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);
  k_ = 0;
  last_k_ = 0;

  x_ = Eigen::VectorXd::Zero(process_model_->state_dim());
  x_(idx.X()) = center_x;
  x_(idx.VX()) = 0.0;
  x_(idx.Y()) = center_y;
  x_(idx.VY()) = 0.0;
  x_(idx.Z()) = obs.z;
  x_(idx.VZ()) = 0.0;
  x_(idx.DELTA()) = center_yaw;
  x_(idx.DELTA_RATE()) = 0.0;
  x_(idx.R1()) = r1;
  x_(idx.R2()) = r2;
  x_(idx.DZA()) = dza;

  P_ = process_model_->get_initial_covariance();
  Q_ = process_model_->build_Q(dt_);

  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;

  initialized_ = true;
}

void Norm4UkfBackendV1::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  Q_ = process_model_->build_Q(dt);

  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        process_model_->state_dim(), config_.ukf.alpha, config_.ukf.beta,
        config_.ukf.kappa);
  }

  Eigen::MatrixXd P_safe = ensure_positive_definite(P_, 1e-6);
  Eigen::MatrixXd sigma_pts = sigma_gen_->generate(x_, P_safe);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd sigma_pred(n_sigma, n);
  for (int i = 0; i < n_sigma; ++i) {
    sigma_pred.row(i) =
        process_model_->predict(sigma_pts.row(i).transpose(), dt);
  }

  auto Wm = sigma_gen_->Wm();
  auto Wc = sigma_gen_->Wc();

  x_ = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n_sigma; ++i) {
    x_ += Wm(i) * sigma_pred.row(i).transpose();
  }

  int delta_idx = state_idx_.DELTA();
  x_(delta_idx) = weighted_angle_mean(sigma_pred, Wm, delta_idx);
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd diff = sigma_pred.row(i).transpose() - x_;
    diff(delta_idx) =
        angle_difference(sigma_pred(i, delta_idx), x_(delta_idx));
    P_pred += Wc(i) * diff * diff.transpose();
  }
  P_pred += Q_;
  P_ = P_pred;
  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

PredictContext Norm4UkfBackendV1::buildPredictContext() const {
  PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  return ctx;
}

std::string Norm4UkfBackendV1::r_type_for_panel(int panel_id) {
  return (panel_id % 2 == 0) ? "r1" : "r2";
}

std::string Norm4UkfBackendV1::armor_layer_for_panel(int panel_id) {
  return (panel_id % 2 == 0) ? "lower" : "upper";
}

Eigen::Vector4d Norm4UkfBackendV1::obs_model_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  (void)k;
  auto idx = state_idx_;
  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double z_mean = x(idx.Z());
  double d_za = x(idx.DZA());

  double radius = (panel_id % 2 == 0) ? x(idx.R1()) : x(idx.R2());
  double center_yaw = normalize_angle(x(idx.DELTA()));
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

void Norm4UkfBackendV1::generate_sigma_points(
    const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
    Eigen::MatrixXd &out_sigma_pts,
    Eigen::VectorXd &out_Wm, Eigen::VectorXd &out_Wc) const {
  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        process_model_->state_dim(), config_.ukf.alpha, config_.ukf.beta,
        config_.ukf.kappa);
  }
  Eigen::MatrixXd P_safe = ensure_positive_definite(P, 1e-6);
  out_sigma_pts = sigma_gen_->generate(x, P_safe);
  out_Wm = sigma_gen_->Wm();
  out_Wc = sigma_gen_->Wc();
}

MeasurementEval Norm4UkfBackendV1::evaluateSingle(
    const PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  MeasurementEval eval;
  const int p = ((panel_id % 4) + 4) % 4;

  double panel_angle = p * (M_PI / 2.0);
  double center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p)
            .transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);

  // Innovation with yaw wrap
  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));

  // Build R using configurable noise parameters
  const auto &v1 = config_.norm4_v3.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  Eigen::Matrix4d R =
      Eigen::Vector4d(sp * sp, sp * sp, sz * sz, sy * sy).asDiagonal();

  // S = Pzz + R
  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
  }

  Eigen::Matrix4d Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
  }

  Eigen::Matrix4d S = Pzz;

  // NIS via LLT
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
  for (int i = 0; i < 4; ++i) {
    logdet += std::log(L(i, i));
  }
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 4.0 * std::log(2.0 * M_PI));

  // Per-component chi2
  double chi2_yaw = (innov(3) * innov(3)) / S(3, 3);
  Eigen::Vector3d innov_pos = innov.head<3>();
  Eigen::Matrix3d S_pos = S.topLeftCorner<3, 3>();
  double chi2_pos =
      innov_pos.transpose() * S_pos.inverse() * innov_pos;

  // Gate check using configurable thresholds
  const auto &gt = config_.norm4_v3.ukf_v1.gate;
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

MeasurementEval Norm4UkfBackendV1::evaluateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  MeasurementEval eval;
  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double center_yaw_obs0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double center_yaw_obs1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, center_yaw_obs0,
           obs1.x, obs1.y, obs1.z, center_yaw_obs1;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d z0 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p0);
    Eigen::Vector4d z1 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p1);
    z_pred_pts.row(i) << z0.transpose(), z1.transpose();
  }

  Eigen::Matrix<double, 8, 1> z_pred = Eigen::Matrix<double, 8, 1>::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);
  z_pred(7) = weighted_angle_mean(z_pred_pts, Wm, 7);

  Eigen::Matrix<double, 8, 1> innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));
  innov(7) = angle_difference(z_obs(7), z_pred(7));

  const auto &v1 = config_.norm4_v3.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  double r_scale = v1.dual_raw_R_scale;
  Eigen::Vector<double, 8> r_diag;
  r_diag << sp * sp, sp * sp, sz * sz, sy * sy,
            sp * sp, sp * sp, sz * sz, sy * sy;
  r_diag *= r_scale;
  Eigen::Matrix<double, 8, 8> R = r_diag.asDiagonal();

  Eigen::MatrixXd diff_z(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
    diff_z(i, 7) = angle_difference(z_pred_pts(i, 7), z_pred(7));
  }

  Eigen::Matrix<double, 8, 8> Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Matrix<double, 8, 1> dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
  }
  Eigen::Matrix<double, 8, 8> S = Pzz;

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
  for (int i = 0; i < 8; ++i) {
    logdet += std::log(L(i, i));
  }
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 8.0 * std::log(2.0 * M_PI));

  double chi2_yaw0 = (innov(3) * innov(3)) / S(3, 3);
  double chi2_yaw1 = (innov(7) * innov(7)) / S(7, 7);
  double chi2_yaw = std::max(chi2_yaw0, chi2_yaw1);

  Eigen::Vector3d innov_pos0 = innov.segment<3>(0);
  Eigen::Vector3d innov_pos1 = innov.segment<3>(4);
  Eigen::Matrix3d S_pos0 = S.block<3, 3>(0, 0);
  Eigen::Matrix3d S_pos1 = S.block<3, 3>(4, 4);
  double chi2_pos0 =
      innov_pos0.transpose() * S_pos0.inverse() * innov_pos0;
  double chi2_pos1 =
      innov_pos1.transpose() * S_pos1.inverse() * innov_pos1;
  double chi2_pos = std::max(chi2_pos0, chi2_pos1);

  const auto &gt = config_.norm4_v3.ukf_v1.gate;
  bool gate_pass = true;
  if (nis > gt.dual_total_nis) gate_pass = false;
  if (chi2_pos0 > gt.dual_each_pos_chi2 || chi2_pos1 > gt.dual_each_pos_chi2) gate_pass = false;
  if (chi2_yaw0 > gt.dual_each_yaw_chi2 || chi2_yaw1 > gt.dual_each_yaw_chi2) gate_pass = false;

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

UkfTrial Norm4UkfBackendV1::tryUpdateSingle(
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

  double panel_angle = p * (M_PI / 2.0);
  double center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd z_pred_pts(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p)
            .transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);

  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));

  const auto &v1 = config_.norm4_v3.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  Eigen::Matrix4d R =
      Eigen::Vector4d(sp * sp, sp * sp, sz * sz, sy * sy).asDiagonal();

  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
  }

  Eigen::Matrix4d Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, 4);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - ctx.x_prior;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  Eigen::Matrix4d S = Pzz;
  Eigen::LLT<Eigen::Matrix4d> llt(S);
  if (llt.info() != Eigen::Success) {
    trial.reject_reason = "S_not_spd_in_tryUpdate";
    return trial;
  }

  Eigen::MatrixXd Pzz_inv = S.inverse();
  Eigen::MatrixXd K = Pxz * Pzz_inv;

  // Conservative structural gain (single obs: freeze r1/r2/dza)
  auto idx = state_idx_;
  // Single update structural gain from config (default 0 = frozen)
  const auto &su = config_.norm4_v3.ukf_v1.single_update;
  K.row(idx.R1()) *= su.structural_gain_r;
  K.row(idx.R2()) *= su.structural_gain_r;
  K.row(idx.DZA()) *= su.structural_gain_dza;

  Eigen::VectorXd x_post = ctx.x_prior + K * innov;
  x_post(idx.DELTA()) = normalize_angle(x_post(idx.DELTA()));
  Eigen::MatrixXd P_post = ctx.P_prior - K * S * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = 0;
  trial.last_k_post = 0;

  // Reconstruction check
  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, trial.k_post, obs, p);

  // Posterior sanity
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  // Clamp structure params in trial
  trial.x_post(idx.R1()) = std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) = std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) = std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

UkfTrial Norm4UkfBackendV1::tryUpdateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  UkfTrial trial;
  trial.hypothesis.kind = HypothesisKind::Dual;
  trial.hypothesis.assignments[0] = {0, panel_id_0};
  trial.hypothesis.assignments[1] = {1, panel_id_1};
  trial.hypothesis.assignment_count = 2;

  MeasurementEval eval = evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double center_yaw_obs0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double center_yaw_obs1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, center_yaw_obs0,
           obs1.x, obs1.y, obs1.z, center_yaw_obs1;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd z_pred_pts(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d z0 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p0);
    Eigen::Vector4d z1 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p1);
    z_pred_pts.row(i) << z0.transpose(), z1.transpose();
  }

  Eigen::Matrix<double, 8, 1> z_pred = Eigen::Matrix<double, 8, 1>::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);
  z_pred(7) = weighted_angle_mean(z_pred_pts, Wm, 7);

  Eigen::Matrix<double, 8, 1> innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));
  innov(7) = angle_difference(z_obs(7), z_pred(7));

  const auto &v1 = config_.norm4_v3.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  double r_scale = v1.dual_raw_R_scale;
  Eigen::Vector<double, 8> r_diag;
  r_diag << sp * sp, sp * sp, sz * sz, sy * sy,
            sp * sp, sp * sp, sz * sz, sy * sy;
  r_diag *= r_scale;
  Eigen::Matrix<double, 8, 8> R = r_diag.asDiagonal();

  Eigen::MatrixXd diff_z(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
    diff_z(i, 7) = angle_difference(z_pred_pts(i, 7), z_pred(7));
  }

  Eigen::Matrix<double, 8, 8> Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Matrix<double, 8, 1> dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - ctx.x_prior;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  Eigen::Matrix<double, 8, 8> S = Pzz;
  Eigen::LLT<Eigen::Matrix<double, 8, 8>> llt(S);
  if (llt.info() != Eigen::Success) {
    trial.reject_reason = "S_not_spd_in_tryUpdate_dual";
    return trial;
  }

  Eigen::Matrix<double, 8, 8> S_inv = S.inverse();
  Eigen::MatrixXd K = Pxz * S_inv;

  // Conservative structural gain for dual: small but non-zero
  auto idx = state_idx_;
  const auto &du = config_.norm4_v3.ukf_v1.dual_update;
  K.row(idx.R1()) *= du.structural_gain_r;
  K.row(idx.R2()) *= du.structural_gain_r;
  K.row(idx.DZA()) *= du.structural_gain_dza;

  Eigen::VectorXd x_post = ctx.x_prior + K * innov;
  x_post(idx.DELTA()) = normalize_angle(x_post(idx.DELTA()));
  Eigen::MatrixXd P_post = ctx.P_prior - K * S * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = 0;
  trial.last_k_post = 0;

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

  // Clamp structure params in trial (dual)
  trial.x_post(idx.R1()) = std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) = std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) = std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

void Norm4UkfBackendV1::commit(const UkfTrial &trial) {
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

  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

// ── SpinFilterInterface queries ──

Eigen::Vector3d Norm4UkfBackendV1::get_center_position() const {
  auto idx = state_idx_;
  return Eigen::Vector3d(x_(idx.X()), x_(idx.Y()), x_(idx.Z()));
}

std::pair<double, double> Norm4UkfBackendV1::get_radii() const {
  auto idx = state_idx_;
  return {x_(idx.R1()), x_(idx.R2())};
}

double Norm4UkfBackendV1::get_dza() const {
  return x_(state_idx_.DZA());
}

double Norm4UkfBackendV1::get_yaw() const {
  return normalize_angle(x_(state_idx_.DELTA()));
}

double Norm4UkfBackendV1::get_delta() const {
  return x_(state_idx_.DELTA());
}

// ── Private helpers ──

bool Norm4UkfBackendV1::check_posterior_sanity(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &x_post,
    const Eigen::MatrixXd &P_post) const {
  auto idx = state_idx_;
  const auto &ps = config_.norm4_v3.ukf_v1.posterior_sanity;

  // Center jump check
  Eigen::Vector3d prior_center(x_prior(idx.X()), x_prior(idx.Y()),
                                x_prior(idx.Z()));
  Eigen::Vector3d post_center(x_post(idx.X()), x_post(idx.Y()),
                               x_post(idx.Z()));
  double center_jump = (post_center - prior_center).norm();
  if (center_jump > ps.max_center_jump) return false;

  // Yaw/delta jump check
  double delta_jump = std::abs(angle_difference(x_post(idx.DELTA()),
                                                  x_prior(idx.DELTA())));
  if (delta_jump > ps.max_yaw_jump) return false;

  // Radius range check
  double r1 = x_post(idx.R1());
  double r2 = x_post(idx.R2());
  if (r1 < ps.min_r || r1 > ps.max_r || r2 < ps.min_r || r2 > ps.max_r) return false;

  // Radius jump check
  if (std::abs(r1 - x_prior(idx.R1())) > ps.max_r_jump) return false;
  if (std::abs(r2 - x_prior(idx.R2())) > ps.max_r_jump) return false;

  // DZA range check
  double dza = x_post(idx.DZA());
  if (dza < ps.min_dza || dza > ps.max_dza) return false;
  if (std::abs(dza - x_prior(idx.DZA())) > ps.max_dza_jump) return false;

  // P positive semi-definite
  if (!P_post.allFinite()) return false;

  return true;
}

double Norm4UkfBackendV1::compute_reconstruction_error(
    const Eigen::VectorXd &x_post, int k, const ObservationData &obs,
    int panel_id) const {
  Eigen::Vector4d z_rebuild = obs_model_single(x_post, k, panel_id);
  Eigen::Vector3d pos_rebuild = z_rebuild.head<3>();
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - pos_rebuild).norm();
}

void Norm4UkfBackendV1::apply_state_constraints() {
  auto idx = state_idx_;
  x_ = fyt::auto_aim::apply_state_constraints(
      x_, idx.R1(), idx.R2(), idx.DZA(), config_.constraints.min_radius,
      config_.constraints.max_radius, 0.0, config_.constraints.max_dz);
  x_(idx.DELTA()) = normalize_angle(x_(idx.DELTA()));
}

}  // namespace fyt::auto_aim::norm4_v3
