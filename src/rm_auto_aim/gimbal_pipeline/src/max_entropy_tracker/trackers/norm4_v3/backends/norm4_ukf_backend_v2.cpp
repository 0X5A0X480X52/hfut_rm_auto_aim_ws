// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v2.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim::norm4_v3 {

namespace {

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

UkfBackendV2::UkfBackendV2(std::unique_ptr<IMotionModelBundle> motion,
                             std::unique_ptr<IMeasurementNoiseModel> noise,
                             const Norm4V3UkfConfig &ukf_config,
                             const UnifiedConfig &config, double dt)
    : config_(config),
      ukf_config_(ukf_config),
      dt_(dt),
      motion_(std::move(motion)),
      noise_(std::move(noise)) {
  int n = motion_->state_dim();
  x_ = Eigen::VectorXd::Zero(n);
  P_ = Eigen::MatrixXd::Identity(n, n) * 100.0;
  Q_ = motion_->build_Q(dt_);
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

void UkfBackendV2::reset(const ObservationData &obs, int panel_id, double r1,
                          double r2, double dza) {
  current_panel_id_ = ((panel_id % 4) + 4) % 4;

  x_ = motion_->initial_state(obs, current_panel_id_, r1, r2, dza);
  P_ = motion_->initial_covariance();
  Q_ = motion_->build_Q(dt_);

  auto idx = motion_->state_idx();
  double panel_angle = current_panel_id_ * (M_PI / 2.0);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);
  k_ = 0;
  last_k_ = 0;
  x_(idx.DELTA()) = center_yaw;

  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;

  initialized_ = true;
}

void UkfBackendV2::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  int n = motion_->state_dim();
  Q_ = motion_->build_Q(dt);

  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        n, config_.ukf.alpha, config_.ukf.beta, config_.ukf.kappa);
  }

  Eigen::MatrixXd P_safe = ensure_positive_definite(P_, 1e-6);
  Eigen::MatrixXd sigma_pts = sigma_gen_->generate(x_, P_safe);
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd sigma_pred(n_sigma, n);
  for (int i = 0; i < n_sigma; ++i) {
    sigma_pred.row(i) =
        motion_->predict(sigma_pts.row(i).transpose(), dt).transpose();
  }

  auto Wm = sigma_gen_->Wm();
  auto Wc = sigma_gen_->Wc();

  x_ = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n_sigma; ++i) {
    x_ += Wm(i) * sigma_pred.row(i).transpose();
  }

  int delta_idx = motion_->state_idx().DELTA();
  x_(delta_idx) = weighted_angle_mean(sigma_pred, Wm, delta_idx);
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd diff = sigma_pred.row(i).transpose() - x_;
    diff(delta_idx) = angle_difference(sigma_pred(i, delta_idx), x_(delta_idx));
    P_pred += Wc(i) * diff * diff.transpose();
  }
  P_pred += Q_;
  P_ = P_pred;
  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

PredictContext UkfBackendV2::buildPredictContext() const {
  PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  return ctx;
}

Eigen::Vector4d UkfBackendV2::obs_model_single(const Eigen::VectorXd &x, int k,
                                                int panel_id) const {
  (void)k;
  auto idx = motion_->state_idx();
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

void UkfBackendV2::generate_sigma_points(const Eigen::VectorXd &x,
                                          const Eigen::MatrixXd &P,
                                          Eigen::MatrixXd &out_sigma_pts,
                                          Eigen::VectorXd &out_Wm,
                                          Eigen::VectorXd &out_Wc) const {
  int n = motion_->state_dim();
  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        n, config_.ukf.alpha, config_.ukf.beta, config_.ukf.kappa);
  }
  Eigen::MatrixXd P_safe = ensure_positive_definite(P, 1e-6);
  out_sigma_pts = sigma_gen_->generate(x, P_safe);
  out_Wm = sigma_gen_->Wm();
  out_Wc = sigma_gen_->Wc();
}

MeasurementEval UkfBackendV2::evaluateSingle(const PredictContext &ctx,
                                              const ObservationData &obs,
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

  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = angle_difference(z_obs(3), z_pred(3));

  Eigen::Matrix4d R = noise_->build_single_R(obs);

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
  double log_likelihood = -0.5 * (nis + logdet + 4.0 * std::log(2.0 * M_PI));

  double chi2_yaw = (innov(3) * innov(3)) / S(3, 3);
  Eigen::Vector3d innov_pos = innov.head<3>();
  Eigen::Matrix3d S_pos = S.topLeftCorner<3, 3>();
  double chi2_pos = innov_pos.transpose() * S_pos.inverse() * innov_pos;

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

MeasurementEval UkfBackendV2::evaluateDual(const PredictContext &ctx,
                                            const ObservationData &obs0,
                                            const ObservationData &obs1,
                                            int panel_id_0,
                                            int panel_id_1) const {
  MeasurementEval eval;
  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double cy0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double cy1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, cy0, obs1.x, obs1.y, obs1.z, cy1;

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

  Eigen::Matrix<double, 8, 8> R = noise_->build_dual_R(obs0, obs1);

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
  double log_likelihood = -0.5 * (nis + logdet + 8.0 * std::log(2.0 * M_PI));

  double chi2_yaw0 = (innov(3) * innov(3)) / S(3, 3);
  double chi2_yaw1 = (innov(7) * innov(7)) / S(7, 7);
  double chi2_yaw = std::max(chi2_yaw0, chi2_yaw1);

  Eigen::Vector3d innov_pos0 = innov.segment<3>(0);
  Eigen::Vector3d innov_pos1 = innov.segment<3>(4);
  Eigen::Matrix3d S_pos0 = S.block<3, 3>(0, 0);
  Eigen::Matrix3d S_pos1 = S.block<3, 3>(4, 4);
  double chi2_pos0 = innov_pos0.transpose() * S_pos0.inverse() * innov_pos0;
  double chi2_pos1 = innov_pos1.transpose() * S_pos1.inverse() * innov_pos1;
  double chi2_pos = std::max(chi2_pos0, chi2_pos1);

  const auto &gt = ukf_config_.gate;
  bool gate_pass = true;
  if (nis > gt.dual_total_nis) gate_pass = false;
  if (chi2_pos0 > gt.dual_each_pos_chi2 || chi2_pos1 > gt.dual_each_pos_chi2)
    gate_pass = false;
  if (chi2_yaw0 > gt.dual_each_yaw_chi2 || chi2_yaw1 > gt.dual_each_yaw_chi2)
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

UkfTrial UkfBackendV2::tryUpdateSingle(const PredictContext &ctx,
                                        const ObservationData &obs,
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
  int n = motion_->state_dim();

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

  Eigen::Matrix4d R = noise_->build_single_R(obs);

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

  auto idx = motion_->state_idx();
  const auto &su = ukf_config_.single_update;
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

  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, trial.k_post, obs, p);
  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  trial.x_post(idx.R1()) = std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) = std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) = std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

UkfTrial UkfBackendV2::tryUpdateDual(const PredictContext &ctx,
                                      const ObservationData &obs0,
                                      const ObservationData &obs1,
                                      int panel_id_0, int panel_id_1) const {
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

  double cy0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double cy1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, cy0, obs1.x, obs1.y, obs1.z, cy1;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();
  int n = motion_->state_dim();

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

  Eigen::Matrix<double, 8, 8> R = noise_->build_dual_R(obs0, obs1);

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

  auto idx = motion_->state_idx();
  const auto &du = ukf_config_.dual_update;
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

  trial.x_post(idx.R1()) = std::clamp(trial.x_post(idx.R1()), 0.05, 0.50);
  trial.x_post(idx.R2()) = std::clamp(trial.x_post(idx.R2()), 0.05, 0.50);
  trial.x_post(idx.DZA()) = std::clamp(trial.x_post(idx.DZA()), 0.0, 0.15);

  return trial;
}

void UkfBackendV2::commit(const UkfTrial &trial) {
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

BackendSnapshot UkfBackendV2::snapshot() const {
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

Eigen::Vector3d UkfBackendV2::get_center_position() const {
  auto idx = motion_->state_idx();
  return Eigen::Vector3d(x_(idx.X()), x_(idx.Y()), x_(idx.Z()));
}

std::pair<double, double> UkfBackendV2::get_radii() const {
  auto idx = motion_->state_idx();
  return {x_(idx.R1()), x_(idx.R2())};
}

double UkfBackendV2::get_dza() const {
  return x_(motion_->state_idx().DZA());
}

double UkfBackendV2::get_yaw() const {
  return normalize_angle(x_(motion_->state_idx().DELTA()));
}

double UkfBackendV2::get_delta() const {
  return x_(motion_->state_idx().DELTA());
}

bool UkfBackendV2::check_posterior_sanity(const Eigen::VectorXd &x_prior,
                                           const Eigen::VectorXd &x_post,
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

  double r1 = x_post(idx.R1());
  double r2 = x_post(idx.R2());
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

double UkfBackendV2::compute_reconstruction_error(const Eigen::VectorXd &x_post,
                                                   int k,
                                                   const ObservationData &obs,
                                                   int panel_id) const {
  Eigen::Vector4d z_rebuild = obs_model_single(x_post, k, panel_id);
  Eigen::Vector3d pos_rebuild = z_rebuild.head<3>();
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - pos_rebuild).norm();
}

void UkfBackendV2::apply_state_constraints() {
  auto idx = motion_->state_idx();
  x_ = fyt::auto_aim::apply_state_constraints(
      x_, idx.R1(), idx.R2(), idx.DZA(), config_.constraints.min_radius,
      config_.constraints.max_radius, 0.0, config_.constraints.max_dz);
  x_(idx.DELTA()) = normalize_angle(x_(idx.DELTA()));
}

}  // namespace fyt::auto_aim::norm4_v3
