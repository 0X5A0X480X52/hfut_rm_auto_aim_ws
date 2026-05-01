// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/filters/dual_radius_spin_ukf.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim {

/* ================================================================ */
/*  Construction                                                     */
/* ================================================================ */

DualRadiusSpinUKF::DualRadiusSpinUKF(
    const UnifiedConfig &config, double dt,
    std::shared_ptr<CompositeProcessModel> process_model)
    : BaseUKF(config, dt),
      motion_model_(process_model ? std::move(process_model)
                                  : create_process_model(config, dt)),
      state_idx_(motion_model_->layout()) {
  x_ = Eigen::VectorXd::Zero(state_dim());
  P_ = Eigen::MatrixXd::Identity(state_dim(), state_dim()) * 100.0;
  Q_ = motion_model_->build_Q(dt_);
  init_sigma_generator();
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

std::shared_ptr<CompositeProcessModel>
DualRadiusSpinUKF::create_process_model(const UnifiedConfig &cfg,
                                        double /*dt*/) const {
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

  return create_default_process_model(
      cfg.motion.translation_model,
      RotationModel::CV, tc, rc, sc, 3);
}

int DualRadiusSpinUKF::state_dim() const { return motion_model_->state_dim(); }

/* ================================================================ */
/*  Initialization                                                   */
/* ================================================================ */

void DualRadiusSpinUKF::initialize(
    const std::vector<ObservationData> &observations, double r1, double r2,
    double dza, int panel_id) {
  if (observations.empty())
    throw std::invalid_argument("At least one observation required");

  const auto &obs = observations.front();

  double use_r = r1;
  double panel_angle = 0.0;
  if (panel_id >= 0) {
    use_r = (panel_id % 2 == 0) ? r1 : r2;
    panel_angle = panel_id * (M_PI / 2.0);
  }

  double center_x = obs.x - use_r * std::cos(obs.yaw);
  double center_y = obs.y - use_r * std::sin(obs.yaw);
  double center_z = obs.z;
  double center_yaw = normalize_angle(obs.yaw - panel_angle);

  auto [k, delta] = decompose_yaw(center_yaw);
  k_ = k;
  last_k_ = k_;

  auto idx = state_idx_;
  x_ = Eigen::VectorXd::Zero(state_dim());
  x_(idx.X()) = center_x;
  x_(idx.VX()) = 0.0;
  x_(idx.Y()) = center_y;
  x_(idx.VY()) = 0.0;
  x_(idx.Z()) = center_z;
  x_(idx.VZ()) = 0.0;
  x_(idx.DELTA()) = delta;
  x_(idx.DELTA_RATE()) = 0.0;
  x_(idx.R1()) = r1;
  x_(idx.R2()) = r2;
  x_(idx.DZA()) = dza;

  P_ = motion_model_->get_initial_covariance();
  initialized_ = true;
}

/* ================================================================ */
/*  Observation models                                               */
/* ================================================================ */

Eigen::VectorXd DualRadiusSpinUKF::observation_model(
    const Eigen::VectorXd &x, const std::string &r_type,
    const std::string &armor_layer, double panel_angle) const {
  auto idx = state_idx_;
  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double z_mean = x(idx.Z());
  double delta = x(idx.DELTA());
  double d_za = x(idx.DZA());

  double radius = (r_type == "r1") ? x(idx.R1()) : x(idx.R2());

  double center_yaw = compose_yaw(k_, delta);
  double armor_yaw = normalize_angle(center_yaw + panel_angle);

  double x_obs = x_c + radius * std::cos(armor_yaw);
  double y_obs = y_c + radius * std::sin(armor_yaw);

  double layer_offset = (armor_layer == "upper") ? d_za : -d_za;
  double z_obs = z_mean + layer_offset;

  Eigen::Vector4d z;
  z << x_obs, y_obs, z_obs, center_yaw;
  return z;
}

Eigen::VectorXd DualRadiusSpinUKF::observation_model_geometry(
    const Eigen::VectorXd &x) const {
  auto idx = state_idx_;
  Eigen::VectorXd z(6);
  z << x(idx.X()), x(idx.Y()), x(idx.Z()), x(idx.R1()), x(idx.R2()),
      x(idx.DZA());
  return z;
}

/* ================================================================ */
/*  Predict                                                          */
/* ================================================================ */

void DualRadiusSpinUKF::predict(std::optional<double> dt_opt) {
  if (!initialized_) return;

  // Reset maneuver cache; will be filled by the subsequent update() call
  last_update_type_ = 0;
  last_nis_         = -1.0;

  double dt = dt_opt.value_or(dt_);
  Q_ = motion_model_->build_Q(dt);

  Eigen::MatrixXd sigma_pts = generate_sigma_points(x_, P_);
  int n_sigma = sigma_pts.rows();

  // Propagate sigma points
  Eigen::MatrixXd sigma_pred(n_sigma, state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    sigma_pred.row(i) = motion_model_->predict(sigma_pts.row(i).transpose(), dt);
  }

  auto [Wm, Wc] = get_sigma_weights();

  // Predicted mean
  x_ = Eigen::VectorXd::Zero(state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    x_ += Wm(i) * sigma_pred.row(i).transpose();
  }

  // Predicted covariance with special delta-angle diff
  int delta_idx = state_idx_.DELTA();
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(state_dim(), state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd diff = sigma_pred.row(i).transpose() - x_;
    diff(delta_idx) =
        delta_angle_diff(sigma_pred(i, delta_idx), x_(delta_idx));
    P_pred += Wc(i) * diff * diff.transpose();
  }
  P_pred += Q_;

  P_ = P_pred;
  apply_constraints();
  ensure_covariance_valid();
}

/* ================================================================ */
/*  Update dispatch                                                  */
/* ================================================================ */

bool DualRadiusSpinUKF::update(
    const std::vector<ObservationData> &observations,
    const std::vector<std::string> &r_types,
    const std::vector<std::string> &armor_layers, double height_confidence,
    double position_confidence, double panel_angle) {
  if (!initialized_ || observations.empty()) return false;

  if (observations.size() == 1) {
    std::string rt = r_types.empty() ? "r1" : r_types[0];
    std::string layer = armor_layers.empty() ? "" : armor_layers[0];
    return update_single(observations[0], rt, layer, height_confidence,
                         position_confidence, panel_angle);
  } else {
    std::string rt1 = r_types.size() > 0 ? r_types[0] : "r1";
    std::string rt2 = r_types.size() > 1 ? r_types[1] : "r2";
    std::string l1 = armor_layers.size() > 0 ? armor_layers[0] : "";
    std::string l2 = armor_layers.size() > 1 ? armor_layers[1] : "";
    return update_dual(observations[0], observations[1], rt1, rt2, l1, l2,
                       height_confidence);
  }
}

/* ================================================================ */
/*  Single-observation update                                        */
/* ================================================================ */

bool DualRadiusSpinUKF::update_single(
    const ObservationData &obs, const std::string &r_type,
    const std::string &armor_layer_in, double height_confidence,
    double position_confidence, double panel_angle) {
  auto idx = state_idx_;

  // Mode selection
  double center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  double current_delta = x_(idx.DELTA());
  int best_k = select_best_k_from_center_yaw(current_delta, center_yaw_obs, k_);

  if (last_k_.has_value() && best_k != last_k_.value()) mode_switches_++;
  k_ = best_k;
  last_k_ = k_;

  // Build observation vector (center_yaw for yaw component)
  center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  // Infer layer
  std::string armor_layer =
      armor_layer_in.empty() ? infer_armor_layer(obs.z) : armor_layer_in;

  // Observation noise R — 来源选择（方向二）
  Eigen::Matrix4d R;

  if (config_.ukf.r_source == "PNP_COVARIANCE" && obs.pos_covariance.has_value() && obs.yaw_variance.has_value()) {
    // 从 PnP 协方差传播结果构建 R
    const Eigen::Matrix3d &cov_xyz = obs.pos_covariance.value();
    double cov_yaw = obs.yaw_variance.value();

    // 对协方差施加最小限幅，防止数值退化
    constexpr double min_pos_var = 1e-6;
    constexpr double min_yaw_var = 1e-6;

    R = Eigen::Vector4d(
        std::max(cov_xyz(0,0), min_pos_var),
        std::max(cov_xyz(1,1), min_pos_var),
        std::max(cov_xyz(2,2), min_pos_var),
        std::max(cov_yaw * config_.ukf.pnp_cov_yaw_inflation, min_yaw_var)
    ).asDiagonal();

    // 协方差传播可能低估了极端角度观测的不确定性，保留 height/position 置信度缩放
    if (height_confidence < 0.3) R(2, 2) *= 100.0;
    if (position_confidence < 0.8) {
      double scale = 100.0 / std::max(position_confidence, 0.1);
      R(0, 0) *= scale;
      R(1, 1) *= scale;
    }
  } else {
    // CONFIG 模式（原行为）：使用 YAML 静态噪声参数
    double np_ = config_.ukf.obs_noise_pos;
    double ny = config_.ukf.obs_noise_yaw;
    R = Eigen::Vector4d(np_ * np_, np_ * np_, np_ * np_, ny * ny).asDiagonal();

    if (height_confidence < 0.3) R(2, 2) *= 100.0;
    if (position_confidence < 0.8) {
      double scale = 100.0 / std::max(position_confidence, 0.1);
      R(0, 0) *= scale;
      R(1, 1) *= scale;
    }
  }

  // Sigma points
  Eigen::MatrixXd sigma_pts = generate_sigma_points(x_, P_);
  auto [Wm, Wc] = get_sigma_weights();
  int n_sigma = sigma_pts.rows();

  // Predicted observations
  Eigen::MatrixXd z_pred_pts(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        observation_model(sigma_pts.row(i).transpose(), r_type, armor_layer,
                          panel_angle)
            .transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }

  // Innovation
  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = normalize_angle(innov(3));

  // Innovation gating
  if (config_.ukf.enable_innovation_gating) {
    if (!check_innovation_gate(innov, z_pred_pts, z_pred, R, Wc)) return false;
  }

  // Cross-covariance and observation covariance
  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = normalize_angle(diff_z(i, 3));
  }

  Eigen::Matrix4d Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(state_dim(), 4);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - x_;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  // Kalman gain
  auto K_opt = compute_kalman_gain(Pxz, Pzz);
  if (!K_opt) return false;
  Eigen::MatrixXd K = K_opt.value();

  // 结构参数更新控制（方向一：BINARY vs SOFT）
  if (config_.ukf.freeze_mode == "SOFT") {
    double delta = x_(idx.DELTA());
    double w = weight_from_delta_angle(delta);
    K.row(idx.R1()) *= w;
    K.row(idx.R2()) *= w;
    K.row(idx.DZA()) *= w;
  } else {
    // BINARY 模式（原行为）：单观测时几何参数行完全置零
    K.row(idx.R1()).setZero();
    K.row(idx.R2()).setZero();
    K.row(idx.DZA()).setZero();
  }

  // Apply update
  apply_kalman_update(K, innov, Pzz);

  // Cache maneuver detection metrics (single-obs, 4D: [x, y, z, yaw])
  last_innov_xyz_   = innov.head<3>();
  last_innov_yaw_   = innov(3);
  last_nis_         = innov.dot(Pzz.inverse() * innov);
  last_update_type_ = 1;

  handle_mode_switch();
  apply_constraints();
  return true;
}

/* ================================================================ */
/*  Dual-observation update                                          */
/* ================================================================ */

bool DualRadiusSpinUKF::update_dual(
    const ObservationData &obs1, const ObservationData &obs2,
    const std::string &r_type_1, const std::string &r_type_2,
    const std::string &layer_1_in, const std::string &layer_2_in,
    double height_confidence) {
  auto idx = state_idx_;

  // Infer layers from z difference
  std::string layer_1 = layer_1_in;
  std::string layer_2 = layer_2_in;
  double z_diff = std::abs(obs1.z - obs2.z);
  if (z_diff > 0.01) {
    if (obs1.z > obs2.z) {
      layer_1 = "upper";
      layer_2 = "lower";
    } else {
      layer_1 = "lower";
      layer_2 = "upper";
    }
  }

  // Ray-intersection to find robot center
  double yaw1_to_center = normalize_angle(obs1.yaw + M_PI);
  double yaw2_to_center = normalize_angle(obs2.yaw + M_PI);

  double c1 = std::cos(yaw1_to_center), s1 = std::sin(yaw1_to_center);
  double c2 = std::cos(yaw2_to_center), s2 = std::sin(yaw2_to_center);

  Eigen::Matrix2d A;
  A << c1, -c2, s1, -s2;
  Eigen::Vector2d b;
  b << obs2.x - obs1.x, obs2.y - obs1.y;

  double det = A.determinant();
  if (std::abs(det) < 1e-10) {
    fprintf(stderr, "[update_dual] parallel rays (det=%.6f)\n", det);
    return false;
  }

  Eigen::Vector2d t_params = A.inverse() * b;
  if (t_params(0) < 0 || t_params(1) < 0) {
    fprintf(stderr, "[update_dual] invalid intersection t1=%.4f t2=%.4f "  \
      "yaw1=%.4f yaw2=%.4f "  \
      "obs1=(%.3f,%.3f) obs2=(%.3f,%.3f)\n",
      t_params(0), t_params(1),
      obs1.yaw, obs2.yaw,
      obs1.x, obs1.y, obs2.x, obs2.y);
    return false;
  }

  double x_center = obs1.x + t_params(0) * c1;
  double y_center = obs1.y + t_params(0) * s1;

  double r_to_1 = std::hypot(obs1.x - x_center, obs1.y - y_center);
  double r_to_2 = std::hypot(obs2.x - x_center, obs2.y - y_center);

  double r1_est, r2_est;
  if (r_type_1 == "r1") {
    r1_est = r_to_1;
    r2_est = (r_type_2 == "r2") ? r_to_2 : x_(idx.R2());
  } else {
    r2_est = r_to_1;
    r1_est = (r_type_2 == "r1") ? r_to_2 : x_(idx.R1());
  }

  // Height parameters
  bool different_layers = (layer_1 != layer_2);
  double dza_est, z_est, dza_noise_factor, z_noise_factor;
  if (different_layers) {
    dza_est = std::abs(obs1.z - obs2.z) / 2.0;
    z_est = (obs1.z + obs2.z) / 2.0;
    dza_noise_factor = 1.5;
    z_noise_factor = 1.0;
  } else {
    dza_est = x_(idx.DZA());
    double z_mean_obs = (obs1.z + obs2.z) / 2.0;
    z_est = (layer_1 == "upper") ? z_mean_obs - dza_est : z_mean_obs + dza_est;
    dza_noise_factor = 100.0;
    z_noise_factor = 5.0;
  }

  if (height_confidence < 0.3) dza_noise_factor *= 100.0;

  // Geometry observation vector [xc, yc, z, r1, r2, dza]
  Eigen::VectorXd z_geometry(6);
  z_geometry << x_center, y_center, z_est, r1_est, r2_est, dza_est;

  fprintf(stderr, "[update_dual] center=(%.3f,%.3f) r1_est=%.4f r2_est=%.4f dza_est=%.4f "
    "x_r1=%.4f x_r2=%.4f x_dza=%.4f layers=(%s,%s) h_conf=%.3f\n",
    x_center, y_center, r1_est, r2_est, dza_est,
    x_(idx.R1()), x_(idx.R2()), x_(idx.DZA()),
    layer_1.c_str(), layer_2.c_str(), height_confidence);

  // Geometry noise — 来源选择（方向二）
  Eigen::VectorXd r_diag(6);

  if (config_.ukf.r_source == "PNP_COVARIANCE" &&
      obs1.pos_covariance.has_value() && obs2.pos_covariance.has_value()) {
    // 保守策略：取两个观测中较大的对角方差
    const auto &c1 = obs1.pos_covariance.value();
    const auto &c2 = obs2.pos_covariance.value();
    double var_x  = std::max(c1(0,0), c2(0,0));
    double var_y  = std::max(c1(1,1), c2(1,1));
    double var_z  = std::max(c1(2,2), c2(2,2));
    constexpr double min_var = 1e-6;
    r_diag << std::max(var_x, min_var),
              std::max(var_y, min_var),
              std::max(var_z, min_var) * z_noise_factor * z_noise_factor,
              config_.ukf.dual_obs_geometry_noise_scale * config_.ukf.dual_obs_geometry_noise_scale,
              config_.ukf.dual_obs_geometry_noise_scale * config_.ukf.dual_obs_geometry_noise_scale,
              config_.ukf.dual_obs_geometry_noise_scale * config_.ukf.dual_obs_geometry_noise_scale
                   * dza_noise_factor * dza_noise_factor;
  } else {
    // CONFIG 模式（原行为）
    double pn = config_.ukf.dual_obs_noise_pos;
    r_diag << pn * pn, pn * pn, pn * pn * z_noise_factor * z_noise_factor,
        pn * pn, pn * pn, pn * pn * dza_noise_factor * dza_noise_factor;
  }

  Eigen::MatrixXd R_geo = r_diag.asDiagonal();

  // UKF update
  Eigen::MatrixXd sigma_pts = generate_sigma_points(x_, P_);
  auto [Wm, Wc] = get_sigma_weights();
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, 6);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        observation_model_geometry(sigma_pts.row(i).transpose()).transpose();
  }

  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(6);
  for (int i = 0; i < n_sigma; ++i) z_pred += Wm(i) * z_pred_pts.row(i).transpose();

  Eigen::VectorXd innov = z_geometry - z_pred;

  Eigen::MatrixXd Pzz = R_geo;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(state_dim(), 6);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd dz = z_pred_pts.row(i).transpose() - z_pred;
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - x_;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  auto K_opt = compute_kalman_gain(Pxz, Pzz);
  if (!K_opt) return false;
  Eigen::MatrixXd K = K_opt.value();

  if (height_confidence < 0.3) K.row(idx.DZA()).setZero();

  apply_kalman_update(K, innov, Pzz);

  // Cache maneuver detection metrics (dual-obs, 6D geometry: [xc, yc, zc, r1, r2, dza])
  last_innov_xyz_   = innov.head<3>();
  last_innov_yaw_   = 0.0;
  last_nis_         = innov.dot(Pzz.inverse() * innov);
  last_update_type_ = 2;

  apply_constraints();
  return true;
}

/* ================================================================ */
/*  Helpers                                                          */
/* ================================================================ */

std::string DualRadiusSpinUKF::infer_armor_layer(double z_obs) const {
  if (!is_dza_converged()) return "lower";
  auto idx = state_idx_;
  double z_mean = x_(idx.Z());
  double d_za = x_(idx.DZA());
  return (std::abs(z_obs - (z_mean + d_za)) < std::abs(z_obs - (z_mean - d_za)))
             ? "upper"
             : "lower";
}

bool DualRadiusSpinUKF::is_dza_converged(double var_threshold,
                                         double min_value) const {
  if (!initialized_) return false;
  auto idx = state_idx_;
  double dza_var = P_(idx.DZA(), idx.DZA());
  double dza_val = std::abs(x_(idx.DZA()));
  return (dza_var < var_threshold) && (dza_val > min_value);
}

void DualRadiusSpinUKF::handle_mode_switch() {
  auto idx = state_idx_;
  double delta = x_(idx.DELTA());
  if (delta > M_PI / 2.0) {
    x_(idx.DELTA()) = delta - M_PI;
    k_ = 1 - k_;
    if (last_k_.has_value() && k_ != last_k_.value()) mode_switches_++;
    last_k_ = k_;
  } else if (delta < -M_PI / 2.0) {
    x_(idx.DELTA()) = delta + M_PI;
    k_ = 1 - k_;
    if (last_k_.has_value() && k_ != last_k_.value()) mode_switches_++;
    last_k_ = k_;
  }
}

void DualRadiusSpinUKF::apply_constraints() {
  auto idx = state_idx_;
  x_ = apply_state_constraints(x_, idx.R1(), idx.R2(), idx.DZA(),
                                config_.constraints.min_radius,
                                config_.constraints.max_radius, 0.0,
                                config_.constraints.max_dz);
}

double DualRadiusSpinUKF::get_yaw() const {
  return compose_yaw(k_, x_(state_idx_.DELTA()));
}

double DualRadiusSpinUKF::get_delta() const {
  return x_(state_idx_.DELTA());
}

Eigen::Vector3d DualRadiusSpinUKF::get_center_position() const {
  auto idx = state_idx_;
  return Eigen::Vector3d(x_(idx.X()), x_(idx.Y()), x_(idx.Z()));
}

std::pair<double, double> DualRadiusSpinUKF::get_radii() const {
  auto idx = state_idx_;
  return {x_(idx.R1()), x_(idx.R2())};
}

double DualRadiusSpinUKF::get_dza() const {
  return x_(state_idx_.DZA());
}

/* ================================================================ */
/*  Panel mismatch correction                                        */
/* ================================================================ */

void DualRadiusSpinUKF::apply_panel_correction(double new_center_yaw) {
  if (!initialized_) return;
  auto idx = state_idx_;

  // 1. Swap R1 ↔ R2 in state vector (parity flipped → radii exchange)
  std::swap(x_(idx.R1()), x_(idx.R2()));

  // 2. Swap the corresponding rows AND columns in P
  //    This preserves the cross-correlations correctly.
  int r1 = idx.R1(), r2 = idx.R2();
  P_.row(r1).swap(P_.row(r2));
  P_.col(r1).swap(P_.col(r2));

  // 3. Recompute delta and k from the new center_yaw
  auto [new_k, new_delta] = decompose_yaw(new_center_yaw);
  k_      = new_k;
  last_k_ = k_;
  x_(idx.DELTA()) = new_delta;

  // 4. Inflate covariances to let UKF re-converge quickly:
  //      DELTA  ×10
  //      R1,R2  ×5
  //      DZA    ×10
  constexpr double kDelta = 10.0;
  constexpr double kR     = 5.0;
  constexpr double kDza   = 10.0;

  int d_idx   = idx.DELTA();
  int dza_idx = idx.DZA();

  P_(d_idx, d_idx)     *= kDelta;
  P_(r1, r1)           *= kR;
  P_(r2, r2)           *= kR;
  P_(dza_idx, dza_idx) *= kDza;

  // Symmetry + positive-definiteness guard
  P_ = 0.5 * (P_ + P_.transpose());
  ensure_covariance_valid();

  fprintf(stderr,
      "[DualRadiusSpinUKF] apply_panel_correction: "
      "new_center_yaw=%.4f k=%d delta=%.4f r1=%.4f r2=%.4f dza=%.4f\n",
      new_center_yaw, k_, new_delta, x_(r1), x_(r2), x_(dza_idx));
}

bool DualRadiusSpinUKF::check_innovation_gate(
    const Eigen::VectorXd &innov, const Eigen::MatrixXd &z_pred_points,
    const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &R,
    const Eigen::VectorXd &Wc) const {
  int n_sigma = z_pred_points.rows();

  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_points.row(i) - z_pred.transpose();
    diff_z(i, 3) = normalize_angle(diff_z(i, 3));
  }

  Eigen::Matrix4d Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
  }

  double threshold = config_.ukf.innovation_gate_chi2_threshold;

  // Yaw chi2 (1-DOF)
  double chi2_yaw = (innov(3) * innov(3)) / Pzz(3, 3);
  bool yaw_pass = (chi2_yaw <= threshold);

  // Position chi2 (3-DOF)
  Eigen::Vector3d innov_pos = innov.head<3>();
  Eigen::Matrix3d Pzz_pos = Pzz.topLeftCorner<3, 3>();
  double det = Pzz_pos.determinant();
  if (std::abs(det) < 1e-12) return true;
  double chi2_pos = innov_pos.transpose() * Pzz_pos.inverse() * innov_pos;
  bool pos_pass = (chi2_pos <= threshold * 3.0);

  return yaw_pass || pos_pass;
}

double DualRadiusSpinUKF::weight_from_delta_angle(double delta) const {
  const double threshold_rad = config_.ukf.soft_freeze_threshold_deg * M_PI / 180.0;

  // |delta| 超过阈值 → 完全冻结
  if (std::abs(delta) >= threshold_rad) return 0.0;

  // w = cos²(π/2 * |delta| / threshold)，在 [0, threshold] 上从 1.0 平滑降至 0.0
  double ratio = std::abs(delta) / threshold_rad;
  double cos_val = std::cos(M_PI / 2.0 * ratio);
  return cos_val * cos_val;
}

}  // namespace fyt::auto_aim
