// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/filters/outpost_spin_ukf.hpp"

#include <cmath>
#include <stdexcept>

#include "max_entropy_tracker/filters/process_models/rotation.hpp"
#include "max_entropy_tracker/filters/process_models/translation.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

OutpostSpinUKF::OutpostSpinUKF(
    const UnifiedConfig &config, double dt,
    std::shared_ptr<CompositeProcessModel> process_model)
    : BaseUKF(config, dt),
      motion_model_(process_model ? std::move(process_model)
                                  : create_process_model(config)),
      state_idx_(motion_model_->layout()) {
  radius_ = std::max(0.05, config.outpost.radius);
  z_offsets_ = {
      config.outpost.z_offset_0,
      config.outpost.z_offset_1,
      config.outpost.z_offset_2,
  };

  const double raw_step = (config.outpost.panel_angle_step > 1e-6)
                              ? config.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  // Keep semantic contract consistent with OutpostArmorTracker:
  // id0=0deg, clockwise order 0->2->1 (CCW-positive: [0, +step, -step]).
  panel_angles_ = {0.0, step, -step};

  x_ = Eigen::VectorXd::Zero(state_dim());
  P_ = Eigen::MatrixXd::Identity(state_dim(), state_dim()) * 10.0;
  Q_ = motion_model_->build_Q(dt_);
  init_sigma_generator();

  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

int OutpostSpinUKF::state_dim() const { return motion_model_->state_dim(); }

std::shared_ptr<CompositeProcessModel> OutpostSpinUKF::create_process_model(
    const UnifiedConfig &cfg) const {
  TranslationConfig tc;
  tc.cv_process_noise_vel = cfg.motion.cv_process_noise_vel;
  tc.ca_process_noise_acc = cfg.motion.ca_process_noise_acc;
  tc.singer_alpha = cfg.outpost.singer_alpha > 1e-6
                        ? cfg.outpost.singer_alpha
                        : cfg.motion.singer_alpha;
  tc.singer_sigma = cfg.outpost.singer_sigma > 1e-6
                        ? cfg.outpost.singer_sigma
                        : cfg.motion.singer_sigma;

  RotationConfig rc;
  rc.cv_process_noise_rate = cfg.outpost.spin_process_noise_theta_rate > 1e-9
                                 ? cfg.outpost.spin_process_noise_theta_rate
                                 : cfg.spin.spin_process_noise_delta_rate;
  rc.ca_process_noise_acc = cfg.outpost.spin_process_noise_theta_acc > 1e-9
                                ? cfg.outpost.spin_process_noise_theta_acc
                                : cfg.spin.spin_process_noise_delta_acc;

  std::vector<std::shared_ptr<ProcessModelComponent>> components;
  components.push_back(create_translation_model(
      cfg.outpost.translation_model, tc, 3));
  components.push_back(create_rotation_model(cfg.outpost.rotation_model, rc));
  return std::make_shared<CompositeProcessModel>(std::move(components));
}

int OutpostSpinUKF::sanitize_panel_id(int panel_id) const {
  const int n = N_PANELS;
  int pid = panel_id % n;
  if (pid < 0) pid += n;
  return pid;
}

void OutpostSpinUKF::initialize(const std::vector<ObservationData> &observations,
                                double /*r1*/, double /*r2*/, double /*dza*/,
                                int panel_id) {
  if (observations.empty()) {
    throw std::invalid_argument("OutpostSpinUKF requires at least one observation");
  }

  const ObservationData &obs = observations.front();
  int pid = panel_id >= 0 ? panel_id : obs.panel_id.value_or(0);
  pid = sanitize_panel_id(pid);
  selected_panel_id_ = pid;

  const double center_yaw = normalize_angle(obs.yaw - panel_angles_[pid]);

  const auto idx = state_idx_;
  x_ = Eigen::VectorXd::Zero(state_dim());
  x_(idx.X()) = obs.x - radius_ * std::cos(obs.yaw);
  x_(idx.VX()) = 0.0;
  x_(idx.Y()) = obs.y - radius_ * std::sin(obs.yaw);
  x_(idx.VY()) = 0.0;
  x_(idx.Z()) = obs.z - z_offsets_[pid];
  x_(idx.VZ()) = 0.0;
  x_(idx.DELTA()) = center_yaw;
  x_(idx.DELTA_RATE()) = 0.0;

  if (idx.has("AX")) {
    x_(idx.AX()) = 0.0;
    x_(idx.AY()) = 0.0;
    x_(idx.AZ()) = 0.0;
  }
  if (idx.has("DELTA_ACC")) {
    x_(idx.get("DELTA_ACC")) = 0.0;
  }

  P_ = motion_model_->get_initial_covariance();
  initialized_ = true;
  apply_angle_constraints();
}

void OutpostSpinUKF::apply_angle_constraints() {
  if (!initialized_) return;
  x_(state_idx_.DELTA()) = normalize_angle(x_(state_idx_.DELTA()));
}

void OutpostSpinUKF::predict(std::optional<double> dt_opt) {
  if (!initialized_) return;

  last_update_type_ = 0;
  last_nis_ = -1.0;

  const double dt = dt_opt.value_or(dt_);
  Q_ = motion_model_->build_Q(dt);

  Eigen::MatrixXd sigma_pts = generate_sigma_points(x_, P_);
  const int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd sigma_pred(n_sigma, state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    sigma_pred.row(i) = motion_model_->predict(
        sigma_pts.row(i).transpose(), dt).transpose();
  }

  auto [Wm, Wc] = get_sigma_weights();

  x_ = Eigen::VectorXd::Zero(state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    x_ += Wm(i) * sigma_pred.row(i).transpose();
  }
  apply_angle_constraints();

  const int delta_idx = state_idx_.DELTA();
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(state_dim(), state_dim());
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd diff = sigma_pred.row(i).transpose() - x_;
    diff(delta_idx) = delta_angle_diff(sigma_pred(i, delta_idx), x_(delta_idx));
    P_pred += Wc(i) * diff * diff.transpose();
  }
  P_ = P_pred + Q_;

  ensure_covariance_valid();
}

Eigen::VectorXd OutpostSpinUKF::observation_model(const Eigen::VectorXd &x,
                                                   int panel_id) const {
  const int pid = sanitize_panel_id(panel_id);
  const auto idx = state_idx_;

  const double center_x = x(idx.X());
  const double center_y = x(idx.Y());
  const double center_z = x(idx.Z());
  const double center_yaw = x(idx.DELTA());

  const double armor_yaw = normalize_angle(center_yaw + panel_angles_[pid]);

  Eigen::Vector4d z;
  z << center_x + radius_ * std::cos(armor_yaw),
      center_y + radius_ * std::sin(armor_yaw),
      center_z + z_offsets_[pid],
      armor_yaw;
  return z;
}

bool OutpostSpinUKF::update_with_panel(const ObservationData &obs, int panel_id,
                                       double position_confidence) {
  if (!initialized_) return false;

  const int pid = sanitize_panel_id(panel_id);
  selected_panel_id_ = pid;

  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, obs.yaw;

  const double np = config_.ukf.obs_noise_pos;
  const double ny = config_.ukf.obs_noise_yaw;
  Eigen::Matrix4d R =
      Eigen::Vector4d(np * np, np * np, np * np, ny * ny).asDiagonal();

  if (position_confidence < 0.8) {
    const double scale = 100.0 / std::max(position_confidence, 0.1);
    R(0, 0) *= scale;
    R(1, 1) *= scale;
  }

  Eigen::MatrixXd sigma_pts = generate_sigma_points(x_, P_);
  auto [Wm, Wc] = get_sigma_weights();
  const int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, obs_dim());
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        observation_model(sigma_pts.row(i).transpose(), pid).transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (int i = 0; i < n_sigma; ++i) {
    z_pred.head<3>() += Wm(i) * z_pred_pts.row(i).head<3>().transpose();
    sin_sum += Wm(i) * std::sin(z_pred_pts(i, 3));
    cos_sum += Wm(i) * std::cos(z_pred_pts(i, 3));
  }
  z_pred(3) = std::atan2(sin_sum, cos_sum);

  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = normalize_angle(innov(3));

  const int delta_idx = state_idx_.DELTA();
  Eigen::Matrix4d Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(state_dim(), obs_dim());
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = z_pred_pts.row(i).transpose() - z_pred;
    dz(3) = normalize_angle(dz(3));
    Pzz += Wc(i) * dz * dz.transpose();

    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - x_;
    dx(delta_idx) = delta_angle_diff(sigma_pts(i, delta_idx), x_(delta_idx));
    Pxz += Wc(i) * dx * dz.transpose();
  }

  auto K_opt = compute_kalman_gain(Pxz, Pzz);
  if (!K_opt.has_value()) return false;

  apply_kalman_update(K_opt.value(), innov, Pzz);
  apply_angle_constraints();

  last_innov_xyz_ = innov.head<3>();
  last_innov_yaw_ = innov(3);
  last_nis_ = innov.dot(Pzz.inverse() * innov);
  last_update_type_ = 1;
  return true;
}

bool OutpostSpinUKF::update(const std::vector<ObservationData> &observations,
                            const std::vector<std::string> & /*r_types*/,
                            const std::vector<std::string> & /*armor_layers*/,
                            double /*height_confidence*/,
                            double position_confidence,
                            double /*panel_angle*/) {
  if (!initialized_ || observations.empty()) return false;

  if (observations.size() == 1) {
    const ObservationData &obs = observations.front();
    const int pid = obs.panel_id.value_or(selected_panel_id_);
    return update_with_panel(obs, pid, position_confidence);
  }

  bool ok_any = false;
  int ok_count = 0;
  for (size_t i = 0; i < std::min<size_t>(2, observations.size()); ++i) {
    const ObservationData &obs = observations[i];
    const int pid = obs.panel_id.value_or(selected_panel_id_ + static_cast<int>(i));
    if (update_with_panel(obs, pid, position_confidence)) {
      ok_any = true;
      ++ok_count;
    }
  }
  if (ok_count >= 2) {
    last_update_type_ = 2;
  }
  return ok_any;
}

Eigen::Vector3d OutpostSpinUKF::get_center_position() const {
  const auto idx = state_idx_;
  return {x_(idx.X()), x_(idx.Y()), x_(idx.Z())};
}

std::pair<double, double> OutpostSpinUKF::get_radii() const {
  return {radius_, radius_};
}

double OutpostSpinUKF::get_yaw() const { return x_(state_idx_.DELTA()); }

double OutpostSpinUKF::get_delta() const { return x_(state_idx_.DELTA()); }

}  // namespace fyt::auto_aim
