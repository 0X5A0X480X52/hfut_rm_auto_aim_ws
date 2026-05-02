// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/filters/outpost_ambiguous_kf.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

namespace {

double sq(double x) { return x * x; }

}  // namespace

OutpostAmbiguousKF::OutpostAmbiguousKF(const UnifiedConfig & config, double dt)
    : config_(config) {
  dt_ = std::clamp(dt, 1e-3, 0.5);
  r_pos_base_ = std::max(1e-4, config_.ukf.obs_noise_pos);
  r_yaw_base_ = std::max(1e-4, config_.ukf.obs_noise_yaw);

  // Use existing config as acceleration spectral densities.
  q_pos_acc_ = std::max(1e-6, config_.motion.ca_process_noise_acc);
  q_yaw_acc_ = std::max(1e-6, config_.spin.spin_process_noise_delta_acc);

  H_pos_.setZero();
  H_pos_(0, 0) = 1.0;
  H_pos_(1, 2) = 1.0;
  H_pos_(2, 4) = 1.0;
  H_yaw_ << 1.0, 0.0;

  rebuild_motion_model(dt_);
}

double OutpostAmbiguousKF::clamp_conf(double c) {
  return std::clamp(c, 0.05, 1.0);
}

void OutpostAmbiguousKF::rebuild_motion_model(double dt) {
  dt_ = std::clamp(dt, 1e-3, 0.5);
  const double dt2 = dt_ * dt_;
  const double dt3 = dt2 * dt_;
  const double dt4 = dt3 * dt_;

  F_pos_.setIdentity();
  F_pos_(0, 1) = dt_;
  F_pos_(2, 3) = dt_;
  F_pos_(4, 5) = dt_;

  const Eigen::Matrix2d q_block =
      (Eigen::Matrix2d() << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2).finished();
  Q_pos_.setZero();
  Q_pos_.block<2, 2>(0, 0) = q_pos_acc_ * q_block;
  Q_pos_.block<2, 2>(2, 2) = q_pos_acc_ * q_block;
  Q_pos_.block<2, 2>(4, 4) = q_pos_acc_ * q_block;

  F_yaw_.setIdentity();
  F_yaw_(0, 1) = dt_;
  Q_yaw_ = q_yaw_acc_ * q_block;
}

double OutpostAmbiguousKF::unwrap_yaw(double yaw_meas) {
  if (!has_unwrap_ref_) {
    yaw_unwrap_ref_ = yaw_meas;
    has_unwrap_ref_ = true;
    return yaw_unwrap_ref_;
  }
  yaw_unwrap_ref_ += normalize_angle(yaw_meas - normalize_angle(yaw_unwrap_ref_));
  return yaw_unwrap_ref_;
}

void OutpostAmbiguousKF::initialize(const ObservationData & obs) {
  has_unwrap_ref_ = false;
  const double yaw_u = unwrap_yaw(obs.yaw);

  x_pos_.setZero();
  x_pos_(0) = obs.x;
  x_pos_(2) = obs.y;
  x_pos_(4) = obs.z;
  P_pos_.setIdentity();
  P_pos_(0, 0) = 0.05;
  P_pos_(2, 2) = 0.05;
  P_pos_(4, 4) = 0.05;
  P_pos_(1, 1) = 1.0;
  P_pos_(3, 3) = 1.0;
  P_pos_(5, 5) = 1.0;

  x_yaw_.setZero();
  x_yaw_(0) = yaw_u;
  x_yaw_(1) = 0.0;
  P_yaw_.setIdentity();
  P_yaw_(0, 0) = 0.10;
  P_yaw_(1, 1) = 1.0;

  initialized_ = true;
}

void OutpostAmbiguousKF::predict(double dt) {
  if (!initialized_) return;
  rebuild_motion_model(dt);

  x_pos_ = F_pos_ * x_pos_;
  P_pos_ = F_pos_ * P_pos_ * F_pos_.transpose() + Q_pos_;

  x_yaw_ = F_yaw_ * x_yaw_;
  P_yaw_ = F_yaw_ * P_yaw_ * F_yaw_.transpose() + Q_yaw_;
}

void OutpostAmbiguousKF::update(const ObservationData & obs,
                                double position_confidence,
                                double yaw_confidence) {
  if (!initialized_) {
    initialize(obs);
    return;
  }

  // Position update
  const double pos_c = clamp_conf(position_confidence);
  const double pos_scale = 1.0 / pos_c;
  Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * sq(r_pos_base_) * pos_scale;
  Eigen::Vector3d z_pos(obs.x, obs.y, obs.z);

  const Eigen::Vector3d y_pos = z_pos - H_pos_ * x_pos_;
  const Eigen::Matrix3d S_pos = H_pos_ * P_pos_ * H_pos_.transpose() + R_pos;
  const Eigen::Matrix<double, 6, 3> K_pos =
      P_pos_ * H_pos_.transpose() * S_pos.inverse();
  x_pos_ = x_pos_ + K_pos * y_pos;
  P_pos_ = (Eigen::Matrix<double, 6, 6>::Identity() - K_pos * H_pos_) * P_pos_;

  // Yaw update (unwrapped domain)
  const double yaw_c = clamp_conf(yaw_confidence);
  const double yaw_scale = 1.0 / yaw_c;
  const double z_yaw = unwrap_yaw(obs.yaw);
  const double y_yaw = z_yaw - H_yaw_ * x_yaw_;
  const double S_yaw = (H_yaw_ * P_yaw_ * H_yaw_.transpose())(0, 0) +
                       sq(r_yaw_base_) * yaw_scale;
  const Eigen::Vector2d K_yaw = P_yaw_ * H_yaw_.transpose() / S_yaw;
  x_yaw_ = x_yaw_ + K_yaw * y_yaw;
  P_yaw_ = (Eigen::Matrix2d::Identity() - K_yaw * H_yaw_) * P_yaw_;
}

Eigen::Vector3d OutpostAmbiguousKF::armor_position() const {
  return Eigen::Vector3d(x_pos_(0), x_pos_(2), x_pos_(4));
}

Eigen::Vector3d OutpostAmbiguousKF::armor_velocity() const {
  return Eigen::Vector3d(x_pos_(1), x_pos_(3), x_pos_(5));
}

double OutpostAmbiguousKF::armor_yaw() const {
  return normalize_angle(x_yaw_(0));
}

double OutpostAmbiguousKF::armor_yaw_rate() const {
  return x_yaw_(1);
}

}  // namespace fyt::auto_aim
