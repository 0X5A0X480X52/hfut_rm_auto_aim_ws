// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/filters/outpost_ambiguous_kf.hpp"
#include "max_entropy_tracker/filters/single_armor_imm_tracker.hpp"

namespace fyt::auto_aim {

AmbiguousSingleArmorFilterAdapter::AmbiguousSingleArmorFilterAdapter(
    const UnifiedConfig &config, double dt)
    : config_(config) {
  dt_ = std::clamp(dt, 1e-3, 0.5);
  use_imm_ = config_.outpost.ambiguous_backend_use_imm_adapter;

  // Legacy KF is always constructed as fallback.
  legacy_kf_ = std::make_unique<OutpostAmbiguousKF>(config_, dt_);

  // IMM tracker is constructed on-demand when the switch is on.
  if (use_imm_) {
    build_imm_config();
  }
}

AmbiguousSingleArmorFilterAdapter::~AmbiguousSingleArmorFilterAdapter() = default;

void AmbiguousSingleArmorFilterAdapter::build_imm_config() {
  kalman::SingleArmorIMMConfig imm_cfg;
  imm_cfg.dt = dt_;

  // Model selectivity: CV + CA always on; CS + CTRV optional
  imm_cfg.enable_cv   = true;
  imm_cfg.enable_ca   = true;
  imm_cfg.enable_cs   = true;
  imm_cfg.enable_ctrv = true;

  // Map UnifiedConfig noise parameters
  imm_cfg.q_cv     = std::max(0.01, config_.motion.cv_process_noise_vel);
  imm_cfg.q_ca     = std::max(0.01, config_.motion.ca_process_noise_acc);
  imm_cfg.q_z_vel  = std::max(0.01, config_.motion.ca_process_noise_acc);
  imm_cfg.q_yaw_rate = std::max(0.01, config_.spin.spin_process_noise_delta_acc);

  imm_cfg.r_pos_base = std::max(1e-4, config_.ukf.obs_noise_pos);
  imm_cfg.r_yaw_base = std::max(1e-4, config_.ukf.obs_noise_yaw);

  imm_cfg.p_stay   = 0.85;
  imm_cfg.p_switch = 0.05;

  imm_tracker_ = std::make_unique<kalman::SingleArmorIMMTracker>(imm_cfg);
}

void AmbiguousSingleArmorFilterAdapter::initialize(const ObservationData &obs) {
  legacy_kf_->initialize(obs);
  if (imm_tracker_) {
    imm_tracker_->initialize(
        Eigen::Vector3d(obs.x, obs.y, obs.z), obs.yaw);
  }
}

void AmbiguousSingleArmorFilterAdapter::predict(double dt) {
  legacy_kf_->predict(dt);
  if (imm_tracker_) {
    imm_tracker_->predict(dt);
  }
}

void AmbiguousSingleArmorFilterAdapter::update(const ObservationData &obs,
                                                double position_confidence,
                                                double yaw_confidence) {
  legacy_kf_->update(obs, position_confidence, yaw_confidence);
  if (imm_tracker_) {
    imm_tracker_->update(
        Eigen::Vector3d(obs.x, obs.y, obs.z),
        obs.yaw,
        position_confidence,
        yaw_confidence);
  }
}

bool AmbiguousSingleArmorFilterAdapter::initialized() const {
  if (imm_tracker_) {
    return imm_tracker_->initialized();
  }
  return legacy_kf_->initialized();
}

Eigen::Vector3d AmbiguousSingleArmorFilterAdapter::armor_position() const {
  if (use_imm_ && imm_tracker_ && imm_tracker_->initialized()) {
    return imm_tracker_->state().pos;
  }
  return legacy_kf_->armor_position();
}

Eigen::Vector3d AmbiguousSingleArmorFilterAdapter::armor_velocity() const {
  if (use_imm_ && imm_tracker_ && imm_tracker_->initialized()) {
    return imm_tracker_->state().vel;
  }
  return legacy_kf_->armor_velocity();
}

double AmbiguousSingleArmorFilterAdapter::armor_yaw() const {
  if (use_imm_ && imm_tracker_ && imm_tracker_->initialized()) {
    return imm_tracker_->state().yaw;
  }
  return legacy_kf_->armor_yaw();
}

double AmbiguousSingleArmorFilterAdapter::armor_yaw_rate() const {
  if (use_imm_ && imm_tracker_ && imm_tracker_->initialized()) {
    return imm_tracker_->state().yaw_rate;
  }
  return legacy_kf_->armor_yaw_rate();
}

}  // namespace fyt::auto_aim
