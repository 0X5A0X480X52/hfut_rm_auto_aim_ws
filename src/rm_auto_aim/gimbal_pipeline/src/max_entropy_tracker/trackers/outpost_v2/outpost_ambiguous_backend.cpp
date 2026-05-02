// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::outpost_v2 {

OutpostAmbiguousBackend::OutpostAmbiguousBackend(const UnifiedConfig &cfg)
    : cfg_(cfg), filter_(cfg, cfg.dt) {
  radius_ = std::max(0.05, cfg_.outpost.radius);
  z_offsets_ = {
      cfg_.outpost.z_offset_0,
      cfg_.outpost.z_offset_1,
      cfg_.outpost.z_offset_2,
  };
  const double raw_step = (cfg_.outpost.panel_angle_step > 1e-6)
                              ? cfg_.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  panel_angles_ = {0.0, step, -step};
  state_.radius = radius_;
}

int OutpostAmbiguousBackend::sanitize_panel_id(int panel_id) const {
  int v = panel_id % 3;
  if (v < 0) v += 3;
  return v;
}

void OutpostAmbiguousBackend::reset(const ObservationData &obs, int panel_id) {
  current_panel_id_ = sanitize_panel_id(panel_id);
  filter_.initialize(obs);
  refresh_center_snapshot();
}

void OutpostAmbiguousBackend::predict(double dt) {
  if (!filter_.initialized()) return;
  filter_.predict(dt);
  refresh_center_snapshot();
}

bool OutpostAmbiguousBackend::update(const ObservationData &obs,
                                     const BackendUpdateHint &hint) {
  if (!filter_.initialized()) {
    reset(obs, hint.panel_id >= 0 ? hint.panel_id : 0);
    return true;
  }

  if (hint.panel_id >= 0) {
    current_panel_id_ = sanitize_panel_id(hint.panel_id);
  }
  const double pos_conf = std::clamp(hint.position_confidence, 0.05, 1.0);
  filter_.update(obs, pos_conf, pos_conf);
  refresh_center_snapshot();
  return true;
}

BackendStateSnapshot OutpostAmbiguousBackend::snapshot() const { return state_; }

const AmbiguousArmorSnapshot &OutpostAmbiguousBackend::ambiguous_snapshot() const {
  return armor_snap_;
}

void OutpostAmbiguousBackend::refresh_center_snapshot() {
  if (!filter_.initialized()) return;

  // 1. Cache raw single-armor state (authoritative in AMBIGUOUS mode).
  armor_snap_.armor_pos = filter_.armor_position();
  armor_snap_.armor_vel = filter_.armor_velocity();
  armor_snap_.armor_yaw = filter_.armor_yaw();
  armor_snap_.armor_yaw_rate = filter_.armor_yaw_rate();
  armor_snap_.panel_id = sanitize_panel_id(current_panel_id_);

  // 2. Compatibility: back-project to center-centric snapshot.
  const int pid = armor_snap_.panel_id;
  state_.panel_id = pid;

  state_.center_pos.x() = armor_snap_.armor_pos.x() - radius_ * std::cos(armor_snap_.armor_yaw);
  state_.center_pos.y() = armor_snap_.armor_pos.y() - radius_ * std::sin(armor_snap_.armor_yaw);
  state_.center_pos.z() = armor_snap_.armor_pos.z() - z_offsets_[pid];

  state_.center_yaw = normalize_angle(armor_snap_.armor_yaw - panel_angles_[pid]);
  state_.yaw_rate = armor_snap_.armor_yaw_rate;

  const Eigen::Vector3d tangential(
      -armor_snap_.armor_yaw_rate * radius_ * std::sin(armor_snap_.armor_yaw),
       armor_snap_.armor_yaw_rate * radius_ * std::cos(armor_snap_.armor_yaw), 0.0);
  state_.center_vel = armor_snap_.armor_vel - tangential;
}

}  // namespace fyt::auto_aim::outpost_v2
