// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::norm4_v2 {

Norm4AmbiguousBackend::Norm4AmbiguousBackend(const UnifiedConfig &cfg)
    : cfg_(cfg), filter_(cfg, cfg.dt) {}

int Norm4AmbiguousBackend::sanitize_panel_id(int panel_id) const {
  int v = panel_id % 4;
  if (v < 0) v += 4;
  return v;
}

double Norm4AmbiguousBackend::panel_z_offset(int panel_id,
                                             binder::HeightLabel label) const {
  if (label == binder::HeightLabel::UPPER) return std::abs(dza_hint_);
  if (label == binder::HeightLabel::LOWER) return -std::abs(dza_hint_);
  return (sanitize_panel_id(panel_id) % 2 == 0) ? -std::abs(dza_hint_)
                                                : std::abs(dza_hint_);
}

void Norm4AmbiguousBackend::reset(const ObservationData &obs, int panel_id,
                                  double r1, double r2, double dza) {
  current_panel_id_ = sanitize_panel_id(panel_id);
  current_height_label_ = (current_panel_id_ % 2 == 0) ? binder::HeightLabel::LOWER
                                                        : binder::HeightLabel::UPPER;
  r1_hint_ = std::max(0.05, r1);
  r2_hint_ = std::max(0.05, r2);
  dza_hint_ = std::max(0.0, dza);
  filter_.initialize(obs);
  refresh_center_snapshot();
}

void Norm4AmbiguousBackend::predict(double dt) {
  if (!filter_.initialized()) return;
  filter_.predict(dt);
  refresh_center_snapshot();
}

bool Norm4AmbiguousBackend::update(const ObservationData &obs,
                                   const BackendUpdateHint &hint) {
  if (!filter_.initialized()) {
    reset(obs, hint.panel_id >= 0 ? hint.panel_id : 0, hint.r1_hint, hint.r2_hint,
          hint.dza_hint);
    return true;
  }

  if (hint.panel_id >= 0) {
    current_panel_id_ = sanitize_panel_id(hint.panel_id);
  }
  if (hint.height_label != binder::HeightLabel::UNKNOWN) {
    current_height_label_ = hint.height_label;
  }
  r1_hint_ = std::max(0.05, hint.r1_hint);
  r2_hint_ = std::max(0.05, hint.r2_hint);
  dza_hint_ = std::max(0.0, hint.dza_hint);

  const double pos_conf = std::clamp(hint.position_confidence, 0.05, 1.0);
  filter_.update(obs, pos_conf, pos_conf);
  refresh_center_snapshot();
  return true;
}

BackendStateSnapshot Norm4AmbiguousBackend::snapshot() const { return state_; }

const AmbiguousArmorSnapshot &Norm4AmbiguousBackend::ambiguous_snapshot() const {
  return armor_snap_;
}

void Norm4AmbiguousBackend::refresh_center_snapshot() {
  if (!filter_.initialized()) return;

  armor_snap_.armor_pos = filter_.armor_position();
  armor_snap_.armor_vel = filter_.armor_velocity();
  armor_snap_.armor_yaw = filter_.armor_yaw();
  armor_snap_.armor_yaw_rate = filter_.armor_yaw_rate();
  armor_snap_.panel_id = sanitize_panel_id(current_panel_id_);
  armor_snap_.height_label = current_height_label_;

  state_.panel_id = armor_snap_.panel_id;
  state_.r1 = r1_hint_;
  state_.r2 = r2_hint_;
  state_.dza = dza_hint_;
  state_.dza_converged = (dza_hint_ > 1e-3);

  const double radius =
      (state_.panel_id % 2 == 0) ? std::max(0.05, r1_hint_) : std::max(0.05, r2_hint_);
  const double panel_yaw = armor_snap_.armor_yaw;
  state_.center_pos.x() = armor_snap_.armor_pos.x() - radius * std::cos(panel_yaw);
  state_.center_pos.y() = armor_snap_.armor_pos.y() - radius * std::sin(panel_yaw);
  state_.center_pos.z() =
      armor_snap_.armor_pos.z() - panel_z_offset(state_.panel_id, current_height_label_);

  state_.center_yaw =
      normalize_angle(armor_snap_.armor_yaw - panel_angles_[state_.panel_id]);
  state_.yaw_rate = armor_snap_.armor_yaw_rate;

  const Eigen::Vector3d tangential(
      -armor_snap_.armor_yaw_rate * radius * std::sin(panel_yaw),
      armor_snap_.armor_yaw_rate * radius * std::cos(panel_yaw), 0.0);
  state_.center_vel = armor_snap_.armor_vel - tangential;
}

}  // namespace fyt::auto_aim::norm4_v2
