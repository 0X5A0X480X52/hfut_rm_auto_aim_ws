// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::norm4_v2 {

Norm4StructuredBackend::Norm4StructuredBackend(const UnifiedConfig &cfg, double dt)
    : cfg_(cfg), ukf_(cfg, dt) {}

std::string Norm4StructuredBackend::panel_r_type(int panel_id) {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p % 2 == 0) ? "r1" : "r2";
}

std::string Norm4StructuredBackend::default_layer(int panel_id) {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p % 2 == 0) ? "lower" : "upper";
}

std::string Norm4StructuredBackend::layer_from_label(binder::HeightLabel label,
                                                     int panel_id) {
  if (label == binder::HeightLabel::UPPER) return "upper";
  if (label == binder::HeightLabel::LOWER) return "lower";
  return default_layer(panel_id);
}

void Norm4StructuredBackend::reset(const ObservationData &obs, int panel_id,
                                   double r1, double r2, double dza) {
  current_panel_id_ = ((panel_id % 4) + 4) % 4;
  default_r1_ = std::max(0.05, r1);
  default_r2_ = std::max(0.05, r2);
  default_dza_ = std::max(0.0, dza);
  ukf_.initialize({obs}, default_r1_, default_r2_, default_dza_, current_panel_id_);
  initialized_ = true;
}

void Norm4StructuredBackend::predict(double dt) {
  if (!initialized_) return;
  ukf_.predict(dt);
}

bool Norm4StructuredBackend::update(const ObservationData &obs,
                                    const BackendUpdateHint &hint) {
  if (!initialized_) {
    reset(obs, hint.panel_id >= 0 ? hint.panel_id : 0, hint.r1_hint, hint.r2_hint,
          hint.dza_hint);
  }
  current_panel_id_ = (hint.panel_id >= 0) ? ((hint.panel_id % 4 + 4) % 4)
                                            : current_panel_id_;

  const std::string r_type = panel_r_type(current_panel_id_);
  const std::string layer = layer_from_label(hint.height_label, current_panel_id_);
  const double h_conf = std::clamp(hint.height_confidence, 0.0, 1.0);
  const double pos_conf = std::clamp(hint.position_confidence, 0.05, 1.0);
  const double panel_angle = current_panel_id_ * (M_PI / 2.0);

  return ukf_.update({obs}, {r_type}, {layer}, h_conf, pos_conf, panel_angle);
}

bool Norm4StructuredBackend::update_dual(const ObservationData &obs1,
                                         const ObservationData &obs2,
                                         int panel_id_1, int panel_id_2,
                                         const std::string &layer_1,
                                         const std::string &layer_2,
                                         double height_confidence) {
  if (!initialized_) return false;
  const std::string rt1 = panel_r_type(panel_id_1);
  const std::string rt2 = panel_r_type(panel_id_2);
  const std::string l1 = layer_1.empty() ? default_layer(panel_id_1) : layer_1;
  const std::string l2 = layer_2.empty() ? default_layer(panel_id_2) : layer_2;
  return ukf_.update({obs1, obs2}, {rt1, rt2}, {l1, l2},
                     std::clamp(height_confidence, 0.0, 1.0));
}

void Norm4StructuredBackend::apply_panel_correction(int new_panel_id,
                                                    double armor_yaw) {
  current_panel_id_ = ((new_panel_id % 4) + 4) % 4;
  const double panel_angle = current_panel_id_ * (M_PI / 2.0);
  const double center_yaw = normalize_angle(armor_yaw - panel_angle);
  ukf_.apply_panel_correction(center_yaw);
}

BackendStateSnapshot Norm4StructuredBackend::snapshot() const {
  BackendStateSnapshot s;
  const auto &x = ukf_.x();
  const auto idx = ukf_.state_idx();
  s.center_pos = ukf_.get_center_position();
  s.center_vel = Eigen::Vector3d(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  s.center_yaw = ukf_.get_yaw();
  s.yaw_rate = x(idx.DELTA_RATE());
  const auto radii = ukf_.get_radii();
  s.r1 = radii.first;
  s.r2 = radii.second;
  s.dza = ukf_.get_dza();
  s.dza_converged = ukf_.is_dza_converged();
  s.panel_id = current_panel_id_;
  return s;
}

}  // namespace fyt::auto_aim::norm4_v2
