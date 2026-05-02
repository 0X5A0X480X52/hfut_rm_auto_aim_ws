// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_structured_backend.hpp"

namespace fyt::auto_aim::outpost_v2 {

OutpostStructuredBackend::OutpostStructuredBackend(const UnifiedConfig & cfg,
                                                   double dt)
    : ukf_(cfg, dt) {}

void OutpostStructuredBackend::reset(const ObservationData & obs, int panel_id) {
  current_panel_id_ = panel_id;
  ukf_.initialize({obs}, 0.15, 0.15, 0.0, panel_id);
  initialized_ = true;
}

void OutpostStructuredBackend::predict(double dt) {
  if (!initialized_) return;
  ukf_.predict(dt);
}

bool OutpostStructuredBackend::update(const ObservationData & obs,
                                      const BackendUpdateHint & hint) {
  if (!initialized_) {
    reset(obs, hint.panel_id >= 0 ? hint.panel_id : 0);
  }
  current_panel_id_ = (hint.panel_id >= 0) ? hint.panel_id : current_panel_id_;
  ObservationData obs_with_panel = obs;
  obs_with_panel.panel_id = current_panel_id_;
  return ukf_.update_with_panel(obs_with_panel, current_panel_id_,
                                hint.position_confidence);
}

BackendStateSnapshot OutpostStructuredBackend::snapshot() const {
  BackendStateSnapshot s;
  const auto &x = ukf_.x();
  const auto idx = ukf_.state_idx();
  s.center_pos = ukf_.get_center_position();
  s.center_vel = Eigen::Vector3d(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  s.center_yaw = ukf_.get_yaw();
  s.yaw_rate = x(idx.DELTA_RATE());
  s.radius = ukf_.get_radii().first;
  s.panel_id = current_panel_id_;
  return s;
}

}  // namespace fyt::auto_aim::outpost_v2
