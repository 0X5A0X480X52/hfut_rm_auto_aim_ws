// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_AMBIGUOUS_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_AMBIGUOUS_BACKEND_HPP_

#include <array>
#include <cmath>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_backend_interface.hpp"

namespace fyt::auto_aim::outpost_v2 {

/// Raw single-armor state, free of any robot-centric back-projection.
/// This is the authoritative output in AMBIGUOUS mode.
struct AmbiguousArmorSnapshot {
  Eigen::Vector3d armor_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_vel = Eigen::Vector3d::Zero();
  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;
  int panel_id = -1;
  double confidence = 1.0;
};

class OutpostAmbiguousBackend : public IOutpostBackend {
 public:
  explicit OutpostAmbiguousBackend(const UnifiedConfig &cfg);

  void reset(const ObservationData &obs, int panel_id) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

  /// Single-armor state in world frame (no center back-projection).
  const AmbiguousArmorSnapshot &ambiguous_snapshot() const;

 private:
  int sanitize_panel_id(int panel_id) const;
  void refresh_center_snapshot();

  UnifiedConfig cfg_;

  // Generalized adapter: routes to legacy KF or IMM core depending on config.
  AmbiguousSingleArmorFilterAdapter filter_;

  int current_panel_id_ = 0;

  // Center-centric snapshot (compatibility with IOutpostBackend).
  BackendStateSnapshot state_;

  // Single-armor snapshot (authoritative in AMBIGUOUS mode).
  AmbiguousArmorSnapshot armor_snap_;

  // Outpost geometry (used only in center <-> armor conversion).
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{0.06, 0.0, -0.06};
  std::array<double, 3> panel_angles_{0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_AMBIGUOUS_BACKEND_HPP_
