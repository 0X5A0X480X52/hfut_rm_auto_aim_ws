// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_AMBIGUOUS_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_AMBIGUOUS_BACKEND_HPP_

#include <array>
#include <cmath>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_backend_interface.hpp"

namespace fyt::auto_aim::norm4_v2 {

struct AmbiguousArmorSnapshot {
  Eigen::Vector3d armor_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_vel = Eigen::Vector3d::Zero();
  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;
  int panel_id = -1;
  binder::HeightLabel height_label = binder::HeightLabel::UNKNOWN;
  double confidence = 1.0;
};

class Norm4AmbiguousBackend : public INorm4Backend {
 public:
  explicit Norm4AmbiguousBackend(const UnifiedConfig &cfg);

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

  const AmbiguousArmorSnapshot &ambiguous_snapshot() const;

 private:
  int sanitize_panel_id(int panel_id) const;
  double panel_z_offset(int panel_id, binder::HeightLabel label) const;
  void refresh_center_snapshot();

  UnifiedConfig cfg_;
  AmbiguousSingleArmorFilterAdapter filter_;

  int current_panel_id_ = 0;
  binder::HeightLabel current_height_label_ = binder::HeightLabel::LOWER;
  double r1_hint_ = 0.15;
  double r2_hint_ = 0.20;
  double dza_hint_ = 0.0;

  BackendStateSnapshot state_;
  AmbiguousArmorSnapshot armor_snap_;

  std::array<double, 4> panel_angles_{{0.0, M_PI / 2.0, M_PI, -M_PI / 2.0}};
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_AMBIGUOUS_BACKEND_HPP_
