// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BACKEND_INTERFACE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BACKEND_INTERFACE_HPP_

#include <string>

#include <Eigen/Dense>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::norm4_v2 {

struct BackendStateSnapshot {
  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;
  double r1 = 0.15;
  double r2 = 0.20;
  double dza = 0.0;
  bool dza_converged = false;
  int panel_id = -1;
};

struct BackendUpdateHint {
  int panel_id = -1;
  binder::HeightLabel height_label = binder::HeightLabel::UNKNOWN;
  double height_confidence = 0.0;
  double position_confidence = 1.0;
  double r1_hint = 0.15;
  double r2_hint = 0.20;
  double dza_hint = 0.0;
  bool enforce_panel_constraint = false;
};

class INorm4Backend {
 public:
  virtual ~INorm4Backend() = default;

  virtual void reset(const ObservationData &obs, int panel_id, double r1,
                     double r2, double dza) = 0;
  virtual void predict(double dt) = 0;
  virtual bool update(const ObservationData &obs,
                      const BackendUpdateHint &hint) = 0;
  virtual BackendStateSnapshot snapshot() const = 0;
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BACKEND_INTERFACE_HPP_
