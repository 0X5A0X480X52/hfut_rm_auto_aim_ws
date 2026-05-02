// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BACKEND_INTERFACE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BACKEND_INTERFACE_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::outpost_v2 {

struct BackendStateSnapshot {
  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;
  double radius = 0.0;
  int panel_id = -1;
};

struct BackendUpdateHint {
  int panel_id = -1;
  double position_confidence = 1.0;
  bool enforce_panel_constraint = false;
};

class IOutpostBackend {
 public:
  virtual ~IOutpostBackend() = default;

  virtual void reset(const ObservationData & obs, int panel_id) = 0;
  virtual void predict(double dt) = 0;
  virtual bool update(const ObservationData & obs,
                      const BackendUpdateHint & hint) = 0;
  virtual BackendStateSnapshot snapshot() const = 0;
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BACKEND_INTERFACE_HPP_
