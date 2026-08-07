// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_baseline/models/norm4_motion_model_bundle.hpp"

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::norm4_baseline {

NativeProcessModelBundle::NativeProcessModelBundle(
    std::shared_ptr<CompositeProcessModel> model)
    : model_(std::move(model)), state_idx_(model_->layout()) {}

Eigen::VectorXd NativeProcessModelBundle::initial_state(
    const ObservationData &obs, int panel_id, double r1, double r2,
    double dza) const {
  const int p = ((panel_id % 4) + 4) % 4;
  double panel_angle = p * (M_PI / 2.0);
  double use_r = (p % 2 == 0) ? r1 : r2;

  double center_x = obs.x - use_r * std::cos(obs.yaw);
  double center_y = obs.y - use_r * std::sin(obs.yaw);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model_->state_dim());
  auto idx = state_idx_;
  x0(idx.X()) = center_x;
  x0(idx.VX()) = 0.0;
  x0(idx.Y()) = center_y;
  x0(idx.VY()) = 0.0;
  x0(idx.Z()) = obs.z;
  x0(idx.VZ()) = 0.0;
  x0(idx.DELTA()) = center_yaw;
  x0(idx.DELTA_RATE()) = 0.0;
  x0(idx.R1()) = r1;
  x0(idx.R2()) = r2;
  x0(idx.DZA()) = dza;
  return x0;
}

}  // namespace fyt::auto_aim::norm4_baseline
