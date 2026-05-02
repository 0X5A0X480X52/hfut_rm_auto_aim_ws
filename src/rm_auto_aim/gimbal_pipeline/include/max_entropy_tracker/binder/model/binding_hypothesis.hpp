// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDING_HYPOTHESIS_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDING_HYPOTHESIS_HPP_

namespace fyt::auto_aim::binder {

struct BindingHypothesis {
  int panel_id = -1;

  double center_yaw = 0.0;
  double center_z = 0.0;

  double yaw_err = 0.0;
  double z_state_err = 0.0;
  double z_hist_err = 0.0;
  double xy_residual = 0.0;
  double switch_penalty = 0.0;

  double cost = 0.0;
  double probability = 0.0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDING_HYPOTHESIS_HPP_
