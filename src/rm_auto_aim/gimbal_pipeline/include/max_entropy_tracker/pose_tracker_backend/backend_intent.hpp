// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_INTENT_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_INTENT_HPP_

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/mode/mode_enums.hpp"

namespace fyt::auto_aim {

/// Describes what the backend stage should do for one frame.
/// Built by the serial pipeline after binding and mode decision.
struct BackendIntent {
  int target_panel_id = -1;
  binder::HeightLabel height_label = binder::HeightLabel::UNKNOWN;
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;

  double r1 = 0.15;
  double r2 = 0.20;
  double dza = 0.0;

  double height_confidence = 1.0;
  double position_confidence = 1.0;

  bool force_reinit = false;

  const ObservationData *obs = nullptr;
  bool has_dual = false;
  const ObservationData *obs1 = nullptr;  // primary
  const ObservationData *obs2 = nullptr;  // dual (set when has_dual)

  int dual_panel_id_1 = -1;
  int dual_panel_id_2 = -1;
  std::string dual_layer_1;
  std::string dual_layer_2;
  double dual_height_confidence = 0.0;

  bool enforce_panel_constraint = true;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_INTENT_HPP_
