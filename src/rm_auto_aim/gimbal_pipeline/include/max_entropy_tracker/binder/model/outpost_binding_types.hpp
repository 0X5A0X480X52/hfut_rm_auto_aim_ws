// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_OUTPOST_BINDING_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_OUTPOST_BINDING_TYPES_HPP_

#include <array>
#include <limits>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"

namespace fyt::auto_aim::binder {

struct OutpostZAuditResult {
  int panel_id = -1;
  double z_jump = std::numeric_limits<double>::quiet_NaN();
  double dz_from_center = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> costs{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
};

struct OutpostBindingFSMInput {
  int candidate_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double same_panel_score = 0.0;
  double switch_score = 0.0;
};

struct OutpostBindingFSMOutput {
  int selected_id = -1;
  int bound_id = -1;
  int pending_id = -1;
  HeightLabel height_label = HeightLabel::UNKNOWN;
  BindingFSMState state = BindingFSMState::LOCKED;
  BindingAction action = BindingAction::HOLD;
  bool switch_occurred = false;
  int switch_reason = 0;
  int transition_state = 0;
};

inline HeightLabel outpostHeightLabelFromPanel(int panel_id) {
  if (panel_id == 0) return HeightLabel::UPPER;
  if (panel_id == 1) return HeightLabel::MIDDLE;
  if (panel_id == 2) return HeightLabel::LOWER;
  return HeightLabel::UNKNOWN;
}

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_OUTPOST_BINDING_TYPES_HPP_
