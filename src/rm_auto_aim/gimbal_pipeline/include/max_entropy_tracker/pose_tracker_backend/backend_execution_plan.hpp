// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTION_PLAN_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTION_PLAN_HPP_

#include <string>
#include <vector>

namespace fyt::auto_aim {

enum class BackendStepOp {
  PREDICT_AMBIGUOUS,
  PREDICT_STRUCTURED,
  UPDATE_AMBIGUOUS,
  UPDATE_STRUCTURED,
  UPDATE_STRUCTURED_DUAL,
  SHADOW_UPDATE_AMBIGUOUS,
  SHADOW_UPDATE_STRUCTURED,
  APPLY_PANEL_MISMATCH,
  SYNC_OUTPUT,
  PUBLISH_STATE,
};

struct BackendStep {
  BackendStepOp op = BackendStepOp::UPDATE_AMBIGUOUS;
  int target_panel_id = -1;
  double height_confidence = 1.0;
  double position_confidence = 1.0;
  double r1_hint = 0.15;
  double r2_hint = 0.20;
  double dza_hint = 0.0;
  bool enforce_panel_constraint = true;

  // For dual-update steps.
  int dual_panel_id_1 = -1;
  int dual_panel_id_2 = -1;
  std::string dual_layer_1;
  std::string dual_layer_2;
  double dual_height_confidence = 0.0;

  static const char *op_name(BackendStepOp op) {
    switch (op) {
      case BackendStepOp::PREDICT_AMBIGUOUS: return "PREDICT_AMBIGUOUS";
      case BackendStepOp::PREDICT_STRUCTURED: return "PREDICT_STRUCTURED";
      case BackendStepOp::UPDATE_AMBIGUOUS: return "UPDATE_AMBIGUOUS";
      case BackendStepOp::UPDATE_STRUCTURED: return "UPDATE_STRUCTURED";
      case BackendStepOp::UPDATE_STRUCTURED_DUAL: return "UPDATE_STRUCTURED_DUAL";
      case BackendStepOp::SHADOW_UPDATE_AMBIGUOUS: return "SHADOW_UPDATE_AMBIGUOUS";
      case BackendStepOp::SHADOW_UPDATE_STRUCTURED: return "SHADOW_UPDATE_STRUCTURED";
      case BackendStepOp::APPLY_PANEL_MISMATCH: return "APPLY_PANEL_MISMATCH";
      case BackendStepOp::SYNC_OUTPUT: return "SYNC_OUTPUT";
      case BackendStepOp::PUBLISH_STATE: return "PUBLISH_STATE";
    }
    return "UNKNOWN";
  }
};

/// Ordered list of backend operations for one frame.
struct BackendExecutionPlan {
  std::vector<BackendStep> steps;

  bool empty() const { return steps.empty(); }
  int size() const { return static_cast<int>(steps.size()); }
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTION_PLAN_HPP_
