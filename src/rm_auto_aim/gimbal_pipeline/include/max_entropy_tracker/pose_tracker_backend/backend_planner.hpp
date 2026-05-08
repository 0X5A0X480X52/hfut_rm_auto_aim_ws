// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_PLANNER_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_PLANNER_HPP_

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/mode/mode_enums.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_execution_plan.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_intent.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"

namespace fyt::auto_aim {

/// Translates a BackendIntent into a concrete BackendExecutionPlan.
///
/// Handles the decision of which backends to update, in what order,
/// and whether shadow updates / dual updates are needed.
class BackendPlanner {
 public:
  explicit BackendPlanner(const UnifiedConfig &config) : config_(config) {}

  BackendExecutionPlan plan(const BackendIntent &intent,
                            const norm4_v2::Norm4RuntimeContext &ctx) const {
    (void)ctx;
    BackendExecutionPlan plan;

    if (intent.mode == mode::TrackMode::STRUCTURED) {
      // Structured is the active backend.
      plan.steps.push_back(make_update_step(BackendStepOp::UPDATE_STRUCTURED,
                                            intent));

      if (intent.has_dual) {
        plan.steps.push_back(
            make_dual_step(BackendStepOp::UPDATE_STRUCTURED_DUAL, intent));
      }

      // Shadow update: keep ambiguous warm at lower confidence.
      plan.steps.push_back(
          make_shadow_step(BackendStepOp::SHADOW_UPDATE_AMBIGUOUS, intent));
    } else {
      // Ambiguous is the active backend.
      plan.steps.push_back(
          make_update_step(BackendStepOp::UPDATE_AMBIGUOUS, intent));

      // Shadow update: keep structured warm.
      plan.steps.push_back(
          make_shadow_step(BackendStepOp::SHADOW_UPDATE_STRUCTURED, intent));

      if (intent.has_dual) {
        plan.steps.push_back(
            make_dual_step(BackendStepOp::UPDATE_STRUCTURED_DUAL, intent));
      }
    }

    if (config_.panel_mismatch.apply_correction &&
        intent.mode == mode::TrackMode::STRUCTURED) {
      BackendStep mismatch_step;
      mismatch_step.op = BackendStepOp::APPLY_PANEL_MISMATCH;
      plan.steps.push_back(mismatch_step);
    }

    BackendStep sync_step;
    sync_step.op = BackendStepOp::SYNC_OUTPUT;
    plan.steps.push_back(sync_step);
    BackendStep publish_step;
    publish_step.op = BackendStepOp::PUBLISH_STATE;
    plan.steps.push_back(publish_step);

    return plan;
  }

 private:
  static BackendStep make_update_step(BackendStepOp op,
                                      const BackendIntent &intent) {
    BackendStep s;
    s.op = op;
    s.target_panel_id = intent.target_panel_id;
    s.height_confidence = intent.height_confidence;
    s.position_confidence = intent.position_confidence;
    s.r1_hint = intent.r1;
    s.r2_hint = intent.r2;
    s.dza_hint = intent.dza;
    s.enforce_panel_constraint = intent.enforce_panel_constraint;
    return s;
  }

  static BackendStep make_shadow_step(BackendStepOp op,
                                      const BackendIntent &intent) {
    BackendStep s = make_update_step(op, intent);
    s.position_confidence =
        std::clamp(0.25 + 0.25 * s.position_confidence, 0.05, 0.60);
    return s;
  }

  static BackendStep make_dual_step(BackendStepOp op,
                                    const BackendIntent &intent) {
    BackendStep s;
    s.op = op;
    s.dual_panel_id_1 = intent.dual_panel_id_1;
    s.dual_panel_id_2 = intent.dual_panel_id_2;
    s.dual_layer_1 = intent.dual_layer_1;
    s.dual_layer_2 = intent.dual_layer_2;
    s.dual_height_confidence = intent.dual_height_confidence;
    return s;
  }

  UnifiedConfig config_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_PLANNER_HPP_
