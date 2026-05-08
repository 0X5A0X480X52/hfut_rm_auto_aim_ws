// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTOR_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTOR_HPP_

#include <string>
#include <utility>

#include "max_entropy_tracker/association/panel_mismatch_detector.hpp"
#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_execution_plan.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend.hpp"

namespace fyt::auto_aim {

/// Executes a BackendExecutionPlan against the real backends.
///
/// Owns no state; just dispatches to the provided backends.
class BackendExecutor {
 public:
  struct Result {
    bool ok = false;
    bool ambiguous_updated = false;
    bool structured_updated = false;
    bool dual_updated = false;
    std::string error_step;
  };

  BackendExecutor(const UnifiedConfig &config,
                  const ObservationData *selected_obs,
                  const ObservationData *dual_obs = nullptr)
      : config_(config), selected_obs_(selected_obs), dual_obs_(dual_obs) {}

  Result execute(const BackendExecutionPlan &plan,
                 norm4_v2::Norm4RuntimeContext *ctx,
                 norm4_v2::Norm4AmbiguousBackend *amb,
                 norm4_v2::Norm4StructuredBackend *str,
                 norm4_v2::Norm4OutputAdapter *output,
                 PanelMismatchDetector *mismatch = nullptr) {
    Result result;
    result.ok = true;

    for (const auto &step : plan.steps) {
      bool step_ok = true;
      switch (step.op) {
        case BackendStepOp::UPDATE_AMBIGUOUS:
          step_ok = do_update(step, ctx, amb);
          result.ambiguous_updated = step_ok;
          break;
        case BackendStepOp::UPDATE_STRUCTURED:
          step_ok = do_update(step, ctx, str);
          result.structured_updated = step_ok;
          break;
        case BackendStepOp::UPDATE_STRUCTURED_DUAL:
          step_ok = do_dual_update(step, str);
          result.dual_updated = step_ok;
          break;
        case BackendStepOp::SHADOW_UPDATE_AMBIGUOUS:
          do_shadow_update(step, ctx, amb);
          break;
        case BackendStepOp::SHADOW_UPDATE_STRUCTURED:
          do_shadow_update(step, ctx, str);
          break;
        case BackendStepOp::APPLY_PANEL_MISMATCH:
          apply_mismatch(step, ctx, str, mismatch);
          break;
        case BackendStepOp::SYNC_OUTPUT:
          sync_output(ctx, amb, str);
          break;
        case BackendStepOp::PUBLISH_STATE:
          publish_state(ctx, amb, str, output);
          break;
        default:
          break;
      }

      if (!step_ok) {
        result.ok = false;
        result.error_step = BackendStep::op_name(step.op);
        break;
      }
    }
    return result;
  }

 private:
  norm4_v2::BackendUpdateHint make_hint(const BackendStep &step,
                                        const norm4_v2::Norm4RuntimeContext &ctx) {
    norm4_v2::BackendUpdateHint hint;
    hint.panel_id = step.target_panel_id;
    hint.height_label = ctx.bound_height_label;
    hint.height_confidence = step.height_confidence;
    hint.position_confidence = step.position_confidence;
    hint.r1_hint = ctx.r1;
    hint.r2_hint = ctx.r2;
    hint.dza_hint = ctx.dza;
    hint.enforce_panel_constraint = step.enforce_panel_constraint;
    return hint;
  }

  bool do_update(const BackendStep &step,
                 norm4_v2::Norm4RuntimeContext *ctx,
                 norm4_v2::INorm4Backend *backend) {
    if (!selected_obs_ || !ctx || !backend) return false;
    return backend->update(*selected_obs_, make_hint(step, *ctx));
  }

  void do_shadow_update(const BackendStep &step,
                        norm4_v2::Norm4RuntimeContext *ctx,
                        norm4_v2::INorm4Backend *backend) {
    if (!selected_obs_ || !ctx || !backend) return;
    backend->update(*selected_obs_, make_hint(step, *ctx));
  }

  bool do_dual_update(const BackendStep &step,
                      norm4_v2::Norm4StructuredBackend *str) {
    if (!str || !selected_obs_ || !dual_obs_) return false;
    return str->update_dual(
        *selected_obs_, *dual_obs_,
        step.dual_panel_id_1, step.dual_panel_id_2,
        step.dual_layer_1, step.dual_layer_2,
        step.dual_height_confidence);
  }

  void apply_mismatch(const BackendStep & /*step*/,
                      norm4_v2::Norm4RuntimeContext *ctx,
                      norm4_v2::Norm4StructuredBackend *str,
                      PanelMismatchDetector *mismatch) {
    if (!mismatch || !str || !selected_obs_ || !ctx) return;
    if (!str->initialized()) return;

    auto &ukf = str->ukf();
    const auto idx = ukf.state_idx();
    auto result = mismatch->update(
        str->current_panel_id(), selected_obs_->z,
        ukf.x()(idx.Z()), ukf.x()(idx.DZA()),
        "", ukf.is_dza_converged(),
        ukf.last_z_innovation());

    if (result.action == PanelMismatchDetector::Action::NONE ||
        !config_.panel_mismatch.apply_correction) {
      return;
    }
    if (result.action == PanelMismatchDetector::Action::REINIT) {
      int corrected =
          ((result.new_panel_id % 4) + 4) % 4;
      str->reset(*selected_obs_, corrected, ctx->r1, ctx->r2, ctx->dza);
    } else {
      str->apply_panel_correction(
          ((result.new_panel_id % 4) + 4) % 4,
          selected_obs_->yaw);
    }
  }

  void sync_output(norm4_v2::Norm4RuntimeContext *ctx,
                   norm4_v2::Norm4AmbiguousBackend *amb,
                   norm4_v2::Norm4StructuredBackend *str) {
    if (!ctx) return;
    const auto active_snap = (ctx->mode == mode::TrackMode::STRUCTURED)
                                 ? str->snapshot()
                                 : amb->snapshot();
    ctx->center_pos = active_snap.center_pos;
    ctx->center_vel = active_snap.center_vel;
    ctx->center_yaw = active_snap.center_yaw;
    ctx->yaw_rate = active_snap.yaw_rate;
    ctx->r1 = active_snap.r1;
    ctx->r2 = active_snap.r2;
    ctx->dza = active_snap.dza;
    ctx->dza_converged = active_snap.dza_converged;
    ctx->selected_panel_id = (active_snap.panel_id >= 0)
                                 ? active_snap.panel_id
                                 : ctx->selected_panel_id;
  }

  void publish_state(norm4_v2::Norm4RuntimeContext *ctx,
                     norm4_v2::Norm4AmbiguousBackend *amb,
                     norm4_v2::Norm4StructuredBackend *str,
                     norm4_v2::Norm4OutputAdapter *output) {
    if (!ctx || !amb || !str || !output) return;
    const auto active_snap = (ctx->mode == mode::TrackMode::STRUCTURED)
                                 ? str->snapshot()
                                 : amb->snapshot();
    norm4_v2::PublishStateInput input;
    input.mode = ctx->mode;
    input.backend_snap = &active_snap;
    if (ctx->mode == mode::TrackMode::AMBIGUOUS) {
      input.armor_snap = &amb->ambiguous_snapshot();
    }
    output->update_publish_state(ctx, input);
  }

  UnifiedConfig config_;
  const ObservationData *selected_obs_;
  const ObservationData *dual_obs_ = nullptr;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_BACKEND_EXECUTOR_HPP_
