// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_PIPELINE_SERIAL_TRACKER_PIPELINE_HPP_
#define MAX_ENTROPY_TRACKER_PIPELINE_SERIAL_TRACKER_PIPELINE_HPP_

#include <memory>
#include <string>

#include "max_entropy_tracker/association/height_identifier.hpp"
#include "max_entropy_tracker/association/panel_mismatch_detector.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/evidence/evidence_builder.hpp"
#include "max_entropy_tracker/mode/evidence_fuser.hpp"
#include "max_entropy_tracker/mode/mode_fsm.hpp"
#include "max_entropy_tracker/pipeline/debug_trace.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_execution_plan.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_executor.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_intent.hpp"
#include "max_entropy_tracker/pose_tracker_backend/backend_planner.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_binder_bridge.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_phase_sequence_memory.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend.hpp"

namespace fyt::auto_aim::pipeline {

/// Phase 7 serial pipeline output structure.
struct SerialPipelineOutput {
  bool ok = false;
  BackendExecutionPlan backend_plan;
  BackendIntent intent;
  PipelineDebugTrace trace;
};

/// Phase 7: unified serial tracker pipeline for Norm4.
///
/// Call chain:
///   ObservationData[] → EvidenceBuilder → Norm4ObsFrontend → BinderBridge
///   → PhaseMemory → ModeFSM → BackendPlanner → BackendExecutor → OutputAdapter
///
/// Wraps Norm4ArmorTracker internals as a pipeline when enable_common_pipeline
/// is true.
class SerialTrackerPipeline {
 public:
  SerialTrackerPipeline(const UnifiedConfig &config, double dt);

  void initialize(const std::vector<ObservationData> &obs,
                  norm4_v2::Norm4RuntimeContext *ctx);

  SerialPipelineOutput step(const std::vector<ObservationData> &obs,
                            norm4_v2::Norm4RuntimeContext *ctx);

  void predict(double dt, norm4_v2::Norm4RuntimeContext *ctx);

  // Read-only access for external wiring.
  norm4_v2::Norm4AmbiguousBackend &ambiguous_backend() {
    return ambiguous_backend_;
  }
  const norm4_v2::Norm4AmbiguousBackend &ambiguous_backend() const {
    return ambiguous_backend_;
  }
  norm4_v2::Norm4StructuredBackend &structured_backend() {
    return structured_backend_;
  }
  const norm4_v2::Norm4StructuredBackend &structured_backend() const {
    return structured_backend_;
  }
  const norm4_v2::Norm4BinderBridge &binder_bridge() const {
    return binder_bridge_;
  }
  const PipelineDebugTrace &last_trace() const { return last_trace_; }

 private:
  void build_backend_intent(const ObservationData &selected,
                            const norm4_v2::BindingCandidate &candidate,
                            const binder::BinderOutput &binder_out,
                            const norm4_v2::Norm4RuntimeContext &ctx,
                            int obs_count,
                            BackendIntent *intent);

  UnifiedConfig config_;
  double dt_;

  norm4_v2::ObservationFrontend obs_frontend_;
  norm4_v2::Norm4BinderBridge binder_bridge_;
  norm4_v2::Norm4AmbiguousBackend ambiguous_backend_;
  norm4_v2::Norm4StructuredBackend structured_backend_;
  norm4_v2::Norm4OutputAdapter output_adapter_;
  norm4_v2::PhaseSequenceMemory phase_memory_;
  evidence::EvidenceBuilder evidence_builder_;
  BackendPlanner backend_planner_;
  mode::EvidenceFuser evidence_fuser_;
  mode::ModeFSM mode_fsm_;
  HeightIdentifier height_identifier_;
  PanelMismatchDetector mismatch_detector_;

  PipelineDebugTrace last_trace_;
  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;
  bool degraded_single_obs_mode_ = false;
};

}  // namespace fyt::auto_aim::pipeline

#endif  // MAX_ENTROPY_TRACKER_PIPELINE_SERIAL_TRACKER_PIPELINE_HPP_
