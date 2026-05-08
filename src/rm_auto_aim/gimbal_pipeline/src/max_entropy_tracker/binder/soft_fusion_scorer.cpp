// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/scorer/soft_fusion_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

SoftFusionScorer::SoftFusionScorer(std::unique_ptr<HypothesisScorer> inner,
                                   const SoftFusionConfig &config)
    : inner_(std::move(inner)), config_(config) {}

double SoftFusionScorer::compute_phase_confidence_penalty(
    const BinderFrameInput &input) const {
  if (!input.has_soft_fusion) return 0.0;

  // w_seq * (1 - ping_pong_risk) → low risk = high sequence consistency.
  double seq = (1.0 - std::min(1.0, input.ping_pong_risk));

  // w_geo: same_panel_residual is a proxy for yaw/xy geometric consistency.
  // Lower residual = better geometry. Normalise to [0,1].
  double geo = std::max(0.0, 1.0 - std::min(1.0, input.same_panel_residual / 0.5));

  // w_dyn: kinematic consistency + velocity direction.
  double dyn = input.kinematic_consistency;

  // w_continuity: track continuity from 2D tracker.
  double cont = input.track_continuity_score;
  // w_topology: left/right panel-order consistency from 2D tracker.
  double topo = input.topology_consistency_score;

  double fused = config_.w_seq * seq +
                 config_.w_geo * geo +
                 config_.w_dyn * dyn +
                 config_.w_continuity * cont +
                 config_.w_topology * topo;

  return std::max(0.0, 1.0 - fused);
}

double SoftFusionScorer::compute_transition_cost(int current_panel,
                                                  int proposed_panel) const {
  if (current_panel < 0 || proposed_panel < 0) return 0.0;
  if (current_panel == proposed_panel) return 0.0;

  // Opposite panel (0↔2, 1↔3) — highest cost.
  int diff = std::abs(current_panel - proposed_panel) % 4;
  if (diff == 2) return 0.35;
  // Adjacent panel — moderate cost.
  return 0.15;
}

void SoftFusionScorer::fuse_into_health(BindingHealth &health,
                                        const BinderFrameInput &input,
                                        const BinderOutput &output) const {
  if (!input.has_soft_fusion) return;

  double penalty = compute_phase_confidence_penalty(input);

  // Add transition cost when switching panels.
  double trans_cost = compute_transition_cost(output.bound_id, input.candidate_id);
  penalty = std::min(1.0, penalty + trans_cost);

  // Blend penalty into health score: reduce health, increase bad-frame counter.
  if (penalty > 0.05) {
    health.score = std::max(0.05, health.score * (1.0 - 0.6 * penalty));
    // Elevated penalty triggers anomaly.
    if (penalty > 0.3) {
      health.anomaly_detected = true;
    }
  }
}

BindingHealth SoftFusionScorer::evaluate(const BinderFrameInput &input,
                                          const BinderOutput &output,
                                          ScorerContext &ctx) {
  BindingHealth health = inner_->evaluate(input, output, ctx);
  fuse_into_health(health, input, output);
  return health;
}

}  // namespace fyt::auto_aim::binder
