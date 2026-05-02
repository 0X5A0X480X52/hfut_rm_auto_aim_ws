// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/scorer/outpost_binding_confidence.hpp"

#include <algorithm>

namespace fyt::auto_aim::binder {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

}  // namespace

OutpostBindingConfidenceScorer::OutpostBindingConfidenceScorer(
    const UnifiedConfig & config)
    : config_(config) {}

double OutpostBindingConfidenceScorer::switch_score(
    int bound_panel_id, int candidate_panel, int z_audit_panel_id,
    double period_confidence, double candidate_prob,
    double candidate_margin) const {
  const double switch_base =
      (bound_panel_id >= 0 && candidate_panel != bound_panel_id)
          ? candidate_prob
          : 0.0;
  const double z_audit_switch_support =
      (z_audit_panel_id >= 0 && candidate_panel == z_audit_panel_id &&
       candidate_panel != bound_panel_id)
          ? 1.0
          : 0.0;
  return clamp01(0.50 * switch_base + 0.20 * period_confidence +
                 0.10 * z_audit_switch_support + 0.20 * candidate_margin);
}

double OutpostBindingConfidenceScorer::binding_confidence(
    double selected_probability, double candidate_margin,
    double same_panel_score, double switch_score,
    double period_confidence, bool pending_transition) const {
  const double consistency_score = std::max(same_panel_score, period_confidence);
  double base = clamp01(0.55 * selected_probability +
                        0.20 * candidate_margin +
                        0.25 * consistency_score);

  if (pending_transition) {
    base *= std::clamp(1.0 - 0.25 * switch_score, 0.55, 1.0);
  }

  const double floor =
      std::clamp(config_.outpost.binding_confidence_floor, 0.0, 0.95);
  return floor + (1.0 - floor) * clamp01(base);
}

bool OutpostBindingConfidenceScorer::conflict_for_update(
    int candidate_panel, int selected_panel, int z_audit_panel_id) const {
  return (candidate_panel >= 0 && candidate_panel != selected_panel) ||
         (z_audit_panel_id >= 0 && z_audit_panel_id != selected_panel);
}

}  // namespace fyt::auto_aim::binder
