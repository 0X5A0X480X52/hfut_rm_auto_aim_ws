// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_BINDING_CONFIDENCE_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_BINDING_CONFIDENCE_HPP_

#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim::binder {

class OutpostBindingConfidenceScorer {
 public:
  explicit OutpostBindingConfidenceScorer(const UnifiedConfig & config);

  double switch_score(int bound_panel_id, int candidate_panel,
                      int z_audit_panel_id, double period_confidence,
                      double candidate_prob, double candidate_margin) const;

  double binding_confidence(double selected_probability,
                            double candidate_margin,
                            double same_panel_score,
                            double switch_score,
                            double period_confidence,
                            bool pending_transition) const;

  bool conflict_for_update(int candidate_panel, int selected_panel,
                           int z_audit_panel_id) const;

 private:
  UnifiedConfig config_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_BINDING_CONFIDENCE_HPP_
