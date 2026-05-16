// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v3/outpost_hypothesis_generator.hpp"

namespace fyt::auto_aim::outpost_v3 {

std::vector<OutpostHypothesis> OutpostHypothesisGenerator::generate(
    const std::vector<ObservationData> &observations) const {
  if (observations.empty()) return {};

  // Phase 1: single-observation only. Take the primary observation.
  return generate_single(0);
}

std::vector<OutpostHypothesis> OutpostHypothesisGenerator::generate_single(
    int obs_index) const {
  std::vector<OutpostHypothesis> hypotheses;
  hypotheses.reserve(kNumPanels);

  for (int pid = 0; pid < kNumPanels; ++pid) {
    OutpostHypothesis hyp;
    hyp.obs_index = obs_index;
    hyp.panel_id = pid;
    hyp.prior_log_weight = 0.0;
    hyp.debug_name = "O" + std::to_string(obs_index) + "_P" +
                     std::to_string(pid);
    hypotheses.push_back(hyp);
  }

  return hypotheses;
}

void OutpostHypothesisGenerator::attach_prior(
    std::vector<OutpostHypothesis> *hypotheses,
    int last_committed_panel, double switch_penalty) const {
  if (!hypotheses) return;
  for (auto &hyp : *hypotheses) {
    hyp.prior_log_weight =
        (hyp.panel_id == last_committed_panel) ? 0.0 : -switch_penalty;
  }
}

}  // namespace fyt::auto_aim::outpost_v3
