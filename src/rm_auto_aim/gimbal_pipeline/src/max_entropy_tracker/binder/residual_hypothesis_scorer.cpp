// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/scorer/residual_hypothesis_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

}  // namespace

ResidualHypothesisScorer::ResidualHypothesisScorer(
    const ResidualHypothesisScorerConfig & config)
    : config_(config) {}

BindingHealth ResidualHypothesisScorer::evaluate(
    const BinderFrameInput & input, const BinderOutput & output,
    ScorerContext & ctx) {
  BindingHealth health;

  (void)output;  // reserved for future use (compare selected_id with prediction)

  const double yaw_gate = std::max(1e-3, config_.same_panel_yaw_gate);
  const double z_gate = std::max(1e-3, config_.same_panel_z_gate);
  const double xy_gate = std::max(1e-3, config_.same_panel_xy_gate);

  // Evaluate consistency of the currently selected binding hypothesis.
  double yaw_err = input.selected_yaw_err;
  if (!std::isfinite(yaw_err)) yaw_err = yaw_gate;

  double z_err = 0.0;
  const int eval_id = (output.selected_id >= 0) ? output.selected_id : input.candidate_id;
  if (input.profile && eval_id >= 0 &&
      eval_id < static_cast<int>(input.profile->z_offsets.size()) &&
      !input.obs_z_values.empty()) {
    const double expected_z = input.profile->z_offsets[eval_id];
    z_err = std::abs(input.obs_z_values[0] - expected_z);
  }

  const double xy_err =
      std::isfinite(input.same_panel_residual) ? input.same_panel_residual : 0.0;

  double score = 1.0 - (0.40 * (yaw_err / yaw_gate) +
                        0.35 * (z_err / z_gate) +
                        0.25 * (xy_err / xy_gate));
  health.score = clamp01(score);

  const bool is_bad = health.score < 0.3;
  health.anomaly_detected = is_bad;

  if (is_bad) {
    ++ctx.consecutive_bad_frames;
  } else {
    ctx.consecutive_bad_frames = 0;
  }
  health.consecutive_bad_frames = ctx.consecutive_bad_frames;
  health.force_rebind_recommend =
      ctx.consecutive_bad_frames >= config_.consecutive_bad_threshold;

  return health;
}

}  // namespace fyt::auto_aim::binder
