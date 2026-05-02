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

  // IMPORTANT:
  // input.obs_z_values are absolute z in odom/world, while profile->z_offsets are
  // relative armor offsets to center. Mixing them produces consistently huge z_err
  // and makes health collapse to zero. For health scoring, only evaluate z on
  // same-panel continuity with relative jump magnitude.
  double z_err = 0.0;
  bool z_err_valid = false;
  if (input.event_type == TrackEventType::CONTINUITY && input.has_z_jump &&
      std::isfinite(input.z_jump)) {
    z_err = std::abs(input.z_jump);
    z_err_valid = true;
  }

  const double xy_err =
      std::isfinite(input.same_panel_residual) ? input.same_panel_residual : 0.0;

  const double yaw_term = std::clamp(yaw_err / yaw_gate, 0.0, 2.0);
  const double xy_term = std::clamp(xy_err / xy_gate, 0.0, 2.0);
  const double z_term = z_err_valid ? std::clamp(z_err / z_gate, 0.0, 2.0) : 0.0;
  const double z_weight = z_err_valid ? 0.25 : 0.0;
  const double base_weight = 0.75 + z_weight;

  double score = 1.0 - ((0.40 * yaw_term + 0.35 * z_term + 0.35 * xy_term) /
                        std::max(1e-6, base_weight));
  health.score = clamp01(score);

  // During reacquire/ambiguous stages, avoid escalating to force-rebind solely
  // by temporary model inconsistency.
  const bool allow_bad_accum =
      input.event_type == TrackEventType::CONTINUITY ||
      input.event_type == TrackEventType::SWITCH_CANDIDATE;
  const bool is_bad = allow_bad_accum && (health.score < 0.3);
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
