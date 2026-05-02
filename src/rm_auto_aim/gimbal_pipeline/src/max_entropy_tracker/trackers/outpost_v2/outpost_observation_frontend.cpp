// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::outpost_v2 {

namespace {

constexpr double kLog3 = 1.0986122886681098;
constexpr double kPanelPeriod = 2.0 * M_PI / 3.0;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

double angle_abs_diff(double a, double b) {
  return std::abs(normalize_angle(a - b));
}

double angle_abs_diff_periodic(double a, double b, double period) {
  const double p = std::max(1e-6, period);
  return std::abs(std::remainder(normalize_angle(a - b), p));
}

}  // namespace

ObservationFrontend::ObservationFrontend(const UnifiedConfig & cfg) : cfg_(cfg) {
  radius_ = std::max(0.05, cfg_.outpost.radius);
  z_offsets_ = {
      cfg_.outpost.z_offset_0,
      cfg_.outpost.z_offset_1,
      cfg_.outpost.z_offset_2,
  };
  const double raw_step = (cfg_.outpost.panel_angle_step > 1e-6)
                              ? cfg_.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  panel_angles_ = {0.0, step, -step};
}

const ObservationData * ObservationFrontend::select_primary_observation(
    const std::vector<ObservationData> & obs,
    const OutpostRuntimeContext & /*ctx*/) const {
  if (obs.empty()) return nullptr;
  const ObservationData * best = &obs.front();
  for (const auto & o : obs) {
    if (o.timestamp.has_value()) {
      if (!best->timestamp.has_value() ||
          o.timestamp.value() > best->timestamp.value()) {
        best = &o;
        continue;
      }
    }
    if ((!o.timestamp.has_value() && !best->timestamp.has_value()) &&
        (o.confidence > best->confidence)) {
      best = &o;
    }
  }
  return best;
}

BindingCandidate ObservationFrontend::build_binding_candidate(
    const ObservationData & obs, const OutpostRuntimeContext & ctx) const {
  BindingCandidate c;
  const bool is_ambiguous = (ctx.mode == mode::TrackMode::AMBIGUOUS);
  const double w_yaw_raw = std::max(0.0, cfg_.outpost.weight_yaw);
  const double w_z_state_raw = std::max(0.0, cfg_.outpost.weight_z_state);
  const double w_xy_raw = std::max(0.0, cfg_.outpost.weight_xy_residual);
  const double w_switch_raw = std::max(0.0, cfg_.outpost.weight_switch_penalty);

  // In AMBIGUOUS mode, center state is back-projected from the currently bound panel.
  // Reduce panel-coupled terms to avoid lock-in positive feedback.
  const double amb_decay = is_ambiguous ? 0.30 : 1.0;
  const double w_yaw = w_yaw_raw * (is_ambiguous ? 0.45 : 1.0);
  const double w_z_state = w_z_state_raw * amb_decay;
  const double w_xy = w_xy_raw * amb_decay;
  const double w_switch = is_ambiguous ? 0.0 : w_switch_raw;
  const double temp = std::max(1e-3, cfg_.outpost.softmax_temperature);

  std::array<double, 3> yaw_errs{0.0, 0.0, 0.0};
  std::array<double, 3> xy_residuals{0.0, 0.0, 0.0};

  for (int i = 0; i < 3; ++i) {
    const double center_yaw = normalize_angle(obs.yaw - panel_angles_[i]);
    const double center_z = obs.z - z_offsets_[i];
    yaw_errs[i] = is_ambiguous
                      ? angle_abs_diff_periodic(center_yaw, ctx.center_yaw,
                                                kPanelPeriod)
                      : angle_abs_diff(center_yaw, ctx.center_yaw);
    const double z_state_err = std::abs(center_z - ctx.center_pos.z());

    const double predicted_panel_yaw = normalize_angle(ctx.center_yaw + panel_angles_[i]);
    const double pred_x = ctx.center_pos.x() + radius_ * std::cos(predicted_panel_yaw);
    const double pred_y = ctx.center_pos.y() + radius_ * std::sin(predicted_panel_yaw);
    xy_residuals[i] = std::hypot(obs.x - pred_x, obs.y - pred_y);

    const double switch_penalty =
        (ctx.bound_panel_id >= 0 && i != ctx.bound_panel_id) ? w_switch : 0.0;
    c.costs[i] = w_yaw * yaw_errs[i] + w_z_state * z_state_err +
                 w_xy * xy_residuals[i] + switch_penalty;
  }

  double min_cost = c.costs[0];
  for (int i = 1; i < 3; ++i) {
    min_cost = std::min(min_cost, c.costs[i]);
  }
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    c.probs[i] = std::exp(-(c.costs[i] - min_cost) / temp);
    sum += c.probs[i];
  }
  sum = std::max(1e-12, sum);
  for (int i = 0; i < 3; ++i) {
    c.probs[i] /= sum;
  }

  int best = 0;
  int second = 1;
  if (c.probs[second] > c.probs[best]) std::swap(best, second);
  for (int i = 2; i < 3; ++i) {
    if (c.probs[i] > c.probs[best]) {
      second = best;
      best = i;
    } else if (c.probs[i] > c.probs[second]) {
      second = i;
    }
  }

  c.candidate_panel_id = best;
  c.candidate_prob = c.probs[best];
  c.max_prob = c.candidate_prob;
  c.candidate_margin = std::max(0.0, c.probs[best] - c.probs[second]);
  c.selected_yaw_err = yaw_errs[best];
  c.selected_xy_residual = xy_residuals[best];

  double entropy = 0.0;
  for (double p : c.probs) {
    const double pp = std::max(p, 1e-12);
    entropy -= pp * std::log(pp);
  }
  c.entropy_norm = clamp01(entropy / kLog3);

  if (ctx.last_obs_z.has_value()) {
    c.z_jump = obs.z - ctx.last_obs_z.value();
    c.has_z_jump = std::isfinite(c.z_jump);
  }
  return c;
}

}  // namespace fyt::auto_aim::outpost_v2
