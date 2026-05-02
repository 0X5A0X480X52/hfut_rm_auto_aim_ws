// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/scorer/outpost_hypothesis_evaluator.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::binder {

namespace {

constexpr double kLog3 = 1.0986122886681098;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

double angle_abs_diff(double a, double b) {
  return std::abs(fyt::auto_aim::normalize_angle(a - b));
}

}  // namespace

OutpostHypothesisEvaluator::OutpostHypothesisEvaluator(
    const UnifiedConfig & config, const RobotBindingProfile & profile)
    : config_(config), profile_(profile) {
  radius_ = std::max(0.05, config_.outpost.radius);
  z_offsets_ = {config_.outpost.z_offset_0, config_.outpost.z_offset_1,
                config_.outpost.z_offset_2};
  const double raw_step = (config_.outpost.panel_angle_step > 1e-6)
                              ? config_.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  panel_angles_ = {0.0, step, -step};
}

std::array<BindingHypothesis, 3> OutpostHypothesisEvaluator::evaluate(
    const ObservationData & obs,
    const Eigen::Vector3d & predicted_center_pos,
    double predicted_center_yaw,
    double history_center_z,
    int bound_panel_id) const {
  std::array<BindingHypothesis, 3> hyps;

  const bool has_history = std::isfinite(history_center_z);
  const double w_yaw = std::max(0.0, config_.outpost.weight_yaw);
  const double w_z_state = std::max(0.0, config_.outpost.weight_z_state);
  const double w_z_hist = std::max(0.0, config_.outpost.weight_z_history);
  const double w_xy = std::max(0.0, config_.outpost.weight_xy_residual);
  const double w_switch = std::max(0.0, config_.outpost.weight_switch_penalty);

  for (int i = 0; i < 3; ++i) {
    BindingHypothesis h;
    h.panel_id = i;
    h.center_yaw =
        fyt::auto_aim::normalize_angle(obs.yaw - panel_angles_[i]);
    h.center_z = obs.z - z_offsets_[i];

    h.yaw_err = angle_abs_diff(h.center_yaw, predicted_center_yaw);
    h.z_state_err = std::abs(h.center_z - predicted_center_pos.z());
    h.z_hist_err =
        has_history ? std::abs(h.center_z - history_center_z) : 0.0;

    const double predicted_panel_yaw =
        fyt::auto_aim::normalize_angle(predicted_center_yaw + panel_angles_[i]);
    const double pred_x =
        predicted_center_pos.x() + radius_ * std::cos(predicted_panel_yaw);
    const double pred_y =
        predicted_center_pos.y() + radius_ * std::sin(predicted_panel_yaw);
    h.xy_residual = std::hypot(obs.x - pred_x, obs.y - pred_y);

    h.switch_penalty =
        (bound_panel_id >= 0 && i != bound_panel_id) ? w_switch : 0.0;
    h.cost = w_yaw * h.yaw_err + w_z_state * h.z_state_err +
             w_z_hist * h.z_hist_err + w_xy * h.xy_residual +
             h.switch_penalty;
    hyps[i] = h;
  }

  return hyps;
}

void OutpostHypothesisEvaluator::apply_z_audit_prior(
    std::array<BindingHypothesis, 3> & hyps,
    const OutpostZAuditResult & audit,
    double audit_confidence,
    bool ambiguous_mode,
    double previous_entropy) const {
  if (!ambiguous_mode || audit.panel_id < 0) return;

  double best_audit = audit.costs[0];
  for (int i = 1; i < 3; ++i) {
    best_audit = std::min(best_audit, audit.costs[i]);
  }

  double prior_weight = 0.35 * audit_confidence;
  if (audit.panel_id == 2) {
    prior_weight += 4.0 * audit_confidence;
  }
  const double ambiguity_boost =
      std::clamp((previous_entropy - 0.55) / 0.20, 0.0, 1.0);
  prior_weight += 0.30 * ambiguity_boost * audit_confidence;

  for (int i = 0; i < 3; ++i) {
    const double normalized_audit =
        std::max(0.0, audit.costs[i] - best_audit);
    hyps[i].cost += prior_weight * normalized_audit;
  }
}

void OutpostHypothesisEvaluator::compute_probabilities(
    std::array<BindingHypothesis, 3> & hyps) const {
  const double temp = std::max(1e-3, config_.outpost.softmax_temperature);
  double min_cost = hyps[0].cost;
  for (int i = 1; i < 3; ++i) {
    min_cost = std::min(min_cost, hyps[i].cost);
  }

  double sum = 0.0;
  for (auto & h : hyps) {
    const double scaled = -(h.cost - min_cost) / temp;
    h.probability = std::exp(scaled);
    sum += h.probability;
  }
  sum = std::max(sum, 1e-12);
  for (auto & h : hyps) {
    h.probability /= sum;
  }
}

int OutpostHypothesisEvaluator::best_index(
    const std::array<BindingHypothesis, 3> & hyps) const {
  int best = 0;
  for (int i = 1; i < 3; ++i) {
    if (hyps[i].probability > hyps[best].probability) best = i;
  }
  return best;
}

int OutpostHypothesisEvaluator::second_index(
    const std::array<BindingHypothesis, 3> & hyps, int best_idx) const {
  int second = (best_idx == 0) ? 1 : 0;
  for (int i = 0; i < 3; ++i) {
    if (i == best_idx) continue;
    if (hyps[i].probability > hyps[second].probability) second = i;
  }
  return second;
}

int OutpostHypothesisEvaluator::hypothesis_index_for_panel(
    const std::array<BindingHypothesis, 3> & hyps, int panel_id) const {
  for (int i = 0; i < 3; ++i) {
    if (hyps[i].panel_id == panel_id) return i;
  }
  return 0;
}

double OutpostHypothesisEvaluator::same_panel_score(
    const BindingHypothesis & hyp, double predicted_center_z) const {
  const double yaw_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_yaw_gate);
  const double z_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_z_gate);
  const double xy_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_xy_gate);

  const double yaw_err = hyp.yaw_err;
  const double z_err = std::abs(hyp.center_z - predicted_center_z);
  const double xy_err = hyp.xy_residual;

  const double score =
      1.0 - (0.40 * (yaw_err / yaw_gate) + 0.35 * (z_err / z_gate) +
             0.25 * (xy_err / xy_gate));
  return clamp01(score);
}

double OutpostHypothesisEvaluator::entropy_norm(
    const std::array<BindingHypothesis, 3> & hyps) const {
  double entropy = 0.0;
  for (const auto & h : hyps) {
    const double p = std::max(h.probability, 1e-12);
    entropy -= p * std::log(p);
  }
  return std::clamp(entropy / kLog3, 0.0, 1.0);
}

}  // namespace fyt::auto_aim::binder
