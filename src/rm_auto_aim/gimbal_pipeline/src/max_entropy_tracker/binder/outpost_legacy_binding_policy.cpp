// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/policy/outpost_legacy_binding_policy.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

namespace {

int directional_successor(int panel_id, int spin_direction) {
  if (panel_id < 0 || panel_id > 2 || spin_direction == 0) return -1;
  // Positive spin is treated as right-to-left image motion: 0 -> 1 -> 2 -> 0.
  if (spin_direction > 0) return (panel_id + 1) % 3;
  // Negative spin is treated as left-to-right image motion: 0 -> 2 -> 1 -> 0.
  return (panel_id + 2) % 3;
}

bool is_legal_directional_candidate(
    int candidate_panel, int bound_panel_id, int spin_direction) {
  if (candidate_panel == bound_panel_id) return true;
  return candidate_panel == directional_successor(bound_panel_id, spin_direction);
}

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

}  // namespace

OutpostLegacyBindingPolicy::OutpostLegacyBindingPolicy(
    const UnifiedConfig & config, const RobotBindingProfile & profile)
    : config_(config),
      profile_(profile),
      z_offsets_{config.outpost.z_offset_0, config.outpost.z_offset_1,
                 config.outpost.z_offset_2},
      z_audit_(z_offsets_),
      periodic_(config_, z_offsets_),
      hypothesis_evaluator_(config_, profile_),
      confidence_scorer_(config_),
      binding_fsm_(config_),
      center_z_history_(std::max(1, config_.outpost.z_history_window)) {}

void OutpostLegacyBindingPolicy::reset(int init_panel_id,
                                       std::optional<double> obs_z) {
  const int selected_panel = std::clamp(init_panel_id, 0, 2);
  binding_fsm_.reset(selected_panel);
  z_audit_.reset();
  periodic_.reset();
  center_z_history_.set_window(std::max(1, config_.outpost.z_history_window));
  center_z_history_.reset();
  if (obs_z.has_value() && std::isfinite(obs_z.value())) {
    center_z_history_.push(obs_z.value() - z_offsets_[selected_panel]);
  }

  entropy_norm_ = 1.0;
  binding_confidence_ = 1.0 / 3.0;
  z_audit_conflict_count_ = 0;
}

OutpostLegacyBindingOutput OutpostLegacyBindingPolicy::step(
    const OutpostLegacyBindingInput & input) {
  if (input.obs == nullptr) {
    const auto fsm_out = binding_fsm_.output();
    OutpostLegacyBindingOutput out;
    out.binder_output.selected_id = fsm_out.selected_id;
    out.binder_output.bound_id = fsm_out.bound_id;
    out.binder_output.pending_id = fsm_out.pending_id;
    out.binder_output.height_label = fsm_out.height_label;
    out.binder_output.fsm_state = fsm_out.state;
    out.binder_output.action = fsm_out.action;
    out.binder_output.switch_occurred = fsm_out.switch_occurred;
    out.binder_output.switch_reason = fsm_out.switch_reason;
    out.binder_output.binding_confidence = binding_confidence_;
    return out;
  }

  const ObservationData & obs = *input.obs;
  const auto audit = z_audit_.update(obs);
  const double audit_confidence = z_audit_.confidence();

  auto hyps = hypothesis_evaluator_.evaluate(
      obs, input.predicted_center_pos, input.predicted_center_yaw,
      center_z_history_.median(), binding_fsm_.bound_id());
  hypothesis_evaluator_.apply_z_audit_prior(
      hyps, audit, audit_confidence, input.ambiguous_mode, entropy_norm_);

  hypothesis_evaluator_.compute_probabilities(hyps);
  const int best_idx_pre = hypothesis_evaluator_.best_index(hyps);
  const int second_idx_pre =
      hypothesis_evaluator_.second_index(hyps, best_idx_pre);
  const double candidate_prob_pre = hyps[best_idx_pre].probability;
  const double candidate_margin_pre =
      std::max(0.0, hyps[best_idx_pre].probability -
                        hyps[second_idx_pre].probability);
  const bool allow_period_update =
      candidate_prob_pre >=
          config_.outpost.binding_period_update_min_confidence &&
      candidate_margin_pre >= config_.outpost.binding_min_candidate_margin;

  periodic_.update(audit.z_jump, allow_period_update, input.yaw_rate_est);
  periodic_.apply_prior(hyps, audit.z_jump, binding_fsm_.bound_id());
  apply_directional_topology_prior(
      hyps, binding_fsm_.bound_id(), periodic_.spin_direction());
  hypothesis_evaluator_.compute_probabilities(hyps);

  const int best_idx = hypothesis_evaluator_.best_index(hyps);
  const int second_idx = hypothesis_evaluator_.second_index(hyps, best_idx);
  const int candidate_panel = hyps[best_idx].panel_id;
  const double candidate_prob = hyps[best_idx].probability;
  const double candidate_margin =
      std::max(0.0, hyps[best_idx].probability - hyps[second_idx].probability);

  const int current_idx =
      (binding_fsm_.bound_id() >= 0)
          ? hypothesis_evaluator_.hypothesis_index_for_panel(
                hyps, binding_fsm_.bound_id())
          : best_idx;
  const double same_panel_score = hypothesis_evaluator_.same_panel_score(
      hyps[current_idx], input.predicted_center_pos.z());
  const double switch_score = confidence_scorer_.switch_score(
      binding_fsm_.bound_id(), candidate_panel, audit.panel_id,
      periodic_.confidence(), candidate_prob, candidate_margin);

  OutpostBindingFSMInput fsm_input;
  fsm_input.candidate_id = candidate_panel;
  fsm_input.candidate_prob = candidate_prob;
  fsm_input.candidate_margin = candidate_margin;
  fsm_input.same_panel_score = same_panel_score;
  fsm_input.switch_score = switch_score;
  auto fsm_out = binding_fsm_.step(fsm_input);

  const bool z_audit_conflicts =
      config_.outpost.z_audit_rebind_enable && audit.panel_id >= 0 &&
      binding_fsm_.bound_id() >= 0 && audit.panel_id != binding_fsm_.bound_id();
  const bool z_audit_topology_legal =
      periodic_.spin_direction() == 0 ||
      is_legal_directional_candidate(
          audit.panel_id, binding_fsm_.bound_id(), periodic_.spin_direction());
  const double min_rebind_conf =
      std::clamp(config_.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
  const double min_rebind_jump =
      std::max(0.0, config_.outpost.z_audit_rebind_min_jump);
  const bool z_audit_has_jump =
      std::isfinite(audit.z_jump) && std::abs(audit.z_jump) >= min_rebind_jump;
  const bool z_audit_strong_level =
      audit_confidence >= std::min(1.0, min_rebind_conf + 0.25);
  if (z_audit_conflicts && z_audit_topology_legal &&
      audit_confidence >= min_rebind_conf &&
      (z_audit_has_jump || z_audit_strong_level)) {
    ++z_audit_conflict_count_;
  } else if (!z_audit_conflicts) {
    z_audit_conflict_count_ = 0;
  }

  const int z_rebind_required =
      std::max(1, config_.outpost.z_audit_rebind_confirm_frames);
  if (z_audit_conflict_count_ >= z_rebind_required) {
    fsm_out = binding_fsm_.force_rebind(audit.panel_id, 5);
    z_audit_conflict_count_ = 0;
  }

  const int selected_idx = hypothesis_evaluator_.hypothesis_index_for_panel(
      hyps, fsm_out.selected_id);
  const double selected_panel_score = hypothesis_evaluator_.same_panel_score(
      hyps[selected_idx], input.predicted_center_pos.z());
  const bool binding_conflict_for_update =
      confidence_scorer_.conflict_for_update(candidate_panel, fsm_out.selected_id,
                                             audit.panel_id);
  binding_confidence_ = confidence_scorer_.binding_confidence(
      hyps[selected_idx].probability, candidate_margin, selected_panel_score,
      switch_score, periodic_.confidence(),
      fsm_out.state == BindingFSMState::PENDING_SWITCH);

  entropy_norm_ = hypothesis_evaluator_.entropy_norm(hyps);
  center_z_history_.push(hyps[selected_idx].center_z);

  return build_output(fsm_out, audit, hyps, candidate_panel, candidate_prob,
                      candidate_margin, selected_idx, selected_panel_score,
                      switch_score, binding_confidence_,
                      binding_conflict_for_update);
}

void OutpostLegacyBindingPolicy::apply_directional_topology_prior(
    std::array<BindingHypothesis, 3> & hyps,
    int bound_panel_id,
    int spin_direction) const {
  if (bound_panel_id < 0 || spin_direction == 0) return;

  const double base_weight =
      std::max(0.0, config_.outpost.binding_topology_prior_weight);
  if (base_weight <= 0.0) return;

  const double period_conf = clamp01(periodic_.confidence());
  const double prior_weight = base_weight * (0.5 + 0.5 * period_conf);
  for (auto & hyp : hyps) {
    if (!is_legal_directional_candidate(
            hyp.panel_id, bound_panel_id, spin_direction)) {
      hyp.cost += prior_weight;
    }
  }
}

OutpostLegacyBindingOutput OutpostLegacyBindingPolicy::build_output(
    const OutpostBindingFSMOutput & fsm_out,
    const OutpostZAuditResult & audit,
    const std::array<BindingHypothesis, 3> & hyps,
    int candidate_panel,
    double candidate_prob,
    double candidate_margin,
    int selected_idx,
    double selected_panel_score,
    double switch_score,
    double binding_confidence,
    bool binding_conflict_for_update) const {
  OutpostLegacyBindingOutput out;
  out.candidate_id = candidate_panel;
  out.candidate_prob = candidate_prob;
  out.candidate_margin = candidate_margin;
  out.max_prob = hyps[selected_idx].probability;
  out.entropy_norm = entropy_norm_;
  out.selected_xy_residual = hyps[selected_idx].xy_residual;
  out.hypotheses = hyps;
  out.z_audit_panel_id = audit.panel_id;
  out.z_audit_confidence = z_audit_.confidence();
  out.z_jump = audit.z_jump;
  out.dz_from_center = audit.dz_from_center;
  out.z_audit_costs = audit.costs;
  out.period_confidence = periodic_.confidence();
  out.period_phase = periodic_.phase();
  out.spin_direction = periodic_.spin_direction();
  out.dz_small_est = periodic_.dz_small_est();
  out.dz_large_est = periodic_.dz_large_est();
  out.transition_state = fsm_out.transition_state;
  out.transition_candidate_id = fsm_out.pending_id;

  out.binder_output.selected_id = fsm_out.selected_id;
  out.binder_output.bound_id = fsm_out.bound_id;
  out.binder_output.pending_id = fsm_out.pending_id;
  out.binder_output.height_label = fsm_out.height_label;
  out.binder_output.fsm_state = fsm_out.state;
  out.binder_output.action = fsm_out.action;
  out.binder_output.switch_occurred = fsm_out.switch_occurred;
  out.binder_output.switch_reason = fsm_out.switch_reason;
  out.binder_output.binding_confidence = binding_confidence;
  out.binder_output.binding_conflict_for_update = binding_conflict_for_update;
  out.binder_output.same_panel_score = selected_panel_score;
  out.binder_output.switch_score = switch_score;

  return out;
}

}  // namespace fyt::auto_aim::binder
