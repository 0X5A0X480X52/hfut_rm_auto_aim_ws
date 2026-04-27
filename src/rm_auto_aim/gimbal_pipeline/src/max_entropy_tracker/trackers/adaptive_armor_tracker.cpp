// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

AdaptiveArmorTracker::AdaptiveArmorTracker(const UnifiedConfig &config,
                                           double dt,
                                           bool enable_oscillation)
    : BaseTracker(dt),
      config_(config),
      ukf_(config, dt),
      osc_detector_(50, 0.05, 5, 100, enable_oscillation),
      mismatch_detector_(config.panel_mismatch.window_size,
                         config.panel_mismatch.threshold_t1,
                         config.panel_mismatch.confirm_count,
                         config.panel_mismatch.reinit_count,
                 config.panel_mismatch.enable),
        maneuver_detector_(config.maneuver) {
      panel_associator_.configure_periodic_binding(
        config_.tracker.periodic_binding_enable,
        config_.tracker.periodic_binding_weight,
        config_.tracker.periodic_binding_spin_rate_gate);
    }

ManeuverResult AdaptiveArmorTracker::assess_maneuver() const {
  const double innov_norm = ukf_.last_innov_xyz().size() >= 3
                                ? ukf_.last_innov_xyz().norm()
                                : 0.0;
  return maneuver_detector_.detect(
      ukf_.last_nis(), innov_norm, ukf_.last_update_type());
}

/* ================================================================ */
/*  Initialize                                                       */
/* ================================================================ */

void AdaptiveArmorTracker::initialize(const std::vector<ObservationData> &obs,
                                      double r1, double r2, double dza) {
  if (obs.empty()) throw std::invalid_argument("At least one observation required");

  panel_associator_.reset_history();

  const auto &o = obs.front();
  auto [panel_id, center_yaw, _err] =
      panel_associator_.associate_panel(o.yaw, std::nullopt);

  current_panel_id_ = panel_id;
  reference_center_yaw_ = center_yaw;

  // Cache defaults for potential re-initialization
  default_r1_  = r1;
  default_r2_  = r2;
  default_dza_ = dza;

  ukf_.initialize(obs, r1, r2, dza, panel_id);
  ukf_.set_structural_noise_scales(1.0, 1.0);

  // Reset detectors on every (re-)initialization
  mismatch_detector_.reset();
  height_identifier_.reset();

  single_obs_streak_ = 0;
  degraded_single_obs_mode_ = false;
  reset_jump_binding(panel_id, default_label_from_panel(panel_id),
                     o.z, o.timestamp);

  if (o.timestamp.has_value()) {
    current_time_ = o.timestamp.value();
    last_update_time_ = o.timestamp.value();
  }

  mark_initialized();
  transition_to(TrackerState::INITIALIZING);
  increment_frame();
}

/* ================================================================ */
/*  Predict                                                          */
/* ================================================================ */

void AdaptiveArmorTracker::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;

  double dt = compute_dt(target_time);
  ukf_.predict(dt);

  if (target_time.has_value())
    current_time_ = target_time.value();
  else if (current_time_.has_value())
    current_time_ = current_time_.value() + dt;

  // When in TEMP_LOST state, decay velocity and acceleration to prevent
  // runaway prediction from CA model. Each predict step multiplies by
  // decay_factor, so velocity exponentially decays toward zero.
  if (is_temp_lost()) {
    const double decay_factor = 0.8;  // ~20% decay per frame
    auto idx = ukf_.state_idx();
    auto &x = ukf_.x();
    x(idx.VX()) *= decay_factor;
    x(idx.VY()) *= decay_factor;
    x(idx.VZ()) *= decay_factor;
    x(idx.DELTA_RATE()) *= decay_factor;
    if (idx.has("AX")) {
      x(idx.AX()) *= decay_factor;
      x(idx.AY()) *= decay_factor;
      x(idx.AZ()) *= decay_factor;
    }
  }

  auto [r1, r2] = ukf_.get_radii();
  if (osc_detector_.update(r1, r2)) reset_parameters();

  reference_center_yaw_ = ukf_.get_yaw();
}

/* ================================================================ */
/*  Update                                                           */
/* ================================================================ */

bool AdaptiveArmorTracker::update(const std::vector<ObservationData> &obs) {
  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
    std::cout << "Observation empty or tracker not initialized, transitioning to "
                 << "state=" << static_cast<int>(state()) << std::endl;
    return false;
  }

  // Auto-predict to observation time
  std::optional<double> obs_time;
  for (const auto &o : obs)
    if (o.timestamp.has_value()) {
      if (!obs_time.has_value() || o.timestamp.value() > obs_time.value())
        obs_time = o.timestamp;
    }

  if (obs_time.has_value() && current_time_.has_value()) {
    update_degraded_single_obs_mode(obs.size() == 1);

    double d = obs_time.value() - current_time_.value();
    if (d > min_dt_) predict(obs_time.value());
  } else {
    update_degraded_single_obs_mode(obs.size() == 1);
  }

  handle_observation_received(config_.tracker.tracking_thres);

  bool success = (obs.size() == 1) ? update_single(obs[0])
                                   : update_dual(obs[0], obs[1]);

  if (obs_time.has_value()) update_time(obs_time.value());
  increment_frame();
  if (success) reference_center_yaw_ = ukf_.get_yaw();
  return success;
}

/* ================================================================ */
/*  Single-observation update                                        */
/* ================================================================ */

bool AdaptiveArmorTracker::update_single(const ObservationData &obs,
                                         double override_pos_confidence) {
  auto idx = ukf_.state_idx();
  const double yaw_rate_hint = ukf_.x()(idx.DELTA_RATE());
  const double dz_unit_hint = std::abs(ukf_.x()(idx.DZA()));

  PanelAssociator::AssociationDiagnostics assoc_diag;
  auto [candidate_panel, center_yaw, matching_error] =
      panel_associator_.associate_panel(
          obs.yaw, reference_center_yaw_, obs.z, ukf_.x()(idx.Z()),
          obs.x, obs.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
          ukf_.x()(idx.R1()), ukf_.x()(idx.R2()), yaw_rate_hint,
          dz_unit_hint, &assoc_diag);

  auto [h_label, h_conf] = height_identifier_.identify_single(
      obs.z, candidate_panel, ukf_.x()(idx.Z()), ukf_.x()(idx.DZA()),
      ukf_.is_dza_converged());

  HeightLabel candidate_label =
      (h_label == HeightLabel::UNKNOWN)
          ? default_label_from_panel(candidate_panel)
          : h_label;
  int panel_id = candidate_panel;
  HeightLabel resolved_label = candidate_label;
  double resolved_h_conf = h_conf;

  if (config_.tracker.jump_binding_enable) {
    update_jump_binding(obs, candidate_panel, assoc_diag, candidate_label,
                        h_conf, &panel_id, &resolved_label, &resolved_h_conf);
  } else {
    bound_panel_id_ = panel_id;
    bound_height_label_ = resolved_label;
    bound_confidence_ = std::clamp(resolved_h_conf, 0.0, 1.0);
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    last_obs_z_ = obs.z;
    last_obs_time_ = obs.timestamp;
    last_panel_id_ = panel_id;
  }

  current_panel_id_ = panel_id;
  height_label_ = resolved_label;
  height_confidence_ = resolved_h_conf;

  std::string r_type = PanelAssociator::get_r_type(panel_id);

  std::string armor_layer;
  if (resolved_label == HeightLabel::UPPER)
    armor_layer = "upper";
  else if (resolved_label == HeightLabel::LOWER)
    armor_layer = "lower";
  else
    armor_layer = PanelAssociator::get_default_layer(panel_id);

  double pos_conf = (override_pos_confidence >= 0.0)
                        ? override_pos_confidence
                        : compute_position_confidence(
                              armor_layer,
                              std::max(resolved_h_conf, bound_confidence_),
                              r_type);

  double panel_angle = panel_id * (M_PI / 2.0);

  bool ok = ukf_.update({obs}, {r_type}, {armor_layer},
                        std::max(resolved_h_conf, bound_confidence_),
                        pos_conf,
                        panel_angle);
  if (!ok) {
    std::cerr << "[adaptive_tracker::update_single] ukf_.update failed "
              << "panel_id=" << panel_id << " r_type=" << r_type
              << " armor_layer=" << armor_layer << " h_conf=" << h_conf
              << " pos_conf=" << pos_conf << std::endl;
    return false;
  }

  // ── Post-update mismatch detection ──
  // Run only when the filter has had a chance to estimate dza (dza_converged).
  // The z-innovation (innov(2)) is available immediately after update().
  {
    double z_innov = ukf_.last_z_innovation();
    auto result = mismatch_detector_.update(
        panel_id, obs.z,
        ukf_.x()(idx.Z()), ukf_.x()(idx.DZA()),
        armor_layer, ukf_.is_dza_converged(),
        z_innov);

    if (result.action != PanelMismatchDetector::Action::NONE) {
      if (result.action == PanelMismatchDetector::Action::REINIT) {
        std::cerr << "[AdaptiveArmorTracker] mismatch REINIT detected "
                  << "panel_id=" << panel_id
                  << " -> " << result.new_panel_id << "\n";
      } else {
        std::cerr << "[AdaptiveArmorTracker] mismatch PATCH detected "
                  << "panel_id=" << panel_id
                  << " -> " << result.new_panel_id << "\n";
      }

      if (config_.panel_mismatch.apply_correction) {
        if (result.action == PanelMismatchDetector::Action::REINIT) {
          reinitialize_tracker(obs);
        } else if (result.action == PanelMismatchDetector::Action::PATCH) {
          correct_panel_id(result.new_panel_id, obs.yaw);
        }
      } else {
        std::cerr << "[AdaptiveArmorTracker] mismatch correction suppressed by "
                     "panel_mismatch.apply_correction=false\n";
      }
    }
  }

  return true;
}

/* ================================================================ */
/*  Dual-observation update                                          */
/* ================================================================ */

bool AdaptiveArmorTracker::update_dual(const ObservationData &obs1,
                                       const ObservationData &obs2) {
  // std::cout << "Updating with dual observations:\n"
  //           << "  obs1: x=" << obs1.x << " y=" << obs1.y << " z=" << obs1.z
  //           << " yaw=" << obs1.yaw << "\n"
  //           << "  obs2: x=" << obs2.x << " y=" << obs2.y << " z=" << obs2.z
  //           << " yaw=" << obs2.yaw << std::endl;

  // First: single update on obs1 with full position confidence
  bool single_ok = update_single(obs1, 1.0);
  if (!single_ok) {
    std::cerr << "[adaptive_tracker::update_dual] update_single(obs1) failed\n";
    return false;
  }

  auto idx = ukf_.state_idx();
    const double yaw_rate_hint = ukf_.x()(idx.DELTA_RATE());
    const double dz_unit_hint = std::abs(ukf_.x()(idx.DZA()));

  auto [pid1, cw1, _e1] = panel_associator_.associate_panel(
      obs1.yaw, reference_center_yaw_, obs1.z, ukf_.x()(idx.Z()),
      obs1.x, obs1.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
      ukf_.x()(idx.R1()), ukf_.x()(idx.R2()), yaw_rate_hint,
      dz_unit_hint);
  auto [pid2, cw2, _e2] = panel_associator_.associate_panel(
      obs2.yaw, reference_center_yaw_, obs2.z, ukf_.x()(idx.Z()),
      obs2.x, obs2.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
      ukf_.x()(idx.R1()), ukf_.x()(idx.R2()), yaw_rate_hint,
      dz_unit_hint);

  std::string rt1 = PanelAssociator::get_r_type(pid1);
  std::string rt2 = PanelAssociator::get_r_type(pid2);

  // std::cout << "identify_dual" << std::endl;
  auto [l1, l2, h_conf] = height_identifier_.identify_dual(obs1.z, obs2.z);
  // std::cout << "identify_dual" << std::endl;
  height_confidence_ = h_conf;

  bool dual_ok = ukf_.update({obs1, obs2}, {rt1, rt2}, {l1, l2}, h_conf);
  if (!dual_ok) {
    std::cerr << "[adaptive_tracker::update_dual] ukf_.update(dual) failed "  \
      "rt1=" << rt1 << " rt2=" << rt2 << " l1=" << l1 << " l2=" << l2 << " h_conf=" << h_conf << std::endl;
  }
  return dual_ok;
}

HeightLabel AdaptiveArmorTracker::default_label_from_panel(int panel_id) {
  return (panel_id % 2 == 0) ? HeightLabel::LOWER : HeightLabel::UPPER;
}

std::string AdaptiveArmorTracker::label_to_layer(HeightLabel label) {
  if (label == HeightLabel::UPPER) return "upper";
  if (label == HeightLabel::LOWER) return "lower";
  return "";
}

void AdaptiveArmorTracker::reset_jump_binding(int panel_id, HeightLabel label,
                                              std::optional<double> obs_z,
                                              std::optional<double> obs_time) {
  bound_panel_id_ = panel_id;
  bound_height_label_ = (label == HeightLabel::UNKNOWN)
                            ? default_label_from_panel(panel_id)
                            : label;
  bound_confidence_ = 0.5;
  binding_transition_state_ = BindingTransitionState::LOCKED;
  transition_candidate_panel_ = -1;
  transition_confirm_count_ = 0;
  switch_cooldown_frames_ = 0;
  last_panel_id_ = panel_id;
  last_obs_z_ = obs_z;
  last_obs_time_ = obs_time;
  z_jump_history_.clear();
  dz_jump_est_ = std::numeric_limits<double>::quiet_NaN();
}

void AdaptiveArmorTracker::update_degraded_single_obs_mode(bool is_single_obs) {
  if (!config_.tracker.degraded_single_obs_enable) {
    single_obs_streak_ = 0;
    degraded_single_obs_mode_ = false;
    ukf_.set_structural_noise_scales(1.0, 1.0);
    return;
  }

  if (is_single_obs)
    ++single_obs_streak_;
  else
    single_obs_streak_ = 0;

  const bool dza_not_converged = !ukf_.is_dza_converged();
  const int streak_thres = std::max(1, config_.tracker.degraded_single_obs_streak);
  degraded_single_obs_mode_ =
      is_single_obs && dza_not_converged && single_obs_streak_ >= streak_thres;

  if (degraded_single_obs_mode_) {
    ukf_.set_structural_noise_scales(config_.tracker.degraded_q_scale_r,
                                     config_.tracker.degraded_q_scale_dza);
  } else {
    ukf_.set_structural_noise_scales(1.0, 1.0);
  }
}

HeightLabel AdaptiveArmorTracker::resolve_layer_from_jump(
    HeightLabel fallback_label, double z_jump, bool has_z_jump) const {
  if (!has_z_jump) return fallback_label;

  const double dz_gate = std::max(0.0, config_.tracker.jump_binding_dz_gate);
  if (z_jump > dz_gate) return HeightLabel::UPPER;
  if (z_jump < -dz_gate) return HeightLabel::LOWER;
  return fallback_label;
}

void AdaptiveArmorTracker::update_jump_statistics(double z_jump,
                                                  bool switch_confirmed) {
  if (!switch_confirmed) return;

  const double abs_jump = std::abs(z_jump);
  if (abs_jump < std::max(0.0, config_.tracker.jump_binding_z_jump_min)) return;

  z_jump_history_.push_back(abs_jump);
  while (z_jump_history_.size() > 20) {
    z_jump_history_.pop_front();
  }

  const double alpha = std::clamp(config_.tracker.jump_binding_dz_ema_alpha,
                                  0.01, 1.0);
  if (!std::isfinite(dz_jump_est_)) {
    dz_jump_est_ = abs_jump;
  } else {
    dz_jump_est_ = (1.0 - alpha) * dz_jump_est_ + alpha * abs_jump;
  }
}

double AdaptiveArmorTracker::compute_jump_binding_confidence(
    const PanelAssociator::AssociationDiagnostics &diag,
    bool jump_gate_passed) const {
  const double margin_base = std::max(1e-3, config_.tracker.jump_binding_cost_margin_min);
  const double yaw_gate = std::max(1e-3, config_.tracker.jump_binding_yaw_err_gate);

  const double margin = std::isfinite(diag.cost_margin) ? diag.cost_margin : 0.0;
  const double margin_score = std::clamp(margin / (2.0 * margin_base), 0.0, 1.0);

  const double yaw_err = std::isfinite(diag.selected_yaw_err)
                             ? diag.selected_yaw_err
                             : yaw_gate;
  const double yaw_score =
      std::clamp(1.0 - yaw_err / yaw_gate, 0.0, 1.0);

  const double jump_score = jump_gate_passed ? 1.0 : 0.0;
  const double base = std::clamp(0.50 * margin_score + 0.30 * yaw_score +
                                     0.20 * jump_score,
                                 0.0, 1.0);

  const double floor = std::clamp(config_.tracker.jump_binding_confidence_floor,
                                  0.0, 0.95);
  return floor + (1.0 - floor) * base;
}

bool AdaptiveArmorTracker::update_jump_binding(
    const ObservationData &obs, int candidate_panel,
    const PanelAssociator::AssociationDiagnostics &diag,
    HeightLabel candidate_label, double candidate_height_conf,
    int *selected_panel, HeightLabel *selected_label,
    double *selected_height_conf) {
  if (selected_panel == nullptr || selected_label == nullptr ||
      selected_height_conf == nullptr) {
    return false;
  }

  if (bound_panel_id_ < 0) {
    reset_jump_binding(candidate_panel, candidate_label, obs.z, obs.timestamp);
  }

  const bool has_z_jump = last_obs_z_.has_value();
  const double z_jump = has_z_jump ? (obs.z - last_obs_z_.value()) : 0.0;

  const int raw_diff = std::abs(candidate_panel - bound_panel_id_);
  const bool adjacent_panel = (raw_diff == 1 || raw_diff == 3);
  const bool jump_mag_ok =
      has_z_jump &&
      (std::abs(z_jump) >= std::max(0.0, config_.tracker.jump_binding_z_jump_min));

  bool dz_match_ok = true;
  if (jump_mag_ok && std::isfinite(dz_jump_est_)) {
    dz_match_ok = std::abs(std::abs(z_jump) - dz_jump_est_) <=
                  std::max(0.0, config_.tracker.jump_binding_dz_match_tolerance);
  }

  const double yaw_err =
      std::isfinite(diag.selected_yaw_err) ? diag.selected_yaw_err : 1e9;
  const bool yaw_ok =
      yaw_err <= std::max(1e-3, config_.tracker.jump_binding_yaw_err_gate);

  const double margin = std::isfinite(diag.cost_margin) ? diag.cost_margin : 0.0;
  const bool margin_ok =
      margin >= std::max(0.0, config_.tracker.jump_binding_cost_margin_min);

  const bool jump_gate_passed =
      adjacent_panel && jump_mag_ok && dz_match_ok && yaw_ok && margin_ok;

  bool switch_confirmed = false;
  const int confirm_required =
      std::max(1, config_.tracker.jump_binding_confirm_frames);

  if (switch_cooldown_frames_ > 0) {
    --switch_cooldown_frames_;
  }

  if (candidate_panel == bound_panel_id_) {
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
  } else if (switch_cooldown_frames_ == 0 && jump_gate_passed) {
    if (binding_transition_state_ == BindingTransitionState::LOCKED) {
      if (confirm_required <= 1) {
        switch_confirmed = true;
      } else {
        binding_transition_state_ = BindingTransitionState::TRANSITION_CANDIDATE;
        transition_candidate_panel_ = candidate_panel;
        transition_confirm_count_ = 1;
      }
    } else if (candidate_panel == transition_candidate_panel_) {
      ++transition_confirm_count_;
      if (transition_confirm_count_ >= confirm_required) {
        switch_confirmed = true;
      }
    } else {
      transition_candidate_panel_ = candidate_panel;
      transition_confirm_count_ = 1;
    }
  } else {
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
  }

  if (switch_confirmed) {
    bound_panel_id_ = candidate_panel;
    const HeightLabel fallback_label = default_label_from_panel(bound_panel_id_);
    bound_height_label_ = resolve_layer_from_jump(fallback_label, z_jump, has_z_jump);
    if (bound_height_label_ == HeightLabel::UNKNOWN) {
      bound_height_label_ = (candidate_label == HeightLabel::UNKNOWN)
                                ? fallback_label
                                : candidate_label;
    }

    update_jump_statistics(z_jump, true);
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_cooldown_frames_ =
        std::max(0, config_.tracker.jump_binding_switch_cooldown);
  }

  if (bound_height_label_ == HeightLabel::UNKNOWN) {
    bound_height_label_ = (candidate_label == HeightLabel::UNKNOWN)
                              ? default_label_from_panel(bound_panel_id_)
                              : candidate_label;
  }

  bound_confidence_ = compute_jump_binding_confidence(diag, jump_gate_passed);

  *selected_panel = bound_panel_id_;
  *selected_label = bound_height_label_;
  *selected_height_conf = std::clamp(
      std::max(candidate_height_conf, bound_confidence_), 0.0, 1.0);

  last_obs_z_ = obs.z;
  last_obs_time_ = obs.timestamp;
  last_panel_id_ = candidate_panel;
  return switch_confirmed;
}

/* ================================================================ */
/*  Helpers                                                          */
/* ================================================================ */

double AdaptiveArmorTracker::compute_position_confidence(
    const std::string &armor_layer, double height_confidence,
    const std::string &r_type) const {
  if (armor_layer.empty()) return 0.1;

  auto idx = ukf_.state_idx();
  int r_idx = (r_type == "r1") ? idx.R1() : idx.R2();
  double r_std = std::sqrt(ukf_.P()(r_idx, r_idx));

  double param_conf;
  if (r_std < 0.01)
    param_conf = 1.0;
  else if (r_std < 0.05)
    param_conf = 1.0 - (r_std - 0.01) * (0.4 / 0.04);
  else if (r_std < 0.1)
    param_conf = 0.6 - (r_std - 0.05) * (0.3 / 0.05);
  else
    param_conf = 0.3;

  return std::sqrt(height_confidence * param_conf);
}

void AdaptiveArmorTracker::reset_parameters() {
  auto [r1, r2] = osc_detector_.get_reset_values();
  auto idx = ukf_.state_idx();
  ukf_.x()(idx.R1()) = r1;
  ukf_.x()(idx.R2()) = r2;
}

/* ================================================================ */
/*  Panel mismatch correction                                        */
/* ================================================================ */

void AdaptiveArmorTracker::correct_panel_id(int new_panel_id,
                                             double armor_yaw) {
  // 1. Recompute center_yaw with the corrected panel offset
  double new_center_yaw =
      normalize_angle(armor_yaw - new_panel_id * (M_PI / 2.0));

  // 2. Update tracker-level bookkeeping
  current_panel_id_     = new_panel_id;
  reference_center_yaw_ = new_center_yaw;

  // 3. Patch the UKF state (swap R1/R2, update delta/k, inflate covariances)
  ukf_.apply_panel_correction(new_center_yaw);

  // 4. Seed the HeightIdentifier with the correct layer for the new panel
  //    even panels → lower;  odd panels → upper
  HeightLabel correct_label = (new_panel_id % 2 == 0)
                                   ? HeightLabel::LOWER
                                   : HeightLabel::UPPER;
  height_identifier_.reset_with_hint(correct_label);

  // 5. Reset the mismatch detector's sliding window to avoid re-triggering on
  //    stale data accumulated under the wrong panel assumption
  mismatch_detector_.reset();
  reset_jump_binding(new_panel_id, correct_label, std::nullopt, std::nullopt);
}

void AdaptiveArmorTracker::reinitialize_tracker(const ObservationData &obs) {
  // Full re-initialization loses velocity/acceleration estimates but gives a
  // clean state without any panel-binding artefacts.
  // initialize() resets mismatch_detector_ and height_identifier_ internally.
  initialize({obs}, default_r1_, default_r2_, default_dza_);
}

/* ================================================================ */
/*  State queries                                                    */
/* ================================================================ */

Eigen::Vector3d AdaptiveArmorTracker::get_center_position() const {
  return ukf_.get_center_position();
}
double AdaptiveArmorTracker::get_yaw() const { return ukf_.get_yaw(); }
std::pair<double, double> AdaptiveArmorTracker::get_radii() const {
  return ukf_.get_radii();
}

AdaptiveArmorTracker::DebugSnapshot AdaptiveArmorTracker::debug_snapshot() const {
  DebugSnapshot snapshot;
  snapshot.valid = is_initialized();
  snapshot.current_panel_id = current_panel_id_;
  snapshot.bound_panel_id = bound_panel_id_;
  snapshot.current_height_label = static_cast<int>(height_label_);
  snapshot.bound_height_label = static_cast<int>(bound_height_label_);
  snapshot.binding_transition_state =
      static_cast<int>(binding_transition_state_);
  snapshot.transition_candidate_panel = transition_candidate_panel_;
  snapshot.transition_confirm_count = transition_confirm_count_;
  snapshot.switch_cooldown_frames = switch_cooldown_frames_;
  snapshot.bound_confidence = bound_confidence_;
  snapshot.height_confidence = height_confidence_;
  snapshot.degraded_single_obs_mode = degraded_single_obs_mode_;
  snapshot.single_obs_streak = single_obs_streak_;
  snapshot.dz_jump_est = dz_jump_est_;
  return snapshot;
}

}  // namespace fyt::auto_aim
