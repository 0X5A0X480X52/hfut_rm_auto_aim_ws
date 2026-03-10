// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"

#include <cmath>
#include <stdexcept>
#include <iostream>

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
      maneuver_detector_(config.maneuver) {}

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

  // Reset detectors on every (re-)initialization
  mismatch_detector_.reset();
  height_identifier_.reset();

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
    double d = obs_time.value() - current_time_.value();
    if (d > min_dt_) predict(obs_time.value());
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
  std::cout << "Updating with single observation: x=" << obs.x << " y=" << obs.y
            << " z=" << obs.z << " yaw=" << obs.yaw << std::endl;
  auto idx = ukf_.state_idx();

  auto [panel_id, center_yaw, matching_error] =
      panel_associator_.associate_panel(
          obs.yaw, reference_center_yaw_, obs.z, ukf_.x()(idx.Z()),
          obs.x, obs.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
          ukf_.x()(idx.R1()), ukf_.x()(idx.R2()));

  current_panel_id_ = panel_id;
  std::string r_type = PanelAssociator::get_r_type(panel_id);

  auto [h_label, h_conf] = height_identifier_.identify_single(
      obs.z, panel_id, ukf_.x()(idx.Z()), ukf_.x()(idx.DZA()),
      ukf_.is_dza_converged());
  height_label_ = h_label;
  height_confidence_ = h_conf;

  std::string armor_layer;
  if (h_label == HeightLabel::UPPER)
    armor_layer = "upper";
  else if (h_label == HeightLabel::LOWER)
    armor_layer = "lower";
  // else stays empty → UKF will infer

  double pos_conf = (override_pos_confidence >= 0.0)
                        ? override_pos_confidence
                        : compute_position_confidence(armor_layer, h_conf, r_type);

  double panel_angle = panel_id * (M_PI / 2.0);

  bool ok = ukf_.update({obs}, {r_type}, {armor_layer}, h_conf, pos_conf,
                        panel_angle);
  if (!ok) return false;

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

    if (result.action == PanelMismatchDetector::Action::REINIT) {
      std::cerr << "[AdaptiveArmorTracker] mismatch REINIT triggered "
                << "panel_id=" << panel_id
                << " -> " << result.new_panel_id << "\n";
      reinitialize_tracker(obs);
      // Return true: update itself succeeded; caller sees a valid (reinit) state
    } else if (result.action == PanelMismatchDetector::Action::PATCH) {
      std::cerr << "[AdaptiveArmorTracker] mismatch PATCH triggered "
                << "panel_id=" << panel_id
                << " -> " << result.new_panel_id << "\n";
      correct_panel_id(result.new_panel_id, obs.yaw);
    }
  }

  return true;
}

/* ================================================================ */
/*  Dual-observation update                                          */
/* ================================================================ */

bool AdaptiveArmorTracker::update_dual(const ObservationData &obs1,
                                       const ObservationData &obs2) {
  std::cout << "Updating with dual observations:\n"
            << "  obs1: x=" << obs1.x << " y=" << obs1.y << " z=" << obs1.z
            << " yaw=" << obs1.yaw << "\n"
            << "  obs2: x=" << obs2.x << " y=" << obs2.y << " z=" << obs2.z
            << " yaw=" << obs2.yaw << std::endl;

  // First: single update on obs1 with full position confidence
  bool single_ok = update_single(obs1, 1.0);
  if (!single_ok) {
    std::cout << "[adaptive_tracker::update_dual] update_single(obs1) failed\n";
    return false;
  }

  auto idx = ukf_.state_idx();

  auto [pid1, cw1, _e1] = panel_associator_.associate_panel(
      obs1.yaw, reference_center_yaw_, obs1.z, ukf_.x()(idx.Z()),
      obs1.x, obs1.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
      ukf_.x()(idx.R1()), ukf_.x()(idx.R2()));
  auto [pid2, cw2, _e2] = panel_associator_.associate_panel(
      obs2.yaw, reference_center_yaw_, obs2.z, ukf_.x()(idx.Z()),
      obs2.x, obs2.y, ukf_.x()(idx.X()), ukf_.x()(idx.Y()),
      ukf_.x()(idx.R1()), ukf_.x()(idx.R2()));

  std::string rt1 = PanelAssociator::get_r_type(pid1);
  std::string rt2 = PanelAssociator::get_r_type(pid2);

  std::cout << "identify_dual" << std::endl;
  auto [l1, l2, h_conf] = height_identifier_.identify_dual(obs1.z, obs2.z);
  std::cout << "identify_dual" << std::endl;
  height_confidence_ = h_conf;

  bool dual_ok = ukf_.update({obs1, obs2}, {rt1, rt2}, {l1, l2}, h_conf);
  if (!dual_ok) {
    std::cout << "[adaptive_tracker::update_dual] ukf_.update(dual) failed "  \
      "rt1=" << rt1 << " rt2=" << rt2 << " l1=" << l1 << " l2=" << l2 << " h_conf=" << h_conf << std::endl;
  }
  return dual_ok;
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
double AdaptiveArmorTracker::get_dza() const { return ukf_.get_dza(); }
int AdaptiveArmorTracker::get_k() const { return ukf_.get_k(); }
double AdaptiveArmorTracker::get_delta() const { return ukf_.get_delta(); }

}  // namespace fyt::auto_aim
