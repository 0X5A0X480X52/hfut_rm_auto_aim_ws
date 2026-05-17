// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_TYPES_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace fyt::auto_aim::outpost_v3 {

static constexpr int kNumPanels = 3;

struct OutpostPanelGeometry {
  double radius = 0.26;
  std::array<double, kNumPanels> z_offsets{0.06, 0.0, -0.06};
  std::array<double, kNumPanels> panel_angles{0.0, 2.0 * M_PI / 3.0,
                                               -2.0 * M_PI / 3.0};
};

struct OutpostHypothesis {
  int obs_index = 0;
  int panel_id = -1;
  double prior_log_weight = 0.0;
  std::string debug_name;
};

struct OutpostGateConfig {
  double single_total_nis = 11.34;
  double single_pos_chi2 = 9.0;
};

struct OutpostHypothesisSelectorConfig {
  int topk = 3;
  double min_top1_confidence = 0.5;
  double min_top1_top2_margin = 1.0;
  double max_reconstruction_pos_error = 0.3;
};

struct OutpostPosteriorSanityConfig {
  double max_center_jump = 0.5;
  double max_yaw_jump = 0.5;
  double max_yaw_rate = 15.0;
  double max_yaw_acc = 30.0;
};

struct OutpostModeRoutingConfig {
  double P_enter_structured = 0.7;
  double M_enter_structured = 1.5;
  int stable_frames = 5;
  double P_exit_structured = 0.4;
  double M_exit_structured = 0.5;
  int degraded_frames = 10;
};

struct OutpostPriorConfig {
  double panel_switch_penalty = 0.5;
};

struct OutpostInitialPConfig {
  double pos = 0.01;
  double vel = 1.0;
  double acc = 10.0;
  double yaw = 0.1;
  double yaw_rate = 1.0;
  double yaw_acc = 5.0;
};

struct OutpostProcessNoiseConfig {
  double acc = 2.0;
  double yaw_acc = 3.0;
};

struct OutpostObservationNoiseConfig {
  double sigma_pos_xy = 0.02;
  double sigma_pos_z = 0.03;
};

struct OutpostWarmupConfig {
  bool enable = true;
  int warmup_frames = 8;
  int min_settle_frames = 3;
  double min_margin_to_commit = 1.2;
  double min_confidence_to_commit = 0.65;
};

struct OutpostPhaseAuditConfig {
  bool enable = true;
  double min_jump = 0.015;
  double dz_gate = 0.035;
  int confirm_frames = 2;
};

struct OutpostV3Config {
  OutpostPanelGeometry geometry;
  OutpostGateConfig gate;
  OutpostHypothesisSelectorConfig hypothesis_selector;
  OutpostPosteriorSanityConfig posterior_sanity;
  OutpostModeRoutingConfig mode_routing;
  OutpostPriorConfig prior;
  OutpostInitialPConfig initial_P;
  OutpostProcessNoiseConfig process_noise;
  OutpostObservationNoiseConfig observation_noise;
  OutpostWarmupConfig warmup;
  OutpostPhaseAuditConfig phase_audit;
};

enum class OutpostV3Mode { AMBIGUOUS = 0, STRUCTURED = 1 };

}  // namespace fyt::auto_aim::outpost_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_TYPES_HPP_
