#pragma once

#include <string>

namespace armor_pnp_refiner {

struct PnpRefinerConfig {
  // Mode: "none" | "single_yaw" | "single_xyz_yaw" | "sliding_window"
  std::string mode{"none"};
  // Phase0 default: prefer legacy single-yaw behavior to avoid drift from
  // armor_detector::BaSolver semantics.
  bool prefer_legacy_single_yaw{true};

  // ── Window ──────────────────────────────────────────
  int window_size{5};
  int min_window_size{3};
  double max_time_span_ms{60.0};
  double max_time_gap_ms{35.0};

  // ── Solver ──────────────────────────────────────────
  double max_solver_time_ms{2.0};
  int max_iterations{5};
  bool use_robust_kernel{true};
  std::string robust_kernel{"huber"};

  // ── Pixel noise ─────────────────────────────────────
  double pixel_sigma{1.5};
  double pixel_sigma_min{0.8};
  double pixel_sigma_max{5.0};
  double huber_delta_px{3.0};

  // ── PnP prior ───────────────────────────────────────
  double prior_sigma_xy{0.08};
  double prior_sigma_z{0.15};
  double prior_sigma_yaw_rad{0.10};

  // ── Second-order smooth ─────────────────────────────
  double acc_sigma_xy{0.15};
  double acc_sigma_z{0.25};
  double acc_sigma_yaw_rad{0.12};

  // ── Third-order smooth (off by default) ─────────────
  bool enable_jerk_smooth{false};
  double jerk_sigma_xy{0.30};
  double jerk_sigma_z{0.45};
  double jerk_sigma_yaw_rad{0.25};

  // ── Short-term association ──────────────────────────
  bool use_external_track_id_if_available{true};
  bool enable_internal_association{true};
  double iou_match_threshold{0.30};
  double center_distance_threshold_px{80.0};
  bool require_same_armor_number{true};
  bool require_same_armor_type{true};
  int min_confirm_hits{2};
  int max_missed_frames{3};

  // ── Quality gate ────────────────────────────────────
  double max_reproj_error_px{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_rad{0.349065850398866};  // 20 deg in rad
  double max_chi2_per_dof{5.0};
  double max_condition_number{1e7};

  // ── Covariance floor ────────────────────────────────
  double min_var_x{0.01 * 0.01};
  double min_var_y{0.01 * 0.01};
  double min_var_z{0.02 * 0.02};
  double min_var_yaw{0.01 * 0.01};

  // ── Confidence thresholds ───────────────────────────
  double good_confidence{0.75};
  double reject_confidence{0.40};

  // ── Safety ──────────────────────────────────────────
  bool require_positive_depth{true};
  bool require_finite{true};

  // ── Fallback ────────────────────────────────────────
  bool use_raw_pnp_on_failure{true};
  double max_pose_delta_m_fallback{0.20};
  double max_yaw_delta_rad_fallback{0.50};

  // ── Diagnostics ─────────────────────────────────────
  bool publish_debug{false};
  bool log_statistics{true};

  // ── Structural prior (temporary hardcoded values) ──
  // Phase1/Phase2 temporary structural prior:
  // armor roll in odom is assumed 0 deg.
  // armor pitch in odom is assumed +15 deg for outpost and -15 deg for others.
  // TODO: move these values to armor_pnp_refiner config after the first version is validated.
  double roll_odom_deg{0.0};
  double pitch_odom_outpost_deg{15.0};
  double pitch_odom_default_deg{-15.0};
};

}  // namespace armor_pnp_refiner
