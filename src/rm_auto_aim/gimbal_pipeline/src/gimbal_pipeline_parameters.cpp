// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalPipelineNode — unified node merging MaxEntropyTracker +
// TargetSelector + GimbalController. Inter-node ROS2 topics are replaced
// by direct C++ function calls to eliminate serialization / scheduling
// latency.

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rm_utils/heartbeat.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"
#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/tracker/norm4_tracker_baseline.hpp"
#include "max_entropy_tracker/visualization.hpp"
#include "rm_utils/logger/log.hpp"

// Gimbal strategies
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

namespace fyt::auto_aim {

void GimbalPipelineNode::declareTrackerParameters() {
  // Basic
  declare_parameter("target_frame", "odom");
  declare_parameter("source_frame", "camera_optical_frame");
  declare_parameter("predict_rate", 100.0);
  declare_parameter("default_r1", 0.15);
  declare_parameter("default_r2", 0.20);
  declare_parameter("default_dza", 0.0);
  declare_parameter("tracker_timeout", 0.5);
  declare_parameter("debug_mode", false);
  declare_parameter("enable_oscillation_detection", false);
  declare_parameter("visualization_frame", "odom");
  declare_parameter("tracker.debug_2d_viz.enable", false);
  declare_parameter("tracker.debug_2d_viz.width", 960);
  declare_parameter("tracker.debug_2d_viz.height", 540);
  declare_parameter("tracker.debug_2d_viz.jpeg_quality", 70);
  declare_parameter("robot_description.strict_unknown_reject", true);
  declare_parameter("robot_description.default_projection_mode", std::string("yaw_plane"));
  declare_parameter("robot_description.full_se3_ids",
                    std::vector<std::string>{"big_buff", "small_buff"});
  declare_parameter("robot_description.full_se3_robot_types", std::vector<int64_t>{});
  declare_parameter("external_targets.enable", false);
  declare_parameter("external_targets.buff.enable", false);
  declare_parameter("external_targets.buff.topic", std::string("auto_buff/tracked_robot"));
  declare_parameter("external_targets.buff.timeout_s", 0.3);
  declare_parameter("external_targets.allowed_ids_by_mode.mode_0", std::vector<std::string>{});
  declare_parameter("external_targets.allowed_ids_by_mode.mode_1", std::vector<std::string>{});
  declare_parameter("external_targets.allowed_ids_by_mode.mode_2",
                    std::vector<std::string>{"small_buff"});
  declare_parameter("external_targets.allowed_ids_by_mode.mode_3",
                    std::vector<std::string>{"small_buff"});
  declare_parameter("external_targets.allowed_ids_by_mode.mode_4",
                    std::vector<std::string>{"big_buff"});
  declare_parameter("external_targets.allowed_ids_by_mode.mode_5",
                    std::vector<std::string>{"big_buff"});

  // UKF
  declare_parameter("ukf.alpha", 0.001);
  declare_parameter("ukf.beta", 2.0);
  declare_parameter("ukf.kappa", 0.0);
  declare_parameter("ukf.obs_noise_pos", 0.05);
  declare_parameter("ukf.obs_noise_yaw", 0.05);
  declare_parameter("ukf.enable_ypd_observation_noise", false);
  declare_parameter("ukf.ypd_sigma_azi", 0.01);
  declare_parameter("ukf.ypd_sigma_ele", 0.01);
  declare_parameter("ukf.ypd_sigma_dist_coeff", 0.08);
  declare_parameter("ukf.dual_obs_noise_pos", 0.01);
  declare_parameter("ukf.dual_obs_noise_yaw", 0.03);
  declare_parameter("ukf.dual_obs_geometry_noise_scale", 0.2);
  declare_parameter("ukf.single_obs_update_weight_pos", 0.05);
  declare_parameter("ukf.enable_innovation_gating", false);
  declare_parameter("ukf.innovation_gate_chi2_threshold", 9.49);

  // Motion
  declare_parameter("motion.translation_model", "CA");
  declare_parameter("motion.cv_process_noise_vel", 0.5);
  declare_parameter("motion.ca_process_noise_acc", 1.0);
  declare_parameter("motion.singer_alpha", 0.5);
  declare_parameter("motion.singer_sigma", 2.0);
  declare_parameter("motion.process_noise_r", 0.02);
  declare_parameter("motion.process_noise_dz", 0.005);

  // Spin
  declare_parameter("spin.spin_process_noise_yaw_rate", 0.3);
  declare_parameter("spin.spin_process_noise_yaw_acc", 1.0);
  declare_parameter("spin.spin_process_noise_delta_rate", 0.3);
  declare_parameter("spin.spin_process_noise_delta_acc", 3.0);

  // Entropy
  declare_parameter("entropy.temperature", 2.0);
  declare_parameter("entropy.use_adaptive", true);
  declare_parameter("entropy.k_prior_weight", 0.7);

  // Tracker
  declare_parameter("tracker.tracking_thres", 2);
  declare_parameter("tracker.lost_thres", 8);
  declare_parameter("tracker.temp_lost_thres", 3);
  declare_parameter("tracker.max_match_distance", 2.0);
  declare_parameter("tracker.max_match_yaw_diff", 1.0);
  declare_parameter("tracker.n_panels", 4);
  declare_parameter("tracker.panel_angle_step", M_PI / 2.0);
  declare_parameter("tracker.periodic_binding_enable", false);
  declare_parameter("tracker.periodic_binding_weight", 0.35);
  declare_parameter("tracker.periodic_binding_spin_rate_gate", 0.8);
  declare_parameter("tracker.jump_binding_enable", true);
  declare_parameter("tracker.jump_binding_confirm_frames", 3);
  declare_parameter("tracker.jump_binding_z_jump_min", 0.015);
  declare_parameter("tracker.jump_binding_dz_match_tolerance", 0.03);
  declare_parameter("tracker.jump_binding_dz_gate", 0.010);
  declare_parameter("tracker.jump_binding_yaw_err_gate", 0.35);
  declare_parameter("tracker.jump_binding_cost_margin_min", 0.08);
  declare_parameter("tracker.jump_binding_switch_cooldown", 2);
  declare_parameter("tracker.jump_binding_dz_ema_alpha", 0.20);
  declare_parameter("tracker.jump_binding_confidence_floor", 0.15);
  declare_parameter("tracker.degraded_single_obs_enable", true);
  declare_parameter("tracker.degraded_single_obs_streak", 8);
  declare_parameter("tracker.degraded_q_scale_r", 4.0);
  declare_parameter("tracker.degraded_q_scale_dza", 4.0);

  // Constraints
  declare_parameter("constraints.min_radius", 0.12);
  declare_parameter("constraints.max_radius", 0.5);
  declare_parameter("constraints.min_dz", -1.0);
  declare_parameter("constraints.max_dz", 1.0);

  // Outpost-specific (known 3-armor geometry + max-entropy mode switch)
  declare_parameter("outpost.translation_model", "CV");
  declare_parameter("outpost.rotation_model", "CV");
  declare_parameter("outpost.tracking_thres", 2);
  declare_parameter("outpost.lost_thres", 40);
  declare_parameter("outpost.temp_lost_thres", 30);
  declare_parameter("outpost.max_match_distance", 2.0);
  declare_parameter("outpost.max_match_yaw_diff", 1.0);
  declare_parameter("outpost.singer_alpha", 0.0);
  declare_parameter("outpost.singer_sigma", 0.0);
  declare_parameter("outpost.spin_process_noise_theta_rate", 0.0);
  declare_parameter("outpost.spin_process_noise_theta_acc", 0.0);
  declare_parameter("outpost.radius", 0.26);
  // Outpost semantic contract:
  //   id0=highest, id1=middle, id2=lowest.
  declare_parameter("outpost.z_offset_0", 0.06);
  declare_parameter("outpost.z_offset_1", 0.0);
  declare_parameter("outpost.z_offset_2", -0.06);
  declare_parameter("outpost.panel_angle_step", 2.0 * M_PI / 3.0);
  declare_parameter("outpost.softmax_temperature", 1.5);
  declare_parameter("outpost.weight_yaw", 1.0);
  declare_parameter("outpost.weight_z_state", 6.0);
  declare_parameter("outpost.weight_z_history", 2.0);
  declare_parameter("outpost.weight_xy_residual", 2.5);
  declare_parameter("outpost.weight_switch_penalty", 0.05);
  declare_parameter("outpost.entropy_enter", 0.75);
  declare_parameter("outpost.entropy_exit", 0.55);
  declare_parameter("outpost.max_prob_enter", 0.60);
  declare_parameter("outpost.max_prob_exit", 0.75);
  declare_parameter("outpost.stable_frames", 4);
  declare_parameter("outpost.z_history_window", 15);
  declare_parameter("outpost.single_mode_confidence_scale", 0.70);
  declare_parameter("outpost.binding_transition_confirm_frames", 3);
  declare_parameter("outpost.binding_same_panel_yaw_gate", 0.35);
  declare_parameter("outpost.binding_same_panel_z_gate", 0.08);
  declare_parameter("outpost.binding_same_panel_xy_gate", 0.18);
  declare_parameter("outpost.binding_min_candidate_prob", 0.40);
  declare_parameter("outpost.binding_min_candidate_margin", 0.12);
  declare_parameter("outpost.binding_switch_strong_score", 0.60);
  declare_parameter("outpost.binding_period_window", 12);
  declare_parameter("outpost.binding_period_weight", 0.60);
  declare_parameter("outpost.binding_topology_prior_weight", 4.0);
  declare_parameter("outpost.binding_period_min_spin_rate", 0.8);
  declare_parameter("outpost.spin_direction_confirm_frames", 3);
  declare_parameter("outpost.binding_period_update_min_confidence", 0.55);
  declare_parameter("outpost.binding_period_update_min_jump", 0.015);
  declare_parameter("outpost.binding_dz_ema_alpha", 0.20);
  declare_parameter("outpost.binding_confidence_floor", 0.15);
  declare_parameter("outpost.z_audit_rebind_enable", true);
  declare_parameter("outpost.z_audit_rebind_confirm_frames", 3);
  declare_parameter("outpost.z_audit_rebind_min_confidence", 0.60);
  declare_parameter("outpost.z_audit_rebind_min_jump", 0.015);
  declare_parameter("outpost.binding_conflict_position_scale", 0.10);
  declare_parameter("outpost.alpha_pos", 0.65);
  declare_parameter("outpost.beta_vel", 0.30);
  declare_parameter("outpost.alpha_yaw", 0.60);
  declare_parameter("outpost.beta_yaw_rate", 0.25);
  declare_parameter("outpost.assume_static_center", true);
  declare_parameter("outpost.linear_velocity_damping", 0.90);
  declare_parameter("outpost.yaw_rate_damping", 0.98);
  declare_parameter("outpost.max_center_speed", 1.00);
  declare_parameter("outpost.max_yaw_rate", 12.0);
  declare_parameter("outpost.max_yaw_rate_step", 3.0);
  declare_parameter("outpost.mode_enter_confirm_frames", 3);
  declare_parameter("outpost.mode_exit_confirm_frames", 4);
  declare_parameter("outpost.mode_min_dwell_frames", 6);
  declare_parameter("outpost.mode_enter_threshold", 0.72);
  declare_parameter("outpost.mode_exit_threshold", 0.45);
  declare_parameter("outpost.mode_weight_jump", 0.30);
  declare_parameter("outpost.mode_weight_dual", 0.20);
  declare_parameter("outpost.mode_weight_margin", 0.20);
  declare_parameter("outpost.mode_weight_health", 0.20);
  declare_parameter("outpost.mode_weight_entropy", 0.10);
  declare_parameter("outpost.ambiguous_publish_single_armor_semantics", true);
  declare_parameter("outpost.ambiguous_single_armor_zero_offset", true);
  declare_parameter("outpost.ambiguous_backend_use_imm_adapter", false);
  declare_parameter("outpost.baseline_warmup_enable", true);
  declare_parameter("outpost.baseline_warmup_min_groups", 3);
  declare_parameter("outpost.baseline_warmup_min_samples_per_group", 2);
  declare_parameter("outpost.baseline_warmup_max_frames", 60);
  declare_parameter("outpost.baseline_warmup_z_jump_gate", 0.025);
  declare_parameter("outpost.baseline_warmup_yaw_jump_gate", 0.75);
  declare_parameter("outpost.baseline_warmup_xyz_jump_gate", 0.18);
  declare_parameter("outpost.baseline_warmup_ratio_min", 1.55);
  declare_parameter("outpost.baseline_warmup_ratio_max", 2.45);
  declare_parameter("outpost.baseline_warmup_min_large_diff", 0.06);
  // Maneuver detection
  declare_parameter("maneuver.enable", true);
  declare_parameter("maneuver.nis_threshold_single", 238.807);
  declare_parameter("maneuver.nis_threshold_dual", 4132.110);
  declare_parameter("maneuver.innov_norm_threshold_single", 0.1279);
  declare_parameter("maneuver.innov_norm_threshold_dual", 0.0613);
  declare_parameter("maneuver.mad_filter_enable", false);
  declare_parameter("maneuver.mad_window", 10);
  declare_parameter("maneuver.mad_k", 3.0);


  // Norm4 baseline tracker
  declare_parameter("norm4_baseline.enable_common_pipeline", false);
  declare_parameter("norm4_baseline.enable_phase_memory", true);
  declare_parameter("norm4_baseline.enable_kinematic_anti_pingpong", true);
  declare_parameter("norm4_baseline.enable_2d_tracker", false);
  declare_parameter("norm4_baseline.enable_proxy_manager", false);
  declare_parameter("norm4_baseline.phase_memory.enable_phase_memory", true);
  declare_parameter("norm4_baseline.phase_memory.enable_kinematic_anti_pingpong", true);
  declare_parameter("norm4_baseline.phase_memory.sequence_window_size", 10);
  declare_parameter("norm4_baseline.phase_memory.ping_pong_pattern_threshold", 0.7);
  declare_parameter("norm4_baseline.phase_memory.enable_opposite_jump_detect", true);
  declare_parameter("norm4_baseline.phase_memory.anti_pingpong.min_consistent_frames_to_commit", 3);
  declare_parameter("norm4_baseline.phase_memory.anti_pingpong.jerk_gate", 1.5);
  declare_parameter("norm4_baseline.phase_memory.anti_pingpong.yaw_rate_jump_gate", 2.0);
  declare_parameter("norm4_baseline.phase_memory.anti_pingpong.velocity_dir_cos_min", 0.2);
  declare_parameter("norm4_baseline.phase_memory.anti_pingpong.pending_timeout_frames", 12);

  declare_parameter("norm4_baseline.ukf_baseline.enabled", true);
  declare_parameter("norm4_baseline.ukf_baseline.force_rotation_ca", false);
  declare_parameter("norm4_baseline.ukf_baseline.dual_raw_batch", true);
  declare_parameter("norm4_baseline.ukf_baseline.sigma_pos_xy", 0.06);
  declare_parameter("norm4_baseline.ukf_baseline.sigma_pos_z", 0.08);
  declare_parameter("norm4_baseline.ukf_baseline.sigma_yaw", 0.12);
  declare_parameter("norm4_baseline.ukf_baseline.dual_raw_R_scale", 1.5);
  declare_parameter("norm4_baseline.ukf_baseline.gate.single_total_nis", 25.0);
  declare_parameter("norm4_baseline.ukf_baseline.gate.single_pos_chi2", 16.0);
  declare_parameter("norm4_baseline.ukf_baseline.gate.single_yaw_chi2", 9.0);
  declare_parameter("norm4_baseline.ukf_baseline.gate.dual_total_nis", 45.0);
  declare_parameter("norm4_baseline.ukf_baseline.gate.dual_each_pos_chi2", 16.0);
  declare_parameter("norm4_baseline.ukf_baseline.gate.dual_each_yaw_chi2", 9.0);
  declare_parameter("norm4_baseline.ukf_baseline.single_update.structural_gain_r", 0.0);
  declare_parameter("norm4_baseline.ukf_baseline.single_update.structural_gain_dza", 0.0);
  declare_parameter("norm4_baseline.ukf_baseline.dual_update.structural_gain_r", 0.05);
  declare_parameter("norm4_baseline.ukf_baseline.dual_update.structural_gain_dza", 0.02);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_center_jump", 0.25);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_yaw_jump", 0.80);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.min_r", 0.05);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_r", 0.50);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_r_jump", 0.05);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.min_dza", 0.0);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_dza", 0.15);
  declare_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_dza_jump", 0.03);

  declare_parameter("norm4_baseline.inekf.enabled", true);
  declare_parameter("norm4_baseline.inekf.force_rotation_ca", false);
  declare_parameter("norm4_baseline.inekf.dual_raw_batch", true);
  declare_parameter("norm4_baseline.inekf.sigma_pos_xy", 0.06);
  declare_parameter("norm4_baseline.inekf.sigma_pos_z", 0.08);
  declare_parameter("norm4_baseline.inekf.sigma_yaw", 0.12);
  declare_parameter("norm4_baseline.inekf.dual_raw_R_scale", 1.5);
  declare_parameter("norm4_baseline.inekf.gate.single_total_nis", 25.0);
  declare_parameter("norm4_baseline.inekf.gate.single_pos_chi2", 16.0);
  declare_parameter("norm4_baseline.inekf.gate.single_yaw_chi2", 9.0);
  declare_parameter("norm4_baseline.inekf.gate.dual_total_nis", 45.0);
  declare_parameter("norm4_baseline.inekf.gate.dual_each_pos_chi2", 16.0);
  declare_parameter("norm4_baseline.inekf.gate.dual_each_yaw_chi2", 9.0);
  declare_parameter("norm4_baseline.inekf.single_update.structural_gain_r", 0.0);
  declare_parameter("norm4_baseline.inekf.single_update.structural_gain_dza", 0.0);
  declare_parameter("norm4_baseline.inekf.dual_update.structural_gain_r", 0.05);
  declare_parameter("norm4_baseline.inekf.dual_update.structural_gain_dza", 0.02);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_center_jump", 0.25);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_yaw_jump", 0.80);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.min_r", 0.05);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_r", 0.50);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_r_jump", 0.05);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.min_dza", 0.0);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_dza", 0.15);
  declare_parameter("norm4_baseline.inekf.posterior_sanity.max_dza_jump", 0.03);

  declare_parameter("norm4_baseline.slow_structure.enable", true);
  declare_parameter("norm4_baseline.slow_structure.q_theta_r1", 1.0e-6);
  declare_parameter("norm4_baseline.slow_structure.q_theta_r2", 1.0e-6);
  declare_parameter("norm4_baseline.slow_structure.q_theta_dza", 5.0e-7);
  declare_parameter("norm4_baseline.slow_structure.prior_r1", 0.15);
  declare_parameter("norm4_baseline.slow_structure.prior_r2", 0.20);
  declare_parameter("norm4_baseline.slow_structure.prior_dza", 0.0);
  declare_parameter("norm4_baseline.slow_structure.prior_sigma_r", 0.06);
  declare_parameter("norm4_baseline.slow_structure.prior_sigma_dza", 0.06);
  declare_parameter("norm4_baseline.slow_structure.alpha_r1_single", 0.0);
  declare_parameter("norm4_baseline.slow_structure.alpha_r2_single", 0.0);
  declare_parameter("norm4_baseline.slow_structure.alpha_dza_single", 0.0);
  declare_parameter("norm4_baseline.slow_structure.alpha_r1_dual", 0.05);
  declare_parameter("norm4_baseline.slow_structure.alpha_r2_dual", 0.05);
  declare_parameter("norm4_baseline.slow_structure.alpha_dza_dual", 0.02);
  declare_parameter("norm4_baseline.slow_structure.prior_pull_gain", 0.002);
  declare_parameter("norm4_baseline.slow_structure.min_r", 0.05);
  declare_parameter("norm4_baseline.slow_structure.max_r", 0.50);
  declare_parameter("norm4_baseline.slow_structure.min_dza", 0.0);
  declare_parameter("norm4_baseline.slow_structure.max_dza", 0.12);

  declare_parameter("norm4_baseline.backend_config.backend_type", "ukf_baseline");
  declare_parameter("norm4_baseline.backend_config.motion_profile", "default");
  declare_parameter("norm4_baseline.backend_config.noise_profile", "default");
  declare_parameter("norm4_baseline.backend_config.structure_profile", "slow");
  declare_parameter("norm4_baseline.inekf_runtime.motion_profile", "default");
  declare_parameter("norm4_baseline.inekf_runtime.noise_profile", "default");
  declare_parameter("norm4_baseline.inekf_runtime.structure_profile", "slow");
  declare_parameter("norm4_baseline.inekf_runtime.translation_model", "");
  declare_parameter("norm4_baseline.inekf_runtime.cv_process_noise_vel", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.ca_process_noise_acc", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.singer_alpha", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.singer_sigma", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.process_noise_r", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.process_noise_dz", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.spin_process_noise_delta_rate", -1.0);
  declare_parameter("norm4_baseline.inekf_runtime.spin_process_noise_delta_acc", -1.0);

  declare_parameter("norm4_baseline.hypothesis_selector.topk", 4);
  declare_parameter("norm4_baseline.hypothesis_selector.commit_top1_only", true);
  declare_parameter("norm4_baseline.hypothesis_selector.min_top1_confidence", 0.55);
  declare_parameter("norm4_baseline.hypothesis_selector.min_top1_top2_margin", 0.0);
  declare_parameter("norm4_baseline.hypothesis_selector.ambiguous_margin", 1.0);
  declare_parameter("norm4_baseline.hypothesis_selector.include_rejected_in_debug", true);
  declare_parameter("norm4_baseline.hypothesis_selector.evidence_prior_enable", false);
  declare_parameter("norm4_baseline.hypothesis_selector.max_reconstruction_pos_error", 0.30);

  declare_parameter("norm4_baseline.warmup.enable_dual_seed_01", true);
  declare_parameter("norm4_baseline.warmup.warmup_frames", 8);
  declare_parameter("norm4_baseline.warmup.min_settle_frames", 3);
  declare_parameter("norm4_baseline.warmup.min_margin_to_commit", 1.5);
  declare_parameter("norm4_baseline.warmup.min_confidence_to_commit", 0.70);

  declare_parameter("norm4_baseline.mode_routing.ambiguous_output", "single_plate_3d");
  declare_parameter("norm4_baseline.mode_routing.structured_output", "structured_ukf");
  declare_parameter("norm4_baseline.mode_routing.ambiguous_structured_backend_mode",
                    "shallow_or_predict");
  declare_parameter("norm4_baseline.mode_routing.structured_single_plate_mode", "shallow");

  declare_parameter("norm4_baseline.single_plate_bridge.enable", false);
  declare_parameter("norm4_baseline.single_plate_bridge.source_semantic", "track2d_id");
  declare_parameter("norm4_baseline.single_plate_bridge.backend_type", "norm4_ambiguous_backend");
  declare_parameter("norm4_baseline.single_plate_bridge.require_semantic_stable_frames", 2);

  declare_parameter("norm4_baseline.fallback.predict_only_on_reject", true);
  declare_parameter("norm4_baseline.fallback.enable_ambiguous_single_fallback", true);
  declare_parameter("norm4_baseline.debug_log.enable", false);
  declare_parameter("norm4_baseline.debug_log.throttle_ms", 500);
  declare_parameter("norm4_baseline.debug_log.verbose", false);

  // Panel mismatch detection
  declare_parameter("panel_mismatch.enable", true);
  declare_parameter("panel_mismatch.window_size", 8);
  declare_parameter("panel_mismatch.threshold_t1", 0.0009);
  declare_parameter("panel_mismatch.confirm_count", 3);
  declare_parameter("panel_mismatch.reinit_count", 5);
  declare_parameter("panel_mismatch.apply_correction", false);

  // Output smoother
  declare_parameter("smoother.enable", true);
  declare_parameter("smoother.enable_position_smooth", true);
  declare_parameter("smoother.enable_yaw_smooth", true);
  declare_parameter("smoother.enable_velocity_smooth", true);
  declare_parameter("smoother.enable_structural_convergence", true);
  declare_parameter("smoother.pos_min_cutoff", 1.5);
  declare_parameter("smoother.pos_beta", 0.01);
  declare_parameter("smoother.pos_d_cutoff", 1.0);
  declare_parameter("smoother.yaw_min_cutoff", 1.0);
  declare_parameter("smoother.yaw_beta", 0.005);
  declare_parameter("smoother.yaw_d_cutoff", 1.0);
  declare_parameter("smoother.vel_min_cutoff", 2.0);
  declare_parameter("smoother.vel_beta", 0.01);
  declare_parameter("smoother.vel_d_cutoff", 1.0);
  declare_parameter("smoother.rm_initial_step", 0.5);
  declare_parameter("smoother.rm_gamma", 0.75);
  declare_parameter("smoother.rm_n0", 5);
  declare_parameter("smoother.rm_dual_obs_boost", 3.0);
  declare_parameter("smoother.rm_min_radius", 0.12);
  declare_parameter("smoother.rm_max_radius", 0.5);
  declare_parameter("smoother.rm_min_dz", -1.0);
  declare_parameter("smoother.rm_max_dz", 1.0);
  declare_parameter("smoother.rm_convergence_eps", 1e-4);
  declare_parameter("smoother.default_freq", 30.0);

  // Outlier filter (independent of smoother.enable)
  declare_parameter("smoother.enable_outlier_filter", false);
  declare_parameter("smoother.outlier_method", std::string("mad"));
  declare_parameter("smoother.outlier_window_size", 10);
  declare_parameter("smoother.outlier_min_samples", 5);
  declare_parameter("smoother.outlier_mad_k", 3.5);
  declare_parameter("smoother.outlier_iqr_k", 1.5);
  declare_parameter("smoother.outlier_mahal_threshold", 9.21);
}

void GimbalPipelineNode::declareTargetSelectorParameters() {
  declare_parameter("selector.reference_yaw", 0.0);
  declare_parameter("selector.max_yaw_deviation", M_PI);
  declare_parameter("selector.max_distance", 10.0);
  declare_parameter("selector.min_confidence", 0.3);
  declare_parameter("selector.priority_robot_ids", std::vector<std::string>{});
}

void GimbalPipelineNode::declareGimbalControllerParameters() {
  declare_parameter("controller.bullet_speed", 20.0);
  declare_parameter("controller.control_rate", 250.0);

  // Solver
  declare_parameter("controller.solver.shooting_range_width", 0.135);
  declare_parameter("controller.solver.shooting_range_height", 0.135);
  declare_parameter("controller.solver.side_angle", 15.0);
  declare_parameter("controller.solver.min_switching_v_yaw", 1.0);
  declare_parameter("controller.solver.gravity", 9.8);
  declare_parameter("controller.solver.resistance", 0.001);
  declare_parameter("controller.solver.iteration_times", 20);
  declare_parameter("controller.solver.facing_enter_angle", 40.0);
  declare_parameter("controller.solver.facing_exit_angle", 55.0);
  declare_parameter("controller.solver.radial_dynamic.enable", false);
  declare_parameter("controller.solver.radial_dynamic.v_yaw_ref", 8.0);
  declare_parameter("controller.solver.radial_dynamic.shrink_ratio", 0.6);
  declare_parameter("controller.solver.radial_dynamic.min_angle_deg", 5.0);
  declare_parameter("controller.solver.radial_dynamic.bias_gain_deg", 0.0);
  declare_parameter("controller.solver.radial_dynamic.max_bias_deg", 0.0);
  declare_parameter("controller.solver.virtual_pose.auto_switch.enable", false);
  declare_parameter("controller.solver.virtual_pose.auto_switch.enter_vyaw", 8.0);
  declare_parameter("controller.solver.virtual_pose.auto_switch.exit_vyaw", 6.0);
  declare_parameter("controller.solver.virtual_pose.auto_switch.selection_method",
                    std::string("virtual_pose"));
  declare_parameter("controller.solver.virtual_pose.auto_switch.fixed_id", 0);
  declare_parameter("controller.solver.virtual_pose.fixed_id", 0);
  declare_parameter("controller.solver.selection_method", std::string("min_movement_with_facing"));

  // Unified delay parameters (preferred)
  declare_parameter("controller.delay.prediction_extra_s", 0.0);
  declare_parameter("controller.delay.control_latency_s", 0.0);
  declare_parameter("controller.delay.trigger_to_muzzle_s", 0.0);
  declare_parameter("controller.delay.max_processing_delay_s", 0.5);
  declare_parameter("controller.delay.flight_time_iters", 2);

  declare_parameter("controller.fire.decision_policy", std::string("axis_threshold"));
  declare_parameter("controller.fire.facing_filter_opening_angle_deg", 180.0);
  declare_parameter("controller.fire.use_gimbal_kinematics", false);
  declare_parameter("controller.fire.target_visibility_policy", std::string("facing_only"));
  declare_parameter("controller.fire.velocity_low_pass.enable", true);
  declare_parameter("controller.fire.velocity_low_pass.alpha", 0.35);
  declare_parameter("controller.fire.velocity_low_pass.reset_timeout_s", 0.25);
  declare_parameter("controller.fire.probability.enable", false);
  declare_parameter("controller.fire.probability.future_window_ms", 50.0);
  declare_parameter("controller.fire.probability.future_step_ms", 10.0);
  declare_parameter("controller.fire.probability.window_fusion", std::string("max"));
  declare_parameter("controller.fire.probability.softmax_beta", 20.0);
  declare_parameter("controller.fire.probability.use_tracker_covariance", true);
  declare_parameter("controller.fire.probability.strict_covariance", false);
  declare_parameter("controller.fire.probability.fallback_sigma_x", 0.02);
  declare_parameter("controller.fire.probability.fallback_sigma_y", 0.02);
  declare_parameter("controller.fire.probability.fallback_sigma_z", 0.03);
  declare_parameter("controller.fire.probability.ballistic_sigma_x0", 0.010);
  declare_parameter("controller.fire.probability.ballistic_sigma_y0", 0.015);
  declare_parameter("controller.fire.probability.ballistic_sigma_z0", 0.015);
  declare_parameter("controller.fire.probability.ballistic_growth_x", 0.03);
  declare_parameter("controller.fire.probability.ballistic_growth_y", 0.06);
  declare_parameter("controller.fire.probability.ballistic_growth_z", 0.08);
  declare_parameter("controller.fire.probability.normal_velocity_weight.enable", false);
  declare_parameter("controller.fire.probability.normal_velocity_weight.v_ref", 28.0);
  declare_parameter("controller.fire.probability.normal_velocity_weight.w_min", 0.5);
  declare_parameter("controller.fire.probability.normal_velocity_gate.enable", true);
  declare_parameter("controller.fire.probability.normal_velocity_gate.require_front_face", true);
  declare_parameter("controller.fire.probability.normal_velocity_gate.v_activate_min", 8.0);
  declare_parameter("controller.fire.probability.normal_velocity_gate.front_epsilon", 1e-4);
  declare_parameter("controller.fire.probability.normal_velocity_gate.max_complement_angle_deg",
                    90.0);
  declare_parameter("controller.fire.probability.sigma_point.enable", true);
  declare_parameter("controller.fire.probability.sigma_point.method", std::string("unscented"));
  declare_parameter("controller.fire.probability.sigma_point.sigma_v0", 0.3);
  declare_parameter("controller.fire.probability.sigma_point.sigma_delay", 0.005);
  declare_parameter("controller.fire.probability.sigma_point.rho", 0.0);
  declare_parameter("controller.fire.probability.sigma_point.alpha", 0.7);
  declare_parameter("controller.fire.probability.sigma_point.beta", 2.0);
  declare_parameter("controller.fire.probability.sigma_point.kappa", 0.0);
  declare_parameter("controller.fire.probability.gate.strategy", std::string("legacy"));
  declare_parameter("controller.fire.probability.gate.mode", std::string("lowpass"));
  declare_parameter("controller.fire.probability.gate.alpha", 0.85);
  declare_parameter("controller.fire.probability.gate.fire_on_th", 0.65);
  declare_parameter("controller.fire.probability.gate.fire_off_th", 0.35);
  declare_parameter("controller.fire.probability.gate.integrator_base_probability", 0.45);
  declare_parameter("controller.fire.probability.gate.integrator_rise", 8.0);
  declare_parameter("controller.fire.probability.gate.integrator_fall", 6.0);
  declare_parameter("controller.fire.probability.burst.burst_bullet_count", 5);
  declare_parameter("controller.fire.probability.burst.min_hit_count", 1);
  declare_parameter("controller.fire.probability.evidence.reference_probability_p0", 0.60);
  declare_parameter("controller.fire.probability.evidence.window_ms", 50.0);
  declare_parameter("controller.fire.probability.evidence.log_clip", 2.0);
  declare_parameter("controller.fire.probability.evidence.epsilon", 1e-3);
  declare_parameter("controller.fire.probability.evidence.neutralize_unshootable_samples", true);
  declare_parameter("controller.fire.probability.evidence.negative_evidence_scale", 0.35);
  declare_parameter("controller.fire.probability.evidence.negative_clip_scale", 0.35);
  declare_parameter("controller.fire.probability.evidence.deadband", 0.10);
  declare_parameter("controller.fire.probability.temperature.value", 0.5);
  declare_parameter("controller.fire.probability.temperature.theta_on_cold", 0.90);
  declare_parameter("controller.fire.probability.temperature.theta_on_hot", 0.75);
  declare_parameter("controller.fire.probability.temperature.theta_hold_cold", 0.70);
  declare_parameter("controller.fire.probability.temperature.theta_hold_hot", 0.55);
  declare_parameter("controller.fire.probability.temperature.theta_reset_cold", 0.45);
  declare_parameter("controller.fire.probability.temperature.theta_reset_hot", 0.35);
  declare_parameter("controller.fire.probability.commit.min_fire_ms", 20.0);
  declare_parameter("controller.fire.probability.commit.cooldown_ms", 80.0);
  declare_parameter("controller.fire.visualization.enable", true);
  declare_parameter("controller.fire.visualization.ellipse_samples", 64);
  declare_parameter("controller.fire.visualization.max_impact_points", 120);
  declare_parameter("controller.fire.visualization.image_debug.enable", false);
  declare_parameter("controller.fire.visualization.image_debug.publish_rate_hz", 10.0);
  declare_parameter("controller.fire.visualization.image_debug.width", 960);
  declare_parameter("controller.fire.visualization.image_debug.height", 540);
  declare_parameter("controller.fire.visualization.image_debug.show_text", true);
  declare_parameter("controller.fire.visualization.image_debug.show_sigma_ellipse", true);
  declare_parameter("controller.fire.visualization.image_debug.show_velocity_fan", true);

  // MPC strategy
  declare_parameter("controller.mpc.N", 20);
  declare_parameter("controller.mpc.dt", 0.01);
  declare_parameter("controller.mpc.max_accel", 30.0);
  declare_parameter("controller.mpc.q_yaw", 100.0);
  declare_parameter("controller.mpc.q_pitch", 100.0);
  declare_parameter("controller.mpc.q_yaw_vel", 10.0);
  declare_parameter("controller.mpc.q_pitch_vel", 10.0);
  declare_parameter("controller.mpc.r_yaw", 0.01);
  declare_parameter("controller.mpc.r_pitch", 0.01);
  declare_parameter("controller.mpc.s_yaw", 5.0);
  declare_parameter("controller.mpc.s_pitch", 5.0);

  // MPC delay compensation
  declare_parameter("controller.mpc.enable_delay_compensation", true);
  declare_parameter("controller.mpc.allow_muzzle_compensation", true);
  declare_parameter("controller.mpc.yaw_feedforward_k_s", 0.0);

  // MPC 机动自适应权重衰减
  declare_parameter("controller.mpc.maneuver_adapt.enable", false);
  declare_parameter("controller.mpc.maneuver_adapt.a_max", 3.0);
  declare_parameter("controller.mpc.maneuver_adapt.eta", 0.2);
  declare_parameter("controller.mpc.maneuver_adapt.tau", 10.0);
  declare_parameter("controller.mpc.maneuver_adapt.r_scale", 10.0);

  // MPC 命中概率权重加权 (Q 权重缩放)
  declare_parameter("controller.mpc.weighting.enable", false);
  declare_parameter("controller.mpc.weighting.alpha", 3.0);
  declare_parameter("controller.mpc.weighting.k_omega", 0.5);
  declare_parameter("controller.mpc.weighting.sigma_min", 0.05);
  declare_parameter("controller.mpc.weighting.sigma_max", 0.5);
  declare_parameter("controller.mpc.weighting.sigma_sys", 0.02);
  declare_parameter("controller.mpc.weighting.target_size", 0.135);
  declare_parameter("controller.mpc.weighting.delay_s", 0.0);
  declare_parameter("controller.mpc.weighting.max_w", 6.0);
  declare_parameter("controller.mpc.weighting.smooth_alpha", 0.7);
  declare_parameter("controller.mpc.weighting.min_distance", 0.1);
  declare_parameter("controller.mpc.weighting.sigma_beta", 0.3);
  declare_parameter("controller.mpc.weighting.gamma", 0.5);

  // MPC 轨迹生成前速度 clamp
  declare_parameter("controller.mpc.vel_clamp.enable", false);
  declare_parameter("controller.mpc.vel_clamp.max_linear_speed", 5.0);
  declare_parameter("controller.mpc.vel_clamp.max_v_yaw", 10.0);

  // MPC FOV 软约束
  declare_parameter("controller.mpc.fov_constraint.enable", false);
  declare_parameter("controller.mpc.fov_constraint.margin", 0.05);
  declare_parameter("controller.mpc.fov_constraint.slack_weight", 1000.0);
  declare_parameter("controller.mpc.fov_constraint.constraint_steps", 0);
  declare_parameter("controller.mpc.fov_constraint.dynamic_margin.enable", false);
  declare_parameter("controller.mpc.fov_constraint.dynamic_margin.vel_scale", 0.01);
  declare_parameter("controller.mpc.fov_constraint.fallback_fov_yaw", 0.35);
  declare_parameter("controller.mpc.fov_constraint.fallback_fov_pitch", 0.26);

  // MPC 数值稳健性: 在线 RMS 归一化
  declare_parameter("controller.mpc.normalization.enable", false);
  declare_parameter("controller.mpc.normalization.mode", std::string("rms"));
  declare_parameter("controller.mpc.normalization.window_size", 80);
  declare_parameter("controller.mpc.normalization.min_samples", 10);
  declare_parameter("controller.mpc.normalization.rms_epsilon", 1e-6);
  declare_parameter("controller.mpc.normalization.typical_state.yaw", 1.0);
  declare_parameter("controller.mpc.normalization.typical_state.pitch", 1.0);
  declare_parameter("controller.mpc.normalization.typical_state.yaw_vel", 1.0);
  declare_parameter("controller.mpc.normalization.typical_state.pitch_vel", 1.0);
  declare_parameter("controller.mpc.normalization.typical_control.yaw_acc", 1.0);
  declare_parameter("controller.mpc.normalization.typical_control.pitch_acc", 1.0);
  declare_parameter("controller.mpc.normalization.typical_delta_control.yaw_acc", 1.0);
  declare_parameter("controller.mpc.normalization.typical_delta_control.pitch_acc", 1.0);

  // MPC 数值稳健性: Hessian 自适应对角正则
  declare_parameter("controller.mpc.regularization.enable", false);
  declare_parameter("controller.mpc.regularization.epsilon_abs", 1e-8);
  declare_parameter("controller.mpc.regularization.epsilon_rel", 1e-6);
  declare_parameter("controller.mpc.regularization.epsilon_max", 1e-2);
  declare_parameter("controller.mpc.regularization.retry_on_fail", true);
  declare_parameter("controller.mpc.regularization.retry_scale", 10.0);

  // MPC 数值诊断: 低成本常开 + 高成本抽样
  declare_parameter("controller.mpc.diagnostics.enable", false);
  declare_parameter("controller.mpc.diagnostics.low_cost_always", true);
  declare_parameter("controller.mpc.diagnostics.high_cost_enable", false);
  declare_parameter("controller.mpc.diagnostics.high_cost_sample_every", 20);
  declare_parameter("controller.mpc.diagnostics.log_every", 50);
  declare_parameter("controller.mpc.diagnostics.log_on_failure", true);
  declare_parameter("controller.mpc.diagnostics.active_tol", 1e-4);
  declare_parameter("controller.mpc.diagnostics.rank_tol_rel", 1e-9);

  // ─── GimbalCmd 输出端保护滤波器 ──────────────────────────────
  // 0. Clamping — 绝对限幅
  declare_parameter("controller.output_filter.enable_clamping", true);
  declare_parameter("controller.output_filter.max_yaw_diff", 15.0);
  declare_parameter("controller.output_filter.max_pitch_diff", 10.0);
  // 1. 外点检测
  declare_parameter("controller.output_filter.enable_outlier_rejection", true);
  declare_parameter("controller.output_filter.outlier_threshold_yaw", 8.0);
  declare_parameter("controller.output_filter.outlier_threshold_pitch", 5.0);
  declare_parameter("controller.output_filter.max_outlier_count", 3);
  // 2. Rate Limiter
  declare_parameter("controller.output_filter.enable_rate_limiter", true);
  declare_parameter("controller.output_filter.max_yaw_rate", 5.0);
  declare_parameter("controller.output_filter.max_pitch_rate", 3.0);
  // 3. 滑动窗口均值
  declare_parameter("controller.output_filter.enable_moving_average", false);
  declare_parameter("controller.output_filter.moving_average_window_size", 3);
  // 4. EMA
  declare_parameter("controller.output_filter.enable_ema", false);
  declare_parameter("controller.output_filter.ema_alpha", 0.7);
  // 5. 1-Euro 自适应滤波
  declare_parameter("controller.output_filter.enable_one_euro", false);
  declare_parameter("controller.output_filter.one_euro_freq", 250.0);
  declare_parameter("controller.output_filter.one_euro_min_cutoff", 1.0);
  declare_parameter("controller.output_filter.one_euro_beta", 0.007);
  declare_parameter("controller.output_filter.one_euro_d_cutoff", 1.0);

  // ─── Prediction logger ────────────────────────────────────────
  declare_parameter("logging.enable", false);
  declare_parameter("logging.output_dir", std::string("/tmp/prediction_logs"));
  declare_parameter("logging.robot_id_filter", std::string(""));
  declare_parameter("logging.flush_every_n", 50);
}

/* ================================================================ */
/*  Apply tracker parameters to config                               */
/* ================================================================ */

void GimbalPipelineNode::applyTrackerParamsToConfig() {
  auto &c = tracker_config_;

  c.ukf.alpha = get_parameter("ukf.alpha").as_double();
  c.ukf.beta = get_parameter("ukf.beta").as_double();
  c.ukf.kappa = get_parameter("ukf.kappa").as_double();
  c.ukf.obs_noise_pos = get_parameter("ukf.obs_noise_pos").as_double();
  c.ukf.obs_noise_yaw = get_parameter("ukf.obs_noise_yaw").as_double();
  c.ukf.enable_ypd_observation_noise = get_parameter("ukf.enable_ypd_observation_noise").as_bool();
  c.ukf.ypd_sigma_azi = get_parameter("ukf.ypd_sigma_azi").as_double();
  c.ukf.ypd_sigma_ele = get_parameter("ukf.ypd_sigma_ele").as_double();
  c.ukf.ypd_sigma_dist_coeff = get_parameter("ukf.ypd_sigma_dist_coeff").as_double();
  c.ukf.dual_obs_noise_pos = get_parameter("ukf.dual_obs_noise_pos").as_double();
  c.ukf.dual_obs_noise_yaw = get_parameter("ukf.dual_obs_noise_yaw").as_double();
  c.ukf.dual_obs_geometry_noise_scale =
    get_parameter("ukf.dual_obs_geometry_noise_scale").as_double();
  c.ukf.single_obs_update_weight_pos =
    get_parameter("ukf.single_obs_update_weight_pos").as_double();
  c.ukf.enable_innovation_gating = get_parameter("ukf.enable_innovation_gating").as_bool();
  c.ukf.innovation_gate_chi2_threshold =
    get_parameter("ukf.innovation_gate_chi2_threshold").as_double();

  auto tm_str = get_parameter("motion.translation_model").as_string();
  c.motion.translation_model = translation_model_from_string(tm_str);
  c.motion.cv_process_noise_vel = get_parameter("motion.cv_process_noise_vel").as_double();
  c.motion.ca_process_noise_acc = get_parameter("motion.ca_process_noise_acc").as_double();
  c.motion.singer_alpha = get_parameter("motion.singer_alpha").as_double();
  c.motion.singer_sigma = get_parameter("motion.singer_sigma").as_double();
  c.motion.process_noise_r = get_parameter("motion.process_noise_r").as_double();
  c.motion.process_noise_dz = get_parameter("motion.process_noise_dz").as_double();

  c.spin.spin_process_noise_yaw_rate =
    get_parameter("spin.spin_process_noise_yaw_rate").as_double();
  c.spin.spin_process_noise_yaw_acc = get_parameter("spin.spin_process_noise_yaw_acc").as_double();
  c.spin.spin_process_noise_delta_rate =
    get_parameter("spin.spin_process_noise_delta_rate").as_double();
  c.spin.spin_process_noise_delta_acc =
    get_parameter("spin.spin_process_noise_delta_acc").as_double();

  c.entropy.temperature = get_parameter("entropy.temperature").as_double();
  c.entropy.use_adaptive = get_parameter("entropy.use_adaptive").as_bool();
  c.entropy.k_prior_weight = get_parameter("entropy.k_prior_weight").as_double();

  c.tracker.tracking_thres = get_parameter("tracker.tracking_thres").as_int();
  c.tracker.lost_thres = get_parameter("tracker.lost_thres").as_int();
  c.tracker.temp_lost_thres = get_parameter("tracker.temp_lost_thres").as_int();
  c.tracker.max_match_distance = get_parameter("tracker.max_match_distance").as_double();
  c.tracker.max_match_yaw_diff = get_parameter("tracker.max_match_yaw_diff").as_double();
  c.tracker.n_panels = get_parameter("tracker.n_panels").as_int();
  c.tracker.panel_angle_step = get_parameter("tracker.panel_angle_step").as_double();
  c.tracker.periodic_binding_enable = get_parameter("tracker.periodic_binding_enable").as_bool();
  c.tracker.periodic_binding_weight = get_parameter("tracker.periodic_binding_weight").as_double();
  c.tracker.periodic_binding_spin_rate_gate =
    get_parameter("tracker.periodic_binding_spin_rate_gate").as_double();
  c.tracker.jump_binding_enable = get_parameter("tracker.jump_binding_enable").as_bool();
  c.tracker.jump_binding_confirm_frames =
    get_parameter("tracker.jump_binding_confirm_frames").as_int();
  c.tracker.jump_binding_z_jump_min = get_parameter("tracker.jump_binding_z_jump_min").as_double();
  c.tracker.jump_binding_dz_match_tolerance =
    get_parameter("tracker.jump_binding_dz_match_tolerance").as_double();
  c.tracker.jump_binding_dz_gate = get_parameter("tracker.jump_binding_dz_gate").as_double();
  c.tracker.jump_binding_yaw_err_gate =
    get_parameter("tracker.jump_binding_yaw_err_gate").as_double();
  c.tracker.jump_binding_cost_margin_min =
    get_parameter("tracker.jump_binding_cost_margin_min").as_double();
  c.tracker.jump_binding_switch_cooldown =
    get_parameter("tracker.jump_binding_switch_cooldown").as_int();
  c.tracker.jump_binding_dz_ema_alpha =
    get_parameter("tracker.jump_binding_dz_ema_alpha").as_double();
  c.tracker.jump_binding_confidence_floor =
    get_parameter("tracker.jump_binding_confidence_floor").as_double();
  c.tracker.degraded_single_obs_enable =
    get_parameter("tracker.degraded_single_obs_enable").as_bool();
  c.tracker.degraded_single_obs_streak =
    get_parameter("tracker.degraded_single_obs_streak").as_int();
  c.tracker.degraded_q_scale_r = get_parameter("tracker.degraded_q_scale_r").as_double();
  c.tracker.degraded_q_scale_dza = get_parameter("tracker.degraded_q_scale_dza").as_double();

  c.tracker.periodic_binding_weight = std::max(0.0, c.tracker.periodic_binding_weight);
  c.tracker.periodic_binding_spin_rate_gate =
    std::max(0.0, c.tracker.periodic_binding_spin_rate_gate);
  c.tracker.jump_binding_confirm_frames = std::max(1, c.tracker.jump_binding_confirm_frames);
  c.tracker.jump_binding_z_jump_min = std::max(0.0, c.tracker.jump_binding_z_jump_min);
  c.tracker.jump_binding_dz_match_tolerance =
    std::max(0.0, c.tracker.jump_binding_dz_match_tolerance);
  c.tracker.jump_binding_dz_gate = std::max(0.0, c.tracker.jump_binding_dz_gate);
  c.tracker.jump_binding_yaw_err_gate = std::max(1e-3, c.tracker.jump_binding_yaw_err_gate);
  c.tracker.jump_binding_cost_margin_min = std::max(0.0, c.tracker.jump_binding_cost_margin_min);
  c.tracker.jump_binding_switch_cooldown = std::max(0, c.tracker.jump_binding_switch_cooldown);
  c.tracker.jump_binding_dz_ema_alpha = std::clamp(c.tracker.jump_binding_dz_ema_alpha, 0.01, 1.0);
  c.tracker.jump_binding_confidence_floor =
    std::clamp(c.tracker.jump_binding_confidence_floor, 0.0, 0.95);
  c.tracker.degraded_single_obs_streak = std::max(1, c.tracker.degraded_single_obs_streak);
  c.tracker.degraded_q_scale_r = std::max(0.1, c.tracker.degraded_q_scale_r);
  c.tracker.degraded_q_scale_dza = std::max(0.1, c.tracker.degraded_q_scale_dza);

  c.constraints.min_radius = get_parameter("constraints.min_radius").as_double();
  c.constraints.max_radius = get_parameter("constraints.max_radius").as_double();
  c.constraints.min_dz = get_parameter("constraints.min_dz").as_double();
  c.constraints.max_dz = get_parameter("constraints.max_dz").as_double();

  c.maneuver.enable = get_parameter("maneuver.enable").as_bool();
  c.maneuver.nis_threshold_single = get_parameter("maneuver.nis_threshold_single").as_double();
  c.maneuver.nis_threshold_dual = get_parameter("maneuver.nis_threshold_dual").as_double();
  c.maneuver.innov_norm_threshold_single =
    get_parameter("maneuver.innov_norm_threshold_single").as_double();
  c.maneuver.innov_norm_threshold_dual =
    get_parameter("maneuver.innov_norm_threshold_dual").as_double();
  c.maneuver.mad_filter_enable = get_parameter("maneuver.mad_filter_enable").as_bool();
  c.maneuver.mad_window = get_parameter("maneuver.mad_window").as_int();
  c.maneuver.mad_k = get_parameter("maneuver.mad_k").as_double();
  c.maneuver.mad_window = std::max(1, c.maneuver.mad_window);
  c.maneuver.mad_k = std::max(0.1, c.maneuver.mad_k);


  c.norm4_baseline.enable_common_pipeline = get_parameter("norm4_baseline.enable_common_pipeline").as_bool();
  c.norm4_baseline.enable_phase_memory = get_parameter("norm4_baseline.enable_phase_memory").as_bool();
  c.norm4_baseline.enable_kinematic_anti_pingpong =
    get_parameter("norm4_baseline.enable_kinematic_anti_pingpong").as_bool();
  c.norm4_baseline.enable_2d_tracker = get_parameter("norm4_baseline.enable_2d_tracker").as_bool();
  c.norm4_baseline.enable_proxy_manager = get_parameter("norm4_baseline.enable_proxy_manager").as_bool();
  c.norm4_baseline.phase_memory.enable_phase_memory =
    get_parameter("norm4_baseline.phase_memory.enable_phase_memory").as_bool();
  c.norm4_baseline.phase_memory.enable_kinematic_anti_pingpong =
    get_parameter("norm4_baseline.phase_memory.enable_kinematic_anti_pingpong").as_bool();
  c.norm4_baseline.phase_memory.sequence_window_size =
    get_parameter("norm4_baseline.phase_memory.sequence_window_size").as_int();
  c.norm4_baseline.phase_memory.ping_pong_pattern_threshold =
    get_parameter("norm4_baseline.phase_memory.ping_pong_pattern_threshold").as_double();
  c.norm4_baseline.phase_memory.enable_opposite_jump_detect =
    get_parameter("norm4_baseline.phase_memory.enable_opposite_jump_detect").as_bool();
  c.norm4_baseline.phase_memory.anti_pingpong.min_consistent_frames_to_commit =
    get_parameter("norm4_baseline.phase_memory.anti_pingpong.min_consistent_frames_to_commit").as_int();
  c.norm4_baseline.phase_memory.anti_pingpong.jerk_gate =
    get_parameter("norm4_baseline.phase_memory.anti_pingpong.jerk_gate").as_double();
  c.norm4_baseline.phase_memory.anti_pingpong.yaw_rate_jump_gate =
    get_parameter("norm4_baseline.phase_memory.anti_pingpong.yaw_rate_jump_gate").as_double();
  c.norm4_baseline.phase_memory.anti_pingpong.velocity_dir_cos_min =
    get_parameter("norm4_baseline.phase_memory.anti_pingpong.velocity_dir_cos_min").as_double();
  c.norm4_baseline.phase_memory.anti_pingpong.pending_timeout_frames =
    get_parameter("norm4_baseline.phase_memory.anti_pingpong.pending_timeout_frames").as_int();

  c.norm4_baseline.phase_memory.enable_phase_memory = c.norm4_baseline.enable_phase_memory;
  c.norm4_baseline.phase_memory.enable_kinematic_anti_pingpong =
    c.norm4_baseline.enable_kinematic_anti_pingpong;
  c.norm4_baseline.phase_memory.sequence_window_size =
    std::max(3, c.norm4_baseline.phase_memory.sequence_window_size);

  c.norm4_baseline.ukf_baseline.enabled = get_parameter("norm4_baseline.ukf_baseline.enabled").as_bool();
  c.norm4_baseline.ukf_baseline.force_rotation_ca =
    get_parameter("norm4_baseline.ukf_baseline.force_rotation_ca").as_bool();
  c.norm4_baseline.ukf_baseline.dual_raw_batch = get_parameter("norm4_baseline.ukf_baseline.dual_raw_batch").as_bool();
  c.norm4_baseline.ukf_baseline.sigma_pos_xy = get_parameter("norm4_baseline.ukf_baseline.sigma_pos_xy").as_double();
  c.norm4_baseline.ukf_baseline.sigma_pos_z = get_parameter("norm4_baseline.ukf_baseline.sigma_pos_z").as_double();
  c.norm4_baseline.ukf_baseline.sigma_yaw = get_parameter("norm4_baseline.ukf_baseline.sigma_yaw").as_double();
  c.norm4_baseline.ukf_baseline.dual_raw_R_scale =
    get_parameter("norm4_baseline.ukf_baseline.dual_raw_R_scale").as_double();
  c.norm4_baseline.ukf_baseline.gate.single_total_nis =
    get_parameter("norm4_baseline.ukf_baseline.gate.single_total_nis").as_double();
  c.norm4_baseline.ukf_baseline.gate.single_pos_chi2 =
    get_parameter("norm4_baseline.ukf_baseline.gate.single_pos_chi2").as_double();
  c.norm4_baseline.ukf_baseline.gate.single_yaw_chi2 =
    get_parameter("norm4_baseline.ukf_baseline.gate.single_yaw_chi2").as_double();
  c.norm4_baseline.ukf_baseline.gate.dual_total_nis =
    get_parameter("norm4_baseline.ukf_baseline.gate.dual_total_nis").as_double();
  c.norm4_baseline.ukf_baseline.gate.dual_each_pos_chi2 =
    get_parameter("norm4_baseline.ukf_baseline.gate.dual_each_pos_chi2").as_double();
  c.norm4_baseline.ukf_baseline.gate.dual_each_yaw_chi2 =
    get_parameter("norm4_baseline.ukf_baseline.gate.dual_each_yaw_chi2").as_double();
  c.norm4_baseline.ukf_baseline.single_update.structural_gain_r =
    get_parameter("norm4_baseline.ukf_baseline.single_update.structural_gain_r").as_double();
  c.norm4_baseline.ukf_baseline.single_update.structural_gain_dza =
    get_parameter("norm4_baseline.ukf_baseline.single_update.structural_gain_dza").as_double();
  c.norm4_baseline.ukf_baseline.dual_update.structural_gain_r =
    get_parameter("norm4_baseline.ukf_baseline.dual_update.structural_gain_r").as_double();
  c.norm4_baseline.ukf_baseline.dual_update.structural_gain_dza =
    get_parameter("norm4_baseline.ukf_baseline.dual_update.structural_gain_dza").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_center_jump =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_center_jump").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_yaw_jump =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_yaw_jump").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.min_r =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.min_r").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_r =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_r").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_r_jump =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_r_jump").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.min_dza =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.min_dza").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_dza =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_dza").as_double();
  c.norm4_baseline.ukf_baseline.posterior_sanity.max_dza_jump =
    get_parameter("norm4_baseline.ukf_baseline.posterior_sanity.max_dza_jump").as_double();

  auto load_norm4_baseline_ukf_cfg = [this](const std::string &prefix, Norm4BaselineUkfConfig *out) {
    out->enabled = get_parameter(prefix + ".enabled").as_bool();
    out->force_rotation_ca = get_parameter(prefix + ".force_rotation_ca").as_bool();
    out->dual_raw_batch = get_parameter(prefix + ".dual_raw_batch").as_bool();
    out->sigma_pos_xy = get_parameter(prefix + ".sigma_pos_xy").as_double();
    out->sigma_pos_z = get_parameter(prefix + ".sigma_pos_z").as_double();
    out->sigma_yaw = get_parameter(prefix + ".sigma_yaw").as_double();
    out->dual_raw_R_scale = get_parameter(prefix + ".dual_raw_R_scale").as_double();
    out->gate.single_total_nis = get_parameter(prefix + ".gate.single_total_nis").as_double();
    out->gate.single_pos_chi2 = get_parameter(prefix + ".gate.single_pos_chi2").as_double();
    out->gate.single_yaw_chi2 = get_parameter(prefix + ".gate.single_yaw_chi2").as_double();
    out->gate.dual_total_nis = get_parameter(prefix + ".gate.dual_total_nis").as_double();
    out->gate.dual_each_pos_chi2 = get_parameter(prefix + ".gate.dual_each_pos_chi2").as_double();
    out->gate.dual_each_yaw_chi2 = get_parameter(prefix + ".gate.dual_each_yaw_chi2").as_double();
    out->single_update.structural_gain_r =
      get_parameter(prefix + ".single_update.structural_gain_r").as_double();
    out->single_update.structural_gain_dza =
      get_parameter(prefix + ".single_update.structural_gain_dza").as_double();
    out->dual_update.structural_gain_r =
      get_parameter(prefix + ".dual_update.structural_gain_r").as_double();
    out->dual_update.structural_gain_dza =
      get_parameter(prefix + ".dual_update.structural_gain_dza").as_double();
    out->posterior_sanity.max_center_jump =
      get_parameter(prefix + ".posterior_sanity.max_center_jump").as_double();
    out->posterior_sanity.max_yaw_jump =
      get_parameter(prefix + ".posterior_sanity.max_yaw_jump").as_double();
    out->posterior_sanity.min_r = get_parameter(prefix + ".posterior_sanity.min_r").as_double();
    out->posterior_sanity.max_r = get_parameter(prefix + ".posterior_sanity.max_r").as_double();
    out->posterior_sanity.max_r_jump =
      get_parameter(prefix + ".posterior_sanity.max_r_jump").as_double();
    out->posterior_sanity.min_dza = get_parameter(prefix + ".posterior_sanity.min_dza").as_double();
    out->posterior_sanity.max_dza = get_parameter(prefix + ".posterior_sanity.max_dza").as_double();
    out->posterior_sanity.max_dza_jump =
      get_parameter(prefix + ".posterior_sanity.max_dza_jump").as_double();
  };
  load_norm4_baseline_ukf_cfg("norm4_baseline.inekf", &c.norm4_baseline.inekf);

  c.norm4_baseline.slow_structure.enable = get_parameter("norm4_baseline.slow_structure.enable").as_bool();
  c.norm4_baseline.slow_structure.q_theta_r1 =
    get_parameter("norm4_baseline.slow_structure.q_theta_r1").as_double();
  c.norm4_baseline.slow_structure.q_theta_r2 =
    get_parameter("norm4_baseline.slow_structure.q_theta_r2").as_double();
  c.norm4_baseline.slow_structure.q_theta_dza =
    get_parameter("norm4_baseline.slow_structure.q_theta_dza").as_double();
  c.norm4_baseline.slow_structure.prior_r1 =
    get_parameter("norm4_baseline.slow_structure.prior_r1").as_double();
  c.norm4_baseline.slow_structure.prior_r2 =
    get_parameter("norm4_baseline.slow_structure.prior_r2").as_double();
  c.norm4_baseline.slow_structure.prior_dza =
    get_parameter("norm4_baseline.slow_structure.prior_dza").as_double();
  c.norm4_baseline.slow_structure.prior_sigma_r =
    get_parameter("norm4_baseline.slow_structure.prior_sigma_r").as_double();
  c.norm4_baseline.slow_structure.prior_sigma_dza =
    get_parameter("norm4_baseline.slow_structure.prior_sigma_dza").as_double();
  c.norm4_baseline.slow_structure.alpha_r1_single =
    get_parameter("norm4_baseline.slow_structure.alpha_r1_single").as_double();
  c.norm4_baseline.slow_structure.alpha_r2_single =
    get_parameter("norm4_baseline.slow_structure.alpha_r2_single").as_double();
  c.norm4_baseline.slow_structure.alpha_dza_single =
    get_parameter("norm4_baseline.slow_structure.alpha_dza_single").as_double();
  c.norm4_baseline.slow_structure.alpha_r1_dual =
    get_parameter("norm4_baseline.slow_structure.alpha_r1_dual").as_double();
  c.norm4_baseline.slow_structure.alpha_r2_dual =
    get_parameter("norm4_baseline.slow_structure.alpha_r2_dual").as_double();
  c.norm4_baseline.slow_structure.alpha_dza_dual =
    get_parameter("norm4_baseline.slow_structure.alpha_dza_dual").as_double();
  c.norm4_baseline.slow_structure.prior_pull_gain =
    get_parameter("norm4_baseline.slow_structure.prior_pull_gain").as_double();
  c.norm4_baseline.slow_structure.min_r = get_parameter("norm4_baseline.slow_structure.min_r").as_double();
  c.norm4_baseline.slow_structure.max_r = get_parameter("norm4_baseline.slow_structure.max_r").as_double();
  c.norm4_baseline.slow_structure.min_dza = get_parameter("norm4_baseline.slow_structure.min_dza").as_double();
  c.norm4_baseline.slow_structure.max_dza = get_parameter("norm4_baseline.slow_structure.max_dza").as_double();

  c.norm4_baseline.backend_config.backend_type =
    get_parameter("norm4_baseline.backend_config.backend_type").as_string();
  c.norm4_baseline.backend_config.motion_profile =
    get_parameter("norm4_baseline.backend_config.motion_profile").as_string();
  c.norm4_baseline.backend_config.noise_profile =
    get_parameter("norm4_baseline.backend_config.noise_profile").as_string();
  c.norm4_baseline.backend_config.structure_profile =
    get_parameter("norm4_baseline.backend_config.structure_profile").as_string();
  c.norm4_baseline.inekf_runtime.motion_profile =
    get_parameter("norm4_baseline.inekf_runtime.motion_profile").as_string();
  c.norm4_baseline.inekf_runtime.noise_profile =
    get_parameter("norm4_baseline.inekf_runtime.noise_profile").as_string();
  c.norm4_baseline.inekf_runtime.structure_profile =
    get_parameter("norm4_baseline.inekf_runtime.structure_profile").as_string();
  c.norm4_baseline.inekf_runtime.translation_model =
    get_parameter("norm4_baseline.inekf_runtime.translation_model").as_string();
  c.norm4_baseline.inekf_runtime.cv_process_noise_vel =
    get_parameter("norm4_baseline.inekf_runtime.cv_process_noise_vel").as_double();
  c.norm4_baseline.inekf_runtime.ca_process_noise_acc =
    get_parameter("norm4_baseline.inekf_runtime.ca_process_noise_acc").as_double();
  c.norm4_baseline.inekf_runtime.singer_alpha =
    get_parameter("norm4_baseline.inekf_runtime.singer_alpha").as_double();
  c.norm4_baseline.inekf_runtime.singer_sigma =
    get_parameter("norm4_baseline.inekf_runtime.singer_sigma").as_double();
  c.norm4_baseline.inekf_runtime.process_noise_r =
    get_parameter("norm4_baseline.inekf_runtime.process_noise_r").as_double();
  c.norm4_baseline.inekf_runtime.process_noise_dz =
    get_parameter("norm4_baseline.inekf_runtime.process_noise_dz").as_double();
  c.norm4_baseline.inekf_runtime.spin_process_noise_delta_rate =
    get_parameter("norm4_baseline.inekf_runtime.spin_process_noise_delta_rate").as_double();
  c.norm4_baseline.inekf_runtime.spin_process_noise_delta_acc =
    get_parameter("norm4_baseline.inekf_runtime.spin_process_noise_delta_acc").as_double();

  c.norm4_baseline.hypothesis_selector.topk = get_parameter("norm4_baseline.hypothesis_selector.topk").as_int();
  c.norm4_baseline.hypothesis_selector.commit_top1_only =
    get_parameter("norm4_baseline.hypothesis_selector.commit_top1_only").as_bool();
  c.norm4_baseline.hypothesis_selector.min_top1_confidence =
    get_parameter("norm4_baseline.hypothesis_selector.min_top1_confidence").as_double();
  c.norm4_baseline.hypothesis_selector.min_top1_top2_margin =
    get_parameter("norm4_baseline.hypothesis_selector.min_top1_top2_margin").as_double();
  c.norm4_baseline.hypothesis_selector.ambiguous_margin =
    get_parameter("norm4_baseline.hypothesis_selector.ambiguous_margin").as_double();
  c.norm4_baseline.hypothesis_selector.include_rejected_in_debug =
    get_parameter("norm4_baseline.hypothesis_selector.include_rejected_in_debug").as_bool();
  c.norm4_baseline.hypothesis_selector.evidence_prior_enable =
    get_parameter("norm4_baseline.hypothesis_selector.evidence_prior_enable").as_bool();
  c.norm4_baseline.hypothesis_selector.max_reconstruction_pos_error =
    get_parameter("norm4_baseline.hypothesis_selector.max_reconstruction_pos_error").as_double();

  c.norm4_baseline.warmup.enable_dual_seed_01 =
    get_parameter("norm4_baseline.warmup.enable_dual_seed_01").as_bool();
  c.norm4_baseline.warmup.warmup_frames = get_parameter("norm4_baseline.warmup.warmup_frames").as_int();
  c.norm4_baseline.warmup.min_settle_frames = get_parameter("norm4_baseline.warmup.min_settle_frames").as_int();
  c.norm4_baseline.warmup.min_margin_to_commit =
    get_parameter("norm4_baseline.warmup.min_margin_to_commit").as_double();
  c.norm4_baseline.warmup.min_confidence_to_commit =
    get_parameter("norm4_baseline.warmup.min_confidence_to_commit").as_double();

  c.norm4_baseline.mode_routing.ambiguous_output =
    get_parameter("norm4_baseline.mode_routing.ambiguous_output").as_string();
  c.norm4_baseline.mode_routing.structured_output =
    get_parameter("norm4_baseline.mode_routing.structured_output").as_string();
  c.norm4_baseline.mode_routing.ambiguous_structured_backend_mode =
    get_parameter("norm4_baseline.mode_routing.ambiguous_structured_backend_mode").as_string();
  c.norm4_baseline.mode_routing.structured_single_plate_mode =
    get_parameter("norm4_baseline.mode_routing.structured_single_plate_mode").as_string();

  c.norm4_baseline.single_plate_bridge.enable =
    get_parameter("norm4_baseline.single_plate_bridge.enable").as_bool();
  c.norm4_baseline.single_plate_bridge.source_semantic =
    get_parameter("norm4_baseline.single_plate_bridge.source_semantic").as_string();
  c.norm4_baseline.single_plate_bridge.backend_type =
    get_parameter("norm4_baseline.single_plate_bridge.backend_type").as_string();
  c.norm4_baseline.single_plate_bridge.require_semantic_stable_frames =
    get_parameter("norm4_baseline.single_plate_bridge.require_semantic_stable_frames").as_int();

  c.norm4_baseline.fallback.predict_only_on_reject =
    get_parameter("norm4_baseline.fallback.predict_only_on_reject").as_bool();
  c.norm4_baseline.fallback.enable_ambiguous_single_fallback =
    get_parameter("norm4_baseline.fallback.enable_ambiguous_single_fallback").as_bool();
  c.norm4_baseline.debug_log.enable = get_parameter("norm4_baseline.debug_log.enable").as_bool();
  c.norm4_baseline.debug_log.throttle_ms =
    std::max<int>(50, static_cast<int>(get_parameter("norm4_baseline.debug_log.throttle_ms").as_int()));
  c.norm4_baseline.debug_log.verbose = get_parameter("norm4_baseline.debug_log.verbose").as_bool();
  c.norm4_baseline.phase_memory.ping_pong_pattern_threshold =
    std::clamp(c.norm4_baseline.phase_memory.ping_pong_pattern_threshold, 0.0, 1.0);
  c.norm4_baseline.phase_memory.anti_pingpong.min_consistent_frames_to_commit =
    std::max(1, c.norm4_baseline.phase_memory.anti_pingpong.min_consistent_frames_to_commit);
  c.norm4_baseline.phase_memory.anti_pingpong.jerk_gate =
    std::max(0.0, c.norm4_baseline.phase_memory.anti_pingpong.jerk_gate);
  c.norm4_baseline.phase_memory.anti_pingpong.yaw_rate_jump_gate =
    std::max(0.0, c.norm4_baseline.phase_memory.anti_pingpong.yaw_rate_jump_gate);
  c.norm4_baseline.phase_memory.anti_pingpong.velocity_dir_cos_min =
    std::clamp(c.norm4_baseline.phase_memory.anti_pingpong.velocity_dir_cos_min, -1.0, 1.0);
  c.norm4_baseline.phase_memory.anti_pingpong.pending_timeout_frames =
    std::max(1, c.norm4_baseline.phase_memory.anti_pingpong.pending_timeout_frames);

  c.panel_mismatch.enable = get_parameter("panel_mismatch.enable").as_bool();
  c.panel_mismatch.window_size = get_parameter("panel_mismatch.window_size").as_int();
  c.panel_mismatch.threshold_t1 = get_parameter("panel_mismatch.threshold_t1").as_double();
  c.panel_mismatch.confirm_count = get_parameter("panel_mismatch.confirm_count").as_int();
  c.panel_mismatch.reinit_count = get_parameter("panel_mismatch.reinit_count").as_int();
  c.panel_mismatch.apply_correction = get_parameter("panel_mismatch.apply_correction").as_bool();

  c.outpost.translation_model =
    translation_model_from_string(get_parameter("outpost.translation_model").as_string());
  c.outpost.rotation_model =
    rotation_model_from_string(get_parameter("outpost.rotation_model").as_string());
  c.outpost.tracking_thres = get_parameter("outpost.tracking_thres").as_int();
  c.outpost.lost_thres = get_parameter("outpost.lost_thres").as_int();
  c.outpost.temp_lost_thres = get_parameter("outpost.temp_lost_thres").as_int();
  c.outpost.max_match_distance = get_parameter("outpost.max_match_distance").as_double();
  c.outpost.max_match_yaw_diff = get_parameter("outpost.max_match_yaw_diff").as_double();
  c.outpost.singer_alpha = get_parameter("outpost.singer_alpha").as_double();
  c.outpost.singer_sigma = get_parameter("outpost.singer_sigma").as_double();
  c.outpost.spin_process_noise_theta_rate =
    get_parameter("outpost.spin_process_noise_theta_rate").as_double();
  c.outpost.spin_process_noise_theta_acc =
    get_parameter("outpost.spin_process_noise_theta_acc").as_double();
  c.outpost.radius = get_parameter("outpost.radius").as_double();
  c.outpost.z_offset_0 = get_parameter("outpost.z_offset_0").as_double();
  c.outpost.z_offset_1 = get_parameter("outpost.z_offset_1").as_double();
  c.outpost.z_offset_2 = get_parameter("outpost.z_offset_2").as_double();
  c.outpost.panel_angle_step = get_parameter("outpost.panel_angle_step").as_double();
  c.outpost.softmax_temperature = get_parameter("outpost.softmax_temperature").as_double();
  c.outpost.weight_yaw = get_parameter("outpost.weight_yaw").as_double();
  c.outpost.weight_z_state = get_parameter("outpost.weight_z_state").as_double();
  c.outpost.weight_z_history = get_parameter("outpost.weight_z_history").as_double();
  c.outpost.weight_xy_residual = get_parameter("outpost.weight_xy_residual").as_double();
  c.outpost.weight_switch_penalty = get_parameter("outpost.weight_switch_penalty").as_double();
  c.outpost.entropy_enter = get_parameter("outpost.entropy_enter").as_double();
  c.outpost.entropy_exit = get_parameter("outpost.entropy_exit").as_double();
  c.outpost.max_prob_enter = get_parameter("outpost.max_prob_enter").as_double();
  c.outpost.max_prob_exit = get_parameter("outpost.max_prob_exit").as_double();
  c.outpost.stable_frames = get_parameter("outpost.stable_frames").as_int();
  c.outpost.z_history_window = get_parameter("outpost.z_history_window").as_int();
  c.outpost.single_mode_confidence_scale =
    get_parameter("outpost.single_mode_confidence_scale").as_double();
  c.outpost.binding_transition_confirm_frames =
    get_parameter("outpost.binding_transition_confirm_frames").as_int();
  c.outpost.binding_same_panel_yaw_gate =
    get_parameter("outpost.binding_same_panel_yaw_gate").as_double();
  c.outpost.binding_same_panel_z_gate =
    get_parameter("outpost.binding_same_panel_z_gate").as_double();
  c.outpost.binding_same_panel_xy_gate =
    get_parameter("outpost.binding_same_panel_xy_gate").as_double();
  c.outpost.binding_min_candidate_prob =
    get_parameter("outpost.binding_min_candidate_prob").as_double();
  c.outpost.binding_min_candidate_margin =
    get_parameter("outpost.binding_min_candidate_margin").as_double();
  c.outpost.binding_switch_strong_score =
    get_parameter("outpost.binding_switch_strong_score").as_double();
  c.outpost.binding_period_window = get_parameter("outpost.binding_period_window").as_int();
  c.outpost.binding_period_weight = get_parameter("outpost.binding_period_weight").as_double();
  c.outpost.binding_topology_prior_weight =
    get_parameter("outpost.binding_topology_prior_weight").as_double();
  c.outpost.binding_period_min_spin_rate =
    get_parameter("outpost.binding_period_min_spin_rate").as_double();
  c.outpost.spin_direction_confirm_frames =
    get_parameter("outpost.spin_direction_confirm_frames").as_int();
  c.outpost.binding_period_update_min_confidence =
    get_parameter("outpost.binding_period_update_min_confidence").as_double();
  c.outpost.binding_period_update_min_jump =
    get_parameter("outpost.binding_period_update_min_jump").as_double();
  c.outpost.binding_dz_ema_alpha = get_parameter("outpost.binding_dz_ema_alpha").as_double();
  c.outpost.binding_confidence_floor =
    get_parameter("outpost.binding_confidence_floor").as_double();
  c.outpost.z_audit_rebind_enable = get_parameter("outpost.z_audit_rebind_enable").as_bool();
  c.outpost.z_audit_rebind_confirm_frames =
    get_parameter("outpost.z_audit_rebind_confirm_frames").as_int();
  c.outpost.z_audit_rebind_min_confidence =
    get_parameter("outpost.z_audit_rebind_min_confidence").as_double();
  c.outpost.z_audit_rebind_min_jump = get_parameter("outpost.z_audit_rebind_min_jump").as_double();
  c.outpost.binding_conflict_position_scale =
    get_parameter("outpost.binding_conflict_position_scale").as_double();
  c.outpost.alpha_pos = get_parameter("outpost.alpha_pos").as_double();
  c.outpost.beta_vel = get_parameter("outpost.beta_vel").as_double();
  c.outpost.alpha_yaw = get_parameter("outpost.alpha_yaw").as_double();
  c.outpost.beta_yaw_rate = get_parameter("outpost.beta_yaw_rate").as_double();
  c.outpost.assume_static_center = get_parameter("outpost.assume_static_center").as_bool();
  c.outpost.linear_velocity_damping = get_parameter("outpost.linear_velocity_damping").as_double();
  c.outpost.yaw_rate_damping = get_parameter("outpost.yaw_rate_damping").as_double();
  c.outpost.max_center_speed = get_parameter("outpost.max_center_speed").as_double();
  c.outpost.max_yaw_rate = get_parameter("outpost.max_yaw_rate").as_double();
  c.outpost.max_yaw_rate_step = get_parameter("outpost.max_yaw_rate_step").as_double();
  c.outpost.mode_enter_confirm_frames = get_parameter("outpost.mode_enter_confirm_frames").as_int();
  c.outpost.mode_exit_confirm_frames = get_parameter("outpost.mode_exit_confirm_frames").as_int();
  c.outpost.mode_min_dwell_frames = get_parameter("outpost.mode_min_dwell_frames").as_int();
  c.outpost.mode_enter_threshold = get_parameter("outpost.mode_enter_threshold").as_double();
  c.outpost.mode_exit_threshold = get_parameter("outpost.mode_exit_threshold").as_double();
  c.outpost.mode_weight_jump = get_parameter("outpost.mode_weight_jump").as_double();
  c.outpost.mode_weight_dual = get_parameter("outpost.mode_weight_dual").as_double();
  c.outpost.mode_weight_margin = get_parameter("outpost.mode_weight_margin").as_double();
  c.outpost.mode_weight_health = get_parameter("outpost.mode_weight_health").as_double();
  c.outpost.mode_weight_entropy = get_parameter("outpost.mode_weight_entropy").as_double();
  c.outpost.ambiguous_publish_single_armor_semantics =
    get_parameter("outpost.ambiguous_publish_single_armor_semantics").as_bool();
  c.outpost.ambiguous_single_armor_zero_offset =
    get_parameter("outpost.ambiguous_single_armor_zero_offset").as_bool();
  c.outpost.ambiguous_backend_use_imm_adapter =
    get_parameter("outpost.ambiguous_backend_use_imm_adapter").as_bool();
  c.outpost.baseline_warmup_enable = get_parameter("outpost.baseline_warmup_enable").as_bool();
  c.outpost.baseline_warmup_min_groups = get_parameter("outpost.baseline_warmup_min_groups").as_int();
  c.outpost.baseline_warmup_min_samples_per_group =
    get_parameter("outpost.baseline_warmup_min_samples_per_group").as_int();
  c.outpost.baseline_warmup_max_frames = get_parameter("outpost.baseline_warmup_max_frames").as_int();
  c.outpost.baseline_warmup_z_jump_gate = get_parameter("outpost.baseline_warmup_z_jump_gate").as_double();
  c.outpost.baseline_warmup_yaw_jump_gate = get_parameter("outpost.baseline_warmup_yaw_jump_gate").as_double();
  c.outpost.baseline_warmup_xyz_jump_gate = get_parameter("outpost.baseline_warmup_xyz_jump_gate").as_double();
  c.outpost.baseline_warmup_ratio_min = get_parameter("outpost.baseline_warmup_ratio_min").as_double();
  c.outpost.baseline_warmup_ratio_max = get_parameter("outpost.baseline_warmup_ratio_max").as_double();
  c.outpost.baseline_warmup_min_large_diff =
    get_parameter("outpost.baseline_warmup_min_large_diff").as_double();
  c.outpost.tracking_thres = std::max(1, c.outpost.tracking_thres);
  c.outpost.lost_thres = std::max(1, c.outpost.lost_thres);
  c.outpost.temp_lost_thres = std::max(1, c.outpost.temp_lost_thres);
  c.outpost.max_match_distance = std::max(0.0, c.outpost.max_match_distance);
  c.outpost.max_match_yaw_diff = std::max(0.0, c.outpost.max_match_yaw_diff);

  c.outpost.binding_transition_confirm_frames =
    std::max(1, c.outpost.binding_transition_confirm_frames);
  c.outpost.binding_same_panel_yaw_gate = std::max(1e-3, c.outpost.binding_same_panel_yaw_gate);
  c.outpost.binding_same_panel_z_gate = std::max(1e-3, c.outpost.binding_same_panel_z_gate);
  c.outpost.binding_same_panel_xy_gate = std::max(1e-3, c.outpost.binding_same_panel_xy_gate);
  c.outpost.binding_min_candidate_prob = std::clamp(c.outpost.binding_min_candidate_prob, 0.0, 1.0);
  c.outpost.binding_min_candidate_margin =
    std::clamp(c.outpost.binding_min_candidate_margin, 0.0, 1.0);
  c.outpost.binding_switch_strong_score =
    std::clamp(c.outpost.binding_switch_strong_score, 0.0, 1.0);
  c.outpost.binding_period_window = std::max(3, c.outpost.binding_period_window);
  c.outpost.binding_period_weight = std::max(0.0, c.outpost.binding_period_weight);
  c.outpost.binding_topology_prior_weight = std::max(0.0, c.outpost.binding_topology_prior_weight);
  c.outpost.binding_period_min_spin_rate = std::max(0.0, c.outpost.binding_period_min_spin_rate);
  c.outpost.spin_direction_confirm_frames = std::max(1, c.outpost.spin_direction_confirm_frames);
  c.outpost.binding_period_update_min_confidence =
    std::clamp(c.outpost.binding_period_update_min_confidence, 0.0, 1.0);
  c.outpost.binding_period_update_min_jump =
    std::max(0.0, c.outpost.binding_period_update_min_jump);
  c.outpost.max_yaw_rate_step = std::max(0.0, c.outpost.max_yaw_rate_step);
  c.outpost.binding_dz_ema_alpha = std::clamp(c.outpost.binding_dz_ema_alpha, 0.01, 1.0);
  c.outpost.binding_confidence_floor = std::clamp(c.outpost.binding_confidence_floor, 0.0, 0.95);
  c.outpost.z_audit_rebind_confirm_frames = std::max(1, c.outpost.z_audit_rebind_confirm_frames);
  c.outpost.z_audit_rebind_min_confidence =
    std::clamp(c.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
  c.outpost.z_audit_rebind_min_jump = std::max(0.0, c.outpost.z_audit_rebind_min_jump);
  c.outpost.baseline_warmup_min_groups = std::clamp(c.outpost.baseline_warmup_min_groups, 2, 6);
  c.outpost.baseline_warmup_min_samples_per_group =
    std::max(1, c.outpost.baseline_warmup_min_samples_per_group);
  c.outpost.baseline_warmup_max_frames = std::max(1, c.outpost.baseline_warmup_max_frames);
  c.outpost.baseline_warmup_z_jump_gate = std::max(0.0, c.outpost.baseline_warmup_z_jump_gate);
  c.outpost.baseline_warmup_yaw_jump_gate = std::max(0.0, c.outpost.baseline_warmup_yaw_jump_gate);
  c.outpost.baseline_warmup_xyz_jump_gate = std::max(0.0, c.outpost.baseline_warmup_xyz_jump_gate);
  c.outpost.baseline_warmup_ratio_min = std::max(1.0, c.outpost.baseline_warmup_ratio_min);
  c.outpost.baseline_warmup_ratio_max =
    std::max(c.outpost.baseline_warmup_ratio_min, c.outpost.baseline_warmup_ratio_max);
  c.outpost.baseline_warmup_min_large_diff = std::max(0.0, c.outpost.baseline_warmup_min_large_diff);
  c.outpost.binding_conflict_position_scale =
    std::clamp(c.outpost.binding_conflict_position_scale, 0.0, 1.0);
  c.outpost.weight_xy_residual = std::max(0.0, c.outpost.weight_xy_residual);
  c.outpost.weight_switch_penalty = std::max(0.0, c.outpost.weight_switch_penalty);
  c.outpost.mode_enter_confirm_frames = std::max(1, c.outpost.mode_enter_confirm_frames);
  c.outpost.mode_exit_confirm_frames = std::max(1, c.outpost.mode_exit_confirm_frames);
  c.outpost.mode_min_dwell_frames = std::max(1, c.outpost.mode_min_dwell_frames);
  c.outpost.mode_enter_threshold = std::clamp(c.outpost.mode_enter_threshold, 0.0, 1.0);
  c.outpost.mode_exit_threshold = std::clamp(c.outpost.mode_exit_threshold, 0.0, 1.0);
  c.outpost.mode_weight_jump = std::max(0.0, c.outpost.mode_weight_jump);
  c.outpost.mode_weight_dual = std::max(0.0, c.outpost.mode_weight_dual);
  c.outpost.mode_weight_margin = std::max(0.0, c.outpost.mode_weight_margin);
  c.outpost.mode_weight_health = std::max(0.0, c.outpost.mode_weight_health);
  c.outpost.mode_weight_entropy = std::max(0.0, c.outpost.mode_weight_entropy);
  const bool z_descending =
    (c.outpost.z_offset_0 > c.outpost.z_offset_1) && (c.outpost.z_offset_1 > c.outpost.z_offset_2);
  if (!z_descending) {
    RCLCPP_WARN(get_logger(),
                "Outpost z-offset semantic mismatch: expected z0>z1>z2 for "
                "[highest,middle,lowest], got [%.4f, %.4f, %.4f]",
                c.outpost.z_offset_0,
                c.outpost.z_offset_1,
                c.outpost.z_offset_2);
  }

  const double expected_step = 2.0 * M_PI / 3.0;
  if (std::abs(std::abs(c.outpost.panel_angle_step) - expected_step) > 1e-3) {
    RCLCPP_WARN(get_logger(),
                "Outpost panel_angle_step=%.6f differs from 2pi/3; semantic contract "
                "(top-down clockwise: 0->2->1) assumes 120deg spacing.",
                c.outpost.panel_angle_step);
  }

  static bool outpost_semantic_logged = false;
  if (!outpost_semantic_logged) {
    outpost_semantic_logged = true;
    RCLCPP_INFO(get_logger(),
                "Outpost semantic contract enabled: id0=highest@0deg, clockwise order "
                "id0->id2->id1.");
  }

  // Output smoother
  smoother_config_.enable = get_parameter("smoother.enable").as_bool();
  smoother_config_.enable_position_smooth =
    get_parameter("smoother.enable_position_smooth").as_bool();
  smoother_config_.enable_yaw_smooth = get_parameter("smoother.enable_yaw_smooth").as_bool();
  smoother_config_.enable_velocity_smooth =
    get_parameter("smoother.enable_velocity_smooth").as_bool();
  smoother_config_.enable_structural_convergence =
    get_parameter("smoother.enable_structural_convergence").as_bool();
  smoother_config_.pos_min_cutoff = get_parameter("smoother.pos_min_cutoff").as_double();
  smoother_config_.pos_beta = get_parameter("smoother.pos_beta").as_double();
  smoother_config_.pos_d_cutoff = get_parameter("smoother.pos_d_cutoff").as_double();
  smoother_config_.yaw_min_cutoff = get_parameter("smoother.yaw_min_cutoff").as_double();
  smoother_config_.yaw_beta = get_parameter("smoother.yaw_beta").as_double();
  smoother_config_.yaw_d_cutoff = get_parameter("smoother.yaw_d_cutoff").as_double();
  smoother_config_.vel_min_cutoff = get_parameter("smoother.vel_min_cutoff").as_double();
  smoother_config_.vel_beta = get_parameter("smoother.vel_beta").as_double();
  smoother_config_.vel_d_cutoff = get_parameter("smoother.vel_d_cutoff").as_double();
  smoother_config_.rm_initial_step = get_parameter("smoother.rm_initial_step").as_double();
  smoother_config_.rm_gamma = get_parameter("smoother.rm_gamma").as_double();
  smoother_config_.rm_n0 = get_parameter("smoother.rm_n0").as_int();
  smoother_config_.rm_dual_obs_boost = get_parameter("smoother.rm_dual_obs_boost").as_double();
  smoother_config_.rm_min_radius = get_parameter("smoother.rm_min_radius").as_double();
  smoother_config_.rm_max_radius = get_parameter("smoother.rm_max_radius").as_double();
  smoother_config_.rm_min_dz = get_parameter("smoother.rm_min_dz").as_double();
  smoother_config_.rm_max_dz = get_parameter("smoother.rm_max_dz").as_double();
  smoother_config_.rm_convergence_eps = get_parameter("smoother.rm_convergence_eps").as_double();
  smoother_config_.default_freq = get_parameter("smoother.default_freq").as_double();

  // Outlier filter parameters
  smoother_config_.enable_outlier_filter =
    get_parameter("smoother.enable_outlier_filter").as_bool();
  smoother_config_.outlier_method = get_parameter("smoother.outlier_method").as_string();
  smoother_config_.outlier_window_size = get_parameter("smoother.outlier_window_size").as_int();
  smoother_config_.outlier_min_samples = get_parameter("smoother.outlier_min_samples").as_int();
  smoother_config_.outlier_mad_k = get_parameter("smoother.outlier_mad_k").as_double();
  smoother_config_.outlier_iqr_k = get_parameter("smoother.outlier_iqr_k").as_double();
  smoother_config_.outlier_mahal_threshold =
    get_parameter("smoother.outlier_mahal_threshold").as_double();

  RCLCPP_INFO(get_logger(),
              "Tracker parameters applied (smoother %s, outlier_filter %s [%s])",
              smoother_config_.enable ? "ON" : "OFF",
              smoother_config_.enable_outlier_filter ? "ON" : "OFF",
              smoother_config_.outlier_method.c_str());
}

/* ================================================================ */
/*  Armors callback — tracker + selector pipeline                    */
/* ================================================================ */

}  // namespace fyt::auto_aim
