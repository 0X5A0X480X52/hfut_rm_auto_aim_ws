// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalPipelineNode — unified node merging MaxEntropyTracker +
// TargetSelector + GimbalController. Inter-node ROS2 topics are replaced
// by direct C++ function calls to eliminate serialization / scheduling
// latency.

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"
#include "gimbal_pipeline/adapters/pipeline_config_loader.hpp"

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

#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/tracker/norm4_tracker_baseline.hpp"
#include "max_entropy_tracker/visualization.hpp"

// Gimbal strategies
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

namespace fyt::auto_aim {

/* ================================================================ */
/*  Construction                                                     */
/* ================================================================ */

GimbalPipelineNode::GimbalPipelineNode(const rclcpp::NodeOptions &options)
: Node("gimbal_pipeline", options) {
  RCLCPP_INFO(get_logger(), "Initializing GimbalPipelineNode (unified pipeline)");

  // ── 1. Declare all parameters ──
  declareTrackerParameters();
  declareTargetSelectorParameters();
  declareGimbalControllerParameters();

  RCLCPP_INFO(get_logger(), "Parameters declared, now loading...");

  // ── 2. Common / tracker params ──
  loadCommonParameters();
  auto pipeline_config = pipeline::ros_adapter::PipelineConfigLoader::load(*this);
  initTrackerConfig();

  // ── 3. Robot description facade ──
  // Independent of tf2/tracker; logged first to match original startup order.
  initRobotDescription();

  // ── 4. TF2 buffer + tracker manager ──
  // tf2_buffer_ must be created before initGimbalComponents() (shared with
  // TFHandler, MessageFilter, and gimbal state).
  initTf2AndTracker();

  // ── 5. External targets adapter ──
  initExternalTargets();

  // ── 6. Gimbal controller components + per-category config ──
  bullet_speed_ = get_parameter("controller.bullet_speed").as_double();
  control_rate_ = get_parameter("controller.control_rate").as_double();
  initGimbalComponents();
  initGimbalControl();
  configureArmorSelector();
  configureFireAdvisor();
  configureOutputFilter();

  RCLCPP_INFO(get_logger(),
              "GimbalPipelineNode initialized: target_frame=%s, control_rate=%.0f Hz, "
              "selector=priority_list, controller=mpc, ballistic_mode=local",
              target_frame_.c_str(),
              control_rate_);

  // ── 7. AutoAim pipeline core ──
  auto_aim_pipeline_ = std::make_unique<pipeline::AutoAimPipeline>(
    [this](const pipeline::ObservationFrame & frame) {
      return processObservationFrame(frame);
    },
    std::move(pipeline_config),
    [this](const pipeline::ControlRequest & request) {
      return computeGimbalCommand(request);
    });

  // ── 8. ROS2 external interfaces ──
  initRosInterfaces();

  // ── 9. Prediction logger + heartbeat ──
  initPredictionLogger();
  heartbeat_ = HeartBeatPublisher::create(this);

  RCLCPP_INFO(get_logger(), "GimbalPipelineNode (unified pipeline) initialized successfully");

  RCLCPP_INFO(get_logger(),
              "GimbalPipelineNode initialized: target_frame=%s, "
              "control_rate=%.0f Hz, strategy=%s, ballistic=local",
              target_frame_.c_str(),
              control_rate_,
              "mpc");
}

/* ================================================================ */
/*  Construction helpers                                             */
/* ================================================================ */

void GimbalPipelineNode::loadCommonParameters() {
  // Common frames / rate / debug flags shared by tracker, selector, and controller.
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  predict_rate_ = get_parameter("predict_rate").as_double();
  debug_mode_ = get_parameter("debug_mode").as_bool();
  visualization_frame_ = get_parameter("visualization_frame").as_string();
  tracker_timeout_s_ = std::max(get_parameter("tracker_timeout").as_double(), 1e-3);

  // 2D tracker debug visualization (clamped to a minimum resolution).
  tracker_2d_image_debug_enable_ = get_parameter("tracker.debug_2d_viz.enable").as_bool();
  tracker_2d_image_debug_width_ =
    std::max(static_cast<int>(get_parameter("tracker.debug_2d_viz.width").as_int()), 320);
  tracker_2d_image_debug_height_ =
    std::max(static_cast<int>(get_parameter("tracker.debug_2d_viz.height").as_int()), 240);
  tracker_2d_image_debug_jpeg_quality_ = std::clamp(
    static_cast<int>(get_parameter("tracker.debug_2d_viz.jpeg_quality").as_int()), 20, 100);
}

void GimbalPipelineNode::initTrackerConfig() {
  tracker_config_ = UnifiedConfig::create_default();
  applyTrackerParamsToConfig();
}

void GimbalPipelineNode::initTf2AndTracker() {
  // TF2 buffer — shared by TFHandler, MessageFilter, and gimbal state.
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                                   get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // TF handler (used by tracker for armor → odom transform).
  // Shares tf2_buffer_ so MessageFilter and TFHandler use the same cache.
  tf_handler_ = std::make_unique<TFHandler>(tf2_buffer_, target_frame_);

  // Tracker manager
  double dt = (predict_rate_ > 0) ? (1.0 / predict_rate_) : 0.01;
  tracker_manager_ =
    std::make_unique<TrackerManager>(tracker_config_,
                                     dt,
                                     get_parameter("default_r1").as_double(),
                                     get_parameter("default_r2").as_double(),
                                     get_parameter("default_dza").as_double(),
                                     tracker_timeout_s_,
                                     get_parameter("enable_oscillation_detection").as_bool());

  RCLCPP_INFO(get_logger(), "Tracker initialized (predict_rate=%.1f Hz)", predict_rate_);
}

void GimbalPipelineNode::initRobotDescription() {
  robot_description_facade_ = std::make_unique<robot_description::RobotDescriptionFacade>();
  robot_description_facade_->setStrictUnknownReject(
    get_parameter("robot_description.strict_unknown_reject").as_bool());

  // Projection mode + full-SE3 robot policy from ROS params.
  const auto mode_raw = get_parameter("robot_description.default_projection_mode").as_string();
  std::string mode_lower = mode_raw;
  std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  using RU = robot_description::TrackedRobotUsage;
  RU::ProjectionMode default_mode = RU::ProjectionMode::YAW_PLANE;
  if (mode_lower == "full_se3") {
    default_mode = RU::ProjectionMode::FULL_SE3;
  } else if (mode_lower == "yaw_plane") {
    default_mode = RU::ProjectionMode::YAW_PLANE;
  } else {
    RCLCPP_WARN(get_logger(),
                "Unknown robot_description.default_projection_mode='%s', fallback to yaw_plane",
                mode_raw.c_str());
  }

  const auto full_se3_ids_vec = get_parameter("robot_description.full_se3_ids").as_string_array();
  std::unordered_set<std::string> full_se3_ids(full_se3_ids_vec.begin(), full_se3_ids_vec.end());

  const auto full_se3_types_vec =
    get_parameter("robot_description.full_se3_robot_types").as_integer_array();
  std::unordered_set<uint8_t> full_se3_robot_types;
  for (const auto v : full_se3_types_vec) {
    if (v < 0 || v > 255) {
      RCLCPP_WARN(
        get_logger(),
        "robot_description.full_se3_robot_types contains out-of-range value %ld, ignored",
        static_cast<long>(v));
      continue;
    }
    full_se3_robot_types.insert(static_cast<uint8_t>(v));
  }

  RU::setProjectionModePolicy(default_mode, full_se3_ids, full_se3_robot_types);

  std::ostringstream oss;
  const auto supported_ids = robot_description_facade_->supportedRobotIds();
  for (size_t i = 0; i < supported_ids.size(); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << supported_ids[i];
  }
  RCLCPP_INFO(get_logger(),
              "RobotDescription initialized (strict_unknown_reject=%s, supported_ids=[%s])",
              robot_description_facade_->strictUnknownReject() ? "true" : "false",
              oss.str().c_str());
}

void GimbalPipelineNode::initExternalTargets() {
  external_targets_enable_ = get_parameter("external_targets.enable").as_bool();
  external_targets_buff_enable_ = get_parameter("external_targets.buff.enable").as_bool();
  external_targets_buff_topic_ = get_parameter("external_targets.buff.topic").as_string();
  external_targets_buff_timeout_s_ = get_parameter("external_targets.buff.timeout_s").as_double();

  if (external_targets_enable_ && external_targets_buff_enable_) {
    adapters::BuffTargetAdapter::Config cfg;
    cfg.enable = true;
    cfg.topic = external_targets_buff_topic_;
    cfg.timeout_s = std::max(0.01, external_targets_buff_timeout_s_);
    cfg.target_frame = target_frame_;
    buff_target_adapter_ = std::make_unique<adapters::BuffTargetAdapter>(*this, cfg);
  }
}

void GimbalPipelineNode::configureArmorSelector() {
  // Solver / facing / radial / virtual-pose selection parameters.
  const double side_angle = get_parameter("controller.solver.side_angle").as_double();
  const double min_switching_v_yaw = get_parameter("controller.solver.min_switching_v_yaw").as_double();
  const double facing_enter_angle = get_parameter("controller.solver.facing_enter_angle").as_double();
  const double facing_exit_angle = get_parameter("controller.solver.facing_exit_angle").as_double();
  const bool radial_dynamic_enable = get_parameter("controller.solver.radial_dynamic.enable").as_bool();
  const double radial_dynamic_v_yaw_ref =
    get_parameter("controller.solver.radial_dynamic.v_yaw_ref").as_double();
  const double radial_dynamic_shrink_ratio =
    get_parameter("controller.solver.radial_dynamic.shrink_ratio").as_double();
  const double radial_dynamic_min_angle_deg =
    get_parameter("controller.solver.radial_dynamic.min_angle_deg").as_double();
  const double radial_dynamic_bias_gain_deg =
    get_parameter("controller.solver.radial_dynamic.bias_gain_deg").as_double();
  const double radial_dynamic_max_bias_deg =
    get_parameter("controller.solver.radial_dynamic.max_bias_deg").as_double();
  const bool virtual_auto_switch_enable =
    get_parameter("controller.solver.virtual_pose.auto_switch.enable").as_bool();
  const double virtual_auto_switch_enter_vyaw =
    get_parameter("controller.solver.virtual_pose.auto_switch.enter_vyaw").as_double();
  const double virtual_auto_switch_exit_vyaw =
    get_parameter("controller.solver.virtual_pose.auto_switch.exit_vyaw").as_double();
  const std::string virtual_auto_switch_method_str =
    get_parameter("controller.solver.virtual_pose.auto_switch.selection_method").as_string();
  const int virtual_auto_switch_fixed_id =
    get_parameter("controller.solver.virtual_pose.auto_switch.fixed_id").as_int();
  const int virtual_fixed_id = get_parameter("controller.solver.virtual_pose.fixed_id").as_int();
  const std::string selection_method_str =
    get_parameter("controller.solver.selection_method").as_string();

  // Cache selector parameters for marker visualization.
  facing_enter_angle_deg_ = facing_enter_angle;
  facing_exit_angle_deg_ = facing_exit_angle;
  radial_dynamic_enable_ = radial_dynamic_enable;
  radial_dynamic_v_yaw_ref_ = std::max(radial_dynamic_v_yaw_ref, 1e-6);
  radial_dynamic_shrink_ratio_ = std::clamp(radial_dynamic_shrink_ratio, 0.0, 1.0);
  radial_dynamic_min_angle_deg_ = std::max(radial_dynamic_min_angle_deg, 0.0);
  radial_dynamic_bias_gain_deg_ = std::max(radial_dynamic_bias_gain_deg, 0.0);
  radial_dynamic_max_bias_deg_ = std::max(radial_dynamic_max_bias_deg, 0.0);
  virtual_auto_switch_enable_ = virtual_auto_switch_enable;
  mpc_dt_debug_ = std::max(get_parameter("controller.mpc.dt").as_double(), 1e-4);

  armor_selector_->setParameters(side_angle, min_switching_v_yaw);
  armor_selector_->setFacingParameters(facing_enter_angle, facing_exit_angle);
  armor_selector_->setRadialDynamicParameters(radial_dynamic_enable,
                                              radial_dynamic_v_yaw_ref,
                                              radial_dynamic_shrink_ratio,
                                              radial_dynamic_min_angle_deg,
                                              radial_dynamic_bias_gain_deg,
                                              radial_dynamic_max_bias_deg);
  armor_selector_->setVirtualPoseParameters(
    virtual_auto_switch_enable, virtual_auto_switch_enter_vyaw, virtual_auto_switch_exit_vyaw);
  gimbal_controller::ArmorSelector::SelectionMethod auto_switch_method =
    gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_POSE;
  if (virtual_auto_switch_method_str == "virtual_fixed_id") {
    auto_switch_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_FIXED_ID;
  }
  armor_selector_->setVirtualAutoSwitchMethod(auto_switch_method);
  armor_selector_->setVirtualAutoSwitchFixedId(virtual_auto_switch_fixed_id);
  armor_selector_->setVirtualFixedId(virtual_fixed_id);

  // 配置选板策略
  gimbal_controller::ArmorSelector::SelectionMethod sel_method =
    gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_FACING;
  if (selection_method_str == "min_movement") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT;
  } else if (selection_method_str == "min_movement_with_radial") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL;
  } else if (selection_method_str == "decision_angle") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::DECISION_ANGLE;
  } else if (selection_method_str == "virtual_pose") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_POSE;
  } else if (selection_method_str == "virtual_fixed_id") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_FIXED_ID;
  } else if (selection_method_str == "facing_or_virtual_pose") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::FACING_OR_VIRTUAL_POSE;
  } else if (selection_method_str == "facing_or_virtual_fixed_id") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::FACING_OR_VIRTUAL_FIXED_ID;
  }
  armor_selector_->setSelectionMethod(sel_method);
  radial_selection_enabled_ =
    (sel_method == gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL);
  RCLCPP_INFO(
    get_logger(), "[GimbalController] selection_method: %s", selection_method_str.c_str());
}

void GimbalPipelineNode::configureFireAdvisor() {
  // Solver hitbox / ballistic params reused by fire advisor & probability engine.
  const double shooting_range_w = get_parameter("controller.solver.shooting_range_width").as_double();
  const double shooting_range_h = get_parameter("controller.solver.shooting_range_height").as_double();
  const double facing_exit_angle = get_parameter("controller.solver.facing_exit_angle").as_double();
  const double prediction_delay = get_parameter("controller.delay.prediction_extra_s").as_double();
  const double controller_delay = get_parameter("controller.delay.control_latency_s").as_double();
  const double trigger_to_muzzle_s = get_parameter("controller.delay.trigger_to_muzzle_s").as_double();
  const double max_processing_delay_s =
    get_parameter("controller.delay.max_processing_delay_s").as_double();
  const double gravity = get_parameter("controller.solver.gravity").as_double();
  const double resistance = get_parameter("controller.solver.resistance").as_double();
  const int iteration_times = get_parameter("controller.solver.iteration_times").as_int();

  // Fire decision / visibility / velocity low-pass params.
  const std::string fire_policy = get_parameter("controller.fire.decision_policy").as_string();
  const int fire_flight_time_iters = get_parameter("controller.delay.flight_time_iters").as_int();
  double fire_facing_filter_opening_angle_deg =
    get_parameter("controller.fire.facing_filter_opening_angle_deg").as_double();
  const bool fire_use_gimbal_kinematics =
    get_parameter("controller.fire.use_gimbal_kinematics").as_bool();
  const bool fire_velocity_low_pass_enable =
    get_parameter("controller.fire.velocity_low_pass.enable").as_bool();
  const double fire_velocity_low_pass_alpha =
    get_parameter("controller.fire.velocity_low_pass.alpha").as_double();
  const double fire_velocity_low_pass_reset_timeout_s =
    get_parameter("controller.fire.velocity_low_pass.reset_timeout_s").as_double();

  // Probability engine params.
  const bool fire_probability_enable =
    get_parameter("controller.fire.probability.enable").as_bool();
  const double fire_probability_window_ms =
    get_parameter("controller.fire.probability.future_window_ms").as_double();
  const double fire_probability_step_ms =
    get_parameter("controller.fire.probability.future_step_ms").as_double();
  const std::string fire_probability_window_fusion =
    get_parameter("controller.fire.probability.window_fusion").as_string();
  const double fire_probability_softmax_beta =
    get_parameter("controller.fire.probability.softmax_beta").as_double();
  const std::string fire_probability_gate_strategy =
    get_parameter("controller.fire.probability.gate.strategy").as_string();
  const int fire_probability_burst_count =
    get_parameter("controller.fire.probability.burst.burst_bullet_count").as_int();
  const int fire_probability_min_hit_count =
    get_parameter("controller.fire.probability.burst.min_hit_count").as_int();
  const double fire_probability_ref_p0 =
    get_parameter("controller.fire.probability.evidence.reference_probability_p0").as_double();
  const double fire_probability_evidence_window_ms =
    get_parameter("controller.fire.probability.evidence.window_ms").as_double();
  const double fire_probability_evidence_log_clip =
    get_parameter("controller.fire.probability.evidence.log_clip").as_double();
  const double fire_probability_evidence_epsilon =
    get_parameter("controller.fire.probability.evidence.epsilon").as_double();
  const bool fire_probability_neutralize_unshootable_samples =
    get_parameter("controller.fire.probability.evidence.neutralize_unshootable_samples").as_bool();
  const double fire_probability_negative_evidence_scale =
    get_parameter("controller.fire.probability.evidence.negative_evidence_scale").as_double();
  const double fire_probability_negative_clip_scale =
    get_parameter("controller.fire.probability.evidence.negative_clip_scale").as_double();
  const double fire_probability_evidence_deadband =
    get_parameter("controller.fire.probability.evidence.deadband").as_double();
  const double fire_probability_temperature =
    get_parameter("controller.fire.probability.temperature.value").as_double();
  const double fire_probability_theta_on_cold =
    get_parameter("controller.fire.probability.temperature.theta_on_cold").as_double();
  const double fire_probability_theta_on_hot =
    get_parameter("controller.fire.probability.temperature.theta_on_hot").as_double();
  const double fire_probability_theta_hold_cold =
    get_parameter("controller.fire.probability.temperature.theta_hold_cold").as_double();
  const double fire_probability_theta_hold_hot =
    get_parameter("controller.fire.probability.temperature.theta_hold_hot").as_double();
  const double fire_probability_theta_reset_cold =
    get_parameter("controller.fire.probability.temperature.theta_reset_cold").as_double();
  const double fire_probability_theta_reset_hot =
    get_parameter("controller.fire.probability.temperature.theta_reset_hot").as_double();
  const double fire_probability_min_fire_ms =
    get_parameter("controller.fire.probability.commit.min_fire_ms").as_double();
  const double fire_probability_cooldown_ms =
    get_parameter("controller.fire.probability.commit.cooldown_ms").as_double();

  // Fire probability visualization / image-debug flags.
  fire_prob_vis_enable_ = get_parameter("controller.fire.visualization.enable").as_bool();
  fire_prob_vis_ellipse_samples_ =
    get_parameter("controller.fire.visualization.ellipse_samples").as_int();
  fire_prob_vis_max_impact_points_ =
    get_parameter("controller.fire.visualization.max_impact_points").as_int();
  fire_prob_image_debug_enable_ =
    get_parameter("controller.fire.visualization.image_debug.enable").as_bool();
  fire_prob_image_debug_publish_rate_hz_ = std::max(
    get_parameter("controller.fire.visualization.image_debug.publish_rate_hz").as_double(), 0.1);
  fire_prob_image_debug_width_ = std::max(
    static_cast<int>(get_parameter("controller.fire.visualization.image_debug.width").as_int()),
    320);
  fire_prob_image_debug_height_ = std::max(
    static_cast<int>(get_parameter("controller.fire.visualization.image_debug.height").as_int()),
    240);
  fire_prob_image_debug_show_text_ =
    get_parameter("controller.fire.visualization.image_debug.show_text").as_bool();
  fire_prob_image_debug_show_sigma_ellipse_ =
    get_parameter("controller.fire.visualization.image_debug.show_sigma_ellipse").as_bool();
  fire_prob_image_debug_show_velocity_fan_ =
    get_parameter("controller.fire.visualization.image_debug.show_velocity_fan").as_bool();
  const std::string fire_target_visibility_policy =
    get_parameter("controller.fire.target_visibility_policy").as_string();

  // ── Resolve facing-filter opening angle from the visibility policy ──
  if (fire_target_visibility_policy == "facing_only") {
    if (fire_facing_filter_opening_angle_deg >= 180.0 - 1e-9) {
      fire_facing_filter_opening_angle_deg = std::clamp(2.0 * facing_exit_angle, 0.0, 180.0);
    }
  } else if (fire_target_visibility_policy == "legacy_all") {
    fire_facing_filter_opening_angle_deg = 180.0;
  } else {
    RCLCPP_WARN(get_logger(),
                "Unknown controller.fire.target_visibility_policy='%s', fallback to facing_only.",
                fire_target_visibility_policy.c_str());
    fire_facing_filter_opening_angle_deg = std::clamp(2.0 * facing_exit_angle, 0.0, 180.0);
  }

  RCLCPP_INFO(
    get_logger(),
    "[DelayUnified] pred_extra=%.4fs ctrl_latency=%.4fs trig2muzzle=%.4fs max_proc=%.4fs iters=%d",
    prediction_delay,
    controller_delay,
    trigger_to_muzzle_s,
    max_processing_delay_s,
    fire_flight_time_iters);
  RCLCPP_INFO(get_logger(),
              "[FireVisibility] policy=%s opening=%.2fdeg use_kinematics=%s vel_lpf=%s alpha=%.2f "
              "reset=%.3fs",
              fire_target_visibility_policy.c_str(),
              fire_facing_filter_opening_angle_deg,
              fire_use_gimbal_kinematics ? "true" : "false",
              fire_velocity_low_pass_enable ? "true" : "false",
              std::clamp(fire_velocity_low_pass_alpha, 0.0, 1.0),
              std::max(fire_velocity_low_pass_reset_timeout_s, 0.0));

  // ── Fire advisor: decision policy ──
  fire_advisor_->setParameters(shooting_range_w, shooting_range_h);
  if (fire_policy == "ellipse") {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::EllipseFireDecisionPolicy>());
  } else {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::AxisThresholdFireDecisionPolicy>());
  }

  // ── Fire advice engine: velocity low-pass + probability / sigma / gate ──
  if (fire_advice_engine_) {
    fire_advice_engine_->setFlightTimeIterations(fire_flight_time_iters);
    fire_advice_engine_->setFacingFilterOpeningAngleDeg(fire_facing_filter_opening_angle_deg);
    fire_advice_engine_->setUseGimbalKinematics(fire_use_gimbal_kinematics);
    gimbal_controller::FireAdviceVelocityLowPassConfig velocity_filter_cfg;
    velocity_filter_cfg.enable = fire_velocity_low_pass_enable;
    velocity_filter_cfg.alpha = fire_velocity_low_pass_alpha;
    velocity_filter_cfg.reset_timeout_s = fire_velocity_low_pass_reset_timeout_s;
    fire_advice_engine_->setVelocityLowPassConfig(velocity_filter_cfg);

    gimbal_controller::fire_advice::ProbabilityConfig prob_cfg;
    prob_cfg.enable = fire_probability_enable;
    prob_cfg.future_window_ms = std::max(fire_probability_window_ms, 0.0);
    prob_cfg.future_step_ms = std::max(fire_probability_step_ms, 1.0);
    prob_cfg.softmax_fusion = (fire_probability_window_fusion == "softmax");
    prob_cfg.softmax_beta = fire_probability_softmax_beta;
    prob_cfg.use_tracker_covariance =
      get_parameter("controller.fire.probability.use_tracker_covariance").as_bool();
    prob_cfg.strict_covariance =
      get_parameter("controller.fire.probability.strict_covariance").as_bool();
    prob_cfg.fallback_sigma_x =
      get_parameter("controller.fire.probability.fallback_sigma_x").as_double();
    prob_cfg.fallback_sigma_y =
      get_parameter("controller.fire.probability.fallback_sigma_y").as_double();
    prob_cfg.fallback_sigma_z =
      get_parameter("controller.fire.probability.fallback_sigma_z").as_double();
    prob_cfg.sigma_x0 = get_parameter("controller.fire.probability.ballistic_sigma_x0").as_double();
    prob_cfg.sigma_y0 = get_parameter("controller.fire.probability.ballistic_sigma_y0").as_double();
    prob_cfg.sigma_z0 = get_parameter("controller.fire.probability.ballistic_sigma_z0").as_double();
    prob_cfg.growth_x = get_parameter("controller.fire.probability.ballistic_growth_x").as_double();
    prob_cfg.growth_y = get_parameter("controller.fire.probability.ballistic_growth_y").as_double();
    prob_cfg.growth_z = get_parameter("controller.fire.probability.ballistic_growth_z").as_double();
    prob_cfg.enable_normal_velocity_weight =
      get_parameter("controller.fire.probability.normal_velocity_weight.enable").as_bool();
    prob_cfg.normal_v_ref =
      get_parameter("controller.fire.probability.normal_velocity_weight.v_ref").as_double();
    prob_cfg.normal_w_min =
      get_parameter("controller.fire.probability.normal_velocity_weight.w_min").as_double();
    prob_cfg.enable_normal_velocity_gate =
      get_parameter("controller.fire.probability.normal_velocity_gate.enable").as_bool();
    prob_cfg.require_front_face =
      get_parameter("controller.fire.probability.normal_velocity_gate.require_front_face")
        .as_bool();
    prob_cfg.normal_v_activate_min =
      get_parameter("controller.fire.probability.normal_velocity_gate.v_activate_min").as_double();
    prob_cfg.front_face_epsilon =
      get_parameter("controller.fire.probability.normal_velocity_gate.front_epsilon").as_double();
    prob_cfg.max_complement_angle_deg =
      get_parameter("controller.fire.probability.normal_velocity_gate.max_complement_angle_deg")
        .as_double();
    // Reuse solver hitbox size as probability hit rectangle in SI meters.
    prob_cfg.armor_width_m = std::max(shooting_range_w, 1e-6);
    prob_cfg.armor_height_m = std::max(shooting_range_h, 1e-6);

    gimbal_controller::fire_advice::SigmaPointConfig sigma_cfg;
    sigma_cfg.enable = get_parameter("controller.fire.probability.sigma_point.enable").as_bool();
    sigma_cfg.use_unscented =
      get_parameter("controller.fire.probability.sigma_point.method").as_string() == "unscented";
    sigma_cfg.sigma_v0 =
      get_parameter("controller.fire.probability.sigma_point.sigma_v0").as_double();
    sigma_cfg.sigma_delay =
      get_parameter("controller.fire.probability.sigma_point.sigma_delay").as_double();
    sigma_cfg.rho = get_parameter("controller.fire.probability.sigma_point.rho").as_double();
    sigma_cfg.alpha = get_parameter("controller.fire.probability.sigma_point.alpha").as_double();
    sigma_cfg.beta = get_parameter("controller.fire.probability.sigma_point.beta").as_double();
    sigma_cfg.kappa = get_parameter("controller.fire.probability.sigma_point.kappa").as_double();

    gimbal_controller::fire_advice::FireGateConfig gate_cfg;
    if (fire_probability_gate_strategy == "burst_evidence") {
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kBurstEvidence;
    } else if (fire_probability_gate_strategy == "legacy") {
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kLegacy;
    } else {
      RCLCPP_WARN(get_logger(),
                  "Unknown controller.fire.probability.gate.strategy='%s', fallback to legacy.",
                  fire_probability_gate_strategy.c_str());
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kLegacy;
    }
    gate_cfg.integrator_mode =
      get_parameter("controller.fire.probability.gate.mode").as_string() == "integrator";
    gate_cfg.alpha = get_parameter("controller.fire.probability.gate.alpha").as_double();
    gate_cfg.fire_on_th = get_parameter("controller.fire.probability.gate.fire_on_th").as_double();
    gate_cfg.fire_off_th =
      get_parameter("controller.fire.probability.gate.fire_off_th").as_double();
    gate_cfg.integrator_base_probability =
      get_parameter("controller.fire.probability.gate.integrator_base_probability").as_double();
    gate_cfg.integrator_rise =
      get_parameter("controller.fire.probability.gate.integrator_rise").as_double();
    gate_cfg.integrator_fall =
      get_parameter("controller.fire.probability.gate.integrator_fall").as_double();
    gate_cfg.burst_bullet_count = std::max(fire_probability_burst_count, 1);
    gate_cfg.min_hit_count = std::max(fire_probability_min_hit_count, 1);
    gate_cfg.reference_probability_p0 = fire_probability_ref_p0;
    gate_cfg.evidence_window_ms = std::max(fire_probability_evidence_window_ms, 0.0);
    gate_cfg.log_evidence_clip = fire_probability_evidence_log_clip;
    gate_cfg.evidence_epsilon = fire_probability_evidence_epsilon;
    gate_cfg.neutralize_unshootable_samples = fire_probability_neutralize_unshootable_samples;
    gate_cfg.negative_evidence_scale = fire_probability_negative_evidence_scale;
    gate_cfg.negative_clip_scale = fire_probability_negative_clip_scale;
    gate_cfg.evidence_deadband = fire_probability_evidence_deadband;
    gate_cfg.temperature = fire_probability_temperature;
    gate_cfg.theta_on_cold = fire_probability_theta_on_cold;
    gate_cfg.theta_on_hot = fire_probability_theta_on_hot;
    gate_cfg.theta_hold_cold = fire_probability_theta_hold_cold;
    gate_cfg.theta_hold_hot = fire_probability_theta_hold_hot;
    gate_cfg.theta_reset_cold = fire_probability_theta_reset_cold;
    gate_cfg.theta_reset_hot = fire_probability_theta_reset_hot;
    gate_cfg.min_fire_ms = std::max(fire_probability_min_fire_ms, 0.0);
    gate_cfg.cooldown_ms = std::max(fire_probability_cooldown_ms, 0.0);

    fire_advice_engine_->setProbabilityConfig(prob_cfg, sigma_cfg, gate_cfg);
  }

  // ── Gimbal control core fire decision config ──
  if (gimbal_control_core_) {
    gimbal_controller::FireDecisionConfig fire_cfg;
    fire_cfg.prediction_delay_s = std::max(prediction_delay, 0.0);
    fire_cfg.control_latency_s = std::max(controller_delay, 0.0);
    fire_cfg.trigger_to_muzzle_s = std::max(trigger_to_muzzle_s, 0.0);
    fire_cfg.max_processing_delay_s = std::max(max_processing_delay_s, 0.0);
    fire_cfg.include_processing_delay = true;
    fire_cfg.include_control_latency_in_target_prediction = false;
    gimbal_control_core_->setFireDecisionConfig(fire_cfg);
  }

  // ── Local ballistic compensator ──
  local_compensator_->setParameters(bullet_speed_, gravity, resistance, iteration_times);
}

void GimbalPipelineNode::configureOutputFilter() {
  // ── 配置 GimbalCmd 输出端保护滤波器 ──
  gimbal_controller::GimbalCmdFilterConfig fcfg;
  fcfg.enable_clamping = get_parameter("controller.output_filter.enable_clamping").as_bool();
  fcfg.max_yaw_diff = get_parameter("controller.output_filter.max_yaw_diff").as_double();
  fcfg.max_pitch_diff = get_parameter("controller.output_filter.max_pitch_diff").as_double();
  fcfg.enable_outlier_rejection =
    get_parameter("controller.output_filter.enable_outlier_rejection").as_bool();
  fcfg.outlier_threshold_yaw =
    get_parameter("controller.output_filter.outlier_threshold_yaw").as_double();
  fcfg.outlier_threshold_pitch =
    get_parameter("controller.output_filter.outlier_threshold_pitch").as_double();
  fcfg.max_outlier_count = get_parameter("controller.output_filter.max_outlier_count").as_int();
  fcfg.enable_rate_limiter =
    get_parameter("controller.output_filter.enable_rate_limiter").as_bool();
  fcfg.max_yaw_rate = get_parameter("controller.output_filter.max_yaw_rate").as_double();
  fcfg.max_pitch_rate = get_parameter("controller.output_filter.max_pitch_rate").as_double();
  fcfg.enable_moving_average =
    get_parameter("controller.output_filter.enable_moving_average").as_bool();
  fcfg.moving_average_window_size =
    get_parameter("controller.output_filter.moving_average_window_size").as_int();
  fcfg.enable_ema = get_parameter("controller.output_filter.enable_ema").as_bool();
  fcfg.ema_alpha = get_parameter("controller.output_filter.ema_alpha").as_double();
  fcfg.enable_one_euro = get_parameter("controller.output_filter.enable_one_euro").as_bool();
  fcfg.one_euro_freq = get_parameter("controller.output_filter.one_euro_freq").as_double();
  fcfg.one_euro_min_cutoff =
    get_parameter("controller.output_filter.one_euro_min_cutoff").as_double();
  fcfg.one_euro_beta = get_parameter("controller.output_filter.one_euro_beta").as_double();
  fcfg.one_euro_d_cutoff =
    get_parameter("controller.output_filter.one_euro_d_cutoff").as_double();

  if (gimbal_control_core_) {
    gimbal_control_core_->setFilterConfig(fcfg);
  }
  RCLCPP_INFO(
    get_logger(),
    "[GimbalCmdFilter] clamp=%s(%.1f°,%.1f°) outlier=%s(%.1f°,%.1f°,max%d)"
    " rate=%s(%.1f°,%.1f°) mean=%s(win=%d) ema=%s(a=%.2f) 1euro=%s(f=%.0f,mc=%.2f,b=%.4f)",
    fcfg.enable_clamping ? "ON" : "off",
    fcfg.max_yaw_diff,
    fcfg.max_pitch_diff,
    fcfg.enable_outlier_rejection ? "ON" : "off",
    fcfg.outlier_threshold_yaw,
    fcfg.outlier_threshold_pitch,
    fcfg.max_outlier_count,
    fcfg.enable_rate_limiter ? "ON" : "off",
    fcfg.max_yaw_rate,
    fcfg.max_pitch_rate,
    fcfg.enable_moving_average ? "ON" : "off",
    fcfg.moving_average_window_size,
    fcfg.enable_ema ? "ON" : "off",
    fcfg.ema_alpha,
    fcfg.enable_one_euro ? "ON" : "off",
    fcfg.one_euro_freq,
    fcfg.one_euro_min_cutoff,
    fcfg.one_euro_beta);
}

void GimbalPipelineNode::initRosInterfaces() {
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();

  // Subscribe: /armor_detector/armors via tf2_ros::MessageFilter
  // This mirrors armor_solver's design: the callback is only invoked once
  // the TF transform at the message's timestamp is available in tf2_buffer_,
  // guaranteeing that TFHandler::transform_pose() uses the correct
  // camera-frame → odom transform (the one that matches the image capture
  // moment) and not a stale/future transform that would cause drift.
  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_armor_filter>(armors_sub_,
                                                   *tf2_buffer_,
                                                   target_frame_,
                                                   /*queue_size=*/10,
                                                   get_node_logging_interface(),
                                                   get_node_clock_interface(),
                                                   std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&GimbalPipelineNode::armorsCallback, this);

  // Subscribe: /joint_states (input from serial driver)
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalPipelineNode::jointStateCallback, this, std::placeholders::_1));

  // Subscribe: camera_info (for FOV soft constraint)
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalPipelineNode::cameraInfoCallback, this, std::placeholders::_1));

  // Publish: cmd_gimbal (output to serial driver)
  gimbal_cmd_pub_ =
    create_publisher<rm_interfaces::msg::GimbalCmd>("cmd_gimbal", rclcpp::SensorDataQoS());

  // Maneuver states publisher (always-on, for chart monitoring)
  maneuver_states_pub_ = create_publisher<rm_interfaces::msg::ManeuverStates>(
    "~/maneuver_states", rclcpp::SensorDataQoS());

  // Debug publishers
  if (debug_mode_) {
    debug_tracked_robots_pub_ =
      create_publisher<rm_interfaces::msg::TrackedRobots>("~/tracked_robots", sensor_qos);
    debug_selected_target_pub_ = create_publisher<rm_interfaces::msg::SelectedTarget>(
      "~/selected_target", rclcpp::SensorDataQoS());
    debug_target_pub_ = create_publisher<rm_interfaces::msg::Target>("~/target", sensor_qos);
    debug_delay_audit_pub_ =
      create_publisher<rm_interfaces::msg::DelayAudit>("~/delay_audit", rclcpp::SensorDataQoS());
    debug_fire_advice_pub_ = create_publisher<rm_interfaces::msg::FireAdviceDebug>(
      "~/fire_advice_debug", rclcpp::SensorDataQoS());
    debug_armor_selection_pub_ =
      create_publisher<std_msgs::msg::String>("~/armor_selection_debug", rclcpp::SensorDataQoS());
    debug_evidence_frame_pub_ =
      create_publisher<std_msgs::msg::String>("~/evidence_frame_debug", rclcpp::SensorDataQoS());
    debug_tracker_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/tracker_markers", 10);
    debug_gimbal_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/gimbal_markers", 10);
    debug_maneuver_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/maneuver_markers", 10);
    if (fire_prob_image_debug_enable_) {
      debug_fire_plane_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/fire_debug/armor_plane", rclcpp::SensorDataQoS());
      debug_fire_normal_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/fire_debug/normal_view", rclcpp::SensorDataQoS());
    }
    if (tracker_2d_image_debug_enable_) {
      debug_tracker_2d_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "~/tracker_debug/TwoD_tracks/compressed", rclcpp::SensorDataQoS());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Subscribed to topics: /armor_detector/armors (with TF sync), /joint_states, camera_info");

  // Service: ~/set_mode
  set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
    "~/set_mode",
    std::bind(
      &GimbalPipelineNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Timer: 250 Hz control loop
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&GimbalPipelineNode::timerCallback, this));

  if (debug_mode_) initMarkers();

  RCLCPP_INFO(get_logger(), "Service ~/set_mode ready");
}

void GimbalPipelineNode::initPredictionLogger() {
  // ─── Prediction logger ────────────────────────────────────────
  if (get_parameter("logging.enable").as_bool()) {
    prediction_logger_ =
      std::make_unique<PredictionLogger>(get_parameter("logging.output_dir").as_string(),
                                         get_parameter("logging.robot_id_filter").as_string(),
                                         get_parameter("logging.flush_every_n").as_int());
    RCLCPP_INFO(get_logger(),
                "PredictionLogger enabled, output: %s",
                get_parameter("logging.output_dir").as_string().c_str());
  }

  RCLCPP_INFO(get_logger(), "PredictionLogger: %s", prediction_logger_ ? "enabled" : "disabled");
}

/* ================================================================ */
/*  Parameter declarations                                           */
/* ================================================================ */

}  // namespace fyt::auto_aim

// Register as composable node
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GimbalPipelineNode)
