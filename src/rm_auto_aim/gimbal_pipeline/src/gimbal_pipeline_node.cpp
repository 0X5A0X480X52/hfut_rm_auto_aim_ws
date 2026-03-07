// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalPipelineNode — unified node merging MaxEntropyTracker +
// TargetSelector + GimbalController. Inter-node ROS2 topics are replaced
// by direct C++ function calls to eliminate serialization / scheduling
// latency.

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"

#include <cmath>
#include <sstream>

#include "rm_utils/logger/log.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/visualization.hpp"

// Gimbal strategies
#include "gimbal_controller/strategies/current_position_strategy.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"
#include "gimbal_controller/strategies/predicted_position_strategy.hpp"
#include "gimbal_controller/strategies/state_machine_strategy.hpp"

namespace fyt::auto_aim {

/* ================================================================ */
/*  Construction                                                     */
/* ================================================================ */

GimbalPipelineNode::GimbalPipelineNode(const rclcpp::NodeOptions &options)
    : Node("gimbal_pipeline", options) {
  RCLCPP_INFO(get_logger(), "Initializing GimbalPipelineNode (unified pipeline)");

  // Register loggers used by inlined target_selector code
  try {
    FYT_REGISTER_LOGGER("target_selector", "logs/gimbal_pipeline", INFO);
  } catch (...) {
    // Logger may already be registered
  }

  // ── 1. Declare all parameters ──
  declareTrackerParameters();
  declareTargetSelectorParameters();
  declareGimbalControllerParameters();

  // ── 2. Read common / tracker params ──
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  predict_rate_ = get_parameter("predict_rate").as_double();
  debug_mode_ = get_parameter("debug_mode").as_bool();
  visualization_frame_ = get_parameter("visualization_frame").as_string();

  tracker_config_ = UnifiedConfig::create_default();
  applyTrackerParamsToConfig();

  // ── TF2 buffer — shared by TFHandler, MessageFilter, and gimbal state ──
  // Must be created before TFHandler and before initGimbalComponents().
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // TF handler (used by tracker for armor → odom transform)
  // Shares tf2_buffer_ so MessageFilter and TFHandler use the same cache.
  tf_handler_ = std::make_unique<TFHandler>(tf2_buffer_, target_frame_);

  // Tracker manager
  double dt = (predict_rate_ > 0) ? (1.0 / predict_rate_) : 0.01;
  tracker_manager_ = std::make_unique<TrackerManager>(
      tracker_config_, dt,
      get_parameter("default_r1").as_double(),
      get_parameter("default_r2").as_double(),
      get_parameter("default_dza").as_double(),
      get_parameter("tracker_timeout").as_double(),
      get_parameter("enable_oscillation_detection").as_bool());

  // ── 3. Target selector ──
  selector_strategy_name_ =
      get_parameter("selector.strategy").as_string();
  selection_config_.reference_yaw =
      get_parameter("selector.reference_yaw").as_double();
  selection_config_.max_yaw_deviation =
      get_parameter("selector.max_yaw_deviation").as_double();
  selection_config_.max_distance =
      get_parameter("selector.max_distance").as_double();
  selection_config_.min_confidence =
      get_parameter("selector.min_confidence").as_double();
  selection_config_.hysteresis_threshold =
      get_parameter("selector.hysteresis_threshold").as_double();
  initSelectionStrategy();

  // ── 4. Gimbal controller ──
  bullet_speed_ = get_parameter("controller.bullet_speed").as_double();
  control_rate_ = get_parameter("controller.control_rate").as_double();
  current_gimbal_strategy_name_ =
      get_parameter("controller.strategy").as_string();
  ballistic_mode_ = get_parameter("controller.ballistic_mode").as_string();

  // TF2 buffer was already created above (shared with TFHandler & MessageFilter).

  initGimbalComponents();
  initGimbalStrategies();

  // Configure solver parameters
  double shooting_range_w = get_parameter("controller.solver.shooting_range_width").as_double();
  double shooting_range_h = get_parameter("controller.solver.shooting_range_height").as_double();
  double side_angle = get_parameter("controller.solver.side_angle").as_double();
  double min_switching_v_yaw = get_parameter("controller.solver.min_switching_v_yaw").as_double();
  double prediction_delay = get_parameter("controller.solver.prediction_delay").as_double();
  double max_prediction_time = get_parameter("controller.solver.max_prediction_time").as_double();
  double max_tracking_v_yaw = get_parameter("controller.solver.max_tracking_v_yaw").as_double();
  int transfer_thresh = get_parameter("controller.solver.transfer_thresh").as_int();
  double gravity = get_parameter("controller.solver.gravity").as_double();
  double resistance = get_parameter("controller.solver.resistance").as_double();
  int iteration_times = get_parameter("controller.solver.iteration_times").as_int();
  double pitch_offset = get_parameter("controller.solver.pitch_offset").as_double();
  double yaw_offset = get_parameter("controller.solver.yaw_offset").as_double();
  double facing_enter_angle = get_parameter("controller.solver.facing_enter_angle").as_double();
  double facing_exit_angle = get_parameter("controller.solver.facing_exit_angle").as_double();
  double controller_delay = get_parameter("controller.solver.controller_delay").as_double();
  std::string selection_method_str = get_parameter("controller.solver.selection_method").as_string();

  armor_selector_->setParameters(side_angle, min_switching_v_yaw);
  armor_selector_->setFacingParameters(facing_enter_angle, facing_exit_angle);

  // 配置选板策略
  gimbal_controller::ArmorSelector::SelectionMethod sel_method =
    gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_FACING;
  if (selection_method_str == "min_movement") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT;
  } else if (selection_method_str == "decision_angle") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::DECISION_ANGLE;
  }
  armor_selector_->setSelectionMethod(sel_method);
  RCLCPP_INFO(get_logger(), "[GimbalController] selection_method: %s", selection_method_str.c_str());

  fire_advisor_->setParameters(shooting_range_w, shooting_range_h);
  local_compensator_->setParameters(bullet_speed_, gravity, resistance,
                                    iteration_times);

  // Strategy-specific configuration
  auto predicted_strategy = std::dynamic_pointer_cast<
      gimbal_controller::PredictedPositionStrategy>(
      gimbal_strategies_["predicted"]);
  if (predicted_strategy) {
    predicted_strategy->setPredictionParameters(prediction_delay, max_prediction_time);
    predicted_strategy->setManualOffset(pitch_offset, yaw_offset);
    predicted_strategy->setTrackingCenterParams(max_tracking_v_yaw, transfer_thresh);
    predicted_strategy->setControllerDelay(controller_delay);
  }
  auto current_strategy = std::dynamic_pointer_cast<
      gimbal_controller::CurrentPositionStrategy>(
      gimbal_strategies_["current"]);
  if (current_strategy) {
    current_strategy->setManualOffset(pitch_offset, yaw_offset);
    current_strategy->setControllerDelay(controller_delay);
  }

  // Configure adaptive controller_delay (AIMD)
  bool   adaptive_enable   = get_parameter("controller.solver.adaptive_delay.enable").as_bool();
  int    adaptive_threshold = get_parameter("controller.solver.adaptive_delay.fire_wait_threshold").as_int();
  double adaptive_mul      = get_parameter("controller.solver.adaptive_delay.mul_factor").as_double();
  double adaptive_step     = get_parameter("controller.solver.adaptive_delay.add_step").as_double();
  double adaptive_max      = get_parameter("controller.solver.adaptive_delay.max_delay").as_double();
  double adaptive_min      = get_parameter("controller.solver.adaptive_delay.min_delay").as_double();
  double adaptive_max_lin  = get_parameter("controller.solver.adaptive_delay.max_linear_speed").as_double();
  double adaptive_max_ang  = get_parameter("controller.solver.adaptive_delay.max_angular_speed").as_double();

  RCLCPP_INFO(get_logger(),
    "[GimbalController] adaptive_delay: %s (init=%.4f s, min=%.4f, max=%.4f, "
    "add_step=%.4f, mul=%.2f, thresh=%d, max_lin=%.1f, max_ang=%.1f)",
    adaptive_enable ? "ENABLED" : "disabled",
    controller_delay, adaptive_min, adaptive_max,
    adaptive_step, adaptive_mul, adaptive_threshold,
    adaptive_max_lin, adaptive_max_ang);

  if (predicted_strategy) {
    predicted_strategy->setAdaptiveDelayParams(
      adaptive_enable, controller_delay,
      adaptive_min, adaptive_max,
      adaptive_step, adaptive_mul, adaptive_threshold,
      adaptive_max_lin, adaptive_max_ang);
  }
  if (current_strategy) {
    current_strategy->setAdaptiveDelayParams(
      adaptive_enable, controller_delay,
      adaptive_min, adaptive_max,
      adaptive_step, adaptive_mul, adaptive_threshold,
      adaptive_max_lin, adaptive_max_ang);
  }

  double sm_facing_enter = get_parameter("controller.state_machine.facing_enter_angle").as_double();
  double sm_facing_exit = get_parameter("controller.state_machine.facing_exit_angle").as_double();
  double sm_spin_thresh = get_parameter("controller.state_machine.spin_v_yaw_thresh").as_double();
  double sm_calm_thresh = get_parameter("controller.state_machine.calm_v_yaw_thresh").as_double();
  int sm_spin_enter = get_parameter("controller.state_machine.spin_enter_count").as_int();
  int sm_spin_exit = get_parameter("controller.state_machine.spin_exit_count").as_int();
  double sm_side_angle = get_parameter("controller.state_machine.side_angle").as_double();
  double sm_prediction_delay = get_parameter("controller.state_machine.prediction_delay").as_double();
  double sm_max_prediction = get_parameter("controller.state_machine.max_prediction_time").as_double();

  auto sm_strategy_ptr = std::dynamic_pointer_cast<
      gimbal_controller::StateMachineStrategy>(
      gimbal_strategies_["state_machine"]);
  if (sm_strategy_ptr) {
    sm_strategy_ptr->setFacingParameters(sm_facing_enter, sm_facing_exit);
    sm_strategy_ptr->setSpinParameters(sm_spin_thresh, sm_calm_thresh,
                                       sm_spin_enter, sm_spin_exit,
                                       sm_side_angle);
    sm_strategy_ptr->setPredictionParameters(sm_prediction_delay,
                                             sm_max_prediction);
    sm_strategy_ptr->setManualOffset(pitch_offset, yaw_offset);
  }

  // ── 配置 GimbalCmd 输出端保护滤波器 ──
  {
    gimbal_controller::GimbalCmdFilterConfig fcfg;
    fcfg.enable_clamping             = get_parameter("controller.output_filter.enable_clamping").as_bool();
    fcfg.max_yaw_diff                = get_parameter("controller.output_filter.max_yaw_diff").as_double();
    fcfg.max_pitch_diff              = get_parameter("controller.output_filter.max_pitch_diff").as_double();
    fcfg.enable_outlier_rejection    = get_parameter("controller.output_filter.enable_outlier_rejection").as_bool();
    fcfg.outlier_threshold_yaw       = get_parameter("controller.output_filter.outlier_threshold_yaw").as_double();
    fcfg.outlier_threshold_pitch     = get_parameter("controller.output_filter.outlier_threshold_pitch").as_double();
    fcfg.max_outlier_count           = get_parameter("controller.output_filter.max_outlier_count").as_int();
    fcfg.enable_rate_limiter         = get_parameter("controller.output_filter.enable_rate_limiter").as_bool();
    fcfg.max_yaw_rate                = get_parameter("controller.output_filter.max_yaw_rate").as_double();
    fcfg.max_pitch_rate              = get_parameter("controller.output_filter.max_pitch_rate").as_double();
    fcfg.enable_ema                  = get_parameter("controller.output_filter.enable_ema").as_bool();
    fcfg.ema_alpha                   = get_parameter("controller.output_filter.ema_alpha").as_double();
    fcfg.enable_one_euro             = get_parameter("controller.output_filter.enable_one_euro").as_bool();
    fcfg.one_euro_freq               = get_parameter("controller.output_filter.one_euro_freq").as_double();
    fcfg.one_euro_min_cutoff         = get_parameter("controller.output_filter.one_euro_min_cutoff").as_double();
    fcfg.one_euro_beta               = get_parameter("controller.output_filter.one_euro_beta").as_double();
    fcfg.one_euro_d_cutoff           = get_parameter("controller.output_filter.one_euro_d_cutoff").as_double();
    cmd_filter_.setConfig(fcfg);
    RCLCPP_INFO(get_logger(),
      "[GimbalCmdFilter] clamp=%s(%.1f°,%.1f°) outlier=%s(%.1f°,%.1f°,max%d)"
      " rate=%s(%.1f°,%.1f°) ema=%s(a=%.2f) 1euro=%s(f=%.0f,mc=%.2f,b=%.4f)",
      fcfg.enable_clamping ? "ON" : "off", fcfg.max_yaw_diff, fcfg.max_pitch_diff,
      fcfg.enable_outlier_rejection ? "ON" : "off",
      fcfg.outlier_threshold_yaw, fcfg.outlier_threshold_pitch, fcfg.max_outlier_count,
      fcfg.enable_rate_limiter ? "ON" : "off", fcfg.max_yaw_rate, fcfg.max_pitch_rate,
      fcfg.enable_ema ? "ON" : "off", fcfg.ema_alpha,
      fcfg.enable_one_euro ? "ON" : "off",
      fcfg.one_euro_freq, fcfg.one_euro_min_cutoff, fcfg.one_euro_beta);
  }

  // ── 5. ROS2 external interfaces ──
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();

  // Subscribe: /armor_detector/armors via tf2_ros::MessageFilter
  // This mirrors armor_solver's design: the callback is only invoked once
  // the TF transform at the message's timestamp is available in tf2_buffer_,
  // guaranteeing that TFHandler::transform_pose() uses the correct
  // camera-frame → odom transform (the one that matches the image capture
  // moment) and not a stale/future transform that would cause drift.
  armors_sub_.subscribe(this, "/armor_detector/armors",
                         rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_armor_filter>(
      armors_sub_, *tf2_buffer_, target_frame_,
      /*queue_size=*/10,
      get_node_logging_interface(), get_node_clock_interface(),
      std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&GimbalPipelineNode::armorsCallback, this);

  // Subscribe: /joint_states (input from serial driver)
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&GimbalPipelineNode::jointStateCallback, this,
                std::placeholders::_1));

  // Publish: cmd_gimbal (output to serial driver)
  gimbal_cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>(
      "cmd_gimbal", rclcpp::SensorDataQoS());

  // Debug publishers
  if (debug_mode_) {
    debug_tracked_robots_pub_ =
        create_publisher<rm_interfaces::msg::TrackedRobots>(
            "~/tracked_robots", sensor_qos);
    debug_selected_target_pub_ =
        create_publisher<rm_interfaces::msg::SelectedTarget>(
            "~/selected_target", rclcpp::SensorDataQoS());
    debug_target_pub_ = create_publisher<rm_interfaces::msg::Target>(
        "~/target", sensor_qos);
    debug_tracker_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/tracker_markers", 10);
    debug_gimbal_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/gimbal_markers", 10);
  }

  // Service: ~/set_mode
  set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
      "~/set_mode",
      std::bind(&GimbalPipelineNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Timer: 250 Hz control loop
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GimbalPipelineNode::timerCallback, this));

  if (debug_mode_) initMarkers();

  // ─── Prediction logger ────────────────────────────────────────
  if (get_parameter("logging.enable").as_bool()) {
    prediction_logger_ = std::make_unique<PredictionLogger>(
        get_parameter("logging.output_dir").as_string(),
        get_parameter("logging.robot_id_filter").as_string(),
        get_parameter("logging.flush_every_n").as_int());
    RCLCPP_INFO(get_logger(), "PredictionLogger enabled, output: %s",
                get_parameter("logging.output_dir").as_string().c_str());
  }

  RCLCPP_INFO(get_logger(),
              "GimbalPipelineNode initialized: target_frame=%s, "
              "control_rate=%.0f Hz, strategy=%s, ballistic=%s",
              target_frame_.c_str(), control_rate_,
              current_gimbal_strategy_name_.c_str(),
              ballistic_mode_.c_str());
}

/* ================================================================ */
/*  Parameter declarations                                           */
/* ================================================================ */

void GimbalPipelineNode::declareTrackerParameters() {
  // Basic
  declare_parameter("target_frame", "odom");
  declare_parameter("source_frame", "camera_optical_frame");
  declare_parameter("predict_rate", 100.0);
  declare_parameter("default_r1", 0.15);
  declare_parameter("default_r2", 0.20);
  declare_parameter("default_dza", 0.0);
  declare_parameter("tracker_timeout", 3.0);
  declare_parameter("debug_mode", false);
  declare_parameter("enable_oscillation_detection", false);
  declare_parameter("visualization_frame", "odom");

  // UKF
  declare_parameter("ukf.alpha", 0.001);
  declare_parameter("ukf.beta", 2.0);
  declare_parameter("ukf.kappa", 0.0);
  declare_parameter("ukf.obs_noise_pos", 0.05);
  declare_parameter("ukf.obs_noise_yaw", 0.05);
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

  // Constraints
  declare_parameter("constraints.min_radius", 0.12);
  declare_parameter("constraints.max_radius", 0.5);
  declare_parameter("constraints.min_dz", -1.0);
  declare_parameter("constraints.max_dz", 1.0);

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
}

void GimbalPipelineNode::declareTargetSelectorParameters() {
  declare_parameter("selector.strategy", "min_yaw_deviation");
  declare_parameter("selector.reference_yaw", 0.0);
  declare_parameter("selector.max_yaw_deviation", M_PI);
  declare_parameter("selector.max_distance", 10.0);
  declare_parameter("selector.min_confidence", 0.3);
  declare_parameter("selector.hysteresis_threshold", 0.1);
}

void GimbalPipelineNode::declareGimbalControllerParameters() {
  declare_parameter("controller.bullet_speed", 20.0);
  declare_parameter("controller.control_rate", 250.0);
  declare_parameter("controller.strategy", "current");
  declare_parameter("controller.ballistic_mode", "service");

  // Solver
  declare_parameter("controller.solver.shooting_range_width", 0.135);
  declare_parameter("controller.solver.shooting_range_height", 0.135);
  declare_parameter("controller.solver.side_angle", 15.0);
  declare_parameter("controller.solver.min_switching_v_yaw", 1.0);
  declare_parameter("controller.solver.prediction_delay", 0.0);
  declare_parameter("controller.solver.max_prediction_time", 0.5);
  declare_parameter("controller.solver.max_tracking_v_yaw", 6.0);
  declare_parameter("controller.solver.transfer_thresh", 5);
  declare_parameter("controller.solver.gravity", 9.8);
  declare_parameter("controller.solver.resistance", 0.001);
  declare_parameter("controller.solver.iteration_times", 20);
  declare_parameter("controller.solver.pitch_offset", 0.0);
  declare_parameter("controller.solver.yaw_offset", 0.0);
  declare_parameter("controller.solver.facing_enter_angle", 40.0);
  declare_parameter("controller.solver.facing_exit_angle", 55.0);
  declare_parameter("controller.solver.controller_delay", 0.0);
  declare_parameter("controller.solver.selection_method", std::string("min_movement_with_facing"));

  // Adaptive controller_delay (AIMD)
  declare_parameter("controller.solver.adaptive_delay.enable",              false);
  declare_parameter("controller.solver.adaptive_delay.fire_wait_threshold", 10);
  declare_parameter("controller.solver.adaptive_delay.mul_factor",          1.2);
  declare_parameter("controller.solver.adaptive_delay.add_step",            0.005);
  declare_parameter("controller.solver.adaptive_delay.max_delay",           0.10);
  declare_parameter("controller.solver.adaptive_delay.min_delay",           0.0);
  declare_parameter("controller.solver.adaptive_delay.max_linear_speed",    3.0);
  declare_parameter("controller.solver.adaptive_delay.max_angular_speed",   10.0);

  // State machine
  declare_parameter("controller.state_machine.facing_enter_angle", 40.0);
  declare_parameter("controller.state_machine.facing_exit_angle", 55.0);
  declare_parameter("controller.state_machine.spin_v_yaw_thresh", 4.0);
  declare_parameter("controller.state_machine.calm_v_yaw_thresh", 2.0);
  declare_parameter("controller.state_machine.spin_enter_count", 5);
  declare_parameter("controller.state_machine.spin_exit_count", 5);
  declare_parameter("controller.state_machine.side_angle", 15.0);
  declare_parameter("controller.state_machine.prediction_delay", 0.0);
  declare_parameter("controller.state_machine.max_prediction_time", 0.5);

  // ─── GimbalCmd 输出端保护滤波器 ──────────────────────────────
  // 0. Clamping — 绝对限幅
  declare_parameter("controller.output_filter.enable_clamping",         true);
  declare_parameter("controller.output_filter.max_yaw_diff",            15.0);
  declare_parameter("controller.output_filter.max_pitch_diff",          10.0);
  // 1. 外点检测
  declare_parameter("controller.output_filter.enable_outlier_rejection", true);
  declare_parameter("controller.output_filter.outlier_threshold_yaw",   8.0);
  declare_parameter("controller.output_filter.outlier_threshold_pitch",  5.0);
  declare_parameter("controller.output_filter.max_outlier_count",        3);
  // 2. Rate Limiter
  declare_parameter("controller.output_filter.enable_rate_limiter",     true);
  declare_parameter("controller.output_filter.max_yaw_rate",            5.0);
  declare_parameter("controller.output_filter.max_pitch_rate",          3.0);
  // 3. EMA
  declare_parameter("controller.output_filter.enable_ema",              false);
  declare_parameter("controller.output_filter.ema_alpha",               0.7);
  // 4. 1-Euro 自适应滤波
  declare_parameter("controller.output_filter.enable_one_euro",         false);
  declare_parameter("controller.output_filter.one_euro_freq",           250.0);
  declare_parameter("controller.output_filter.one_euro_min_cutoff",     1.0);
  declare_parameter("controller.output_filter.one_euro_beta",           0.007);
  declare_parameter("controller.output_filter.one_euro_d_cutoff",       1.0);

  // ─── Prediction logger ────────────────────────────────────────
  declare_parameter("logging.enable",          false);
  declare_parameter("logging.output_dir",       std::string("/tmp/prediction_logs"));
  declare_parameter("logging.robot_id_filter",  std::string(""));
  declare_parameter("logging.flush_every_n",    50);
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
  c.ukf.dual_obs_noise_pos = get_parameter("ukf.dual_obs_noise_pos").as_double();
  c.ukf.dual_obs_noise_yaw = get_parameter("ukf.dual_obs_noise_yaw").as_double();
  c.ukf.dual_obs_geometry_noise_scale =
      get_parameter("ukf.dual_obs_geometry_noise_scale").as_double();
  c.ukf.single_obs_update_weight_pos =
      get_parameter("ukf.single_obs_update_weight_pos").as_double();
  c.ukf.enable_innovation_gating =
      get_parameter("ukf.enable_innovation_gating").as_bool();
  c.ukf.innovation_gate_chi2_threshold =
      get_parameter("ukf.innovation_gate_chi2_threshold").as_double();

  auto tm_str = get_parameter("motion.translation_model").as_string();
  c.motion.translation_model = translation_model_from_string(tm_str);
  c.motion.cv_process_noise_vel =
      get_parameter("motion.cv_process_noise_vel").as_double();
  c.motion.ca_process_noise_acc =
      get_parameter("motion.ca_process_noise_acc").as_double();
  c.motion.singer_alpha = get_parameter("motion.singer_alpha").as_double();
  c.motion.singer_sigma = get_parameter("motion.singer_sigma").as_double();
  c.motion.process_noise_r = get_parameter("motion.process_noise_r").as_double();
  c.motion.process_noise_dz =
      get_parameter("motion.process_noise_dz").as_double();

  c.spin.spin_process_noise_yaw_rate =
      get_parameter("spin.spin_process_noise_yaw_rate").as_double();
  c.spin.spin_process_noise_yaw_acc =
      get_parameter("spin.spin_process_noise_yaw_acc").as_double();
  c.spin.spin_process_noise_delta_rate =
      get_parameter("spin.spin_process_noise_delta_rate").as_double();
  c.spin.spin_process_noise_delta_acc =
      get_parameter("spin.spin_process_noise_delta_acc").as_double();

  c.entropy.temperature = get_parameter("entropy.temperature").as_double();
  c.entropy.use_adaptive = get_parameter("entropy.use_adaptive").as_bool();
  c.entropy.k_prior_weight =
      get_parameter("entropy.k_prior_weight").as_double();

  c.tracker.tracking_thres = get_parameter("tracker.tracking_thres").as_int();
  c.tracker.lost_thres = get_parameter("tracker.lost_thres").as_int();
  c.tracker.temp_lost_thres =
      get_parameter("tracker.temp_lost_thres").as_int();
  c.tracker.max_match_distance =
      get_parameter("tracker.max_match_distance").as_double();
  c.tracker.max_match_yaw_diff =
      get_parameter("tracker.max_match_yaw_diff").as_double();
  c.tracker.n_panels = get_parameter("tracker.n_panels").as_int();
  c.tracker.panel_angle_step =
      get_parameter("tracker.panel_angle_step").as_double();

  c.constraints.min_radius =
      get_parameter("constraints.min_radius").as_double();
  c.constraints.max_radius =
      get_parameter("constraints.max_radius").as_double();
  c.constraints.min_dz = get_parameter("constraints.min_dz").as_double();
  c.constraints.max_dz = get_parameter("constraints.max_dz").as_double();

  // Output smoother
  smoother_config_.enable = get_parameter("smoother.enable").as_bool();
  smoother_config_.enable_position_smooth =
      get_parameter("smoother.enable_position_smooth").as_bool();
  smoother_config_.enable_yaw_smooth =
      get_parameter("smoother.enable_yaw_smooth").as_bool();
  smoother_config_.enable_velocity_smooth =
      get_parameter("smoother.enable_velocity_smooth").as_bool();
  smoother_config_.enable_structural_convergence =
      get_parameter("smoother.enable_structural_convergence").as_bool();
  smoother_config_.pos_min_cutoff =
      get_parameter("smoother.pos_min_cutoff").as_double();
  smoother_config_.pos_beta = get_parameter("smoother.pos_beta").as_double();
  smoother_config_.pos_d_cutoff =
      get_parameter("smoother.pos_d_cutoff").as_double();
  smoother_config_.yaw_min_cutoff =
      get_parameter("smoother.yaw_min_cutoff").as_double();
  smoother_config_.yaw_beta = get_parameter("smoother.yaw_beta").as_double();
  smoother_config_.yaw_d_cutoff =
      get_parameter("smoother.yaw_d_cutoff").as_double();
  smoother_config_.vel_min_cutoff =
      get_parameter("smoother.vel_min_cutoff").as_double();
  smoother_config_.vel_beta = get_parameter("smoother.vel_beta").as_double();
  smoother_config_.vel_d_cutoff =
      get_parameter("smoother.vel_d_cutoff").as_double();
  smoother_config_.rm_initial_step =
      get_parameter("smoother.rm_initial_step").as_double();
  smoother_config_.rm_gamma = get_parameter("smoother.rm_gamma").as_double();
  smoother_config_.rm_n0 = get_parameter("smoother.rm_n0").as_int();
  smoother_config_.rm_dual_obs_boost =
      get_parameter("smoother.rm_dual_obs_boost").as_double();
  smoother_config_.rm_min_radius =
      get_parameter("smoother.rm_min_radius").as_double();
  smoother_config_.rm_max_radius =
      get_parameter("smoother.rm_max_radius").as_double();
  smoother_config_.rm_min_dz =
      get_parameter("smoother.rm_min_dz").as_double();
  smoother_config_.rm_max_dz =
      get_parameter("smoother.rm_max_dz").as_double();
  smoother_config_.rm_convergence_eps =
      get_parameter("smoother.rm_convergence_eps").as_double();
  smoother_config_.default_freq =
      get_parameter("smoother.default_freq").as_double();

  RCLCPP_INFO(get_logger(), "Tracker parameters applied (smoother %s)",
              smoother_config_.enable ? "ON" : "OFF");
}

/* ================================================================ */
/*  Armors callback — tracker + selector pipeline                    */
/* ================================================================ */

void GimbalPipelineNode::armorsCallback(
    const rm_interfaces::msg::Armors::SharedPtr msg) {
  if (msg->armors.empty()) return;

  rclcpp::Time msg_time(msg->header.stamp);
  double current_time = msg_time.seconds();

  // ── Step 1: Predict all existing trackers ──
  tracker_manager_->predict_all(current_time);
  auto removed = tracker_manager_->remove_stale(current_time);
  if (!removed.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu stale trackers", removed.size());
  }

  // ── Step 2: Group observations by robot ID ──
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  std::string sf =
      msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;

  for (const auto &armor : msg->armors) {
    auto obs =
        tf_handler_->transform_armor_to_observation(armor, sf, msg_time);
    if (!obs.has_value()) {
      if (debug_mode_)
        RCLCPP_WARN(get_logger(), "TF failed for armor %s",
                    armor.number.c_str());
      continue;
    }
    obs_by_robot[armor.number].push_back(obs.value());
  }

  // ── Log observations (before update, so we record incoming sensor data) ──
  if (prediction_logger_) {
    int64_t ts_ns = msg_time.nanoseconds();
    for (const auto &[rid, obs_list] : obs_by_robot) {
      bool is_dual = (obs_list.size() >= 2);
      std::vector<LogObservation> log_obs;
      log_obs.reserve(obs_list.size());
      for (const auto &o : obs_list) {
        LogObservation lo;
        lo.x          = o.x;
        lo.y          = o.y;
        lo.z          = o.z;
        lo.yaw        = o.yaw;
        lo.panel_id   = o.panel_id.value_or(-1);
        lo.confidence = o.confidence;
        lo.is_dual_obs = is_dual;
        log_obs.push_back(lo);
      }
      prediction_logger_->logObservations(ts_ns, rid, log_obs);
    }
  }

  // ── Step 3: Update trackers ──
  for (auto &[rid, obs_list] : obs_by_robot) {
    last_obs_counts_[rid] = static_cast<int>(obs_list.size());
    last_dual_obs_[rid] = (obs_list.size() >= 2);
    bool is_ok = tracker_manager_->update(rid, obs_list, current_time);

    if (is_ok) {
      auto *t = tracker_manager_->get(rid);
      if (t && t->is_initialized() &&
          smoothers_.find(rid) == smoothers_.end()) {
        OutputSmoother sm(smoother_config_);
        auto [r1, r2] = t->get_radii();
        double dza = t->get_dza();
        sm.initialize(r1, r2, dza);
        smoothers_.emplace(rid, std::move(sm));
      }
    }
  }

  // Clean up smoothers for removed trackers
  for (const auto &rid : removed) {
    smoothers_.erase(rid);
    last_dual_obs_.erase(rid);
  }

  // ── Step 4: Build TrackedRobots message (internal) ──
  auto tracked_msg = buildTrackedRobotsMsg(msg->header);

  // ── Log tracker posterior states (after update, before selection) ──
  if (prediction_logger_) {
    int64_t ts_ns = msg_time.nanoseconds();
    for (const auto &robot : tracked_msg.robots) {
      LogTrackerState st;
      st.center_x           = robot.center_position.x;
      st.center_y           = robot.center_position.y;
      st.center_z           = robot.center_position.z;
      st.vel_x              = robot.center_velocity.x;
      st.vel_y              = robot.center_velocity.y;
      st.vel_z              = robot.center_velocity.z;
      st.yaw                = robot.yaw;
      st.yaw_velocity       = robot.yaw_velocity;
      st.yaw_acceleration   = robot.yaw_acceleration;
      st.radius_1           = robot.radius;
      st.radius_2           = robot.radius_2;
      st.dza                = robot.d_za;
      st.track_state        = robot.track_state;
      st.num_armors         = robot.num_armors;
      st.visible_armor_count = robot.visible_armor_count;
      st.is_visible         = robot.is_visible;
      st.confidence         = robot.confidence;
      prediction_logger_->logTrackerState(ts_ns, robot.robot_id, st);
    }
  }

  // ── Step 5: Target selection (direct C++ call, no ROS topic!) ──
  SelectionResult sel_result;
  if (!tracked_msg.robots.empty()) {
    sel_result = selectTargetInternal(tracked_msg);
  }

  // ── Step 6: Store results for timerCallback (thread-safe) ──
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    latest_tracked_robots_ =
        std::make_shared<rm_interfaces::msg::TrackedRobots>(tracked_msg);
    latest_selected_target_id_ = sel_result.robot_id;
    latest_selected_confidence_ = sel_result.confidence;
    latest_update_time_ = now();  // record local clock for processing_delay
  }

  // ── Step 7: Debug publishing ──
  if (debug_mode_) {
    if (debug_tracked_robots_pub_ && !tracked_msg.robots.empty())
      debug_tracked_robots_pub_->publish(tracked_msg);

    if (debug_selected_target_pub_) {
      rm_interfaces::msg::SelectedTarget sel_msg;
      sel_msg.header.stamp = now();
      sel_msg.header.frame_id = target_frame_;
      sel_msg.robot_id = sel_result.robot_id;
      sel_msg.confidence = sel_result.confidence;
      sel_msg.selection_strategy = selector_strategy_name_;
      debug_selected_target_pub_->publish(sel_msg);
    }

    if (debug_tracker_marker_pub_) {
      rclcpp::Time stamp(msg->header.stamp);
      auto marker_array = build_tracker_markers(
          visualization_frame_, tracker_manager_->trackers(), stamp);
      debug_tracker_marker_pub_->publish(marker_array);
    }
  }
}

/* ================================================================ */
/*  Build TrackedRobots message (from tracker state)                 */
/* ================================================================ */

rm_interfaces::msg::TrackedRobots GimbalPipelineNode::buildTrackedRobotsMsg(
    const std_msgs::msg::Header &header) {
  rm_interfaces::msg::TrackedRobots tracked_msg;
  tracked_msg.header = header;
  tracked_msg.header.frame_id = target_frame_;

  auto tracking_ids = tracker_manager_->tracking_robot_ids();
  for (const auto &rid : tracking_ids) {
    auto *tracker = tracker_manager_->get(rid);
    if (!tracker || !tracker->is_tracking()) continue;

    // Apply output smoothing
    SmoothedOutput smoothed;
    bool has_smoothed = false;
    auto sm_it = smoothers_.find(rid);
    if (sm_it != smoothers_.end() && smoother_config_.enable) {
      auto pos = tracker->get_center_position();
      auto idx = tracker->ukf().state_idx();
      const auto &x = tracker->ukf().x();
      Eigen::Vector3d vel(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
      double yaw = tracker->get_yaw();
      double v_yaw = x(idx.DELTA_RATE());
      auto [r1, r2] = tracker->get_radii();
      double dza = tracker->get_dza();

      bool is_dual = false;
      auto dual_it = last_dual_obs_.find(rid);
      if (dual_it != last_dual_obs_.end()) is_dual = dual_it->second;

      rclcpp::Time stamp(header.stamp);
      double ts = stamp.seconds();

      smoothed = sm_it->second.smooth(pos, yaw, vel, v_yaw, r1, r2, dza,
                                       is_dual, ts);
      has_smoothed = true;
    }

    // Publish target for debug
    if (debug_mode_ && debug_target_pub_) {
      auto target = has_smoothed
          ? buildTargetMessage(header, rid, *tracker, &smoothed)
          : buildTargetMessage(header, rid, *tracker, nullptr);
      debug_target_pub_->publish(target);
    }

    auto robot = has_smoothed
        ? buildTrackedRobotMessage(header, rid, *tracker, &smoothed)
        : buildTrackedRobotMessage(header, rid, *tracker, nullptr);
    tracked_msg.robots.push_back(robot);
  }

  return tracked_msg;
}

/* ================================================================ */
/*  Target message builders (from MaxEntropyTrackerNode)             */
/* ================================================================ */

rm_interfaces::msg::Target GimbalPipelineNode::buildTargetMessage(
    const std_msgs::msg::Header &header, const std::string &robot_id,
    AdaptiveArmorTracker &tracker, const SmoothedOutput *smoothed) {
  rm_interfaces::msg::Target target;
  target.header = header;
  target.header.frame_id = target_frame_;
  target.tracking = true;
  target.id = robot_id;
  target.armors_num = 4;

  if (smoothed) {
    target.position.x = smoothed->center_position.x();
    target.position.y = smoothed->center_position.y();
    target.position.z = smoothed->center_position.z();
    target.velocity.x = smoothed->velocity.x();
    target.velocity.y = smoothed->velocity.y();
    target.velocity.z = smoothed->velocity.z();
    target.yaw = smoothed->yaw;
    target.v_yaw = smoothed->yaw_velocity;
    target.radius_1 = smoothed->r1;
    target.radius_2 = smoothed->r2;
    target.d_za = smoothed->dza;
  } else {
    auto pos = tracker.get_center_position();
    target.position.x = pos.x();
    target.position.y = pos.y();
    target.position.z = pos.z();
    auto idx = tracker.ukf().state_idx();
    const auto &x = tracker.ukf().x();
    target.velocity.x = x(idx.VX());
    target.velocity.y = x(idx.VY());
    target.velocity.z = x(idx.VZ());
    target.yaw = tracker.get_yaw();
    target.v_yaw = x(idx.DELTA_RATE());
    auto [r1, r2] = tracker.get_radii();
    target.radius_1 = r1;
    target.radius_2 = r2;
    target.d_za = tracker.get_dza();
  }

  target.d_zc = 0.0;
  target.yaw_diff = 0.0;
  target.position_diff = 0.0;
  return target;
}

rm_interfaces::msg::TrackedRobot GimbalPipelineNode::buildTrackedRobotMessage(
    const std_msgs::msg::Header &header, const std::string &robot_id,
    AdaptiveArmorTracker &tracker, const SmoothedOutput *smoothed) {
  rm_interfaces::msg::TrackedRobot msg;
  msg.header = header;
  msg.header.frame_id = target_frame_;
  msg.robot_id = robot_id;
  msg.robot_type = static_cast<uint8_t>(inferRobotType(robot_id));

  if (tracker.is_tracking())
    msg.track_state = rm_interfaces::msg::TrackedRobot::TRACKING;
  else if (tracker.is_temp_lost())
    msg.track_state = rm_interfaces::msg::TrackedRobot::TEMP_LOST;
  else
    msg.track_state = rm_interfaces::msg::TrackedRobot::DETECTING;

  auto idx = tracker.ukf().state_idx();
  const auto &x = tracker.ukf().x();

  if (smoothed) {
    msg.center_position.x = smoothed->center_position.x();
    msg.center_position.y = smoothed->center_position.y();
    msg.center_position.z = smoothed->center_position.z();
    msg.center_velocity.x = smoothed->velocity.x();
    msg.center_velocity.y = smoothed->velocity.y();
    msg.center_velocity.z = smoothed->velocity.z();
    msg.yaw = smoothed->yaw;
    msg.yaw_velocity = smoothed->yaw_velocity;
    msg.radius = smoothed->r1;
    msg.radius_2 = smoothed->r2;
    msg.d_za = smoothed->dza;
  } else {
    auto pos = tracker.get_center_position();
    msg.center_position.x = pos.x();
    msg.center_position.y = pos.y();
    msg.center_position.z = pos.z();
    msg.center_velocity.x = x(idx.VX());
    msg.center_velocity.y = x(idx.VY());
    msg.center_velocity.z = x(idx.VZ());
    msg.yaw = tracker.get_yaw();
    msg.yaw_velocity = x(idx.DELTA_RATE());
    auto [r1, r2] = tracker.get_radii();
    msg.radius = r1;
    msg.radius_2 = r2;
    msg.d_za = tracker.get_dza();
  }

  if (idx.has("AX")) {
    msg.center_acceleration.x = x(idx.AX());
    msg.center_acceleration.y = x(idx.AY());
    msg.center_acceleration.z = x(idx.AZ());
  } else {
    msg.center_acceleration.x = 0.0;
    msg.center_acceleration.y = 0.0;
    msg.center_acceleration.z = 0.0;
  }

  msg.yaw_acceleration =
      idx.has("DELTA_ACC") ? x(idx.get("DELTA_ACC")) : 0.0;
  msg.d_zc = 0.0;
  msg.num_armors = inferNumArmors(robot_id, msg.robot_type);

  double off_r1 = smoothed ? smoothed->r1 : msg.radius;
  double off_r2 = smoothed ? smoothed->r2 : msg.radius_2;
  double off_dza = smoothed ? smoothed->dza : msg.d_za;
  msg.armors_offset =
      generateArmorsOffset(msg.num_armors, off_r1, off_r2, off_dza, msg.d_zc);

  try {
    const auto &P = tracker.ukf().P();
    int dim = static_cast<int>(P.rows());
    msg.covariance_dim = dim;
    msg.state_covariance.resize(dim * dim);
    for (int r = 0; r < dim; ++r)
      for (int c = 0; c < dim; ++c)
        msg.state_covariance[r * dim + c] = P(r, c);
  } catch (...) {
    msg.state_covariance.clear();
    msg.covariance_dim = 0;
  }

  msg.bound_armor_ids = {robot_id};

  if (tracker.is_tracking())
    msg.confidence = 1.0;
  else if (tracker.is_temp_lost())
    msg.confidence = 0.7;
  else
    msg.confidence = 0.3;

  msg.is_visible = tracker.is_tracking() || tracker.is_temp_lost();
  auto it = last_obs_counts_.find(robot_id);
  msg.visible_armor_count =
      (msg.is_visible && it != last_obs_counts_.end()) ? it->second : 0;

  return msg;
}

/* ================================================================ */
/*  Tracker helpers                                                  */
/* ================================================================ */

uint8_t GimbalPipelineNode::inferRobotType(
    const std::string &robot_id) const {
  if (robot_id == "outpost")
    return rm_interfaces::msg::TrackedRobot::OUTPOST_3;
  if (robot_id == "base") return rm_interfaces::msg::TrackedRobot::BASE;
  if (robot_id == "sentry") return rm_interfaces::msg::TrackedRobot::SENTRY;
  if (robot_id == "1") return rm_interfaces::msg::TrackedRobot::HERO_4;
  if (robot_id == "2" || robot_id == "3" || robot_id == "4" ||
      robot_id == "5")
    return rm_interfaces::msg::TrackedRobot::STANDARD_4;
  return rm_interfaces::msg::TrackedRobot::UNKNOWN;
}

int GimbalPipelineNode::inferNumArmors(const std::string & /*robot_id*/,
                                       int robot_type) const {
  using T = rm_interfaces::msg::TrackedRobot;
  if (robot_type == T::OUTPOST_3 || robot_type == T::BASE) return 3;
  if (robot_type == T::BALANCE_2) return 2;
  return 4;
}

std::vector<geometry_msgs::msg::Pose>
GimbalPipelineNode::generateArmorsOffset(int num_armors, double r1,
                                          double r2, double d_za,
                                          double d_zc) const {
  std::vector<geometry_msgs::msg::Pose> offsets;
  bool is_current_pair = true;

  for (int i = 0; i < num_armors; ++i) {
    double angle = i * (2.0 * M_PI / num_armors);
    double r, dz;
    if (num_armors == 4) {
      r = is_current_pair ? r1 : r2;
      dz = d_zc + (is_current_pair ? -d_za : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      dz = d_zc;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = -r * std::cos(angle);
    pose.position.y = -r * std::sin(angle);
    pose.position.z = dz;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    offsets.push_back(pose);
  }
  return offsets;
}

/* ================================================================ */
/*  Target selection (internal, no ROS topic)                        */
/* ================================================================ */

void GimbalPipelineNode::initSelectionStrategy() {
  if (selector_strategy_name_ == "min_yaw_deviation") {
    selection_strategy_ = std::make_unique<MinYawDeviationStrategy>();
  } else {
    RCLCPP_WARN(get_logger(), "Unknown selector strategy '%s', using min_yaw_deviation",
                selector_strategy_name_.c_str());
    selection_strategy_ = std::make_unique<MinYawDeviationStrategy>();
  }
  RCLCPP_INFO(get_logger(), "Selection strategy: %s",
              selection_strategy_->getName().c_str());
}

SelectionResult GimbalPipelineNode::selectTargetInternal(
    const rm_interfaces::msg::TrackedRobots &robots) {
  selection_config_.current_target_id = current_target_id_;

  auto result = selection_strategy_->selectTarget(robots, selection_config_);

  if (!result.has_value()) {
    current_target_id_ = "";
    return SelectionResult();
  }

  bool target_changed = (result->robot_id != current_target_id_);
  if (target_changed) {
    RCLCPP_INFO(get_logger(), "Target changed: %s -> %s (yaw_dev=%.3f, dist=%.2f)",
                current_target_id_.empty() ? "none" : current_target_id_.c_str(),
                result->robot_id.c_str(), result->yaw_deviation,
                result->distance);
    current_target_id_ = result->robot_id;
  }

  return *result;
}

/* ================================================================ */
/*  Gimbal controller initialization                                 */
/* ================================================================ */

void GimbalPipelineNode::initGimbalComponents() {
  position_calculator_ =
      std::make_shared<gimbal_controller::ArmorPositionCalculator>();
  armor_selector_ = std::make_shared<gimbal_controller::ArmorSelector>();
  ballistic_client_ =
      std::make_shared<gimbal_controller::BallisticSolverClient>(this);
  local_compensator_ =
      std::make_shared<gimbal_controller::LocalTrajectoryCompensator>();
  fire_advisor_ = std::make_shared<gimbal_controller::FireAdvisor>();
}

void GimbalPipelineNode::initGimbalStrategies() {
  auto current_s =
      std::make_shared<gimbal_controller::CurrentPositionStrategy>();
  current_s->setComponents(position_calculator_, armor_selector_,
                           ballistic_client_, local_compensator_,
                           fire_advisor_);
  gimbal_strategies_["current"] = current_s;

  auto predicted_s =
      std::make_shared<gimbal_controller::PredictedPositionStrategy>();
  predicted_s->setComponents(position_calculator_, armor_selector_,
                             ballistic_client_, local_compensator_,
                             fire_advisor_);
  gimbal_strategies_["predicted"] = predicted_s;

  auto mpc_s = std::make_shared<gimbal_controller::MpcControlStrategy>();
  mpc_s->setComponents(position_calculator_, armor_selector_,
                       ballistic_client_, local_compensator_, fire_advisor_);
  gimbal_strategies_["mpc"] = mpc_s;

  auto sm_s = std::make_shared<gimbal_controller::StateMachineStrategy>();
  sm_s->setComponents(position_calculator_, armor_selector_,
                      ballistic_client_, local_compensator_, fire_advisor_);
  gimbal_strategies_["state_machine"] = sm_s;
}

/* ================================================================ */
/*  Joint state callback                                             */
/* ================================================================ */

void GimbalPipelineNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "yaw_joint")
      current_yaw_ = msg->position[i];
    else if (msg->name[i] == "pitch_joint")
      current_pitch_ = msg->position[i];
  }
}

void GimbalPipelineNode::updateGimbalState() {
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform(target_frame_,
                                                   "gimbal_link",
                                                   tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    current_pitch_ = -pitch;
  } catch (const tf2::TransformException &) {
    // fall through — use joint_states values
  }
}

/* ================================================================ */
/*  Timer callback — 250 Hz control loop                             */
/* ================================================================ */

void GimbalPipelineNode::timerCallback() {
  if (!enable_) {
    rm_interfaces::msg::GimbalCmd idle_cmd;
    idle_cmd.yaw_diff = 0;
    idle_cmd.pitch_diff = 0;
    idle_cmd.distance = -1;
    idle_cmd.fire_advice = false;
    gimbal_cmd_pub_->publish(idle_cmd);
    // idle 期间清空滤波器状态，恢复跟踪时允许首帧自由跳变
    cmd_filter_.reset();
    prev_tracking_target_id_.clear();
    return;
  }

  updateGimbalState();

  auto strategy = getGimbalStrategy(current_gimbal_strategy_name_);
  if (!strategy) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Gimbal strategy '%s' not found",
                         current_gimbal_strategy_name_.c_str());
    return;
  }

  gimbal_controller::GimbalControlContext context;
  context.current_yaw = current_yaw_;
  context.current_pitch = current_pitch_;
  context.bullet_speed = bullet_speed_;
  context.current_time = now();
  context.is_tracking = false;

  // Read shared state (thread-safe)
  rm_interfaces::msg::TrackedRobots::SharedPtr robots;
  std::string selected_id;
  rclcpp::Time data_update_time{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    robots = latest_tracked_robots_;
    selected_id = latest_selected_target_id_;
    data_update_time = latest_update_time_;
  }

  if (robots && !robots->robots.empty()) {
    if (!selected_id.empty()) {
      for (const auto &robot : robots->robots) {
        if (robot.robot_id == selected_id) {
          context.target_robot = robot;
          // Use local clock timestamp to avoid cross-clock-domain mismatch
          // (camera hardware stamps vs system wall clock)
          context.target_stamp = data_update_time;
          context.is_tracking =
              (robot.track_state ==
                   rm_interfaces::msg::TrackedRobot::TRACKING ||
               robot.track_state ==
                   rm_interfaces::msg::TrackedRobot::TEMP_LOST);
          break;
        }
      }
    } else {
      // No selection — use first robot
      context.target_robot = robots->robots[0];
      context.target_stamp = data_update_time;
      context.is_tracking =
          (context.target_robot.track_state ==
               rm_interfaces::msg::TrackedRobot::TRACKING ||
           context.target_robot.track_state ==
               rm_interfaces::msg::TrackedRobot::TEMP_LOST);
    }
  }

  auto cmd = strategy->solve(context);

  // ── GimbalCmd 输出端保护滤波 ──
  // 目标切换（或从无目标变为有目标）时重置滤波器，允许首帧自由跳变快速锁定
  const std::string &current_target = context.is_tracking ? selected_id : std::string("");
  if (current_target != prev_tracking_target_id_) {
    cmd_filter_.reset();
  }
  cmd_filter_.filter(cmd);
  prev_tracking_target_id_ = current_target;

  gimbal_cmd_pub_->publish(cmd);

  if (debug_mode_ && context.is_tracking)
    publishGimbalMarkers(context.target_robot, cmd);
}

/* ================================================================ */
/*  Service callback                                                 */
/* ================================================================ */

void GimbalPipelineNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  int mode = request->mode;
  if (mode == 1 || mode == 2) {
    enable_ = true;
    RCLCPP_INFO(get_logger(), "GimbalPipeline enabled (mode=%d)", mode);
  } else {
    enable_ = false;
    RCLCPP_INFO(get_logger(), "GimbalPipeline disabled (mode=%d)", mode);
  }
}

gimbal_controller::GimbalControlStrategy::SharedPtr
GimbalPipelineNode::getGimbalStrategy(const std::string &name) const {
  auto it = gimbal_strategies_.find(name);
  return (it != gimbal_strategies_.end()) ? it->second : nullptr;
}

/* ================================================================ */
/*  Visualization                                                    */
/* ================================================================ */

void GimbalPipelineNode::initMarkers() {
  position_marker_.ns = "target_position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y =
      position_marker_.scale.z = 0.15;
  position_marker_.color.a = 1.0;
  position_marker_.color.r = 1.0;

  target_velocity_marker_.type = visualization_msgs::msg::Marker::ARROW;
  target_velocity_marker_.ns = "target_velocity";
  target_velocity_marker_.scale.x = 0.03;
  target_velocity_marker_.scale.y = 0.05;
  target_velocity_marker_.color.a = 1.0;
  target_velocity_marker_.color.g = 1.0;
  target_velocity_marker_.color.b = 1.0;

  armors_marker_.ns = "armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.y = 0.23;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 0.7;
  armors_marker_.color.g = 0.5;
  armors_marker_.color.b = 1.0;

  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y =
      selection_marker_.scale.z = 0.12;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.r = 1.0;
  selection_marker_.color.g = 1.0;

  predicted_marker_.ns = "predicted_hit";
  predicted_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  predicted_marker_.scale.x = predicted_marker_.scale.y =
      predicted_marker_.scale.z = 0.1;
  predicted_marker_.color.a = 1.0;
  predicted_marker_.color.g = 1.0;

  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker_.scale.x = 0.02;
  trajectory_marker_.color.a = 0.8;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;

  color_palette_.clear();
  for (int i = 0; i < 10; ++i) {
    float hue = i * 36.0f;
    color_palette_.push_back(hsvToRgb(hue, 1.0f, 1.0f));
  }
}

void GimbalPipelineNode::publishGimbalMarkers(
    const rm_interfaces::msg::TrackedRobot &target_robot,
    const rm_interfaces::msg::GimbalCmd &cmd) {
  if (!debug_gimbal_marker_pub_) return;

  visualization_msgs::msg::MarkerArray marker_array;

  // Position
  position_marker_.header = target_robot.header;
  position_marker_.id = 0;
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  position_marker_.pose.position = target_robot.center_position;
  position_marker_.pose.orientation.w = 1.0;
  marker_array.markers.push_back(position_marker_);

  // Velocity arrow
  target_velocity_marker_.header = target_robot.header;
  target_velocity_marker_.id = 0;
  target_velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
  target_velocity_marker_.points.clear();
  geometry_msgs::msg::Point vel_start = target_robot.center_position;
  geometry_msgs::msg::Point vel_end = target_robot.center_position;
  vel_end.x += target_robot.center_velocity.x * 0.5;
  vel_end.y += target_robot.center_velocity.y * 0.5;
  vel_end.z += target_robot.center_velocity.z * 0.5;
  target_velocity_marker_.points.push_back(vel_start);
  target_velocity_marker_.points.push_back(vel_end);
  marker_array.markers.push_back(target_velocity_marker_);

  // Armor plates
  if (!target_robot.armors_offset.empty()) {
    for (size_t i = 0; i < target_robot.armors_offset.size(); ++i) {
      auto armor_marker = armors_marker_;
      armor_marker.header = target_robot.header;
      armor_marker.id = static_cast<int>(i);
      armor_marker.action = visualization_msgs::msg::Marker::ADD;
      double cos_yaw = std::cos(target_robot.yaw);
      double sin_yaw = std::sin(target_robot.yaw);
      const auto &offset = target_robot.armors_offset[i];
      armor_marker.pose.position.x =
          target_robot.center_position.x +
          offset.position.x * cos_yaw - offset.position.y * sin_yaw;
      armor_marker.pose.position.y =
          target_robot.center_position.y +
          offset.position.x * sin_yaw + offset.position.y * cos_yaw;
      armor_marker.pose.position.z =
          target_robot.center_position.z + offset.position.z;
      tf2::Quaternion q;
      q.setRPY(0, 0.2618,
               target_robot.yaw +
                   i * (2 * M_PI / target_robot.num_armors));
      armor_marker.pose.orientation.x = q.x();
      armor_marker.pose.orientation.y = q.y();
      armor_marker.pose.orientation.z = q.z();
      armor_marker.pose.orientation.w = q.w();
      marker_array.markers.push_back(armor_marker);
    }
  }

  // Selection target
  if (cmd.distance > 0) {
    selection_marker_.header = target_robot.header;
    selection_marker_.id = 0;
    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    double yaw_rad = cmd.yaw * M_PI / 180.0;
    double pitch_rad = cmd.pitch * M_PI / 180.0;
    selection_marker_.pose.position.x =
        cmd.distance * std::cos(pitch_rad) * std::cos(yaw_rad);
    selection_marker_.pose.position.y =
        cmd.distance * std::cos(pitch_rad) * std::sin(yaw_rad);
    selection_marker_.pose.position.z =
        cmd.distance * std::sin(pitch_rad);
    selection_marker_.pose.orientation.w = 1.0;
    marker_array.markers.push_back(selection_marker_);
  }

  // Predicted hit (for predicted strategy)
  if (current_gimbal_strategy_name_ == "predicted" && cmd.distance > 0) {
    predicted_marker_.header = target_robot.header;
    predicted_marker_.id = 0;
    predicted_marker_.action = visualization_msgs::msg::Marker::ADD;
    predicted_marker_.pose = selection_marker_.pose;
    predicted_marker_.color.a = 0.6;
    marker_array.markers.push_back(predicted_marker_);
  }

  // Trajectory
  if (cmd.distance > 0) {
    trajectory_marker_.header.frame_id = "gimbal_link";
    trajectory_marker_.header.stamp = target_robot.header.stamp;
    trajectory_marker_.id = 0;
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    int num_points = 20;
    double yaw_rad = cmd.yaw * M_PI / 180.0;
    double pitch_rad = cmd.pitch * M_PI / 180.0;
    for (int i = 0; i <= num_points; ++i) {
      double t = static_cast<double>(i) / num_points;
      double distance = cmd.distance * t;
      geometry_msgs::msg::Point p;
      p.x = distance * std::cos(pitch_rad) * std::cos(yaw_rad);
      p.y = distance * std::cos(pitch_rad) * std::sin(yaw_rad);
      double flight_time = cmd.distance / bullet_speed_ * t;
      p.z = distance * std::sin(pitch_rad) -
            0.5 * 9.8 * flight_time * flight_time;
      trajectory_marker_.points.push_back(p);
    }
    if (cmd.fire_advice) {
      trajectory_marker_.color.r = 0.0;
      trajectory_marker_.color.g = 1.0;
      trajectory_marker_.color.b = 0.0;
    } else {
      trajectory_marker_.color.r = 1.0;
      trajectory_marker_.color.g = 0.75;
      trajectory_marker_.color.b = 0.79;
    }
    marker_array.markers.push_back(trajectory_marker_);
  }

  debug_gimbal_marker_pub_->publish(marker_array);
}

std::array<float, 4> GimbalPipelineNode::hsvToRgb(float h, float s,
                                                    float v) {
  float c = v * s;
  float x = c * (1 - std::abs(std::fmod(h / 60.0f, 2.0f) - 1));
  float m = v - c;
  float r, g, b;
  if (h < 60) { r = c; g = x; b = 0; }
  else if (h < 120) { r = x; g = c; b = 0; }
  else if (h < 180) { r = 0; g = c; b = x; }
  else if (h < 240) { r = 0; g = x; b = c; }
  else if (h < 300) { r = x; g = 0; b = c; }
  else { r = c; g = 0; b = x; }
  return {r + m, g + m, b + m, 1.0f};
}

}  // namespace fyt::auto_aim

// Register as composable node
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GimbalPipelineNode)
