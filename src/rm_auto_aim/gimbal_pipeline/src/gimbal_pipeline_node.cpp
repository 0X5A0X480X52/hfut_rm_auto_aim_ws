// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalPipelineNode — unified node merging MaxEntropyTracker +
// TargetSelector + GimbalController. Inter-node ROS2 topics are replaced
// by direct C++ function calls to eliminate serialization / scheduling
// latency.

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"

#include <cmath>
#include <limits>
#include <rm_utils/heartbeat.hpp>
#include <sstream>
#include <unordered_set>

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
#include "gimbal_controller/fire_advisor.hpp"

namespace
{

bool hasParameterOverride(rclcpp::Node & node, const std::string & key)
{
  const auto params_interface = node.get_node_parameters_interface();
  if (!params_interface) {
    return false;
  }
  const auto & overrides = params_interface->get_parameter_overrides();
  return overrides.find(key) != overrides.end();
}

bool shouldWarnDeprecatedOnce(const std::string & deprecated_key)
{
  static std::unordered_set<std::string> warned_keys;
  return warned_keys.insert(deprecated_key).second;
}

double readCompatDoubleParameter(
  rclcpp::Node & node,
  const std::string & canonical_key,
  const std::string & deprecated_key,
  double conflict_eps = 1e-9)
{
  const double canonical_value = node.get_parameter(canonical_key).as_double();
  const double deprecated_value = node.get_parameter(deprecated_key).as_double();
  const bool canonical_overridden = hasParameterOverride(node, canonical_key);
  const bool deprecated_overridden = hasParameterOverride(node, deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %.6f",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value);
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden &&
    std::abs(canonical_value - deprecated_value) > conflict_eps)
  {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%.6f, canonical=%.6f). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value,
        canonical_value);
    }
  }

  return canonical_value;
}

int readCompatIntParameter(
  rclcpp::Node & node,
  const std::string & canonical_key,
  const std::string & deprecated_key)
{
  const int canonical_value = node.get_parameter(canonical_key).as_int();
  const int deprecated_value = node.get_parameter(deprecated_key).as_int();
  const bool canonical_overridden = hasParameterOverride(node, canonical_key);
  const bool deprecated_overridden = hasParameterOverride(node, deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %d",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value);
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden && canonical_value != deprecated_value) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%d, canonical=%d). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value,
        canonical_value);
    }
  }

  return canonical_value;
}

bool readCompatBoolParameter(
  rclcpp::Node & node,
  const std::string & canonical_key,
  const std::string & deprecated_key)
{
  const bool canonical_value = node.get_parameter(canonical_key).as_bool();
  const bool deprecated_value = node.get_parameter(deprecated_key).as_bool();
  const bool canonical_overridden = hasParameterOverride(node, canonical_key);
  const bool deprecated_overridden = hasParameterOverride(node, deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %s",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value ? "true" : "false");
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden && canonical_value != deprecated_value) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%s, canonical=%s). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value ? "true" : "false",
        canonical_value ? "true" : "false");
    }
  }

  return canonical_value;
}

}  // namespace

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

  RCLCPP_INFO(get_logger(), "Parameters declared, now loading...");

  // ── 2. Read common / tracker params ──
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  predict_rate_ = get_parameter("predict_rate").as_double();
  debug_mode_ = get_parameter("debug_mode").as_bool();
  visualization_frame_ = get_parameter("visualization_frame").as_string();
  tracker_timeout_s_ = std::max(get_parameter("tracker_timeout").as_double(), 1e-3);

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
      tracker_timeout_s_,
      get_parameter("enable_oscillation_detection").as_bool());

  robot_description_facade_ = std::make_unique<robot_description::RobotDescriptionFacade>();
  robot_description_facade_->setStrictUnknownReject(
      get_parameter("robot_description.strict_unknown_reject").as_bool());

  {
    std::ostringstream oss;
    const auto supported_ids = robot_description_facade_->supportedRobotIds();
    for (size_t i = 0; i < supported_ids.size(); ++i) {
      if (i != 0) {
        oss << ",";
      }
      oss << supported_ids[i];
    }
    RCLCPP_INFO(
      get_logger(),
      "RobotDescription initialized (strict_unknown_reject=%s, supported_ids=[%s])",
      robot_description_facade_->strictUnknownReject() ? "true" : "false",
      oss.str().c_str());
  }

  RCLCPP_INFO(get_logger(), "Tracker initialized (predict_rate=%.1f Hz)",
              predict_rate_);

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
    selection_config_.priority_robot_ids =
      get_parameter("selector.priority_robot_ids").as_string_array();
    selection_config_.sticky_lock_frames =
      get_parameter("selector.sticky_lock_frames").as_int();
    selection_config_.sticky_lost_frames =
      get_parameter("selector.sticky_lost_frames").as_int();
  initSelectionStrategy();

  RCLCPP_INFO(get_logger(), "[GimbalPipelineNode] selector_strategy: %s", selector_strategy_name_.c_str());

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
  double prediction_delay = readCompatDoubleParameter(
    *this, "controller.solver.prediction_delay", "solver.prediction_delay");
  double max_prediction_time = readCompatDoubleParameter(
    *this, "controller.solver.max_prediction_time", "solver.max_prediction_time");
  double max_tracking_v_yaw = get_parameter("controller.solver.max_tracking_v_yaw").as_double();
  int transfer_thresh = get_parameter("controller.solver.transfer_thresh").as_int();
  double gravity = get_parameter("controller.solver.gravity").as_double();
  double resistance = get_parameter("controller.solver.resistance").as_double();
  int iteration_times = get_parameter("controller.solver.iteration_times").as_int();
  double pitch_offset = get_parameter("controller.solver.pitch_offset").as_double();
  double yaw_offset = get_parameter("controller.solver.yaw_offset").as_double();
  double facing_enter_angle = get_parameter("controller.solver.facing_enter_angle").as_double();
  double facing_exit_angle = get_parameter("controller.solver.facing_exit_angle").as_double();
  bool radial_dynamic_enable = get_parameter("controller.solver.radial_dynamic.enable").as_bool();
  double radial_dynamic_v_yaw_ref = get_parameter("controller.solver.radial_dynamic.v_yaw_ref").as_double();
  double radial_dynamic_shrink_ratio = get_parameter("controller.solver.radial_dynamic.shrink_ratio").as_double();
  double radial_dynamic_min_angle_deg = get_parameter("controller.solver.radial_dynamic.min_angle_deg").as_double();
  double radial_dynamic_bias_gain_deg = get_parameter("controller.solver.radial_dynamic.bias_gain_deg").as_double();
  double radial_dynamic_max_bias_deg = get_parameter("controller.solver.radial_dynamic.max_bias_deg").as_double();
  double controller_delay = readCompatDoubleParameter(
    *this, "controller.solver.controller_delay", "solver.controller_delay");
  double trigger_to_muzzle_s = readCompatDoubleParameter(
    *this, "controller.solver.trigger_to_muzzle_s", "solver.trigger_to_muzzle_s");
  if (hasParameterOverride(*this, "controller.fire.trigger_to_muzzle_s")) {
    trigger_to_muzzle_s = get_parameter("controller.fire.trigger_to_muzzle_s").as_double();
  }
  double max_processing_delay_s = readCompatDoubleParameter(
    *this, "controller.mpc.max_processing_delay_s", "mpc.max_processing_delay_s");
  std::string selection_method_str = get_parameter("controller.solver.selection_method").as_string();
  std::string fire_policy = get_parameter("controller.fire.decision_policy").as_string();
  int fire_flight_time_iters = get_parameter("controller.fire.flight_time_iters").as_int();
  bool fire_use_gimbal_kinematics =
    get_parameter("controller.fire.use_gimbal_kinematics").as_bool();

  facing_enter_angle_deg_ = facing_enter_angle;
  facing_exit_angle_deg_ = facing_exit_angle;
  radial_dynamic_enable_ = radial_dynamic_enable;
  radial_dynamic_v_yaw_ref_ = std::max(radial_dynamic_v_yaw_ref, 1e-6);
  radial_dynamic_shrink_ratio_ = std::clamp(radial_dynamic_shrink_ratio, 0.0, 1.0);
  radial_dynamic_min_angle_deg_ = std::max(radial_dynamic_min_angle_deg, 0.0);
  radial_dynamic_bias_gain_deg_ = std::max(radial_dynamic_bias_gain_deg, 0.0);
  radial_dynamic_max_bias_deg_ = std::max(radial_dynamic_max_bias_deg, 0.0);

  armor_selector_->setParameters(side_angle, min_switching_v_yaw);
  armor_selector_->setFacingParameters(facing_enter_angle, facing_exit_angle);
  armor_selector_->setRadialDynamicParameters(
    radial_dynamic_enable,
    radial_dynamic_v_yaw_ref,
    radial_dynamic_shrink_ratio,
    radial_dynamic_min_angle_deg,
    radial_dynamic_bias_gain_deg,
    radial_dynamic_max_bias_deg);

  // 配置选板策略
  gimbal_controller::ArmorSelector::SelectionMethod sel_method =
    gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_FACING;
  if (selection_method_str == "min_movement") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT;
  } else if (selection_method_str == "min_movement_with_radial") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL;
  } else if (selection_method_str == "decision_angle") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::DECISION_ANGLE;
  }
  armor_selector_->setSelectionMethod(sel_method);
  radial_selection_enabled_ =
    (sel_method == gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL);
  RCLCPP_INFO(get_logger(), "[GimbalController] selection_method: %s", selection_method_str.c_str());

  fire_advisor_->setParameters(shooting_range_w, shooting_range_h);
  if (fire_policy == "ellipse") {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::EllipseFireDecisionPolicy>());
  } else {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::AxisThresholdFireDecisionPolicy>());
  }
  if (fire_advice_engine_) {
    fire_advice_engine_->setFlightTimeIterations(fire_flight_time_iters);
    fire_advice_engine_->setUseGimbalKinematics(fire_use_gimbal_kinematics);
  }
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
  local_compensator_->setParameters(bullet_speed_, gravity, resistance,
                                    iteration_times);

  // Strategy-specific configuration
  auto predicted_strategy = std::dynamic_pointer_cast<
      gimbal_controller::PredictedPositionStrategy>(
      gimbal_strategies_["predicted"]);
  if (predicted_strategy) {
    predicted_strategy->setPredictionParameters(prediction_delay, max_prediction_time);
    predicted_strategy->setMaxProcessingDelay(max_processing_delay_s);
    predicted_strategy->setManualOffset(pitch_offset, yaw_offset);
    predicted_strategy->setTrackingCenterParams(max_tracking_v_yaw, transfer_thresh);
    predicted_strategy->setControllerDelay(controller_delay);
    predicted_strategy->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
  }
  auto current_strategy = std::dynamic_pointer_cast<
      gimbal_controller::CurrentPositionStrategy>(
      gimbal_strategies_["current"]);
  if (current_strategy) {
    current_strategy->setManualOffset(pitch_offset, yaw_offset);
    current_strategy->setControllerDelay(controller_delay);
    current_strategy->setMaxProcessingDelay(max_processing_delay_s);
    current_strategy->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
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
  double sm_prediction_delay = readCompatDoubleParameter(
    *this,
    "controller.state_machine.prediction_delay",
    "state_machine.prediction_delay");
  double sm_max_prediction = readCompatDoubleParameter(
    *this,
    "controller.state_machine.max_prediction_time",
    "state_machine.max_prediction_time");

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
    sm_strategy_ptr->setMaxProcessingDelay(max_processing_delay_s);
    sm_strategy_ptr->setManualOffset(pitch_offset, yaw_offset);
    sm_strategy_ptr->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
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
    fcfg.enable_moving_average       = get_parameter("controller.output_filter.enable_moving_average").as_bool();
    fcfg.moving_average_window_size  = get_parameter("controller.output_filter.moving_average_window_size").as_int();
    fcfg.enable_ema                  = get_parameter("controller.output_filter.enable_ema").as_bool();
    fcfg.ema_alpha                   = get_parameter("controller.output_filter.ema_alpha").as_double();
    fcfg.enable_one_euro             = get_parameter("controller.output_filter.enable_one_euro").as_bool();
    fcfg.one_euro_freq               = get_parameter("controller.output_filter.one_euro_freq").as_double();
    fcfg.one_euro_min_cutoff         = get_parameter("controller.output_filter.one_euro_min_cutoff").as_double();
    fcfg.one_euro_beta               = get_parameter("controller.output_filter.one_euro_beta").as_double();
    fcfg.one_euro_d_cutoff           = get_parameter("controller.output_filter.one_euro_d_cutoff").as_double();
    if (gimbal_control_core_) {
      gimbal_control_core_->setFilterConfig(fcfg);
    }
    RCLCPP_INFO(get_logger(),
      "[GimbalCmdFilter] clamp=%s(%.1f°,%.1f°) outlier=%s(%.1f°,%.1f°,max%d)"
      " rate=%s(%.1f°,%.1f°) mean=%s(win=%d) ema=%s(a=%.2f) 1euro=%s(f=%.0f,mc=%.2f,b=%.4f)",
      fcfg.enable_clamping ? "ON" : "off", fcfg.max_yaw_diff, fcfg.max_pitch_diff,
      fcfg.enable_outlier_rejection ? "ON" : "off",
      fcfg.outlier_threshold_yaw, fcfg.outlier_threshold_pitch, fcfg.max_outlier_count,
      fcfg.enable_rate_limiter ? "ON" : "off", fcfg.max_yaw_rate, fcfg.max_pitch_rate,
      fcfg.enable_moving_average ? "ON" : "off", fcfg.moving_average_window_size,
      fcfg.enable_ema ? "ON" : "off", fcfg.ema_alpha,
      fcfg.enable_one_euro ? "ON" : "off",
      fcfg.one_euro_freq, fcfg.one_euro_min_cutoff, fcfg.one_euro_beta);
  }

  RCLCPP_INFO(get_logger(),
     "GimbalPipelineNode initialized: target_frame=%s, control_rate=%.0f Hz, "
     "selector_strategy=%s, gimbal_strategy=%s, ballistic_mode=%s",
     target_frame_.c_str(), control_rate_,
     selector_strategy_name_.c_str(),
     current_gimbal_strategy_name_.c_str(),
     ballistic_mode_.c_str());

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

  // Subscribe: camera_info (for FOV soft constraint)
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      std::bind(&GimbalPipelineNode::cameraInfoCallback, this,
                std::placeholders::_1));

  // Publish: cmd_gimbal (output to serial driver)
  gimbal_cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>(
      "cmd_gimbal", rclcpp::SensorDataQoS());

  // Maneuver states publisher (always-on, for chart monitoring)
  maneuver_states_pub_ =
      create_publisher<rm_interfaces::msg::ManeuverStates>(
          "~/maneuver_states", rclcpp::SensorDataQoS());

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
    debug_delay_audit_pub_ = create_publisher<rm_interfaces::msg::DelayAudit>(
      "~/delay_audit", rclcpp::SensorDataQoS());
    debug_tracker_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/tracker_markers", 10);
    debug_gimbal_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/gimbal_markers", 10);
    debug_maneuver_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/maneuver_markers", 10);
  }

  RCLCPP_INFO(get_logger(), "Subscribed to topics: /armor_detector/armors (with TF sync), /joint_states, camera_info");

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

  RCLCPP_INFO(get_logger(), "Service ~/set_mode ready");

  // ─── Prediction logger ────────────────────────────────────────
  if (get_parameter("logging.enable").as_bool()) {
    prediction_logger_ = std::make_unique<PredictionLogger>(
        get_parameter("logging.output_dir").as_string(),
        get_parameter("logging.robot_id_filter").as_string(),
        get_parameter("logging.flush_every_n").as_int());
    RCLCPP_INFO(get_logger(), "PredictionLogger enabled, output: %s",
                get_parameter("logging.output_dir").as_string().c_str());
  }

  RCLCPP_INFO(get_logger(), "PredictionLogger: %s", prediction_logger_ ? "enabled" : "disabled");

  // ── 6. Heartbeat ──
  heartbeat_ = HeartBeatPublisher::create(this);

  RCLCPP_INFO(get_logger(), "GimbalPipelineNode (unified pipeline) initialized successfully");

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
  declare_parameter("tracker_timeout", 0.5);
  declare_parameter("debug_mode", false);
  declare_parameter("enable_oscillation_detection", false);
  declare_parameter("visualization_frame", "odom");
  declare_parameter("robot_description.strict_unknown_reject", true);

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

  // Outpost-specific (known 3-armor geometry + max-entropy mode switch)
  declare_parameter("outpost.translation_model", "CV");
  declare_parameter("outpost.rotation_model", "CV");
  declare_parameter("outpost.singer_alpha", 0.0);
  declare_parameter("outpost.singer_sigma", 0.0);
  declare_parameter("outpost.spin_process_noise_theta_rate", 0.0);
  declare_parameter("outpost.spin_process_noise_theta_acc", 0.0);
  declare_parameter("outpost.radius", 0.26);
  declare_parameter("outpost.z_offset_0", -0.06);
  declare_parameter("outpost.z_offset_1", 0.0);
  declare_parameter("outpost.z_offset_2", 0.06);
  declare_parameter("outpost.panel_angle_step", 2.0 * M_PI / 3.0);
  declare_parameter("outpost.softmax_temperature", 1.5);
  declare_parameter("outpost.weight_yaw", 1.0);
  declare_parameter("outpost.weight_z_state", 6.0);
  declare_parameter("outpost.weight_z_history", 2.0);
  declare_parameter("outpost.entropy_enter", 0.75);
  declare_parameter("outpost.entropy_exit", 0.55);
  declare_parameter("outpost.max_prob_enter", 0.60);
  declare_parameter("outpost.max_prob_exit", 0.75);
  declare_parameter("outpost.stable_frames", 4);
  declare_parameter("outpost.z_history_window", 15);
  declare_parameter("outpost.single_mode_confidence_scale", 0.70);
  declare_parameter("outpost.alpha_pos", 0.65);
  declare_parameter("outpost.beta_vel", 0.30);
  declare_parameter("outpost.alpha_yaw", 0.60);
  declare_parameter("outpost.beta_yaw_rate", 0.25);
  declare_parameter("outpost.assume_static_center", true);
  declare_parameter("outpost.linear_velocity_damping", 0.90);
  declare_parameter("outpost.yaw_rate_damping", 0.98);
  declare_parameter("outpost.max_center_speed", 1.00);
  declare_parameter("outpost.max_yaw_rate", 12.0);

  // Maneuver detection
  declare_parameter("maneuver.enable", true);
  declare_parameter("maneuver.nis_threshold_single", 238.807);
  declare_parameter("maneuver.nis_threshold_dual", 4132.110);
  declare_parameter("maneuver.innov_norm_threshold_single", 0.1279);
  declare_parameter("maneuver.innov_norm_threshold_dual", 0.0613);

  // Panel mismatch detection
  declare_parameter("panel_mismatch.enable", true);
  declare_parameter("panel_mismatch.window_size", 8);
  declare_parameter("panel_mismatch.threshold_t1", 0.0009);
  declare_parameter("panel_mismatch.confirm_count", 3);
  declare_parameter("panel_mismatch.reinit_count", 5);

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
  declare_parameter("smoother.enable_outlier_filter",    false);
  declare_parameter("smoother.outlier_method",           std::string("mad"));
  declare_parameter("smoother.outlier_window_size",      10);
  declare_parameter("smoother.outlier_min_samples",      5);
  declare_parameter("smoother.outlier_mad_k",            3.5);
  declare_parameter("smoother.outlier_iqr_k",            1.5);
  declare_parameter("smoother.outlier_mahal_threshold",  9.21);
}

void GimbalPipelineNode::declareTargetSelectorParameters() {
  declare_parameter("selector.strategy", "min_yaw_deviation");
  declare_parameter("selector.reference_yaw", 0.0);
  declare_parameter("selector.max_yaw_deviation", M_PI);
  declare_parameter("selector.max_distance", 10.0);
  declare_parameter("selector.min_confidence", 0.3);
  declare_parameter("selector.hysteresis_threshold", 0.1);
  declare_parameter("selector.priority_robot_ids", std::vector<std::string>{});
  declare_parameter("selector.sticky_lock_frames", 3);
  declare_parameter("selector.sticky_lost_frames", 3);
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
  declare_parameter("controller.solver.radial_dynamic.enable", false);
  declare_parameter("controller.solver.radial_dynamic.v_yaw_ref", 8.0);
  declare_parameter("controller.solver.radial_dynamic.shrink_ratio", 0.6);
  declare_parameter("controller.solver.radial_dynamic.min_angle_deg", 5.0);
  declare_parameter("controller.solver.radial_dynamic.bias_gain_deg", 0.0);
  declare_parameter("controller.solver.radial_dynamic.max_bias_deg", 0.0);
  declare_parameter("controller.solver.controller_delay", 0.0);
  declare_parameter("controller.solver.trigger_to_muzzle_s", 0.0);
  declare_parameter("controller.solver.selection_method", std::string("min_movement_with_facing"));
  declare_parameter("controller.fire.trigger_to_muzzle_s", 0.0);
  declare_parameter("controller.fire.decision_policy", std::string("axis_threshold"));
  declare_parameter("controller.fire.flight_time_iters", 2);
  declare_parameter("controller.fire.use_gimbal_kinematics", false);

  // Deprecated aliases (for migration from legacy gimbal_controller keys)
  declare_parameter("solver.prediction_delay", 0.0);
  declare_parameter("solver.max_prediction_time", 0.5);
  declare_parameter("solver.controller_delay", 0.0);
  declare_parameter("solver.trigger_to_muzzle_s", 0.0);

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

  // Deprecated aliases (for migration from legacy gimbal_controller keys)
  declare_parameter("state_machine.prediction_delay", 0.0);
  declare_parameter("state_machine.max_prediction_time", 0.5);

  // MPC strategy
  declare_parameter("controller.mpc.N", 20);
  declare_parameter("controller.mpc.dt", 0.01);
  declare_parameter("controller.mpc.control_delay_s", 0.0);
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
  declare_parameter("controller.mpc.enable_delay_compensation", false);
  declare_parameter("controller.mpc.prediction_delay_s", 0.0);
  declare_parameter("controller.mpc.flight_time_iters", 2);
  declare_parameter("controller.mpc.max_processing_delay_s", 0.5);
  declare_parameter("controller.mpc.yaw_feedforward_k_s", 0.0);

  // Deprecated aliases (for migration from old unscoped mpc delay keys)
  declare_parameter("mpc.control_delay_s", 0.0);
  declare_parameter("mpc.enable_delay_compensation", false);
  declare_parameter("mpc.prediction_delay_s", 0.0);
  declare_parameter("mpc.flight_time_iters", 2);
  declare_parameter("mpc.max_processing_delay_s", 0.5);

  // MPC 机动自适应权重衰减
  declare_parameter("controller.mpc.maneuver_adapt.enable",  false);
  declare_parameter("controller.mpc.maneuver_adapt.a_max",   3.0);
  declare_parameter("controller.mpc.maneuver_adapt.eta",     0.2);
  declare_parameter("controller.mpc.maneuver_adapt.tau",    10.0);
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
  declare_parameter("controller.mpc.vel_clamp.enable",          false);
  declare_parameter("controller.mpc.vel_clamp.max_linear_speed", 5.0);
  declare_parameter("controller.mpc.vel_clamp.max_v_yaw",        10.0);

  // MPC FOV 软约束
  declare_parameter("controller.mpc.fov_constraint.enable",                  false);
  declare_parameter("controller.mpc.fov_constraint.margin",                  0.05);
  declare_parameter("controller.mpc.fov_constraint.slack_weight",            1000.0);
  declare_parameter("controller.mpc.fov_constraint.constraint_steps",        0);
  declare_parameter("controller.mpc.fov_constraint.dynamic_margin.enable",   false);
  declare_parameter("controller.mpc.fov_constraint.dynamic_margin.vel_scale",0.01);
  declare_parameter("controller.mpc.fov_constraint.fallback_fov_yaw",        0.35);
  declare_parameter("controller.mpc.fov_constraint.fallback_fov_pitch",      0.26);

  // MPC 数值稳健性: 在线 RMS 归一化
  declare_parameter("controller.mpc.normalization.enable", false);
  declare_parameter("controller.mpc.normalization.window_size", 80);
  declare_parameter("controller.mpc.normalization.min_samples", 10);
  declare_parameter("controller.mpc.normalization.rms_epsilon", 1e-6);

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
  // 3. 滑动窗口均值
  declare_parameter("controller.output_filter.enable_moving_average",   false);
  declare_parameter("controller.output_filter.moving_average_window_size", 3);
  // 4. EMA
  declare_parameter("controller.output_filter.enable_ema",              false);
  declare_parameter("controller.output_filter.ema_alpha",               0.7);
  // 5. 1-Euro 自适应滤波
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

  c.maneuver.enable = get_parameter("maneuver.enable").as_bool();
  c.maneuver.nis_threshold_single =
      get_parameter("maneuver.nis_threshold_single").as_double();
  c.maneuver.nis_threshold_dual =
      get_parameter("maneuver.nis_threshold_dual").as_double();
  c.maneuver.innov_norm_threshold_single =
      get_parameter("maneuver.innov_norm_threshold_single").as_double();
  c.maneuver.innov_norm_threshold_dual =
      get_parameter("maneuver.innov_norm_threshold_dual").as_double();

  c.panel_mismatch.enable =
      get_parameter("panel_mismatch.enable").as_bool();
  c.panel_mismatch.window_size =
      get_parameter("panel_mismatch.window_size").as_int();
  c.panel_mismatch.threshold_t1 =
      get_parameter("panel_mismatch.threshold_t1").as_double();
  c.panel_mismatch.confirm_count =
      get_parameter("panel_mismatch.confirm_count").as_int();
  c.panel_mismatch.reinit_count =
      get_parameter("panel_mismatch.reinit_count").as_int();

    c.outpost.translation_model = translation_model_from_string(
      get_parameter("outpost.translation_model").as_string());
    c.outpost.rotation_model = rotation_model_from_string(
      get_parameter("outpost.rotation_model").as_string());
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
    c.outpost.panel_angle_step =
      get_parameter("outpost.panel_angle_step").as_double();
    c.outpost.softmax_temperature =
      get_parameter("outpost.softmax_temperature").as_double();
    c.outpost.weight_yaw = get_parameter("outpost.weight_yaw").as_double();
    c.outpost.weight_z_state =
      get_parameter("outpost.weight_z_state").as_double();
    c.outpost.weight_z_history =
      get_parameter("outpost.weight_z_history").as_double();
    c.outpost.entropy_enter =
      get_parameter("outpost.entropy_enter").as_double();
    c.outpost.entropy_exit =
      get_parameter("outpost.entropy_exit").as_double();
    c.outpost.max_prob_enter =
      get_parameter("outpost.max_prob_enter").as_double();
    c.outpost.max_prob_exit =
      get_parameter("outpost.max_prob_exit").as_double();
    c.outpost.stable_frames =
      get_parameter("outpost.stable_frames").as_int();
    c.outpost.z_history_window =
      get_parameter("outpost.z_history_window").as_int();
    c.outpost.single_mode_confidence_scale =
      get_parameter("outpost.single_mode_confidence_scale").as_double();
    c.outpost.alpha_pos = get_parameter("outpost.alpha_pos").as_double();
    c.outpost.beta_vel = get_parameter("outpost.beta_vel").as_double();
    c.outpost.alpha_yaw = get_parameter("outpost.alpha_yaw").as_double();
    c.outpost.beta_yaw_rate =
      get_parameter("outpost.beta_yaw_rate").as_double();
    c.outpost.assume_static_center =
      get_parameter("outpost.assume_static_center").as_bool();
    c.outpost.linear_velocity_damping =
      get_parameter("outpost.linear_velocity_damping").as_double();
    c.outpost.yaw_rate_damping =
      get_parameter("outpost.yaw_rate_damping").as_double();
    c.outpost.max_center_speed =
      get_parameter("outpost.max_center_speed").as_double();
    c.outpost.max_yaw_rate =
      get_parameter("outpost.max_yaw_rate").as_double();

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

  // Outlier filter parameters
  smoother_config_.enable_outlier_filter =
      get_parameter("smoother.enable_outlier_filter").as_bool();
  smoother_config_.outlier_method =
      get_parameter("smoother.outlier_method").as_string();
  smoother_config_.outlier_window_size =
      get_parameter("smoother.outlier_window_size").as_int();
  smoother_config_.outlier_min_samples =
      get_parameter("smoother.outlier_min_samples").as_int();
  smoother_config_.outlier_mad_k =
      get_parameter("smoother.outlier_mad_k").as_double();
  smoother_config_.outlier_iqr_k =
      get_parameter("smoother.outlier_iqr_k").as_double();
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

void GimbalPipelineNode::armorsCallback(
    const rm_interfaces::msg::Armors::SharedPtr msg) {
  rclcpp::Time msg_time(msg->header.stamp);
  double current_time = msg_time.seconds();
  if (msg->armors.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received empty armors message, running missing-target update");
  }

  // ── Step 1: Predict all existing trackers ──
  tracker_manager_->predict_all(current_time);
  auto removed_stale = tracker_manager_->remove_stale(current_time);
  if (!removed_stale.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu stale trackers", removed_stale.size());
  }

  // ── Step 2: Group observations by robot ID ──
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  std::string sf =
      msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;

  const bool strict_unknown_reject =
      robot_description_facade_ && robot_description_facade_->strictUnknownReject();

  for (const auto &armor : msg->armors) {
    const bool supported_robot_id =
        robot_description_facade_ &&
        robot_description_facade_->isSupportedRobotId(armor.number);

    // Strict mode: skip unsupported IDs to prevent false tracker creation.
    if (strict_unknown_reject && !supported_robot_id) {
      if (debug_mode_)
        RCLCPP_WARN(get_logger(), "Ignoring armor with invalid ID: '%s'",
                    armor.number.c_str());
      continue;
    }
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
        double dza = t->spin_filter().get_dza();
        sm.initialize(r1, r2, dza);
        smoothers_.emplace(rid, std::move(sm));

        // Co-initialise outlier filter for this robot
        OutlierFilterConfig ocfg;
        ocfg.enable          = smoother_config_.enable_outlier_filter;
        ocfg.method          = smoother_config_.outlier_method;
        ocfg.window_size     = smoother_config_.outlier_window_size;
        ocfg.min_samples     = smoother_config_.outlier_min_samples;
        ocfg.mad_k           = smoother_config_.outlier_mad_k;
        ocfg.iqr_k           = smoother_config_.outlier_iqr_k;
        ocfg.mahal_threshold = smoother_config_.outlier_mahal_threshold;
        outlier_filters_.emplace(rid, ObservationOutlierFilter(ocfg));
      }
    }
  }

  // ── Step 3.5: Notify trackers that did NOT receive observations this frame ──
  // This drives the state machine: TRACKING → TEMP_LOST → LOST for
  // missing targets, preventing "ghost tracking" of disappeared targets.
  {
    std::set<std::string> observed_ids;
    for (const auto &[rid, _] : obs_by_robot) {
      observed_ids.insert(rid);
    }
    tracker_manager_->notify_missing(observed_ids, current_time);
  }
  auto removed_lost = tracker_manager_->remove_lost();
  if (!removed_lost.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu lost trackers", removed_lost.size());
  }

  // Clean up per-robot caches for trackers that no longer exist.
  std::vector<std::string> removed_ids;
  removed_ids.reserve(removed_stale.size() + removed_lost.size());
  removed_ids.insert(removed_ids.end(), removed_stale.begin(), removed_stale.end());
  removed_ids.insert(removed_ids.end(), removed_lost.begin(), removed_lost.end());
  for (const auto &rid : removed_ids) {
    if (tracker_manager_->get(rid) != nullptr) {
      continue;
    }
    smoothers_.erase(rid);
    last_dual_obs_.erase(rid);
    outlier_filters_.erase(rid);
    last_smoothed_outputs_.erase(rid);
  }

  // ── Step 4: Build TrackedRobots message (internal) ──
  auto tracked_msg = buildTrackedRobotsMsg(msg->header);

  // ── Log tracker posterior states (after update, before selection) ──
  if (prediction_logger_) {
    int64_t ts_ns = msg_time.nanoseconds();
    for (const auto &robot : tracked_msg.robots) {
      const auto center_position = robot_description::TrackedRobotUsage::centerPosition(robot);
      const auto linear_velocity = robot_description::TrackedRobotUsage::linearVelocity(robot);
      LogTrackerState st;
      st.center_x           = center_position.x();
      st.center_y           = center_position.y();
      st.center_z           = center_position.z();
      st.vel_x              = linear_velocity.x();
      st.vel_y              = linear_velocity.y();
      st.vel_z              = linear_velocity.z();
      st.yaw                = robot_description::TrackedRobotUsage::yaw(robot);
      st.yaw_velocity       = robot_description::TrackedRobotUsage::yawVelocity(robot);
      st.yaw_acceleration   = robot_description::TrackedRobotUsage::yawAcceleration(robot);
      st.radius_1           = robot.radius;
      st.radius_2           = robot.radius_2;
      st.dza                = robot.d_za;
      st.track_state        = robot.track_state;
      st.num_armors         = robot.num_armors;
      st.visible_armor_count = robot.visible_armor_count;
      st.is_visible         = robot.is_visible;
      st.confidence         = robot.confidence;

      // ── 机动检测指标：从对应 tracker 的 UKF 内部读取 ──
      auto *tracker = tracker_manager_->get(robot.robot_id);
      if (tracker && tracker->is_initialized()) {
        const auto &ukf = tracker->spin_filter();
        const auto &idx = ukf.state_idx();
        const auto &xv  = ukf.x();
        const auto &Pv  = ukf.P();

        // 创新向量
        const auto &iv = ukf.last_innov_xyz();
        if (iv.size() >= 3) {
          st.innov_x = iv(0);
          st.innov_y = iv(1);
          st.innov_z = iv(2);
        }
        st.innov_yaw   = ukf.last_innov_yaw();
        st.nis         = ukf.last_nis();
        st.update_type = ukf.last_update_type();

        // P 对角线 —— 位置与速度
        st.p_var_x  = Pv(idx.X(),  idx.X());
        st.p_var_y  = Pv(idx.Y(),  idx.Y());
        st.p_var_z  = Pv(idx.Z(),  idx.Z());
        st.p_var_vx = Pv(idx.VX(), idx.VX());
        st.p_var_vy = Pv(idx.VY(), idx.VY());
        st.p_var_vz = Pv(idx.VZ(), idx.VZ());

        // 加速度状态（仅 CA / Singer 过程模型存在 AX/AY/AZ）
        constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
        if (idx.has("AX")) {
          st.p_var_ax = Pv(idx.AX(), idx.AX());
          st.p_var_ay = Pv(idx.AY(), idx.AY());
          st.p_var_az = Pv(idx.AZ(), idx.AZ());
          st.accel_x  = xv(idx.AX());
          st.accel_y  = xv(idx.AY());
          st.accel_z  = xv(idx.AZ());
          st.accel_magnitude = std::sqrt(xv(idx.AX()) * xv(idx.AX()) +
                                         xv(idx.AY()) * xv(idx.AY()) +
                                         xv(idx.AZ()) * xv(idx.AZ()));
        } else {
          st.p_var_ax = kNaN;
          st.p_var_ay = kNaN;
          st.p_var_az = kNaN;
          st.accel_x         = kNaN;
          st.accel_y         = kNaN;
          st.accel_z         = kNaN;
          st.accel_magnitude = kNaN;
        }
      }

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

    if (debug_maneuver_pub_) {
      publishManeuverMarkers(msg->header);
    }
  }

  // ── Step 8: Publish maneuver states (always-on, for chart monitoring) ──
  if (maneuver_states_pub_) {
    rm_interfaces::msg::ManeuverStates states_msg;
    states_msg.header.stamp    = msg->header.stamp;
    states_msg.header.frame_id = target_frame_;
    for (const auto &[robot_id, entry] : tracker_manager_->trackers()) {
      if (!entry.tracker || !entry.tracker->is_initialized()) continue;
      const auto result = entry.tracker->assess_maneuver();
      const auto &ukf   = entry.tracker->spin_filter();
      rm_interfaces::msg::ManeuverState s;
      s.robot_id        = robot_id;
      s.is_maneuvering  = result.is_maneuvering;
      s.nis             = result.nis;
      s.innov_norm      = result.innov_norm;
      s.innov_yaw_abs   = std::abs(ukf.last_innov_yaw());
      s.update_type     = result.update_type;
      states_msg.states.push_back(s);
    }
    maneuver_states_pub_->publish(states_msg);
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

  auto tracking_ids = tracker_manager_->active_robot_ids();
  for (const auto &rid : tracking_ids) {
    auto *tracker = tracker_manager_->get(rid);
    if (!tracker || (!tracker->is_tracking() && !tracker->is_temp_lost())) continue;

    // ── Extract raw tracker state ─────────────────────────────────
    const auto pos  = tracker->get_center_position();
    const auto &filter = tracker->spin_filter();
    const auto idx  = filter.state_idx();
    const auto &x   = filter.x();
    Eigen::Vector3d vel = tracker->get_publish_velocity();
    const double yaw   = tracker->get_yaw();
    const double v_yaw = x(idx.DELTA_RATE());
    const auto [r1, r2] = tracker->get_radii();
    const double dza  = filter.get_dza();

    bool is_dual = false;
    {
      auto dual_it = last_dual_obs_.find(rid);
      if (dual_it != last_dual_obs_.end()) is_dual = dual_it->second;
    }
    const rclcpp::Time stamp(header.stamp);
    const double ts = stamp.seconds();

    // ── Stage 1: Outlier detection (independent of smoother.enable) ──
    // If an outlier is detected, hold the last valid SmoothedOutput so
    // the One-Euro filter internal state is never corrupted by a jump.
    SmoothedOutput smoothed;
    bool has_smoothed = false;
    bool is_outlier   = false;

    if (smoother_config_.enable_outlier_filter) {
      auto of_it = outlier_filters_.find(rid);
      if (of_it != outlier_filters_.end()) {
        is_outlier = of_it->second.update(pos, yaw);
      }
    }

    if (is_outlier) {
      // Hold strategy: reuse last valid smoothed output
      auto prev_it = last_smoothed_outputs_.find(rid);
      if (prev_it != last_smoothed_outputs_.end()) {
        smoothed     = prev_it->second;
        has_smoothed = true;
      }
      // If no previous smoothed output exists (outlier on very first frame)
      // fall through with has_smoothed = false → raw output is used downstream
    } else {
      // ── Stage 2: Output smoothing (independent switch) ────────────
      auto sm_it = smoothers_.find(rid);
      if (sm_it != smoothers_.end() && smoother_config_.enable) {
        smoothed = sm_it->second.smooth(pos, yaw, vel, v_yaw, r1, r2, dza,
                                         is_dual, ts);
        has_smoothed = true;
      }
      // Update hold-cache with valid (non-outlier) smoothed result
      if (has_smoothed) {
        last_smoothed_outputs_[rid] = smoothed;
      }
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

    if (robot.robot_id.empty()) {
      continue;
    }

    tracked_msg.robots.push_back(robot);
  }

  return tracked_msg;
}

/* ================================================================ */
/*  Target message builders (from MaxEntropyTrackerNode)             */
/* ================================================================ */

rm_interfaces::msg::Target GimbalPipelineNode::buildTargetMessage(
    const std_msgs::msg::Header &header, const std::string &robot_id,
  BaseTracker &tracker, const SmoothedOutput *smoothed) {
  rm_interfaces::msg::Target target;
  target.header = header;
  target.header.frame_id = target_frame_;
  target.tracking = true;
  target.id = robot_id;

  // Keep debug target semantic aligned with tracked robot profile.
  target.armors_num = 4;
  if (robot_id == "outpost" || robot_id == "base") {
    target.armors_num = 3;
  }
  const int runtime_num_armors = tracker.effective_num_armors();
  if (runtime_num_armors > 0) {
    target.armors_num = runtime_num_armors;
  }

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
    const auto &filter = tracker.spin_filter();
    auto idx = filter.state_idx();
    const auto &x = filter.x();
    const auto pub_vel = tracker.get_publish_velocity();
    target.velocity.x = pub_vel.x();
    target.velocity.y = pub_vel.y();
    target.velocity.z = pub_vel.z();
    target.yaw = tracker.get_yaw();
    target.v_yaw = x(idx.DELTA_RATE());
    auto [r1, r2] = tracker.get_radii();
    target.radius_1 = r1;
    target.radius_2 = r2;
    target.d_za = filter.get_dza();
  }

  target.d_zc = 0.0;
  target.yaw_diff = 0.0;
  target.position_diff = 0.0;
  return target;
}

rm_interfaces::msg::TrackedRobot GimbalPipelineNode::buildTrackedRobotMessage(
    const std_msgs::msg::Header &header, const std::string &robot_id,
  BaseTracker &tracker, const SmoothedOutput *smoothed) {
  rm_interfaces::msg::TrackedRobot empty_msg;

  if (!robot_description_facade_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "RobotDescriptionFacade is not initialized, skip TrackedRobot build");
    return empty_msg;
  }

  int visible_armor_count = 0;
  auto obs_it = last_obs_counts_.find(robot_id);
  if (obs_it != last_obs_counts_.end()) {
    visible_armor_count = obs_it->second;
  }

  robot_description::TrackedRobotBuildInput input{
    header,
    target_frame_,
    robot_id,
    tracker,
    smoothed,
    visible_armor_count};

  auto build_result = robot_description_facade_->tryBuildTrackedRobot(input);
  if (!build_result.ok()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected TrackedRobot build for id='%s': %s",
      robot_id.c_str(),
      build_result.reason.c_str());
    return empty_msg;
  }

  return build_result.robot;
}

/* ================================================================ */
/*  Target selection (internal, no ROS topic)                        */
/* ================================================================ */

void GimbalPipelineNode::initSelectionStrategy() {
  if (selector_strategy_name_ == "min_yaw_deviation") {
    selection_strategy_ = std::make_unique<MinYawDeviationStrategy>();
  } else if (selector_strategy_name_ == "priority_list") {
    selection_strategy_ = std::make_unique<PriorityListStrategy>();
  } else if (selector_strategy_name_ == "sticky_min_yaw_deviation") {
    selection_strategy_ = std::make_unique<StickyMinYawDeviationStrategy>();
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
  fire_advice_engine_ = std::make_shared<gimbal_controller::FireAdviceEngine>();
  fire_advice_engine_->setComponents(
    position_calculator_, ballistic_client_, local_compensator_, fire_advisor_);
  gimbal_control_core_ = std::make_shared<gimbal_controller::GimbalControlCore>();
  gimbal_control_core_->setFireModules(fire_advice_engine_, fire_advisor_);
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
  mpc_s->initReferenceGenerator();

  const double mpc_control_delay_s = readCompatDoubleParameter(
    *this, "controller.mpc.control_delay_s", "mpc.control_delay_s");
  const bool mpc_enable_delay_compensation = readCompatBoolParameter(
    *this, "controller.mpc.enable_delay_compensation", "mpc.enable_delay_compensation");
  const double mpc_prediction_delay_s = readCompatDoubleParameter(
    *this, "controller.mpc.prediction_delay_s", "mpc.prediction_delay_s");
  double mpc_trigger_to_muzzle_s = readCompatDoubleParameter(
    *this, "controller.solver.trigger_to_muzzle_s", "solver.trigger_to_muzzle_s");
  if (hasParameterOverride(*this, "controller.fire.trigger_to_muzzle_s")) {
    mpc_trigger_to_muzzle_s = get_parameter("controller.fire.trigger_to_muzzle_s").as_double();
  }
  const int mpc_flight_time_iters = readCompatIntParameter(
    *this, "controller.mpc.flight_time_iters", "mpc.flight_time_iters");
  const double mpc_max_processing_delay_s = readCompatDoubleParameter(
    *this, "controller.mpc.max_processing_delay_s", "mpc.max_processing_delay_s");

  mpc_s->setMpcParameters(
    get_parameter("controller.mpc.N").as_int(),
    get_parameter("controller.mpc.dt").as_double(),
    mpc_control_delay_s,
    get_parameter("controller.mpc.max_accel").as_double(),
    get_parameter("controller.mpc.q_yaw").as_double(),
    get_parameter("controller.mpc.q_pitch").as_double(),
    get_parameter("controller.mpc.q_yaw_vel").as_double(),
    get_parameter("controller.mpc.q_pitch_vel").as_double(),
    get_parameter("controller.mpc.r_yaw").as_double(),
    get_parameter("controller.mpc.r_pitch").as_double(),
    get_parameter("controller.mpc.s_yaw").as_double(),
    get_parameter("controller.mpc.s_pitch").as_double());
  mpc_s->setDelayCompensation(
    mpc_enable_delay_compensation,
    mpc_prediction_delay_s,
    mpc_trigger_to_muzzle_s,
    mpc_flight_time_iters,
    mpc_max_processing_delay_s);
  mpc_s->setYawFeedforward(
    get_parameter("controller.mpc.yaw_feedforward_k_s").as_double());
  mpc_s->setManeuverAdaptParameters(
    get_parameter("controller.mpc.maneuver_adapt.enable").as_bool(),
    get_parameter("controller.mpc.maneuver_adapt.a_max").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.eta").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.tau").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.r_scale").as_double());
  mpc_s->setWeightingParameters(
    get_parameter("controller.mpc.weighting.enable").as_bool(),
    get_parameter("controller.mpc.weighting.alpha").as_double(),
    get_parameter("controller.mpc.weighting.k_omega").as_double(),
    get_parameter("controller.mpc.weighting.sigma_min").as_double(),
    get_parameter("controller.mpc.weighting.sigma_max").as_double(),
    get_parameter("controller.mpc.weighting.sigma_sys").as_double(),
    get_parameter("controller.mpc.weighting.target_size").as_double(),
    get_parameter("controller.mpc.weighting.delay_s").as_double(),
    get_parameter("controller.mpc.weighting.max_w").as_double(),
    get_parameter("controller.mpc.weighting.smooth_alpha").as_double(),
    get_parameter("controller.mpc.weighting.min_distance").as_double(),
    get_parameter("controller.bullet_speed").as_double(),
    get_parameter("controller.mpc.weighting.sigma_beta").as_double(),
    get_parameter("controller.mpc.weighting.gamma").as_double());
  {
    gimbal_controller::mpc::VelocityClampConfig vel_clamp_cfg;
    vel_clamp_cfg.enable =
      get_parameter("controller.mpc.vel_clamp.enable").as_bool();
    vel_clamp_cfg.max_linear_speed =
      get_parameter("controller.mpc.vel_clamp.max_linear_speed").as_double();
    vel_clamp_cfg.max_v_yaw =
      get_parameter("controller.mpc.vel_clamp.max_v_yaw").as_double();
    mpc_s->setVelocityClamp(vel_clamp_cfg);
  }
  mpc_s->setFovConstraintParameters(
    get_parameter("controller.mpc.fov_constraint.enable").as_bool(),
    get_parameter("controller.mpc.fov_constraint.margin").as_double(),
    get_parameter("controller.mpc.fov_constraint.slack_weight").as_double(),
    get_parameter("controller.mpc.fov_constraint.constraint_steps").as_int(),
    get_parameter("controller.mpc.fov_constraint.dynamic_margin.enable").as_bool(),
    get_parameter("controller.mpc.fov_constraint.dynamic_margin.vel_scale").as_double(),
    get_parameter("controller.mpc.fov_constraint.fallback_fov_yaw").as_double(),
    get_parameter("controller.mpc.fov_constraint.fallback_fov_pitch").as_double());
  mpc_s->setNumericalNormalizationParameters(
    get_parameter("controller.mpc.normalization.enable").as_bool(),
    get_parameter("controller.mpc.normalization.window_size").as_int(),
    get_parameter("controller.mpc.normalization.min_samples").as_int(),
    get_parameter("controller.mpc.normalization.rms_epsilon").as_double());
  mpc_s->setHessianRegularizationParameters(
    get_parameter("controller.mpc.regularization.enable").as_bool(),
    get_parameter("controller.mpc.regularization.epsilon_abs").as_double(),
    get_parameter("controller.mpc.regularization.epsilon_rel").as_double(),
    get_parameter("controller.mpc.regularization.epsilon_max").as_double(),
    get_parameter("controller.mpc.regularization.retry_on_fail").as_bool(),
    get_parameter("controller.mpc.regularization.retry_scale").as_double());
  mpc_s->setDiagnosticsParameters(
    get_parameter("controller.mpc.diagnostics.enable").as_bool(),
    get_parameter("controller.mpc.diagnostics.low_cost_always").as_bool(),
    get_parameter("controller.mpc.diagnostics.high_cost_enable").as_bool(),
    get_parameter("controller.mpc.diagnostics.high_cost_sample_every").as_int(),
    get_parameter("controller.mpc.diagnostics.log_every").as_int(),
    get_parameter("controller.mpc.diagnostics.log_on_failure").as_bool(),
    get_parameter("controller.mpc.diagnostics.active_tol").as_double(),
    get_parameter("controller.mpc.diagnostics.rank_tol_rel").as_double());
  gimbal_strategies_["mpc"] = mpc_s;

  auto sm_s = std::make_shared<gimbal_controller::StateMachineStrategy>();
  sm_s->setComponents(position_calculator_, armor_selector_,
                      ballistic_client_, local_compensator_, fire_advisor_);
  gimbal_strategies_["state_machine"] = sm_s;

  if (gimbal_control_core_) {
    gimbal_control_core_->setStrategies(&gimbal_strategies_);
  }
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

void GimbalPipelineNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (msg->k[0] < 1e-6 || msg->k[4] < 1e-6 || msg->width == 0 || msg->height == 0) {
    return;  // 无效的相机内参
  }
  double fx = msg->k[0];
  double fy = msg->k[4];
  double fov_half_yaw = std::atan(static_cast<double>(msg->width) / (2.0 * fx));
  double fov_half_pitch = std::atan(static_cast<double>(msg->height) / (2.0 * fy));

  // 通过核心类透传 FOV 更新，避免 node 直接耦合具体策略实现。
  if (gimbal_control_core_) {
    gimbal_control_core_->updateFov(fov_half_yaw, fov_half_pitch);
  }

  RCLCPP_INFO_ONCE(get_logger(),
      "[FOV] camera_info received: fov_yaw=%.1f° fov_pitch=%.1f° (fx=%.1f fy=%.1f %dx%d)",
      fov_half_yaw * 2.0 * 180.0 / M_PI, fov_half_pitch * 2.0 * 180.0 / M_PI,
      fx, fy, msg->width, msg->height);
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

void GimbalPipelineNode::buildControlContextFromCache(
    gimbal_controller::GimbalControlContext & context,
    std::string & selected_id) {
  context.is_tracking = false;
  context.is_temp_lost = false;
  context.is_maneuvering = false;

  // Read shared state (thread-safe)
  rm_interfaces::msg::TrackedRobots::SharedPtr robots;
  rclcpp::Time data_update_time{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    robots = latest_tracked_robots_;
    selected_id = latest_selected_target_id_;
    data_update_time = latest_update_time_;
  }

  if (!robots || robots->robots.empty()) {
    return;
  }

  // Cache freshness check: stale target cache should not drive control.
  const double data_age = (context.current_time - data_update_time).seconds();
  const double max_data_age = std::max(tracker_timeout_s_, 1e-3);
  if (data_age > max_data_age) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Stale tracking data (age=%.3fs > %.3fs), ignoring",
      data_age, max_data_age);
    return;
  }

  const rm_interfaces::msg::TrackedRobot * selected_robot = nullptr;
  if (!selected_id.empty()) {
    for (const auto & robot : robots->robots) {
      if (robot.robot_id == selected_id) {
        selected_robot = &robot;
        break;
      }
    }
  } else {
    selected_robot = &robots->robots[0];
    selected_id = selected_robot->robot_id;
  }

  if (!selected_robot) {
    return;
  }

  context.target_robot = *selected_robot;
  context.target_stamp = data_update_time;
  context.is_tracking =
      (selected_robot->track_state == rm_interfaces::msg::TrackedRobot::TRACKING);
  context.is_temp_lost =
      (selected_robot->track_state == rm_interfaces::msg::TrackedRobot::TEMP_LOST);

  auto * tracker = tracker_manager_->get(selected_robot->robot_id);
  context.is_maneuvering = (tracker && tracker->is_initialized()) ?
    tracker->assess_maneuver().is_maneuvering : false;
}

void GimbalPipelineNode::publishDelayAuditDebug(
    const gimbal_controller::GimbalControlContext & context,
    const gimbal_controller::DelayAuditSnapshot & audit,
    const std::string & strategy_name) {
  if (!debug_delay_audit_pub_) {
    return;
  }

  rm_interfaces::msg::DelayAudit msg;
  msg.header.stamp = context.current_time;
  msg.header.frame_id = target_frame_;
  msg.strategy_name = audit.strategy_name.empty() ? strategy_name : audit.strategy_name;
  msg.valid = audit.valid;
  msg.tracking = audit.tracking;
  msg.processing_delay_s = audit.processing_delay_s;
  msg.prediction_extra_s = audit.prediction_extra_s;
  msg.flight_time_s = audit.flight_time_s;
  msg.total_prediction_time_s = audit.total_prediction_time_s;
  msg.control_latency_s = audit.control_latency_s;
  msg.fire_control_compensation_s = audit.fire_control_compensation_s;
  msg.control_delay_steps = audit.control_delay_steps;
  msg.uses_delayed_b = audit.uses_delayed_b;
  msg.double_compensation_risk = audit.double_compensation_risk;
  debug_delay_audit_pub_->publish(msg);
}

/* ================================================================ */
/*  Timer callback — 250 Hz control loop                             */
/* ================================================================ */

void GimbalPipelineNode::timerCallback() {
  // Step 0: 核心类可用性检查
  if (!gimbal_control_core_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "GimbalControlCore is not initialized, skipping control cycle");
    return;
  }

  gimbal_controller::GimbalControlContext context;
  context.current_time = now();

  // Step 1: 控制禁用时发布 idle 命令并早返回
  if (!enable_) {
    const auto idle_result = gimbal_control_core_->compute(
      context, current_gimbal_strategy_name_, std::string(), false);
    gimbal_cmd_pub_->publish(idle_result.cmd);
    return;
  }

  // Step 2: 更新云台姿态并填充控制上下文基础字段
  updateGimbalState();
  context.current_yaw = current_yaw_;
  context.current_pitch = current_pitch_;
  context.bullet_speed = bullet_speed_;

  // Step 3: 从共享缓存构建目标上下文
  std::string selected_id;
  buildControlContextFromCache(context, selected_id);

  // Step 4: 核心类统一生成命令（strategy + finalize + filter + audit）
  const auto control_result = gimbal_control_core_->compute(
    context, current_gimbal_strategy_name_, selected_id, true);
  if (!control_result.strategy_found) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Gimbal strategy '%s' not found, fallback to idle cmd",
      current_gimbal_strategy_name_.c_str());
  }

  // Step 5: 发布控制命令
  gimbal_cmd_pub_->publish(control_result.cmd);

  // Step 6: 发布调试信息（audit + marker）
  if (debug_mode_ && debug_delay_audit_pub_) {
    publishDelayAuditDebug(
      context,
      control_result.delay_audit,
      current_gimbal_strategy_name_);
  }

  if (debug_mode_ && control_result.has_tracking) {
    publishGimbalMarkers(context.target_robot, control_result.cmd);
  }
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

  radial_allowed_arc_marker_.ns = "radial_allowed_arc";
  radial_allowed_arc_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  radial_allowed_arc_marker_.scale.x = 0.018;
  radial_allowed_arc_marker_.color.a = 0.95;
  radial_allowed_arc_marker_.color.r = 1.0;
  radial_allowed_arc_marker_.color.g = 0.6;
  radial_allowed_arc_marker_.color.b = 0.0;

  radial_allowed_bounds_marker_.ns = "radial_allowed_bounds";
  radial_allowed_bounds_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  radial_allowed_bounds_marker_.scale.x = 0.012;
  radial_allowed_bounds_marker_.color.a = 0.95;
  radial_allowed_bounds_marker_.color.r = 1.0;
  radial_allowed_bounds_marker_.color.g = 0.85;
  radial_allowed_bounds_marker_.color.b = 0.2;

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

  const auto normalized_target = robot_description::TrackedRobotUsage::normalizeState(target_robot);
  const auto center_position = robot_description::TrackedRobotUsage::centerPosition(normalized_target);
  const auto linear_velocity = robot_description::TrackedRobotUsage::linearVelocity(normalized_target);
  const double target_yaw = robot_description::TrackedRobotUsage::yaw(normalized_target);
  const double target_yaw_velocity =
      robot_description::TrackedRobotUsage::yawVelocity(normalized_target);

  visualization_msgs::msg::MarkerArray marker_array;
  const bool has_valid_measurement =
    cmd.mode == rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT &&
    cmd.distance > 0.0;

  // Position
  position_marker_.header = target_robot.header;
  position_marker_.id = 0;
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  position_marker_.pose.position =
    robot_description::TrackedRobotUsage::toPoint(center_position);
  position_marker_.pose.orientation.w = 1.0;
  marker_array.markers.push_back(position_marker_);

  // Velocity arrow
  target_velocity_marker_.header = target_robot.header;
  target_velocity_marker_.id = 0;
  target_velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
  target_velocity_marker_.points.clear();
  geometry_msgs::msg::Point vel_start =
    robot_description::TrackedRobotUsage::toPoint(center_position);
  geometry_msgs::msg::Point vel_end = vel_start;
  vel_end.x += linear_velocity.x() * 0.5;
  vel_end.y += linear_velocity.y() * 0.5;
  vel_end.z += linear_velocity.z() * 0.5;
  target_velocity_marker_.points.push_back(vel_start);
  target_velocity_marker_.points.push_back(vel_end);
  marker_array.markers.push_back(target_velocity_marker_);

  // Armor plates
  if (!normalized_target.armors_offset.empty()) {
    for (size_t i = 0; i < normalized_target.armors_offset.size(); ++i) {
      auto armor_marker = armors_marker_;
      armor_marker.header = target_robot.header;
      armor_marker.id = static_cast<int>(i);
      armor_marker.action = visualization_msgs::msg::Marker::ADD;
      double cos_yaw = std::cos(target_yaw);
      double sin_yaw = std::sin(target_yaw);
      const auto &offset = normalized_target.armors_offset[i];
      armor_marker.pose.position.x =
          center_position.x() +
          offset.position.x * cos_yaw - offset.position.y * sin_yaw;
      armor_marker.pose.position.y =
          center_position.y() +
          offset.position.x * sin_yaw + offset.position.y * cos_yaw;
      armor_marker.pose.position.z =
          center_position.z() + offset.position.z;
      tf2::Quaternion q;
      q.setRPY(0, 0.2618,
               target_yaw +
                   i * (2 * M_PI / normalized_target.num_armors));
      armor_marker.pose.orientation.x = q.x();
      armor_marker.pose.orientation.y = q.y();
      armor_marker.pose.orientation.z = q.z();
      armor_marker.pose.orientation.w = q.w();
      marker_array.markers.push_back(armor_marker);
    }
  }

  // Allowed radial selection range marker (for min_movement_with_radial)
  if (radial_selection_enabled_ && armor_selector_ && normalized_target.num_armors > 0) {
    const double center_x = center_position.x();
    const double center_y = center_position.y();
    const double center_z = center_position.z();

    // Direction from robot center to our gimbal origin (world origin approximation).
    double axis_yaw = std::atan2(-center_y, -center_x);

    double enter_deg = facing_enter_angle_deg_;
    double speed_norm = 0.0;
    double bias_deg = 0.0;
    if (radial_dynamic_enable_) {
      speed_norm = std::clamp(
        std::abs(target_yaw_velocity) / radial_dynamic_v_yaw_ref_, 0.0, 1.0);
      const double scale = 1.0 - radial_dynamic_shrink_ratio_ * speed_norm;
      enter_deg = std::max(enter_deg * scale, radial_dynamic_min_angle_deg_);
      const double bias_mag = std::min(
        radial_dynamic_bias_gain_deg_ * speed_norm,
        radial_dynamic_max_bias_deg_);
      bias_deg = (target_yaw_velocity >= 0.0 ? 1.0 : -1.0) * bias_mag;
      axis_yaw += bias_deg * M_PI / 180.0;
    }
    const double enter_rad = enter_deg * M_PI / 180.0;

    double radius = 0.25;
    for (const auto & offset : normalized_target.armors_offset) {
      const double r = std::hypot(offset.position.x, offset.position.y);
      if (r > radius) {
        radius = r;
      }
    }
    radius = std::clamp(radius * 1.2, 0.2, 0.8);

    radial_allowed_arc_marker_.header = target_robot.header;
    radial_allowed_arc_marker_.id = 100;
    radial_allowed_arc_marker_.action = visualization_msgs::msg::Marker::ADD;
    radial_allowed_arc_marker_.points.clear();

    constexpr int kArcSamples = 48;
    for (int i = 0; i <= kArcSamples; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(kArcSamples);
      const double yaw = axis_yaw - enter_rad + 2.0 * enter_rad * t;
      geometry_msgs::msg::Point p;
      p.x = center_x + radius * std::cos(yaw);
      p.y = center_y + radius * std::sin(yaw);
      p.z = center_z + 0.08;
      radial_allowed_arc_marker_.points.push_back(p);
    }
    marker_array.markers.push_back(radial_allowed_arc_marker_);

    radial_allowed_bounds_marker_.header = target_robot.header;
    radial_allowed_bounds_marker_.id = 101;
    radial_allowed_bounds_marker_.action = visualization_msgs::msg::Marker::ADD;
    radial_allowed_bounds_marker_.points.clear();

    geometry_msgs::msg::Point c;
    c.x = center_x;
    c.y = center_y;
    c.z = center_z + 0.08;

    geometry_msgs::msg::Point p_min;
    p_min.x = center_x + radius * std::cos(axis_yaw - enter_rad);
    p_min.y = center_y + radius * std::sin(axis_yaw - enter_rad);
    p_min.z = center_z + 0.08;

    geometry_msgs::msg::Point p_max;
    p_max.x = center_x + radius * std::cos(axis_yaw + enter_rad);
    p_max.y = center_y + radius * std::sin(axis_yaw + enter_rad);
    p_max.z = center_z + 0.08;

    geometry_msgs::msg::Point p_axis;
    p_axis.x = center_x + radius * std::cos(axis_yaw);
    p_axis.y = center_y + radius * std::sin(axis_yaw);
    p_axis.z = center_z + 0.08;

    radial_allowed_bounds_marker_.points.push_back(c);
    radial_allowed_bounds_marker_.points.push_back(p_min);
    radial_allowed_bounds_marker_.points.push_back(c);
    radial_allowed_bounds_marker_.points.push_back(p_max);
    radial_allowed_bounds_marker_.points.push_back(c);
    radial_allowed_bounds_marker_.points.push_back(p_axis);
    marker_array.markers.push_back(radial_allowed_bounds_marker_);
  }

  // Selection target
  if (has_valid_measurement) {
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
  if (current_gimbal_strategy_name_ == "predicted" && has_valid_measurement) {
    predicted_marker_.header = target_robot.header;
    predicted_marker_.id = 0;
    predicted_marker_.action = visualization_msgs::msg::Marker::ADD;
    predicted_marker_.pose = selection_marker_.pose;
    predicted_marker_.color.a = 0.6;
    marker_array.markers.push_back(predicted_marker_);
  }

  // Trajectory
  if (has_valid_measurement) {
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

void GimbalPipelineNode::publishManeuverMarkers(
    const std_msgs::msg::Header &header) {
  visualization_msgs::msg::MarkerArray arr;

  // Tracker positions are expressed in target_frame_ (== visualization_frame_).
  // Use visualization_frame_ explicitly to avoid the camera source frame mismatch.
  std_msgs::msg::Header viz_header;
  viz_header.stamp    = header.stamp;
  viz_header.frame_id = visualization_frame_;

  int id = 0;

  for (const auto &[robot_id, entry] : tracker_manager_->trackers()) {
    if (!entry.tracker || !entry.tracker->is_initialized()) continue;

    const auto result = entry.tracker->assess_maneuver();
    const auto pos    = entry.tracker->get_center_position();

    // Estimate robot top: center pos + half robot height (~0.25 m)
    const double top_z = pos.z() + 0.25;

    // ── Sphere marker (at robot top) ───────────────────────────
    visualization_msgs::msg::Marker sphere;
    sphere.header       = viz_header;
    sphere.ns           = "maneuver";
    sphere.id           = id++;
    sphere.type         = visualization_msgs::msg::Marker::SPHERE;
    sphere.action       = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = pos.x();
    sphere.pose.position.y = pos.y();
    sphere.pose.position.z = top_z;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.15);
    if (result.is_maneuvering) {
      sphere.color.r = 0.9f; sphere.color.g = 0.1f;
      sphere.color.b = 0.1f; sphere.color.a = 0.8f;
    } else {
      sphere.color.r = 0.1f; sphere.color.g = 0.9f;
      sphere.color.b = 0.1f; sphere.color.a = 0.6f;
    }
    arr.markers.push_back(sphere);

    // ── Text marker (above sphere) ─────────────────────────────
    visualization_msgs::msg::Marker text;
    text.header    = viz_header;
    text.ns        = "maneuver_text";
    text.id        = id++;
    text.type      = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action    = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = pos.x();
    text.pose.position.y = pos.y();
    text.pose.position.z = top_z + 0.15;
    text.pose.orientation.w = 1.0;
    text.scale.z   = 0.08;
    text.color.r   = 1.0f; text.color.g = 1.0f;
    text.color.b   = 1.0f; text.color.a = 1.0f;
    text.lifetime  = rclcpp::Duration::from_seconds(0.15);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "NIS=%.0f\nIN=%.3f",
                  result.nis, result.innov_norm);
    text.text = buf;
    arr.markers.push_back(text);
  }

  if (!arr.markers.empty()) debug_maneuver_pub_->publish(arr);
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
