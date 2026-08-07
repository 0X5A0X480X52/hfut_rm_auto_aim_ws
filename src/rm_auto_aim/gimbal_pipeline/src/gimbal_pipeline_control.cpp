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
#include "max_entropy_tracker/trackers/norm4_v3/tracker/norm4_tracker_v2.hpp"
#include "max_entropy_tracker/visualization.hpp"
#include "rm_utils/logger/log.hpp"

// Gimbal strategies
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

namespace fyt::auto_aim {

void GimbalPipelineNode::initGimbalComponents() {
  position_calculator_ = std::make_shared<gimbal_controller::ArmorPositionCalculator>();
  armor_selector_ = std::make_shared<gimbal_controller::ArmorSelector>();
  local_compensator_ = std::make_shared<gimbal_controller::LocalTrajectoryCompensator>();
  fire_advisor_ = std::make_shared<gimbal_controller::FireAdvisor>();
  fire_advice_engine_ = std::make_shared<gimbal_controller::FireAdviceEngine>();
  fire_advice_engine_->setComponents(position_calculator_, local_compensator_, fire_advisor_);
  gimbal_control_core_ = std::make_shared<gimbal_controller::GimbalControlCore>();
  gimbal_control_core_->setFireModules(fire_advice_engine_, fire_advisor_);
}

void GimbalPipelineNode::initGimbalStrategies() {
  auto mpc_s = std::make_shared<gimbal_controller::MpcControlStrategy>();
  mpc_s->setComponents(position_calculator_, armor_selector_, local_compensator_, fire_advisor_);
  mpc_s->initReferenceGenerator();

  const double mpc_control_delay_s =
    get_parameter("controller.delay.control_latency_s").as_double();
  const bool mpc_enable_delay_compensation =
    get_parameter("controller.mpc.enable_delay_compensation").as_bool();
  const bool mpc_allow_muzzle_compensation =
    get_parameter("controller.mpc.allow_muzzle_compensation").as_bool();
  const double mpc_prediction_delay_s =
    get_parameter("controller.delay.prediction_extra_s").as_double();
  const double mpc_trigger_to_muzzle_s =
    get_parameter("controller.delay.trigger_to_muzzle_s").as_double();
  const int mpc_flight_time_iters = get_parameter("controller.delay.flight_time_iters").as_int();
  const double mpc_max_processing_delay_s =
    get_parameter("controller.delay.max_processing_delay_s").as_double();

  mpc_s->setMpcParameters(get_parameter("controller.mpc.N").as_int(),
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
  mpc_s->setDelayCompensation(mpc_enable_delay_compensation,
                              mpc_prediction_delay_s,
                              mpc_trigger_to_muzzle_s,
                              mpc_allow_muzzle_compensation,
                              mpc_flight_time_iters,
                              mpc_max_processing_delay_s);
  mpc_s->setYawFeedforward(get_parameter("controller.mpc.yaw_feedforward_k_s").as_double());
  mpc_s->setManeuverAdaptParameters(
    get_parameter("controller.mpc.maneuver_adapt.enable").as_bool(),
    get_parameter("controller.mpc.maneuver_adapt.a_max").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.eta").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.tau").as_double(),
    get_parameter("controller.mpc.maneuver_adapt.r_scale").as_double());
  mpc_s->setWeightingParameters(get_parameter("controller.mpc.weighting.enable").as_bool(),
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
    vel_clamp_cfg.enable = get_parameter("controller.mpc.vel_clamp.enable").as_bool();
    vel_clamp_cfg.max_linear_speed =
      get_parameter("controller.mpc.vel_clamp.max_linear_speed").as_double();
    vel_clamp_cfg.max_v_yaw = get_parameter("controller.mpc.vel_clamp.max_v_yaw").as_double();
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
    get_parameter("controller.mpc.normalization.rms_epsilon").as_double(),
    get_parameter("controller.mpc.normalization.mode").as_string(),
    Eigen::Vector4d(
      get_parameter("controller.mpc.normalization.typical_state.yaw").as_double(),
      get_parameter("controller.mpc.normalization.typical_state.pitch").as_double(),
      get_parameter("controller.mpc.normalization.typical_state.yaw_vel").as_double(),
      get_parameter("controller.mpc.normalization.typical_state.pitch_vel").as_double()),
    Eigen::Vector2d(
      get_parameter("controller.mpc.normalization.typical_control.yaw_acc").as_double(),
      get_parameter("controller.mpc.normalization.typical_control.pitch_acc").as_double()),
    Eigen::Vector2d(
      get_parameter("controller.mpc.normalization.typical_delta_control.yaw_acc").as_double(),
      get_parameter("controller.mpc.normalization.typical_delta_control.pitch_acc").as_double()));
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

  if (gimbal_control_core_) {
    gimbal_control_core_->setStrategies(&gimbal_strategies_);
  }
}

/* ================================================================ */
/*  Joint state callback                                             */
/* ================================================================ */

void GimbalPipelineNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "yaw_joint")
      current_yaw_ = msg->position[i];
    else if (msg->name[i] == "pitch_joint")
      current_pitch_ = msg->position[i];
  }
}

void GimbalPipelineNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
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

  RCLCPP_INFO_ONCE(
    get_logger(),
    "[FOV] camera_info received: fov_yaw=%.1f° fov_pitch=%.1f° (fx=%.1f fy=%.1f %dx%d)",
    fov_half_yaw * 2.0 * 180.0 / M_PI,
    fov_half_pitch * 2.0 * 180.0 / M_PI,
    fx,
    fy,
    msg->width,
    msg->height);
}

void GimbalPipelineNode::updateGimbalState() {
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform(target_frame_, "gimbal_link", tf2::TimePointZero);
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
  gimbal_controller::GimbalControlContext &context, std::string &selected_id) {
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
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "Stale tracking data (age=%.3fs > %.3fs), ignoring",
                         data_age,
                         max_data_age);
    return;
  }

  const rm_interfaces::msg::TrackedRobot *selected_robot = nullptr;
  if (!selected_id.empty()) {
    for (const auto &robot : robots->robots) {
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
  context.is_tracking = (selected_robot->track_state == rm_interfaces::msg::TrackedRobot::TRACKING);
  context.is_temp_lost =
    (selected_robot->track_state == rm_interfaces::msg::TrackedRobot::TEMP_LOST);

  auto *tracker = tracker_manager_->get(selected_robot->robot_id);
  context.is_maneuvering =
    (tracker && tracker->is_initialized()) ? tracker->assess_maneuver().is_maneuvering : false;
}

void GimbalPipelineNode::publishDelayAuditDebug(
  const gimbal_controller::GimbalControlContext &context,
  const gimbal_controller::DelayAuditSnapshot &audit,
  const std::string &strategy_name) {
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

void GimbalPipelineNode::publishFireAdviceDebug(
  const gimbal_controller::GimbalControlContext &context,
  const rm_interfaces::msg::GimbalCmd &cmd,
  const gimbal_controller::FireAdviceDebugSnapshot &snapshot) {
  if (!debug_fire_advice_pub_) {
    return;
  }

  rm_interfaces::msg::FireAdviceDebug msg;
  msg.header.stamp = context.current_time;
  msg.header.frame_id = target_frame_;
  msg.target_id = snapshot.target_id.empty() ? cmd.target_id : snapshot.target_id;
  msg.mode = snapshot.mode;
  msg.track_state = snapshot.track_state;
  msg.evaluated = snapshot.evaluated;
  msg.valid = snapshot.valid;
  msg.fire_advice = snapshot.fire_advice;
  msg.best_candidate_index = snapshot.best_candidate_index;
  msg.yaw_error = snapshot.yaw_error;
  msg.pitch_error = snapshot.pitch_error;
  msg.best_candidate_facing_ok = snapshot.best_candidate_facing_ok;
  msg.candidate_count_total = snapshot.candidate_count_total;
  msg.candidate_count_facing_eligible = snapshot.candidate_count_facing_eligible;
  msg.candidate_count_facing_rejected = snapshot.candidate_count_facing_rejected;
  msg.probability_enabled = snapshot.probability_enabled;
  msg.p_hit_window = snapshot.p_hit_window;
  msg.fire_score = snapshot.fire_score;
  msg.best_tau_ms = snapshot.best_tau_ms;
  msg.e_u = snapshot.e_u;
  msg.e_v = snapshot.e_v;
  msg.sigma_u = snapshot.sigma_u;
  msg.sigma_v = snapshot.sigma_v;
  msg.armor_width_m = snapshot.armor_width_m;
  msg.armor_height_m = snapshot.armor_height_m;
  msg.burst_probability = snapshot.burst_probability;
  msg.log_evidence = snapshot.log_evidence;
  msg.evidence_sum = snapshot.evidence_sum;
  msg.evidence_strength = snapshot.evidence_strength;
  msg.gate_strategy = snapshot.gate_strategy;
  msg.gate_state = snapshot.gate_state;

  debug_fire_advice_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "[FireAdvice] target=%s mode=%d track_state=%u evaluated=%d valid=%d fire=%d "
    "best=%d yaw_err=%.4fdeg pitch_err=%.4fdeg facing_ok=%d rejected=%d/%d eligible=%d",
    msg.target_id.empty() ? "none" : msg.target_id.c_str(),
    static_cast<int>(msg.mode),
    static_cast<unsigned>(msg.track_state),
    msg.evaluated ? 1 : 0,
    msg.valid ? 1 : 0,
    msg.fire_advice ? 1 : 0,
    msg.best_candidate_index,
    msg.yaw_error * 180.0 / M_PI,
    msg.pitch_error * 180.0 / M_PI,
    msg.best_candidate_facing_ok ? 1 : 0,
    msg.candidate_count_facing_rejected,
    msg.candidate_count_total,
    msg.candidate_count_facing_eligible);
}

/* ================================================================ */
/*  Timer callback — 250 Hz control loop                             */
/* ================================================================ */

void GimbalPipelineNode::timerCallback() {
  // Step 0: 核心类可用性检查
  if (!gimbal_control_core_) {
    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          2000,
                          "GimbalControlCore is not initialized, skipping control cycle");
    return;
  }

  gimbal_controller::GimbalControlContext context;
  context.current_time = now();

  // Step 1: 控制禁用时发布 idle 命令并早返回
  if (!enable_) {
    const auto idle_result =
      gimbal_control_core_->compute(context, current_gimbal_strategy_name_, std::string(), false);
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
  const auto control_result =
    gimbal_control_core_->compute(context, current_gimbal_strategy_name_, selected_id, true);
  if (!control_result.strategy_found) {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2000,
                         "Gimbal strategy '%s' not found, fallback to idle cmd",
                         current_gimbal_strategy_name_.c_str());
  }

  // Step 5: 发布控制命令
  gimbal_cmd_pub_->publish(control_result.cmd);

  // Step 6: 发布调试信息（audit + marker）
  if (debug_mode_ && debug_delay_audit_pub_) {
    publishDelayAuditDebug(context, control_result.delay_audit, current_gimbal_strategy_name_);
  }

  if (debug_mode_ && debug_fire_advice_pub_) {
    publishFireAdviceDebug(context, control_result.cmd, control_result.fire_advice_debug);
  }

  if (debug_mode_ && debug_armor_selection_pub_ && control_result.has_tracking) {
    publishArmorSelectionDebug(context, current_gimbal_strategy_name_);
  }

  if (debug_mode_ && control_result.has_tracking) {
    publishGimbalMarkers(
      context.target_robot, control_result.cmd, control_result.fire_advice_debug);
    publishFireProbabilityDebugImages(context.target_robot.header,
                                      control_result.fire_advice_debug);
  }
}

/* ================================================================ */
/*  Service callback                                                 */
/* ================================================================ */

void GimbalPipelineNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  const int mode = request->mode;
  if (mode >= 0 && mode <= 5) {
    enable_ = true;
    refreshExternalTargetAllowlist(mode);
    if (buff_target_adapter_) {
      const bool enable_buff = external_targets_enable_ && external_targets_buff_enable_;
      buff_target_adapter_->setEnabled(enable_buff);
    }
    RCLCPP_INFO(get_logger(), "GimbalPipeline enabled (mode=%d)", mode);
  } else {
    enable_ = false;
    if (buff_target_adapter_) {
      buff_target_adapter_->setEnabled(false);
    }
    RCLCPP_INFO(get_logger(), "GimbalPipeline disabled (mode=%d)", mode);
  }
}

gimbal_controller::GimbalControlStrategy::SharedPtr GimbalPipelineNode::getGimbalStrategy(
  const std::string &name) const {
  auto it = gimbal_strategies_.find(name);
  return (it != gimbal_strategies_.end()) ? it->second : nullptr;
}

}  // namespace fyt::auto_aim
