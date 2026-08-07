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
#include "gimbal_pipeline/adapters/pipeline_ros_conversions.hpp"
#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/tracker/norm4_tracker_baseline.hpp"
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

void GimbalPipelineNode::initGimbalControl() {
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
  if (gimbal_control_core_) {
    gimbal_control_core_->setStrategy(mpc_s);
  }
}

/* ================================================================ */
/*  Joint state callback                                             */
/* ================================================================ */

void GimbalPipelineNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(sensor_state_mutex_);
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
    std::lock_guard<std::mutex> lock(control_core_mutex_);
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
    std::lock_guard<std::mutex> lock(sensor_state_mutex_);
    current_yaw_ = yaw;
    current_pitch_ = -pitch;
  } catch (const tf2::TransformException &) {
    // fall through — use joint_states values
  }
}

pipeline::GimbalCommand GimbalPipelineNode::computeGimbalCommand(
  const pipeline::ControlRequest & request)
{
  gimbal_controller::GimbalControlContext context;
  context.current_time = rclcpp::Time(request.now_ns, RCL_ROS_TIME);
  context.current_yaw = request.gimbal.yaw;
  context.current_pitch = request.gimbal.pitch;
  context.bullet_speed = request.gimbal.bullet_speed;

  std::string selected_id;
  if (request.target) {
    std_msgs::msg::Header header;
    header.frame_id = target_frame_;
    header.stamp = pipeline::ros_adapter::toRosTime(request.target->timestamp_ns);
    context.target_robot = pipeline::ros_adapter::toRos(*request.target, header);
    context.target_stamp = rclcpp::Time(request.target->timestamp_ns, RCL_ROS_TIME);
    context.is_tracking = request.target->track_state == pipeline::TrackState::TRACKING;
    context.is_temp_lost = request.target->track_state == pipeline::TrackState::TEMP_LOST;
    context.is_maneuvering = request.target->is_maneuvering;
    selected_id = request.target->robot_id;
  }

  std::lock_guard<std::mutex> lock(control_core_mutex_);
  last_control_context_ = context;
  last_control_result_ = gimbal_control_core_->compute(
    context, selected_id, request.enabled);
  auto command = pipeline::ros_adapter::toDomain(last_control_result_.cmd);
  command.timestamp_ns = request.now_ns;
  return command;
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
  if (!auto_aim_pipeline_ || !gimbal_control_core_) {
    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          2000,
                          "AutoAimPipeline is not initialized, skipping control cycle");
    return;
  }

  updateGimbalState();
  const auto cycle_time = now();
  pipeline::GimbalState gimbal_state;
  int mode = 0;
  bool enabled = false;
  {
    std::lock_guard<std::mutex> lock(sensor_state_mutex_);
    gimbal_state = {current_yaw_, current_pitch_, bullet_speed_};
    mode = current_mode_;
    enabled = enable_;
  }
  pipeline::PipelineCycleInput input;
  input.now_ns = cycle_time.nanoseconds();
  input.detection_frames = pipeline_inbox_.takeAll();
  input.external_targets = collectExternalTargets(input.now_ns);
  input.gimbal = gimbal_state;
  input.mode = mode;
  input.enabled = enabled;

  // The 250 Hz timer is the only caller of the ROS-free end-to-end business cycle.
  const auto result = auto_aim_pipeline_->runCycle(input);
  gimbal_cmd_pub_->publish(
    pipeline::ros_adapter::toRos(result.command, target_frame_));

  for (const auto & event : result.events) {
    if (event.type == pipeline::PipelineEvent::Type::OUT_OF_ORDER_FRAME) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Rejected out-of-order detector frame");
    }
  }

  publishPipelineTelemetry(result);
  if (debug_mode_ && debug_delay_audit_pub_) {
    publishDelayAuditDebug(last_control_context_, last_control_result_.delay_audit, "mpc");
  }

  if (debug_mode_ && debug_fire_advice_pub_) {
    publishFireAdviceDebug(
      last_control_context_, last_control_result_.cmd,
      last_control_result_.fire_advice_debug);
  }

  if (debug_mode_ && debug_armor_selection_pub_ && last_control_result_.has_tracking) {
    publishArmorSelectionDebug(last_control_context_, "mpc");
  }

  if (debug_mode_ && last_control_result_.has_tracking) {
    publishGimbalMarkers(
      last_control_context_.target_robot, last_control_result_.cmd,
      last_control_result_.fire_advice_debug);
    publishFireProbabilityDebugImages(
      last_control_context_.target_robot.header,
      last_control_result_.fire_advice_debug);
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
    {
      std::lock_guard<std::mutex> lock(sensor_state_mutex_);
      enable_ = true;
      current_mode_ = mode;
    }
    if (buff_target_adapter_) {
      const bool enable_buff = external_targets_enable_ && external_targets_buff_enable_;
      buff_target_adapter_->setEnabled(enable_buff);
    }
    RCLCPP_INFO(get_logger(), "GimbalPipeline enabled (mode=%d)", mode);
  } else {
    {
      std::lock_guard<std::mutex> lock(sensor_state_mutex_);
      enable_ = false;
    }
    if (buff_target_adapter_) {
      buff_target_adapter_->setEnabled(false);
    }
    RCLCPP_INFO(get_logger(), "GimbalPipeline disabled (mode=%d)", mode);
  }
}

}  // namespace fyt::auto_aim
