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

void GimbalPipelineNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg) {
  rclcpp::Time msg_time(msg->header.stamp);
  double current_time = msg_time.seconds();
  if (msg->armors.empty()) {
    RCLCPP_DEBUG_THROTTLE(get_logger(),
                          *get_clock(),
                          1000,
                          "Received empty armors message, running missing-target update");
  }

  // ── Step 1: Group observations by robot ID ──
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  std::string sf = msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;

  const bool strict_unknown_reject =
    robot_description_facade_ && robot_description_facade_->strictUnknownReject();

  for (const auto &armor : msg->armors) {
    const bool supported_robot_id =
      robot_description_facade_ && robot_description_facade_->isSupportedRobotId(armor.number);

    // Strict mode: skip unsupported IDs to prevent false tracker creation.
    if (strict_unknown_reject && !supported_robot_id) {
      if (debug_mode_)
        RCLCPP_WARN(get_logger(), "Ignoring armor with invalid ID: '%s'", armor.number.c_str());
      continue;
    }
    auto obs = tf_handler_->transform_armor_to_observation(armor, sf, msg_time);
    if (!obs.has_value()) {
      if (debug_mode_) RCLCPP_WARN(get_logger(), "TF failed for armor %s", armor.number.c_str());
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
        lo.x = o.x;
        lo.y = o.y;
        lo.z = o.z;
        lo.yaw = o.yaw;
        lo.panel_id = o.panel_id.value_or(-1);
        lo.confidence = o.confidence;
        lo.is_dual_obs = is_dual;
        log_obs.push_back(lo);
      }
      prediction_logger_->logObservations(ts_ns, rid, log_obs);
    }
  }

  // ── Step 2: Run tracker core frame process in manager ──
  auto frame_result = tracker_manager_->process_frame(obs_by_robot, current_time, smoother_config_);
  if (!frame_result.removed_stale_ids.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu stale trackers", frame_result.removed_stale_ids.size());
  }
  if (!frame_result.removed_lost_ids.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu lost trackers", frame_result.removed_lost_ids.size());
  }

  // ── Step 3: Build TrackedRobots message (internal) ──
  auto tracked_msg = buildTrackedRobotsMsg(msg->header);

  // ── Log tracker posterior states (after update, before selection) ──
  if (prediction_logger_) {
    int64_t ts_ns = msg_time.nanoseconds();
    for (const auto &robot : tracked_msg.robots) {
      const auto center_position = robot_description::TrackedRobotUsage::centerPosition(robot);
      const auto linear_velocity = robot_description::TrackedRobotUsage::linearVelocity(robot);
      LogTrackerState st;
      st.center_x = center_position.x();
      st.center_y = center_position.y();
      st.center_z = center_position.z();
      st.vel_x = linear_velocity.x();
      st.vel_y = linear_velocity.y();
      st.vel_z = linear_velocity.z();
      st.yaw = robot_description::TrackedRobotUsage::yaw(robot);
      st.yaw_velocity = robot_description::TrackedRobotUsage::yawVelocity(robot);
      st.yaw_acceleration = robot_description::TrackedRobotUsage::yawAcceleration(robot);
      st.radius_1 = robot.radius;
      st.radius_2 = robot.radius_2;
      st.dza = robot.d_za;
      st.track_state = robot.track_state;
      st.num_armors = robot.num_armors;
      st.visible_armor_count = robot.visible_armor_count;
      st.is_visible = robot.is_visible;
      st.confidence = robot.confidence;

      // ── 机动检测指标：从对应 tracker 的 UKF 内部读取 ──
      auto *tracker = tracker_manager_->get(robot.robot_id);
      if (tracker && tracker->is_initialized()) {
        const auto &ukf = tracker->spin_filter();
        const auto &idx = ukf.state_idx();
        const auto &xv = ukf.x();
        const auto &Pv = ukf.P();

        // 创新向量
        const auto &iv = ukf.last_innov_xyz();
        if (iv.size() >= 3) {
          st.innov_x = iv(0);
          st.innov_y = iv(1);
          st.innov_z = iv(2);
        }
        st.innov_yaw = ukf.last_innov_yaw();
        st.nis = ukf.last_nis();
        st.update_type = ukf.last_update_type();

        // P 对角线 —— 位置与速度
        st.p_var_x = Pv(idx.X(), idx.X());
        st.p_var_y = Pv(idx.Y(), idx.Y());
        st.p_var_z = Pv(idx.Z(), idx.Z());
        st.p_var_vx = Pv(idx.VX(), idx.VX());
        st.p_var_vy = Pv(idx.VY(), idx.VY());
        st.p_var_vz = Pv(idx.VZ(), idx.VZ());

        // 加速度状态（仅 CA / Singer 过程模型存在 AX/AY/AZ）
        constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
        if (idx.has("AX")) {
          st.p_var_ax = Pv(idx.AX(), idx.AX());
          st.p_var_ay = Pv(idx.AY(), idx.AY());
          st.p_var_az = Pv(idx.AZ(), idx.AZ());
          st.accel_x = xv(idx.AX());
          st.accel_y = xv(idx.AY());
          st.accel_z = xv(idx.AZ());
          st.accel_magnitude = std::sqrt(xv(idx.AX()) * xv(idx.AX()) + xv(idx.AY()) * xv(idx.AY()) +
                                         xv(idx.AZ()) * xv(idx.AZ()));
        } else {
          st.p_var_ax = kNaN;
          st.p_var_ay = kNaN;
          st.p_var_az = kNaN;
          st.accel_x = kNaN;
          st.accel_y = kNaN;
          st.accel_z = kNaN;
          st.accel_magnitude = kNaN;
        }

        if (robot.robot_id == "outpost") {
          auto apply_outpost_snapshot = [&](const auto &snap) {
            if (!snap.valid) return;
            st.outpost_mode = snap.track_mode;
            st.estimated_id = snap.estimated_id;
            st.runtime_panel_id = snap.runtime_panel_id;
            st.bound_height_label = snap.bound_height_label;
            st.obs_inferred_id = snap.obs_inferred_id;
            st.obs_inferred_id_z = snap.obs_inferred_id_z;
            st.candidate_panel_id = snap.candidate_panel_id;
            st.candidate_prob = snap.candidate_prob;
            st.candidate_margin = snap.candidate_margin;
            st.selected_xy_residual = snap.selected_xy_residual;
            st.outpost_entropy = snap.entropy_norm;
            st.outpost_max_prob = snap.max_prob;
            st.hyp_cost_0 = snap.hyp_costs[0];
            st.hyp_cost_1 = snap.hyp_costs[1];
            st.hyp_cost_2 = snap.hyp_costs[2];
            st.hyp_prob_0 = snap.hyp_probs[0];
            st.hyp_prob_1 = snap.hyp_probs[1];
            st.hyp_prob_2 = snap.hyp_probs[2];
            st.center_yaw_est = snap.center_yaw_est;
            st.has_observation = snap.has_observation ? 1 : 0;
            st.obs_x = snap.obs_x;
            st.obs_y = snap.obs_y;
            st.obs_z = snap.obs_z;
            st.obs_yaw = snap.obs_yaw;
            st.obs_z_jump = snap.obs_z_jump;
            st.obs_dz_from_audit_center = snap.obs_dz_from_audit_center;
            st.obs_z_audit_cost_0 = snap.obs_z_audit_costs[0];
            st.obs_z_audit_cost_1 = snap.obs_z_audit_costs[1];
            st.obs_z_audit_cost_2 = snap.obs_z_audit_costs[2];
            st.binding_confidence = snap.binding_confidence;
            st.switch_event = snap.switch_event;
            st.switch_reason = snap.switch_reason;
            st.transition_state = snap.transition_state;
            st.z_audit_conflict_count = snap.z_audit_conflict_count;
            st.z_audit_confidence = snap.z_audit_confidence;
            st.publish_x = snap.publish_x;
            st.publish_y = snap.publish_y;
            st.publish_z = snap.publish_z;
            st.period_confidence = snap.period_confidence;
            st.period_update_applied = snap.period_update_applied;
            st.period_phase_index = snap.period_phase_index;
            st.spin_direction = snap.spin_direction;
            st.dz_small_est = snap.dz_small_est;
            st.dz_large_est = snap.dz_large_est;
          };

          if (const auto *outpost_v2_tracker = dynamic_cast<const OutpostTrackerV2 *>(tracker);
              outpost_v2_tracker != nullptr) {
            apply_outpost_snapshot(outpost_v2_tracker->debug_snapshot());
          }
        }
      }

      prediction_logger_->logTrackerState(ts_ns, robot.robot_id, st);
    }
  }

  // ── Step 4: Target selection (direct C++ call, no ROS topic!) ──
  SelectionResult sel_result;
  if (!tracked_msg.robots.empty()) {
    sel_result = selectTargetInternal(tracked_msg);
  }

  // ── Step 5: Store results for timerCallback (thread-safe) ──
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    latest_tracked_robots_ = std::make_shared<rm_interfaces::msg::TrackedRobots>(tracked_msg);
    latest_selected_target_id_ = sel_result.robot_id;
    latest_selected_confidence_ = sel_result.confidence;
    latest_update_time_ = now();  // record local clock for processing_delay
  }

  // ── Step 6: Debug publishing ──
  const auto tracker_views = tracker_manager_->initialized_tracker_views();
  logNorm4V3TrackerDebug(tracker_views);
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
      auto marker_array = build_tracker_markers(visualization_frame_, tracker_views, stamp);
      debug_tracker_marker_pub_->publish(marker_array);
    }

    if (debug_maneuver_pub_) {
      publishManeuverMarkers(msg->header);
    }

    if (debug_tracker_2d_image_pub_) {
      publish2DTrackerDebugImage(msg->header, tracker_views);
    }
    if (debug_evidence_frame_pub_) {
      publishEvidenceFrameDebug(msg->header, tracker_views);
    }
  }

  // ── Step 7: Publish maneuver states (always-on, for chart monitoring) ──
  if (maneuver_states_pub_) {
    rm_interfaces::msg::ManeuverStates states_msg;
    states_msg.header.stamp = msg->header.stamp;
    states_msg.header.frame_id = target_frame_;
    for (const auto &view : tracker_views) {
      if (!view.tracker) continue;
      const auto result = view.tracker->assess_maneuver();
      const auto &ukf = view.tracker->spin_filter();
      rm_interfaces::msg::ManeuverState s;
      s.robot_id = view.robot_id;
      s.is_maneuvering = result.is_maneuvering;
      s.nis = result.nis;
      s.innov_norm = result.innov_norm;
      s.innov_yaw_abs = std::abs(ukf.last_innov_yaw());
      s.update_type = result.update_type;
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

    const rclcpp::Time stamp(header.stamp);
    const double ts = stamp.seconds();
    auto post = tracker_manager_->post_process_output(rid, ts, smoother_config_);
    const SmoothedOutput *smoothed = post.has_smoothed ? &post.smoothed : nullptr;
    const int visible_armor_count = tracker_manager_->visible_observation_count(rid);

    // Publish target for debug
    if (debug_mode_ && debug_target_pub_) {
      auto target = buildTargetMessage(header, rid, *tracker, smoothed);
      debug_target_pub_->publish(target);
    }

    auto robot = buildTrackedRobotMessage(header, rid, *tracker, smoothed, visible_armor_count);

    if (robot.robot_id.empty()) {
      continue;
    }

    tracked_msg.robots.push_back(robot);
  }

  mergeExternalTargets(tracked_msg, header);

  return tracked_msg;
}

void GimbalPipelineNode::mergeExternalTargets(rm_interfaces::msg::TrackedRobots &tracked_msg,
                                              const std_msgs::msg::Header &header) {
  if (!external_targets_enable_ || !external_targets_buff_enable_ || !buff_target_adapter_) {
    return;
  }

  const auto now = rclcpp::Time(header.stamp);
  auto buff_robot_opt = buff_target_adapter_->latestValid(now);
  if (!buff_robot_opt.has_value()) {
    return;
  }

  auto buff_robot = buff_robot_opt.value();
  if (!active_external_allowed_ids_.empty() &&
      active_external_allowed_ids_.find(buff_robot.robot_id) ==
        active_external_allowed_ids_.end()) {
    return;
  }

  buff_robot.header = tracked_msg.header;
  bool replaced = false;
  for (auto &robot : tracked_msg.robots) {
    if (robot.robot_id == buff_robot.robot_id) {
      robot = buff_robot;
      replaced = true;
      break;
    }
  }
  if (!replaced) {
    tracked_msg.robots.push_back(buff_robot);
  }
}

void GimbalPipelineNode::refreshExternalTargetAllowlist(int mode) {
  current_mode_ = mode;
  auto it = allowed_ids_by_mode_.find(mode);
  if (it == allowed_ids_by_mode_.end()) {
    active_external_allowed_ids_.clear();
    return;
  }
  active_external_allowed_ids_ = it->second;
}

/* ================================================================ */
/*  Target message builders (from MaxEntropyTrackerNode)             */
/* ================================================================ */

rm_interfaces::msg::Target GimbalPipelineNode::buildTargetMessage(
  const std_msgs::msg::Header &header,
  const std::string &robot_id,
  BaseTracker &tracker,
  const SmoothedOutput *smoothed) {
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
  const std_msgs::msg::Header &header,
  const std::string &robot_id,
  BaseTracker &tracker,
  const SmoothedOutput *smoothed,
  int visible_armor_count) {
  rm_interfaces::msg::TrackedRobot empty_msg;

  if (!robot_description_facade_) {
    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          2000,
                          "RobotDescriptionFacade is not initialized, skip TrackedRobot build");
    return empty_msg;
  }

  robot_description::TrackedRobotBuildInput input{
    header, target_frame_, robot_id, tracker, smoothed, visible_armor_count};

  auto build_result = robot_description_facade_->tryBuildTrackedRobot(input);
  if (!build_result.ok()) {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2000,
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

}  // namespace fyt::auto_aim
