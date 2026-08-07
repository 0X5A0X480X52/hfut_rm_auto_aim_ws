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

// Gimbal strategies
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

namespace fyt::auto_aim {

void GimbalPipelineNode::armorsCallback(
  const rm_interfaces::msg::Armors::SharedPtr msg)
{
  const rclcpp::Time message_time(msg->header.stamp);
  pipeline::ObservationFrame frame;
  frame.timestamp_ns = message_time.nanoseconds();

  const std::string source_frame =
    msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;
  const bool strict_unknown_reject =
    robot_description_facade_ && robot_description_facade_->strictUnknownReject();

  for (const auto & armor : msg->armors) {
    const bool supported =
      robot_description_facade_ && robot_description_facade_->isSupportedRobotId(armor.number);
    if (strict_unknown_reject && !supported) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring armor with unsupported ID '%s'", armor.number.c_str());
      continue;
    }

    auto observation =
      tf_handler_->transform_armor_to_observation(armor, source_frame, message_time);
    if (!observation) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF conversion failed for armor '%s'", armor.number.c_str());
      continue;
    }
    frame.observations[armor.number].push_back(std::move(*observation));
  }

  if (prediction_logger_) {
    for (const auto & [robot_id, observations] : frame.observations) {
      std::vector<LogObservation> log_observations;
      log_observations.reserve(observations.size());
      for (const auto & observation : observations) {
        LogObservation item;
        item.x = observation.x;
        item.y = observation.y;
        item.z = observation.z;
        item.yaw = observation.yaw;
        item.panel_id = observation.panel_id.value_or(-1);
        item.confidence = observation.confidence;
        item.is_dual_obs = observations.size() >= 2;
        log_observations.push_back(item);
      }
      prediction_logger_->logObservations(
        frame.timestamp_ns, robot_id, log_observations);
    }
  }

  const auto push_result = pipeline_inbox_.push(std::move(frame));
  if (push_result.dropped_oldest) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Detector frame inbox overflow; dropped the oldest frame");
  }
}

pipeline::RobotTrackSet GimbalPipelineNode::processObservationFrame(
  const pipeline::ObservationFrame & frame)
{
  const double timestamp_s = static_cast<double>(frame.timestamp_ns) * 1e-9;
  tracker_manager_->process_frame(frame.observations, timestamp_s, smoother_config_);

  std_msgs::msg::Header header;
  header.stamp = pipeline::ros_adapter::toRosTime(frame.timestamp_ns);
  header.frame_id = target_frame_;
  auto tracks = pipeline::ros_adapter::toDomain(buildTrackedRobotsMsg(header));

  for (auto & robot : tracks.robots) {
    auto * tracker = tracker_manager_->get(robot.robot_id);
    robot.is_maneuvering = tracker && tracker->is_initialized() &&
      tracker->assess_maneuver().is_maneuvering;
  }
  return tracks;
}

pipeline::RobotTrackSet GimbalPipelineNode::collectExternalTargets(
  pipeline::TimestampNs now_ns)
{
  pipeline::RobotTrackSet result;
  result.timestamp_ns = now_ns;
  if (!external_targets_enable_ || !external_targets_buff_enable_ || !buff_target_adapter_) {
    return result;
  }

  const auto target = buff_target_adapter_->latestValid(rclcpp::Time(now_ns, RCL_ROS_TIME));
  if (target) {
    result.robots.push_back(pipeline::ros_adapter::toDomain(*target));
  }
  return result;
}

void GimbalPipelineNode::publishPipelineTelemetry(
  const pipeline::PipelineCycleResult & result)
{
  if (!result.tracking_updated) {
    return;
  }

  const auto tracked_msg = pipeline::ros_adapter::toRos(result.tracks, target_frame_);
  const auto tracker_views = tracker_manager_->initialized_tracker_views();
  logNorm4BaselineTrackerDebug(tracker_views);

  if (prediction_logger_) {
    for (const auto & robot : result.tracks.robots) {
      LogTrackerState state;
      state.center_x = robot.center_position.x();
      state.center_y = robot.center_position.y();
      state.center_z = robot.center_position.z();
      state.vel_x = robot.center_velocity.x();
      state.vel_y = robot.center_velocity.y();
      state.vel_z = robot.center_velocity.z();
      state.yaw = robot.yaw;
      state.yaw_velocity = robot.yaw_velocity;
      state.yaw_acceleration = robot.yaw_acceleration;
      state.radius_1 = robot.radius;
      state.radius_2 = robot.radius_2;
      state.dza = robot.d_za;
      state.track_state = static_cast<std::uint8_t>(robot.track_state);
      state.num_armors = robot.num_armors;
      state.visible_armor_count = robot.visible_armor_count;
      state.is_visible = robot.is_visible;
      state.confidence = robot.confidence;
      prediction_logger_->logTrackerState(
        result.tracks.timestamp_ns, robot.robot_id, state);
    }
  }

  if (debug_mode_) {
    if (debug_tracked_robots_pub_ && !tracked_msg.robots.empty()) {
      debug_tracked_robots_pub_->publish(tracked_msg);
    }
    if (debug_selected_target_pub_) {
      rm_interfaces::msg::SelectedTarget selected;
      selected.header = tracked_msg.header;
      selected.selection_strategy = "priority_list";
      if (result.selected_target) {
        selected.robot_id = result.selected_target->robot_id;
        selected.confidence = result.selected_target->confidence;
      }
      debug_selected_target_pub_->publish(selected);
    }
    if (debug_target_pub_ && result.selected_target) {
      const auto selected_it = std::find_if(
        result.tracks.robots.begin(), result.tracks.robots.end(),
        [&](const pipeline::RobotTrack & robot) {
          return robot.robot_id == result.selected_target->robot_id;
        });
      if (selected_it != result.tracks.robots.end()) {
        rm_interfaces::msg::Target target;
        target.header = tracked_msg.header;
        target.tracking = selected_it->track_state == pipeline::TrackState::TRACKING;
        target.id = selected_it->robot_id;
        target.armors_num = selected_it->num_armors;
        target.position.x = selected_it->center_position.x();
        target.position.y = selected_it->center_position.y();
        target.position.z = selected_it->center_position.z();
        target.velocity.x = selected_it->center_velocity.x();
        target.velocity.y = selected_it->center_velocity.y();
        target.velocity.z = selected_it->center_velocity.z();
        target.yaw = selected_it->yaw;
        target.v_yaw = selected_it->yaw_velocity;
        target.radius_1 = selected_it->radius;
        target.radius_2 = selected_it->radius_2;
        target.d_za = selected_it->d_za;
        target.d_zc = selected_it->d_zc;
        debug_target_pub_->publish(target);
      }
    }
    if (debug_tracker_marker_pub_) {
      debug_tracker_marker_pub_->publish(
        build_tracker_markers(
          visualization_frame_, tracker_views,
          rclcpp::Time(tracked_msg.header.stamp)));
    }
    if (debug_maneuver_pub_) {
      publishManeuverMarkers(tracked_msg.header);
    }
    if (debug_tracker_2d_image_pub_) {
      publish2DTrackerDebugImage(tracked_msg.header, tracker_views);
    }
    if (debug_evidence_frame_pub_) {
      publishEvidenceFrameDebug(tracked_msg.header, tracker_views);
    }
  }

  if (maneuver_states_pub_) {
    rm_interfaces::msg::ManeuverStates states;
    states.header = tracked_msg.header;
    for (const auto & robot : result.tracks.robots) {
      rm_interfaces::msg::ManeuverState state;
      state.robot_id = robot.robot_id;
      state.is_maneuvering = robot.is_maneuvering;
      auto * tracker = tracker_manager_->get(robot.robot_id);
      if (tracker && tracker->is_initialized()) {
        const auto assessment = tracker->assess_maneuver();
        state.nis = assessment.nis;
        state.innov_norm = assessment.innov_norm;
        state.innov_yaw_abs = std::abs(tracker->spin_filter().last_innov_yaw());
        state.update_type = assessment.update_type;
      }
      states.states.push_back(state);
    }
    maneuver_states_pub_->publish(states);
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

    auto robot = buildTrackedRobotMessage(header, rid, *tracker, smoothed, visible_armor_count);

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
