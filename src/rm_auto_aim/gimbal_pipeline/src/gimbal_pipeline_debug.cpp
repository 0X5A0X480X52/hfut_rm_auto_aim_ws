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

void GimbalPipelineNode::publish2DTrackerDebugImage(
  const std_msgs::msg::Header &header,
  const std::vector<TrackerManager::TrackerConstView> &tracker_views) {
  if (!debug_tracker_2d_image_pub_) return;

  cv::Mat canvas(
    tracker_2d_image_debug_height_, tracker_2d_image_debug_width_, CV_8UC3, cv::Scalar(20, 20, 20));

  int draw_count = 0;
  for (const auto &view : tracker_views) {
    if (!view.tracker) continue;
    const evidence::ArmorEvidenceFrame *frame = nullptr;

    if (const auto *norm4_baseline = dynamic_cast<const Norm4TrackerBaseline *>(view.tracker)) {
      frame = &norm4_baseline->last_evidence_frame();
    }
    if (!frame) continue;

    const auto &f = *frame;
    if (f.observations.empty()) continue;

    for (const auto &obs : f.observations) {
      if (!obs.image.has_value() || !obs.image->valid) continue;
      const auto &img = obs.image.value();

      int track_id = obs.track2d_id.value_or(-1);
      int color_seed = (track_id >= 0 ? track_id : draw_count);
      cv::Scalar color(
        50 + (color_seed * 71) % 205, 50 + (color_seed * 131) % 205, 50 + (color_seed * 193) % 205);

      const int x = std::max(0, static_cast<int>(std::lround(img.bbox_x)));
      const int y = std::max(0, static_cast<int>(std::lround(img.bbox_y)));
      const int w = std::max(1, static_cast<int>(std::lround(img.bbox_w)));
      const int h = std::max(1, static_cast<int>(std::lround(img.bbox_h)));
      cv::rectangle(canvas, cv::Rect(x, y, w, h), color, 2);

      bool has_corners = true;
      for (const auto &c : img.corners) {
        if (!std::isfinite(c.x()) || !std::isfinite(c.y())) {
          has_corners = false;
          break;
        }
      }
      if (has_corners) {
        std::vector<cv::Point> poly;
        poly.reserve(4);
        for (const auto &c : img.corners) {
          poly.emplace_back(static_cast<int>(std::lround(c.x())),
                            static_cast<int>(std::lround(c.y())));
        }
        const cv::Point *pts = poly.data();
        int npts = static_cast<int>(poly.size());
        cv::polylines(canvas, &pts, &npts, 1, true, color, 1, cv::LINE_AA);
      }

      std::ostringstream oss;
      oss << view.robot_id << " t2d=" << track_id;
      cv::putText(canvas,
                  oss.str(),
                  cv::Point(x, std::max(16, y - 5)),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.45,
                  color,
                  1,
                  cv::LINE_AA);
      ++draw_count;
    }
  }

  cv::putText(canvas,
              "2DTracker tracks: " + std::to_string(draw_count),
              cv::Point(10, 22),
              cv::FONT_HERSHEY_SIMPLEX,
              0.6,
              cv::Scalar(200, 220, 255),
              1,
              cv::LINE_AA);

  std::vector<uchar> encoded;
  std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, tracker_2d_image_debug_jpeg_quality_};
  if (!cv::imencode(".jpg", canvas, encoded, encode_params)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to encode 2D tracker debug image");
    return;
  }

  sensor_msgs::msg::CompressedImage out;
  out.header = header;
  out.format = "jpeg";
  out.data = std::move(encoded);
  debug_tracker_2d_image_pub_->publish(out);
}

void GimbalPipelineNode::publishEvidenceFrameDebug(
  const std_msgs::msg::Header &header,
  const std::vector<TrackerManager::TrackerConstView> &tracker_views) {
  if (!debug_evidence_frame_pub_) return;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "stamp=" << rclcpp::Time(header.stamp).seconds();

  int norm4_count = 0;
  for (const auto &view : tracker_views) {
    if (!view.tracker) continue;
    const evidence::ArmorEvidenceFrame *frame = nullptr;
    std::string rid = view.robot_id;

    if (const auto *norm4_baseline = dynamic_cast<const Norm4TrackerBaseline *>(view.tracker)) {
      frame = &norm4_baseline->last_evidence_frame();
    }
    if (!frame) continue;

    ++norm4_count;
    const auto &f = *frame;
    oss << "\nrobot_id=" << view.robot_id << " ts=" << f.timestamp << " obs=" << f.obs_count
        << " obs_vec=" << f.observations.size() << " t2d=" << f.track2d_evidence.size()
        << " proxy=" << f.proxy_evidence.size()
        << " comp={3d:" << (f.completeness.has_3d_obs ? 1 : 0)
        << ",2d:" << (f.completeness.has_2d_tracks ? 1 : 0)
        << ",proxy:" << (f.completeness.has_proxy ? 1 : 0)
        << ",geo:" << (f.completeness.has_geometry ? 1 : 0)
        << ",rel:" << (f.completeness.has_relation ? 1 : 0)
        << ",ratio:" << f.completeness.fraction() << "}"
        << " relation={valid:" << (f.relation.valid ? 1 : 0)
        << ",has_z_jump:" << (f.relation.has_z_jump ? 1 : 0) << ",z_jump:" << f.relation.z_jump
        << ",yaw_delta:" << f.relation.yaw_delta << ",spatial:" << f.relation.spatial_consistency
        << ",dual:" << (f.relation.has_dual_obs ? 1 : 0) << ",p1:" << f.relation.dual_panel_id_1
        << ",p2:" << f.relation.dual_panel_id_2 << "}";
  }

  if (norm4_count == 0) {
    oss << "\nno_norm4_tracker";
  }

  std_msgs::msg::String out;
  out.data = oss.str();
  debug_evidence_frame_pub_->publish(out);
}

void GimbalPipelineNode::logNorm4BaselineTrackerDebug(
  const std::vector<TrackerManager::TrackerConstView> &tracker_views) {
  const auto &dbg_cfg = tracker_config_.norm4_baseline.debug_log;
  if (!dbg_cfg.enable) return;

  std::ostringstream oss;
  bool has_norm4_baseline = false;
  for (const auto &view : tracker_views) {
    if (!view.tracker) continue;
    const auto *norm4_baseline = dynamic_cast<const Norm4TrackerBaseline *>(view.tracker);
    if (!norm4_baseline) continue;

    has_norm4_baseline = true;
    const auto &h = norm4_baseline->last_hypothesis_debug();
    const auto &s = norm4_baseline->debug_snapshot();

    oss << " [" << view.robot_id << " committed=" << (h.committed ? 1 : 0)
        << " panel=" << s.current_panel_id << " cand=" << s.candidate_panel_id
        << " conf=" << h.top1_confidence << " margin=" << h.top1_top2_margin;
    if (dbg_cfg.verbose) {
      oss << " mode=" << static_cast<int>(norm4_baseline->current_mode()) << " top1_nis=" << s.top1_nis
          << " degraded=" << (h.degraded ? 1 : 0);
    }
    oss << " reason=" << h.decision_reason << "]";
  }

  if (!has_norm4_baseline) return;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), dbg_cfg.throttle_ms, "norm4_baseline_debug:%s", oss.str().c_str());
}

/* ================================================================ */
/*  Visualization                                                    */
/* ================================================================ */

void GimbalPipelineNode::initMarkers() {
  position_marker_.ns = "target_position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.15;
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
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.12;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.r = 1.0;
  selection_marker_.color.g = 1.0;

  predicted_marker_.ns = "predicted_hit";
  predicted_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  predicted_marker_.scale.x = predicted_marker_.scale.y = predicted_marker_.scale.z = 0.1;
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

  virtual_armor_marker_.ns = "virtual_armor";
  virtual_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  virtual_armor_marker_.scale.x = 0.03;
  virtual_armor_marker_.scale.y = 0.23;
  virtual_armor_marker_.scale.z = 0.125;
  virtual_armor_marker_.color.a = 0.95;
  virtual_armor_marker_.color.r = 0.1;
  virtual_armor_marker_.color.g = 0.95;
  virtual_armor_marker_.color.b = 0.35;

  virtual_armor_text_marker_.ns = "virtual_armor_text";
  virtual_armor_text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  virtual_armor_text_marker_.scale.z = 0.12;
  virtual_armor_text_marker_.color.a = 1.0;
  virtual_armor_text_marker_.color.r = 0.1;
  virtual_armor_text_marker_.color.g = 1.0;
  virtual_armor_text_marker_.color.b = 0.6;

  color_palette_.clear();
  for (int i = 0; i < 10; ++i) {
    float hue = i * 36.0f;
    color_palette_.push_back(hsvToRgb(hue, 1.0f, 1.0f));
  }
}

void GimbalPipelineNode::publishGimbalMarkers(
  const rm_interfaces::msg::TrackedRobot &target_robot,
  const rm_interfaces::msg::GimbalCmd &cmd,
  const gimbal_controller::FireAdviceDebugSnapshot &fire_snapshot) {
  if (!debug_gimbal_marker_pub_) return;

  const auto normalized_target = robot_description::TrackedRobotUsage::normalizeState(target_robot);
  const auto center_position =
    robot_description::TrackedRobotUsage::centerPosition(normalized_target);
  const auto linear_velocity =
    robot_description::TrackedRobotUsage::linearVelocity(normalized_target);
  const double target_yaw = robot_description::TrackedRobotUsage::yaw(normalized_target);
  const double target_yaw_velocity =
    robot_description::TrackedRobotUsage::yawVelocity(normalized_target);

  visualization_msgs::msg::MarkerArray marker_array;
  const bool has_valid_measurement =
    cmd.mode == rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT && cmd.distance > 0.0;

  // Position
  position_marker_.header = target_robot.header;
  position_marker_.id = 0;
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  position_marker_.pose.position = robot_description::TrackedRobotUsage::toPoint(center_position);
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
        center_position.x() + offset.position.x * cos_yaw - offset.position.y * sin_yaw;
      armor_marker.pose.position.y =
        center_position.y() + offset.position.x * sin_yaw + offset.position.y * cos_yaw;
      armor_marker.pose.position.z = center_position.z() + offset.position.z;
      tf2::Quaternion q_offset;
      tf2::fromMsg(offset.orientation, q_offset);
      if (q_offset.length2() <= 1e-12) {
        q_offset.setRPY(0.0, 0.0, 0.0);
      } else {
        q_offset.normalize();
      }
      tf2::Quaternion q_world_yaw;
      q_world_yaw.setRPY(0.0, 0.0, target_yaw);
      const tf2::Quaternion q_world_armor = q_world_yaw * q_offset;
      armor_marker.pose.orientation = tf2::toMsg(q_world_armor);
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
      speed_norm = std::clamp(std::abs(target_yaw_velocity) / radial_dynamic_v_yaw_ref_, 0.0, 1.0);
      const double scale = 1.0 - radial_dynamic_shrink_ratio_ * speed_norm;
      enter_deg = std::max(enter_deg * scale, radial_dynamic_min_angle_deg_);
      const double bias_mag =
        std::min(radial_dynamic_bias_gain_deg_ * speed_norm, radial_dynamic_max_bias_deg_);
      bias_deg = (target_yaw_velocity >= 0.0 ? 1.0 : -1.0) * bias_mag;
      axis_yaw += bias_deg * M_PI / 180.0;
    }
    const double enter_rad = enter_deg * M_PI / 180.0;

    double radius = 0.25;
    for (const auto &offset : normalized_target.armors_offset) {
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

  // Virtual armor marker (only when auto-switch virtual mode is enabled and active)
  {
    virtual_armor_marker_.header = target_robot.header;
    virtual_armor_marker_.id = 0;
    virtual_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    virtual_armor_text_marker_.header = target_robot.header;
    virtual_armor_text_marker_.id = 0;
    virtual_armor_text_marker_.action = visualization_msgs::msg::Marker::DELETE;

    if (virtual_auto_switch_enable_ && armor_selector_ && position_calculator_) {
      auto armor_positions = position_calculator_->calculate(normalized_target);
      if (!armor_positions.empty()) {
        // Use a local copy to avoid mutating runtime selector state during debug visualization.
        auto debug_selector = *armor_selector_;
        auto virtual_selection = debug_selector.selectBest(armor_positions,
                                                           center_position,
                                                           target_yaw,
                                                           normalized_target.num_armors,
                                                           target_yaw_velocity,
                                                           current_yaw_,
                                                           current_pitch_);

        if (virtual_selection.is_virtual_target && !virtual_selection.is_center_fallback) {
          virtual_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
          virtual_armor_marker_.pose.position =
            robot_description::TrackedRobotUsage::toPoint(virtual_selection.position);

          const double normal_yaw = std::atan2(-center_position.y(), -center_position.x());
          tf2::Quaternion q_virtual;
          q_virtual.setRPY(0.0, -0.2618, normal_yaw);
          virtual_armor_marker_.pose.orientation.x = q_virtual.x();
          virtual_armor_marker_.pose.orientation.y = q_virtual.y();
          virtual_armor_marker_.pose.orientation.z = q_virtual.z();
          virtual_armor_marker_.pose.orientation.w = q_virtual.w();

          virtual_armor_text_marker_.action = visualization_msgs::msg::Marker::ADD;
          virtual_armor_text_marker_.pose.position = virtual_armor_marker_.pose.position;
          virtual_armor_text_marker_.pose.position.z += 0.18;
          virtual_armor_text_marker_.pose.orientation.w = 1.0;
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(3)
              << "vidx=" << virtual_selection.real_selected_index
              << " dYaw=" << virtual_selection.virtual_delta_yaw;
          virtual_armor_text_marker_.text = oss.str();
        }
      }
    }
    marker_array.markers.push_back(virtual_armor_marker_);
    marker_array.markers.push_back(virtual_armor_text_marker_);
  }

  // Selection target
  if (has_valid_measurement) {
    selection_marker_.header = target_robot.header;
    selection_marker_.id = 0;
    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    double yaw_rad = cmd.yaw * M_PI / 180.0;
    double pitch_rad = cmd.pitch * M_PI / 180.0;
    selection_marker_.pose.position.x = cmd.distance * std::cos(pitch_rad) * std::cos(yaw_rad);
    selection_marker_.pose.position.y = cmd.distance * std::cos(pitch_rad) * std::sin(yaw_rad);
    selection_marker_.pose.position.z = cmd.distance * std::sin(pitch_rad);
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
    trajectory_marker_.header = target_robot.header;
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
      p.z = distance * std::sin(pitch_rad) - 0.5 * 9.8 * flight_time * flight_time;
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

  if (fire_prob_vis_enable_) {
    publishFireProbabilityMarkers(target_robot.header, fire_snapshot, marker_array);
  }

  debug_gimbal_marker_pub_->publish(marker_array);
}

void GimbalPipelineNode::publishFireProbabilityMarkers(
  const std_msgs::msg::Header &header,
  const gimbal_controller::FireAdviceDebugSnapshot &fire_snapshot,
  visualization_msgs::msg::MarkerArray &marker_array) {
  if (!fire_snapshot.probability_enabled || fire_snapshot.tau_samples.empty()) {
    const std::vector<std::pair<std::string, int>> stale_markers = {
      {"fire_prob/tau_candidates", 0},
      {"fire_prob/error_ellipse_1sigma", 0},
      {"fire_prob/error_ellipse_2sigma", 0},
      {"fire_prob/trajectory_mean", 0},
      {"fire_prob/impact_cloud", 0},
      {"fire_prob/armor_plane", 0},
      {"fire_prob/mean_error_point", 0},
      {"fire_prob/text", 0}};
    for (const auto &[ns, id] : stale_markers) {
      visualization_msgs::msg::Marker del;
      del.header = header;
      del.ns = ns;
      del.id = id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(del);
    }
    return;
  }

  visualization_msgs::msg::Marker traj;
  traj.header = header;
  traj.ns = "fire_prob/trajectory_mean";
  traj.id = 0;
  traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
  traj.action = visualization_msgs::msg::Marker::ADD;
  traj.scale.x = 0.01;
  traj.color.a = 0.95;
  traj.color.r = 0.1;
  traj.color.g = 0.95;
  traj.color.b = 0.2;
  for (const auto &s : fire_snapshot.tau_samples) {
    geometry_msgs::msg::Point p;
    p.x = s.impact_x;
    p.y = s.impact_y;
    p.z = s.impact_z;
    traj.points.push_back(p);
  }
  marker_array.markers.push_back(traj);

  visualization_msgs::msg::Marker tau_pts;
  tau_pts.header = header;
  tau_pts.ns = "fire_prob/tau_candidates";
  tau_pts.id = 0;
  tau_pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  tau_pts.action = visualization_msgs::msg::Marker::ADD;
  tau_pts.scale.x = 0.03;
  tau_pts.scale.y = 0.03;
  tau_pts.scale.z = 0.03;
  tau_pts.color.a = 0.9;
  tau_pts.points.clear();
  tau_pts.colors.clear();
  for (const auto &s : fire_snapshot.tau_samples) {
    geometry_msgs::msg::Point p;
    p.x = s.impact_x;
    p.y = s.impact_y;
    p.z = s.impact_z;
    tau_pts.points.push_back(p);
    std_msgs::msg::ColorRGBA c;
    c.a = 0.9f;
    c.r = static_cast<float>(1.0 - s.p_hit);
    c.g = static_cast<float>(s.p_hit);
    c.b = 0.1f;
    tau_pts.colors.push_back(c);
  }
  marker_array.markers.push_back(tau_pts);

  visualization_msgs::msg::Marker armor_plane;
  armor_plane.header = header;
  armor_plane.ns = "fire_prob/armor_plane";
  armor_plane.id = 0;
  armor_plane.type = visualization_msgs::msg::Marker::LINE_LIST;
  armor_plane.action = visualization_msgs::msg::Marker::ADD;
  armor_plane.scale.x = 0.008;
  armor_plane.color.a = 0.9;
  armor_plane.color.r = 0.8;
  armor_plane.color.g = 0.95;
  armor_plane.color.b = 1.0;
  const double hw = std::max(fire_snapshot.armor_width_m, 1e-6) * 0.5;
  const double hh = std::max(fire_snapshot.armor_height_m, 1e-6) * 0.5;
  const Eigen::Vector3d c = fire_snapshot.armor_center;
  const Eigen::Vector3d r = fire_snapshot.armor_right;
  const Eigen::Vector3d u = fire_snapshot.armor_up;
  const Eigen::Vector3d p0 = c + r * hw + u * hh;
  const Eigen::Vector3d p1 = c - r * hw + u * hh;
  const Eigen::Vector3d p2 = c - r * hw - u * hh;
  const Eigen::Vector3d p3 = c + r * hw - u * hh;
  const std::array<Eigen::Vector3d, 4> ps = {p0, p1, p2, p3};
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point a;
    geometry_msgs::msg::Point b;
    a.x = ps[i].x();
    a.y = ps[i].y();
    a.z = ps[i].z();
    b.x = ps[(i + 1) % 4].x();
    b.y = ps[(i + 1) % 4].y();
    b.z = ps[(i + 1) % 4].z();
    armor_plane.points.push_back(a);
    armor_plane.points.push_back(b);
  }
  marker_array.markers.push_back(armor_plane);

  visualization_msgs::msg::Marker ell1;
  ell1.header = header;
  ell1.ns = "fire_prob/error_ellipse_1sigma";
  ell1.id = 0;
  ell1.type = visualization_msgs::msg::Marker::LINE_STRIP;
  ell1.action = visualization_msgs::msg::Marker::ADD;
  ell1.scale.x = 0.01;
  ell1.color.a = 0.95;
  ell1.color.r = 1.0;
  ell1.color.g = 0.9;
  ell1.color.b = 0.1;
  const int samples = std::max(16, fire_prob_vis_ellipse_samples_);
  for (int i = 0; i <= samples; ++i) {
    const double th = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
    const double du = fire_snapshot.sigma_u * std::cos(th);
    const double dv = fire_snapshot.sigma_v * std::sin(th);
    const Eigen::Vector3d p3 =
      fire_snapshot.armor_center + fire_snapshot.armor_right * du + fire_snapshot.armor_up * dv;
    geometry_msgs::msg::Point p;
    p.x = p3.x();
    p.y = p3.y();
    p.z = p3.z();
    ell1.points.push_back(p);
  }
  marker_array.markers.push_back(ell1);

  visualization_msgs::msg::Marker ell2 = ell1;
  ell2.ns = "fire_prob/error_ellipse_2sigma";
  ell2.id = 0;
  ell2.color.r = 1.0;
  ell2.color.g = 0.5;
  ell2.color.b = 0.1;
  ell2.points.clear();
  for (int i = 0; i <= samples; ++i) {
    const double th = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
    const double du = 2.0 * fire_snapshot.sigma_u * std::cos(th);
    const double dv = 2.0 * fire_snapshot.sigma_v * std::sin(th);
    const Eigen::Vector3d p3 =
      fire_snapshot.armor_center + fire_snapshot.armor_right * du + fire_snapshot.armor_up * dv;
    geometry_msgs::msg::Point p;
    p.x = p3.x();
    p.y = p3.y();
    p.z = p3.z();
    ell2.points.push_back(p);
  }
  marker_array.markers.push_back(ell2);

  visualization_msgs::msg::Marker mean_pt;
  mean_pt.header = header;
  mean_pt.ns = "fire_prob/mean_error_point";
  mean_pt.id = 0;
  mean_pt.type = visualization_msgs::msg::Marker::SPHERE;
  mean_pt.action = visualization_msgs::msg::Marker::ADD;
  mean_pt.scale.x = 0.04;
  mean_pt.scale.y = 0.04;
  mean_pt.scale.z = 0.04;
  mean_pt.color.a = 0.95;
  mean_pt.color.r = static_cast<float>(1.0 - fire_snapshot.p_hit_window);
  mean_pt.color.g = static_cast<float>(fire_snapshot.p_hit_window);
  mean_pt.color.b = 0.1f;
  const Eigen::Vector3d mean3 = fire_snapshot.armor_center +
                                fire_snapshot.armor_right * fire_snapshot.e_u +
                                fire_snapshot.armor_up * fire_snapshot.e_v;
  mean_pt.pose.position.x = mean3.x();
  mean_pt.pose.position.y = mean3.y();
  mean_pt.pose.position.z = mean3.z();
  mean_pt.pose.orientation.w = 1.0;
  marker_array.markers.push_back(mean_pt);

  visualization_msgs::msg::Marker cloud;
  cloud.header = header;
  cloud.ns = "fire_prob/impact_cloud";
  cloud.id = 0;
  cloud.type = visualization_msgs::msg::Marker::POINTS;
  cloud.action = visualization_msgs::msg::Marker::ADD;
  cloud.scale.x = 0.012;
  cloud.scale.y = 0.012;
  cloud.color.a = 0.25;
  cloud.color.r = 0.2;
  cloud.color.g = 0.9;
  cloud.color.b = 1.0;
  const int cloud_n = std::max(8, std::min(fire_prob_vis_max_impact_points_, 512));
  for (int i = 0; i < cloud_n; ++i) {
    const double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(cloud_n);
    const double rn = std::sqrt(static_cast<double>(i) / static_cast<double>(cloud_n));
    const double du = fire_snapshot.e_u + fire_snapshot.sigma_u * rn * std::cos(t);
    const double dv = fire_snapshot.e_v + fire_snapshot.sigma_v * rn * std::sin(t);
    const Eigen::Vector3d q =
      fire_snapshot.armor_center + fire_snapshot.armor_right * du + fire_snapshot.armor_up * dv;
    geometry_msgs::msg::Point p;
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
    cloud.points.push_back(p);
  }
  marker_array.markers.push_back(cloud);

  visualization_msgs::msg::Marker txt;
  txt.header = header;
  txt.ns = "fire_prob/text";
  txt.id = 0;
  txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  txt.action = visualization_msgs::msg::Marker::ADD;
  txt.scale.z = 0.10;
  txt.color.a = 1.0;
  txt.color.r = 0.95;
  txt.color.g = 0.95;
  txt.color.b = 0.95;
  txt.pose.position.x = fire_snapshot.armor_center.x();
  txt.pose.position.y = fire_snapshot.armor_center.y();
  txt.pose.position.z = fire_snapshot.armor_center.z() + 0.20;
  txt.pose.orientation.w = 1.0;
  std::ostringstream oss;
  oss << "Pwin=" << std::fixed << std::setprecision(2) << fire_snapshot.p_hit_window
      << " Score=" << fire_snapshot.fire_score;
  if (fire_snapshot.gate_strategy == 1) {
    oss << " Pb=" << std::setprecision(2) << fire_snapshot.burst_probability
        << " S=" << std::setprecision(2) << fire_snapshot.evidence_strength
        << " G=" << fire_snapshot.gate_state;
  }
  oss << " tau=" << std::setprecision(1) << fire_snapshot.best_tau_ms << "ms"
      << " eu=" << std::setprecision(3) << fire_snapshot.e_u << "m"
      << " ev=" << fire_snapshot.e_v << "m"
      << " su=" << fire_snapshot.sigma_u << "m"
      << " sv=" << fire_snapshot.sigma_v << "m"
      << " fire=" << (fire_snapshot.fire_advice ? 1 : 0);
  txt.text = oss.str();
  marker_array.markers.push_back(txt);
}

void GimbalPipelineNode::publishFireProbabilityDebugImages(
  const std_msgs::msg::Header &header,
  const gimbal_controller::FireAdviceDebugSnapshot &fire_snapshot) {
  if (!fire_prob_image_debug_enable_ || !debug_fire_plane_image_pub_ ||
      !debug_fire_normal_image_pub_) {
    return;
  }
  if (!fire_snapshot.probability_enabled || fire_snapshot.tau_samples.empty()) {
    return;
  }

  const rclcpp::Time stamp = header.stamp;
  const double min_period = 1.0 / std::max(fire_prob_image_debug_publish_rate_hz_, 0.1);
  if (last_fire_prob_image_pub_time_.nanoseconds() > 0 &&
      (stamp - last_fire_prob_image_pub_time_).seconds() < min_period) {
    return;
  }
  last_fire_prob_image_pub_time_ = stamp;

  const int w = fire_prob_image_debug_width_;
  const int h = fire_prob_image_debug_height_;
  cv::Mat plane(h, w, CV_8UC3, cv::Scalar(18, 18, 18));
  cv::Mat normal(h, w, CV_8UC3, cv::Scalar(18, 18, 18));

  const int margin = 40;
  const cv::Point2d center_plane(w * 0.45, h * 0.55);
  const double half_w = std::max(fire_snapshot.armor_width_m * 0.5, 1e-6);
  const double half_h = std::max(fire_snapshot.armor_height_m * 0.5, 1e-6);
  const double sx = (w * 0.35 - margin) / half_w;
  const double sy = (h * 0.35 - margin) / half_h;
  const double scale = std::min(sx, sy);

  const cv::Rect armor_rect(static_cast<int>(center_plane.x - half_w * scale),
                            static_cast<int>(center_plane.y - half_h * scale),
                            static_cast<int>(2.0 * half_w * scale),
                            static_cast<int>(2.0 * half_h * scale));
  cv::rectangle(plane, armor_rect, cv::Scalar(220, 220, 220), 2);
  cv::line(plane,
           cv::Point(armor_rect.x, static_cast<int>(center_plane.y)),
           cv::Point(armor_rect.x + armor_rect.width, static_cast<int>(center_plane.y)),
           cv::Scalar(80, 80, 80),
           1);
  cv::line(plane,
           cv::Point(static_cast<int>(center_plane.x), armor_rect.y),
           cv::Point(static_cast<int>(center_plane.x), armor_rect.y + armor_rect.height),
           cv::Scalar(80, 80, 80),
           1);

  for (const auto &s : fire_snapshot.tau_samples) {
    const int px = static_cast<int>(center_plane.x + s.e_u * scale);
    const int py = static_cast<int>(center_plane.y - s.e_v * scale);
    const int g = static_cast<int>(255.0 * std::clamp(s.p_hit, 0.0, 1.0));
    const int r = 255 - g;
    cv::circle(plane, cv::Point(px, py), 3, cv::Scalar(30, g, r), -1);
  }

  if (fire_prob_image_debug_show_sigma_ellipse_) {
    const int a1 = std::max(1, static_cast<int>(std::abs(fire_snapshot.sigma_u) * scale));
    const int b1 = std::max(1, static_cast<int>(std::abs(fire_snapshot.sigma_v) * scale));
    cv::ellipse(
      plane, center_plane, cv::Size(a1, b1), 0.0, 0.0, 360.0, cv::Scalar(80, 200, 255), 2);
    cv::ellipse(
      plane, center_plane, cv::Size(2 * a1, 2 * b1), 0.0, 0.0, 360.0, cv::Scalar(80, 130, 255), 1);
  }

  const cv::Point best_pt(static_cast<int>(center_plane.x + fire_snapshot.e_u * scale),
                          static_cast<int>(center_plane.y - fire_snapshot.e_v * scale));
  cv::circle(plane, best_pt, 6, cv::Scalar(0, 255, 255), 2);

  const gimbal_controller::fire_advice::TauDebugSample *best_s = &fire_snapshot.tau_samples.front();
  double min_tau_diff = std::numeric_limits<double>::max();
  const double best_tau_s = fire_snapshot.best_tau_ms * 1e-3;
  for (const auto &sample : fire_snapshot.tau_samples) {
    const double d = std::abs(sample.tau_s - best_tau_s);
    if (d < min_tau_diff) {
      min_tau_diff = d;
      best_s = &sample;
    }
  }
  const bool best_front_ok = best_s->front_ok;
  const bool best_gate_ok = best_s->normal_gate_pass;

  if (fire_prob_image_debug_show_text_) {
    std::ostringstream oss1;
    oss1 << "Pwin=" << std::fixed << std::setprecision(2) << fire_snapshot.p_hit_window
         << " Score=" << fire_snapshot.fire_score << " Tau=" << std::setprecision(1)
         << fire_snapshot.best_tau_ms << "ms"
         << " Fire=" << (fire_snapshot.fire_advice ? "Y" : "N");
    cv::putText(plane,
                oss1.str(),
                cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(230, 230, 230),
                1);
    std::ostringstream oss2;
    oss2 << "eu=" << std::setprecision(3) << fire_snapshot.e_u << " ev=" << fire_snapshot.e_v
         << " su=" << fire_snapshot.sigma_u << " sv=" << fire_snapshot.sigma_v;
    cv::putText(plane,
                oss2.str(),
                cv::Point(20, 55),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(200, 200, 200),
                1);
  }

  const cv::Point2d c2(w * 0.50, h * 0.58);
  const double axis_len = std::min(w, h) * 0.28;
  const cv::Point2d n_tip(c2.x + axis_len, c2.y);
  cv::arrowedLine(normal, c2, n_tip, cv::Scalar(80, 220, 80), 3, cv::LINE_AA, 0, 0.05);
  cv::putText(normal,
              "armor normal +n",
              cv::Point(static_cast<int>(n_tip.x) - 30, static_cast<int>(n_tip.y) - 10),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(120, 240, 120),
              1);

  const double v_n = std::max(0.0, best_s->normal_velocity);
  const double v_ref = std::max(
    1e-3, get_parameter("controller.fire.probability.normal_velocity_weight.v_ref").as_double());
  const double ratio = std::clamp(v_n / v_ref, 0.0, 1.0);
  const double spread_deg = (1.0 - ratio) * 35.0;
  const double center_deg = best_front_ok ? 180.0 : 0.0;
  const double theta_deg = center_deg + (best_front_ok ? -spread_deg : spread_deg);
  const double theta = theta_deg * M_PI / 180.0;
  const cv::Point2d v_tip(c2.x + axis_len * std::cos(theta), c2.y - axis_len * std::sin(theta));
  cv::arrowedLine(normal, c2, v_tip, cv::Scalar(80, 180, 255), 3, cv::LINE_AA, 0, 0.05);
  cv::putText(normal,
              "bullet velocity",
              cv::Point(static_cast<int>(v_tip.x) - 30, static_cast<int>(v_tip.y) - 8),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(120, 200, 255),
              1);

  if (fire_prob_image_debug_show_velocity_fan_) {
    const cv::Scalar fan_color = best_front_ok ? cv::Scalar(60, 200, 60) : cv::Scalar(60, 60, 220);
    const double fan_half = 15.0 + (1.0 - ratio) * 30.0;
    cv::ellipse(normal,
                c2,
                cv::Size(static_cast<int>(axis_len * 0.7), static_cast<int>(axis_len * 0.7)),
                0.0,
                center_deg - fan_half,
                center_deg + fan_half,
                fan_color,
                2,
                cv::LINE_AA);
  }

  cv::line(normal,
           cv::Point(static_cast<int>(c2.x), static_cast<int>(c2.y)),
           cv::Point(static_cast<int>(c2.x + axis_len * ratio), static_cast<int>(c2.y)),
           cv::Scalar(0, 255, 255),
           4,
           cv::LINE_AA);
  cv::putText(normal,
              "v_n along normal",
              cv::Point(static_cast<int>(c2.x), static_cast<int>(c2.y) + 24),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(0, 255, 255),
              1);

  const cv::Scalar status_color =
    (best_front_ok && best_gate_ok) ? cv::Scalar(80, 240, 80) : cv::Scalar(80, 80, 240);
  const std::string status_text =
    (best_front_ok && best_gate_ok) ? "HIT GATE: PASS" : "HIT GATE: BLOCK";
  cv::putText(
    normal, status_text, cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.85, status_color, 2);

  std::ostringstream ns1;
  ns1 << "front_ok=" << (best_front_ok ? "Y" : "N") << " gate_ok=" << (best_gate_ok ? "Y" : "N")
      << " normal_v=" << std::fixed << std::setprecision(2) << best_s->normal_velocity << " m/s";
  cv::putText(normal,
              ns1.str(),
              cv::Point(20, 65),
              cv::FONT_HERSHEY_SIMPLEX,
              0.55,
              cv::Scalar(220, 220, 220),
              1);
  std::ostringstream ns2;
  ns2 << "normal_weight=" << std::setprecision(2) << best_s->normal_weight
      << " p_hit=" << best_s->p_hit;
  cv::putText(normal,
              ns2.str(),
              cv::Point(20, 90),
              cv::FONT_HERSHEY_SIMPLEX,
              0.55,
              cv::Scalar(220, 220, 220),
              1);

  auto plane_msg = cv_bridge::CvImage(header, "bgr8", plane).toImageMsg();
  auto normal_msg = cv_bridge::CvImage(header, "bgr8", normal).toImageMsg();
  debug_fire_plane_image_pub_->publish(*plane_msg);
  debug_fire_normal_image_pub_->publish(*normal_msg);
}

void GimbalPipelineNode::publishManeuverMarkers(const std_msgs::msg::Header &header) {
  visualization_msgs::msg::MarkerArray arr;

  // Tracker positions are expressed in target_frame_ (== visualization_frame_).
  // Use visualization_frame_ explicitly to avoid the camera source frame mismatch.
  std_msgs::msg::Header viz_header;
  viz_header.stamp = header.stamp;
  viz_header.frame_id = visualization_frame_;

  int id = 0;

  const auto tracker_views = tracker_manager_->initialized_tracker_views();
  for (const auto &view : tracker_views) {
    if (!view.tracker) continue;

    const auto result = view.tracker->assess_maneuver();
    const auto pos = view.tracker->get_center_position();

    // Estimate robot top: center pos + half robot height (~0.25 m)
    const double top_z = pos.z() + 0.25;

    // ── Sphere marker (at robot top) ───────────────────────────
    visualization_msgs::msg::Marker sphere;
    sphere.header = viz_header;
    sphere.ns = "maneuver";
    sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = pos.x();
    sphere.pose.position.y = pos.y();
    sphere.pose.position.z = top_z;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.15);
    if (result.is_maneuvering) {
      sphere.color.r = 0.9f;
      sphere.color.g = 0.1f;
      sphere.color.b = 0.1f;
      sphere.color.a = 0.8f;
    } else {
      sphere.color.r = 0.1f;
      sphere.color.g = 0.9f;
      sphere.color.b = 0.1f;
      sphere.color.a = 0.6f;
    }
    arr.markers.push_back(sphere);

    // ── Text marker (above sphere) ─────────────────────────────
    visualization_msgs::msg::Marker text;
    text.header = viz_header;
    text.ns = "maneuver_text";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = pos.x();
    text.pose.position.y = pos.y();
    text.pose.position.z = top_z + 0.15;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.08;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.lifetime = rclcpp::Duration::from_seconds(0.15);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "NIS=%.0f\nIN=%.3f", result.nis, result.innov_norm);
    text.text = buf;
    arr.markers.push_back(text);
  }

  if (!arr.markers.empty()) debug_maneuver_pub_->publish(arr);
}

void GimbalPipelineNode::publishArmorSelectionDebug(
  const gimbal_controller::GimbalControlContext &context, const std::string &strategy_name) {
  if (!debug_armor_selection_pub_ || !armor_selector_ || !position_calculator_) {
    return;
  }
  if (!context.is_tracking) {
    return;
  }

  const auto normalized_target =
    robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const auto center_position =
    robot_description::TrackedRobotUsage::centerPosition(normalized_target);
  const double target_yaw = robot_description::TrackedRobotUsage::yaw(normalized_target);
  const double target_yaw_velocity =
    robot_description::TrackedRobotUsage::yawVelocity(normalized_target);

  std::vector<Eigen::Vector3d> armor_positions;
  Eigen::Vector3d select_center = center_position;
  double select_yaw = target_yaw;

  if (strategy_name == "mpc") {
    const double t_ahead = mpc_dt_debug_;
    armor_positions = position_calculator_->calculatePredicted(normalized_target, t_ahead);
    select_center = robot_description::TrackedRobotUsage::predictCenter(
      normalized_target,
      t_ahead,
      robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
    select_yaw = robot_description::TrackedRobotUsage::predictYaw(
      normalized_target,
      t_ahead,
      robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
  } else {
    armor_positions = position_calculator_->calculate(normalized_target);
  }

  if (armor_positions.empty()) {
    return;
  }

  auto debug_selector = *armor_selector_;
  const auto selection = debug_selector.selectBest(armor_positions,
                                                   select_center,
                                                   select_yaw,
                                                   normalized_target.num_armors,
                                                   target_yaw_velocity,
                                                   context.current_yaw,
                                                   context.current_pitch);

  std_msgs::msg::String out;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4) << "strategy=" << strategy_name
      << " sel=" << selection.selected_index << " real_sel=" << selection.real_selected_index
      << " is_virtual=" << (selection.is_virtual_target ? 1 : 0)
      << " fallback=" << (selection.is_center_fallback ? 1 : 0) << " dist=" << selection.distance
      << " move=" << selection.gimbal_movement << " v_yaw=" << selection.virtual_robot_yaw
      << " d_yaw=" << selection.virtual_delta_yaw << " pos=(" << selection.position.x() << ","
      << selection.position.y() << "," << selection.position.z() << ")"
      << " real=(" << selection.real_position.x() << "," << selection.real_position.y() << ","
      << selection.real_position.z() << ")";
  if (strategy_name == "mpc") {
    oss << " note=first_predicted_selection";
  }
  out.data = oss.str();
  debug_armor_selection_pub_->publish(out);
}

std::array<float, 4> GimbalPipelineNode::hsvToRgb(float h, float s, float v) {
  float c = v * s;
  float x = c * (1 - std::abs(std::fmod(h / 60.0f, 2.0f) - 1));
  float m = v - c;
  float r, g, b;
  if (h < 60) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 120) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 180) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 240) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 300) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }
  return {r + m, g + m, b + m, 1.0f};
}

}  // namespace fyt::auto_aim
