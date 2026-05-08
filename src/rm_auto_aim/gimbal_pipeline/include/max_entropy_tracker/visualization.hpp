// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
//
// Visualization module — generates MarkerArray for RViz display.
// Markers include:
//   - Robot center (SPHERE, green/orange)
//   - Robot ID text label (TEXT_VIEW_FACING)
//   - Predicted armor positions (CUBE, blue)
//   - Velocity arrow (ARROW, yellow)
//   - Yaw direction arrow (ARROW, red)
//
// Designed to match the Python visualization.py behaviour, with
// armor rendering style inspired by robot_pose_estimator_node.

#ifndef MAX_ENTROPY_TRACKER_VISUALIZATION_HPP_
#define MAX_ENTROPY_TRACKER_VISUALIZATION_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"
#include "max_entropy_tracker/tracker_manager.hpp"

namespace fyt::auto_aim {

/// Build a MarkerArray visualising every initialised tracker.
inline visualization_msgs::msg::MarkerArray build_tracker_markers(
    const std::string &target_frame,
    const std::vector<TrackerManager::TrackerConstView> &tracker_views,
    const rclcpp::Time &stamp) {
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  for (const auto &view : tracker_views) {
    if (!view.tracker) {
      continue;
    }
    const auto &robot_id = view.robot_id;
    const auto &tracker = *view.tracker;
    if (!tracker.is_initialized()) continue;

    // ---------- gather state ----------
    auto pos  = tracker.get_center_position();
    double yaw = tracker.get_yaw();
    auto [r1, r2] = tracker.get_radii();
    const auto &filter = tracker.spin_filter();
    double dza = filter.get_dza();
    const auto vel = tracker.get_publish_velocity();
    double vx = vel.x();
    double vy = vel.y();
    double vz = vel.z();

    // ---------- 1. Robot center SPHERE ----------
    {
      Marker m;
      m.header.frame_id = target_frame;
      m.header.stamp    = stamp;
      m.ns   = "robot_center_" + robot_id;
      m.id   = id++;
      m.type = Marker::SPHERE;
      m.action = Marker::ADD;

      m.pose.position.x = pos.x();
      m.pose.position.y = pos.y();
      m.pose.position.z = pos.z();
      m.pose.orientation.w = 1.0;

      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;

      if (tracker.is_tracking()) {
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f; // green
      } else {
        m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.6f; // orange
      }

      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      marker_array.markers.push_back(m);
    }

    // ---------- 2. Robot ID text label ----------
    {
      Marker m;
      m.header.frame_id = target_frame;
      m.header.stamp    = stamp;
      m.ns   = "robot_label_" + robot_id;
      m.id   = id++;
      m.type = Marker::TEXT_VIEW_FACING;
      m.action = Marker::ADD;

      m.pose.position.x = pos.x();
      m.pose.position.y = pos.y();
      m.pose.position.z = pos.z() + 0.3;
      m.pose.orientation.w = 1.0;

      m.text = "ID:" + robot_id;
      m.scale.z = 0.15;

      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;

      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      marker_array.markers.push_back(m);
    }

    // ---------- 3. Predicted armor CUBE ----------
    {
      // Determine armor count from robot type
      using T = rm_interfaces::msg::TrackedRobot;
      uint8_t rtype = T::STANDARD_4;
      if (robot_id == "outpost") rtype = T::OUTPOST_3;
      else if (robot_id == "base") rtype = T::BASE;
      else if (robot_id == "1") rtype = T::HERO_4;

      int n_armors = 4;
      if (rtype == T::OUTPOST_3 || rtype == T::BASE) n_armors = 3;
      else if (rtype == T::BALANCE_2) n_armors = 2;

      const int runtime_num_armors = tracker.effective_num_armors();
      if (runtime_num_armors > 0) n_armors = runtime_num_armors;

      // Armor size heuristic (same as robot_pose_estimator)
      bool is_large = (rtype == T::BALANCE_2 || rtype == T::HERO_4 ||
                       rtype == T::OUTPOST_3 || rtype == T::BASE);
      double armor_width = is_large ? 0.23 : 0.135;

      auto armor_offsets = tracker.build_armors_offset_for_message();
      if (armor_offsets.empty()) {
        armor_offsets = robot_description::TrackedRobotUsage::generateArmorsOffsetFromProfile(
            n_armors, r1, r2, dza, 0.0);
      }

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      tf2::Quaternion q_world_yaw;
      q_world_yaw.setRPY(0.0, 0.0, yaw);

      for (size_t i = 0; i < armor_offsets.size(); ++i) {
        const auto &offset = armor_offsets[i];
        const double world_ox =
            offset.position.x * cos_yaw - offset.position.y * sin_yaw;
        const double world_oy =
            offset.position.x * sin_yaw + offset.position.y * cos_yaw;

        Marker m;
        m.header.frame_id = target_frame;
        m.header.stamp    = stamp;
        m.ns   = "armor_" + robot_id;
        m.id   = id++;
        m.type = Marker::CUBE;
        m.action = Marker::ADD;

        m.pose.position.x = pos.x() + world_ox;
        m.pose.position.y = pos.y() + world_oy;
        m.pose.position.z = pos.z() + offset.position.z;

        tf2::Quaternion q_offset;
        tf2::fromMsg(offset.orientation, q_offset);
        if (q_offset.length2() <= 1e-12) {
          q_offset.setRPY(0.0, 0.0, 0.0);
        } else {
          q_offset.normalize();
        }
        m.pose.orientation = tf2::toMsg(q_world_yaw * q_offset);

        m.scale.x = 0.03;           // thickness
        m.scale.y = armor_width;    // width
        m.scale.z = 0.125;          // height

        m.color.r = 0.0f; m.color.g = 0.5f; m.color.b = 1.0f; m.color.a = 0.8f;

        m.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(m);
      }
    }

    // ---------- 4. Velocity ARROW ----------
    {
      double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (speed > 0.1) {
        Marker m;
        m.header.frame_id = target_frame;
        m.header.stamp    = stamp;
        m.ns   = "velocity_" + robot_id;
        m.id   = id++;
        m.type = Marker::ARROW;
        m.action = Marker::ADD;

        double scale = std::min(speed * 0.5, 1.0);
        geometry_msgs::msg::Point start, end;
        start.x = pos.x(); start.y = pos.y(); start.z = pos.z();
        end.x = pos.x() + vx * scale / speed;
        end.y = pos.y() + vy * scale / speed;
        end.z = pos.z() + vz * scale / speed;
        m.points = {start, end};

        m.scale.x = 0.03;  // shaft diameter
        m.scale.y = 0.06;  // head diameter
        m.scale.z = 0.0;

        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f; // yellow

        m.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(m);
      }
    }

    // ---------- 5. Yaw direction ARROW ----------
    {
      constexpr double yaw_length = 0.3;

      Marker m;
      m.header.frame_id = target_frame;
      m.header.stamp    = stamp;
      m.ns   = "yaw_" + robot_id;
      m.id   = id++;
      m.type = Marker::ARROW;
      m.action = Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = pos.x(); start.y = pos.y(); start.z = pos.z();
      end.x = pos.x() + yaw_length * std::cos(yaw);
      end.y = pos.y() + yaw_length * std::sin(yaw);
      end.z = pos.z();
      m.points = {start, end};

      m.scale.x = 0.02;
      m.scale.y = 0.04;
      m.scale.z = 0.0;

      m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.8f; // red

      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      marker_array.markers.push_back(m);
    }
  }

  return marker_array;
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_VISUALIZATION_HPP_
