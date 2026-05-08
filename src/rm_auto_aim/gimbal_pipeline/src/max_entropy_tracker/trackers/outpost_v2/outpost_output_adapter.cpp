// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::outpost_v2 {

namespace {

// RViz/tf2 positive pitch rotates the local armor normal toward -Z.
constexpr double kOutpostPitchDown = 0.2618;

}  // namespace

OutpostOutputAdapter::OutpostOutputAdapter(const UnifiedConfig &cfg) {
  radius_ = std::max(0.05, cfg.outpost.radius);
  z_offsets_ = {
      cfg.outpost.z_offset_0,
      cfg.outpost.z_offset_1,
      cfg.outpost.z_offset_2,
  };
  const double raw_step = (cfg.outpost.panel_angle_step > 1e-6)
                              ? cfg.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  panel_angles_ = {0.0, step, -step};
  publish_single_semantics_ = cfg.outpost.ambiguous_publish_single_armor_semantics;
  ambiguous_zero_offset_ = cfg.outpost.ambiguous_single_armor_zero_offset;
}

void OutpostOutputAdapter::fill_from_armor(
    OutpostRuntimeContext *ctx, const AmbiguousArmorSnapshot &snap) const {
  // Publish the raw single-armor state directly (no center back-projection).
  ctx->publish_pos = snap.armor_pos;
  ctx->publish_vel = snap.armor_vel;

  // Also set center fields to the armor state, so that BaseTracker
  // accessors (get_center_position / get_yaw) return single-armor values
  // in ambiguous mode — the FixedProfileTrackedRobotBuilder will then
  // interpret these as single-armor data (num_armors=1).
  ctx->center_pos = snap.armor_pos;
  ctx->center_vel = snap.armor_vel;
  ctx->center_yaw = snap.armor_yaw;
  ctx->yaw_rate = snap.armor_yaw_rate;
}

void OutpostOutputAdapter::fill_from_center(
    OutpostRuntimeContext *ctx, const BackendStateSnapshot &snap) const {
  ctx->center_pos = snap.center_pos;
  ctx->center_vel = snap.center_vel;
  ctx->center_yaw = snap.center_yaw;
  ctx->yaw_rate = snap.yaw_rate;
  ctx->publish_pos = snap.center_pos;
  ctx->publish_vel = snap.center_vel;
}

void OutpostOutputAdapter::update_publish_state(
    OutpostRuntimeContext *ctx, const PublishStateInput &input) const {
  if (ctx == nullptr) return;

  if (input.mode == mode::TrackMode::AMBIGUOUS && input.armor_snap != nullptr &&
      publish_single_semantics_) {
    fill_from_armor(ctx, *input.armor_snap);
  } else if (input.backend_snap != nullptr) {
    fill_from_center(ctx, *input.backend_snap);
  }
}

std::vector<geometry_msgs::msg::Pose>
OutpostOutputAdapter::build_armors_offset_for_message(
    const OutpostRuntimeContext &ctx) const {
  std::vector<geometry_msgs::msg::Pose> offsets;

  double OutpostArmorPitchDown = 0.2618;  // +15 deg

  if (ctx.mode == mode::TrackMode::AMBIGUOUS) {
    if (ambiguous_zero_offset_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, OutpostArmorPitchDown, 0.0);
      pose.orientation = tf2::toMsg(q);
      offsets.push_back(pose);
      return offsets;
    }
    int pid = ctx.selected_panel_id;
    if (pid < 0 || pid >= 3) pid = 0;
    geometry_msgs::msg::Pose pose;
    const double angle = panel_angles_[pid];
    pose.position.x = -radius_ * std::cos(angle);
    pose.position.y = -radius_ * std::sin(angle);
    pose.position.z = z_offsets_[pid];
    tf2::Quaternion q;
    q.setRPY(0.0, OutpostArmorPitchDown, angle + M_PI);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
    return offsets;
  }

  offsets.reserve(3);
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::Pose pose;
    const double angle = panel_angles_[i];
    pose.position.x = -radius_ * std::cos(angle);
    pose.position.y = -radius_ * std::sin(angle);
    pose.position.z = z_offsets_[i];
    tf2::Quaternion q;
    q.setRPY(0.0, OutpostArmorPitchDown, angle + M_PI);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
  }
  return offsets;
}

}  // namespace fyt::auto_aim::outpost_v2
