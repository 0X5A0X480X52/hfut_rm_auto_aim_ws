// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace fyt::auto_aim::norm4_v2 {

namespace {

int sanitize_panel_id(int panel_id) {
  int p = panel_id % 4;
  if (p < 0) p += 4;
  return p;
}

double extractPitch(const geometry_msgs::msg::Quaternion &msg) {
  tf2::Quaternion q;
  tf2::fromMsg(msg, q);
  if (q.length2() <= 1e-12) {
    return 0.0;
  }
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

}  // namespace

Norm4OutputAdapter::Norm4OutputAdapter(const UnifiedConfig &cfg) {
  const double raw_step = std::abs(cfg.tracker.panel_angle_step) > 1e-6
                              ? std::abs(cfg.tracker.panel_angle_step)
                              : (M_PI / 2.0);
  panel_angles_ = {0.0, raw_step, 2.0 * raw_step, -raw_step};
  publish_single_semantics_ = true;
}

void Norm4OutputAdapter::fill_from_armor(Norm4RuntimeContext *ctx,
                                         const AmbiguousArmorSnapshot &snap) const {
  ctx->publish_pos = snap.armor_pos;
  ctx->publish_vel = snap.armor_vel;
  ctx->center_pos = snap.armor_pos;
  ctx->center_vel = snap.armor_vel;
  ctx->center_yaw = snap.armor_yaw;
  ctx->yaw_rate = snap.armor_yaw_rate;
}

void Norm4OutputAdapter::fill_from_center(Norm4RuntimeContext *ctx,
                                          const BackendStateSnapshot &snap) const {
  ctx->center_pos = snap.center_pos;
  ctx->center_vel = snap.center_vel;
  ctx->center_yaw = snap.center_yaw;
  ctx->yaw_rate = snap.yaw_rate;
  ctx->publish_pos = snap.center_pos;
  ctx->publish_vel = snap.center_vel;
}

void Norm4OutputAdapter::update_publish_state(
    Norm4RuntimeContext *ctx, const PublishStateInput &input) const {
  if (ctx == nullptr) return;
  if (input.mode == mode::TrackMode::AMBIGUOUS && input.armor_snap != nullptr &&
      publish_single_semantics_) {
    fill_from_armor(ctx, *input.armor_snap);
  } else if (input.backend_snap != nullptr) {
    fill_from_center(ctx, *input.backend_snap);
  }
}

std::vector<geometry_msgs::msg::Pose>
Norm4OutputAdapter::build_armors_offset_for_message(
    const Norm4RuntimeContext &ctx) const {
  std::vector<geometry_msgs::msg::Pose> offsets;

  double Norm4ArmorPitchUp = -0.2618;  // -15 deg

  if (ctx.mode == mode::TrackMode::AMBIGUOUS && publish_single_semantics_) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, Norm4ArmorPitchUp, 0.0);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
    return offsets;
  }

  offsets.reserve(4);
  const double r1 = std::max(0.05, ctx.r1);
  const double r2 = std::max(0.05, ctx.r2);
  const double dza = std::abs(ctx.dza);

  for (int i = 0; i < 4; ++i) {
    const int panel_id = sanitize_panel_id(i);
    const double angle = panel_angles_[panel_id];
    const double radius = (panel_id % 2 == 0) ? r1 : r2;
    const double dz = (panel_id % 2 == 0) ? -dza : dza;

    geometry_msgs::msg::Pose pose;
    pose.position.x = radius * std::cos(angle);
    pose.position.y = radius * std::sin(angle);
    pose.position.z = dz;

    tf2::Quaternion q;
    const double pitch = Norm4ArmorPitchUp;
    q.setRPY(0.0, pitch, angle);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
  }

  return offsets;
}

}  // namespace fyt::auto_aim::norm4_v2
