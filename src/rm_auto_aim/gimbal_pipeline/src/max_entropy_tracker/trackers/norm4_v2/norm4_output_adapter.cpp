// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"

namespace fyt::auto_aim::norm4_v2 {

Norm4OutputAdapter::Norm4OutputAdapter(const UnifiedConfig &cfg) {
  (void)cfg;
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
    const Norm4RuntimeContext & /*ctx*/) const {
  return {};
}

}  // namespace fyt::auto_aim::norm4_v2
