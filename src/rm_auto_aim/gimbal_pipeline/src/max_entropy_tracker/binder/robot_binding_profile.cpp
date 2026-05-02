// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

HeightLabel RobotBindingProfile::height_label_for(int panel_id) const {
  if (panel_id < 0 || panel_id >= panel_count) return HeightLabel::UNKNOWN;
  if (z_offsets.empty() || z_offsets.size() != static_cast<size_t>(panel_count))
    return HeightLabel::UNKNOWN;

  if (height_levels == 2) {
    return (panel_id % 2 == 0) ? HeightLabel::LOWER : HeightLabel::UPPER;
  }

  if (height_levels == 3) {
    std::vector<std::pair<double, int>> z_with_id;
    z_with_id.reserve(z_offsets.size());
    for (int i = 0; i < panel_count; ++i) {
      z_with_id.emplace_back(z_offsets[i], i);
    }
    std::sort(z_with_id.begin(), z_with_id.end(),
              [](const auto & a, const auto & b) { return a.first < b.first; });

    const int low_id = z_with_id.front().second;
    const int high_id = z_with_id.back().second;
    int mid_id = low_id;
    if (panel_count >= 3) {
      mid_id = z_with_id[panel_count / 2].second;
    }

    if (panel_id == low_id) return HeightLabel::LOWER;
    if (panel_id == high_id) return HeightLabel::UPPER;
    if (panel_id == mid_id) return HeightLabel::MIDDLE;
  }

  // Fallback: preserve a stable 2-level behavior for unsupported layouts.
  const double z_this = z_offsets[panel_id];
  std::vector<double> sorted = z_offsets;
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted[panel_count / 2];
  return (z_this >= median) ? HeightLabel::UPPER : HeightLabel::LOWER;
}

RobotBindingProfile RobotBindingProfileProvider::from_robot_id(
    const std::string & robot_id,
    const std::vector<double> & armors_offset) {
  RobotBindingProfile p;

  if (robot_id == "outpost") {
    p.robot_type = 3;  // OUTPOST_3
    p.panel_count = 3;
    p.z_offsets = {0.06, 0.0, -0.06};
    p.cyclic_order = {0, 2, 1};
    p.has_dual_obs_capability = false;
    p.primary_jump_kind = JumpKind::DOUBLE_DZ;
    p.height_levels = 3;
  } else if (robot_id == "base") {
    p.robot_type = 5;  // BASE
    p.panel_count = 3;
    p.z_offsets = {0.06, 0.0, -0.06};
    p.cyclic_order = {0, 2, 1};
    p.has_dual_obs_capability = false;
    p.primary_jump_kind = JumpKind::DZ;
    p.height_levels = 3;
  } else if (robot_id == "sentry") {
    p.robot_type = 4;  // SENTRY
    p.panel_count = 4;
    p.cyclic_order = {0, 1, 2, 3};
    p.has_dual_obs_capability = true;
    p.primary_jump_kind = JumpKind::DZ;
    p.height_levels = 2;
    if (!armors_offset.empty()) {
      p.z_offsets = armors_offset;
    }
  } else {
    // "1", "2", "3", "4", "5", "hero" → STANDARD_4 or HERO_4
    p.robot_type = (robot_id == "1" || robot_id == "hero") ? 2 : 1;
    p.panel_count = 4;
    p.cyclic_order = {0, 1, 2, 3};
    p.has_dual_obs_capability = true;
    p.primary_jump_kind = JumpKind::DZ;
    p.height_levels = 2;
    if (!armors_offset.empty()) {
      p.z_offsets = armors_offset;
    }
  }

  return p;
}

RobotBindingProfile RobotBindingProfileProvider::from_robot_type(
    uint8_t robot_type, int num_armors) {
  RobotBindingProfile p;
  p.robot_type = robot_type;
  p.panel_count = num_armors;

  if (num_armors == 3) {
    p.z_offsets = {0.06, 0.0, -0.06};
    p.cyclic_order = {0, 2, 1};
    p.has_dual_obs_capability = false;
    p.primary_jump_kind = JumpKind::DOUBLE_DZ;
    p.height_levels = 3;
  } else {
    p.cyclic_order.clear();
    for (int i = 0; i < num_armors; ++i) p.cyclic_order.push_back(i);
    p.has_dual_obs_capability = (num_armors >= 4);
    p.primary_jump_kind = JumpKind::DZ;
    p.height_levels = 2;
  }

  return p;
}

bool RobotBindingProfileProvider::validate(const RobotBindingProfile & profile) {
  if (profile.panel_count < 2) return false;
  if (!profile.z_offsets.empty() &&
      profile.z_offsets.size() != static_cast<size_t>(profile.panel_count))
    return false;
  if (profile.cyclic_order.size() != static_cast<size_t>(profile.panel_count))
    return false;
  if (profile.height_levels != 2 && profile.height_levels != 3) return false;
  return true;
}

}  // namespace fyt::auto_aim::binder
