// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"

namespace {

using fyt::auto_aim::AdaptiveArmorTracker;
using fyt::auto_aim::ObservationData;
using fyt::auto_aim::UnifiedConfig;
using fyt::auto_aim::HeightLabel;

ObservationData makeObservation(double center_x,
                                double center_y,
                                double center_yaw,
                                double radius,
                                int panel_id,
                                double z,
                                double timestamp,
                                double panel_step) {
  ObservationData obs;
  const double armor_yaw = center_yaw + panel_id * panel_step;
  obs.x = center_x + radius * std::cos(armor_yaw);
  obs.y = center_y + radius * std::sin(armor_yaw);
  obs.z = z;
  obs.yaw = armor_yaw;
  obs.timestamp = timestamp;
  return obs;
}

UnifiedConfig makeConfig() {
  UnifiedConfig cfg = UnifiedConfig::create_default();
  cfg.tracker.jump_binding_enable = true;
  cfg.tracker.jump_binding_confirm_frames = 1;
  cfg.tracker.jump_binding_z_jump_min = 0.001;
  cfg.tracker.jump_binding_dz_match_tolerance = 0.20;
  cfg.tracker.jump_binding_dz_gate = 0.0;
  cfg.tracker.jump_binding_yaw_err_gate = 1.0;
  cfg.tracker.jump_binding_cost_margin_min = 0.0;
  cfg.tracker.jump_binding_switch_cooldown = 0;
  cfg.tracker.jump_binding_confidence_floor = 0.0;
  cfg.tracker.degraded_single_obs_enable = false;
  cfg.tracker.panel_angle_step = M_PI / 2.0;
  return cfg;
}

}  // namespace

TEST(AdaptiveJumpBinding, ConfirmsAdjacentPanelJump) {
  auto cfg = makeConfig();
  AdaptiveArmorTracker tracker(cfg, 0.05, false);

  constexpr double center_x = 0.0;
  constexpr double center_y = 0.0;
  constexpr double center_yaw = 0.2;
  constexpr double radius = 0.20;
  constexpr double z0 = 0.10;

  const auto init_obs =
      makeObservation(center_x, center_y, center_yaw, radius, 0, z0, 0.00,
                      cfg.tracker.panel_angle_step);
  tracker.initialize({init_obs});

  auto snap = tracker.debug_snapshot();
  ASSERT_TRUE(snap.valid);
  EXPECT_EQ(snap.current_panel_id, 0);
  EXPECT_EQ(snap.bound_panel_id, 0);
  EXPECT_EQ(snap.binding_transition_state, 0);
  EXPECT_TRUE(std::isfinite(snap.bound_confidence));

  const auto switched_obs =
      makeObservation(center_x, center_y, center_yaw, radius, 1, z0 + 0.05,
                      0.05, cfg.tracker.panel_angle_step);
  ASSERT_TRUE(tracker.update({switched_obs}));

  snap = tracker.debug_snapshot();
  EXPECT_TRUE(snap.valid);
  EXPECT_EQ(snap.current_panel_id, 1);
  EXPECT_EQ(snap.bound_panel_id, 1);
  EXPECT_EQ(snap.binding_transition_state, 0);
  EXPECT_EQ(snap.transition_confirm_count, 0);
  EXPECT_EQ(snap.switch_cooldown_frames, 0);
  EXPECT_TRUE(std::isfinite(snap.bound_confidence));
  EXPECT_TRUE(std::isfinite(snap.height_confidence));
}
