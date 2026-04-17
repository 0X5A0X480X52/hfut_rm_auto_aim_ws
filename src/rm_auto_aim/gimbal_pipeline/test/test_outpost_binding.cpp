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

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/outpost_armor_tracker.hpp"

namespace {

using fyt::auto_aim::ObservationData;
using fyt::auto_aim::OutpostArmorTracker;
using fyt::auto_aim::UnifiedConfig;

std::array<double, 3> panelAngles(double step) {
  const double s = std::abs(step);
  return {0.0, s, -s};
}

std::array<double, 3> zOffsets(const UnifiedConfig &cfg) {
  return {cfg.outpost.z_offset_0, cfg.outpost.z_offset_1, cfg.outpost.z_offset_2};
}

ObservationData makeOutpostObservation(
    const UnifiedConfig &cfg,
    double center_x,
    double center_y,
    double center_z,
    double center_yaw,
    int panel_id,
    double ts) {
  const auto angles = panelAngles(cfg.outpost.panel_angle_step);
  const auto offsets = zOffsets(cfg);

  ObservationData obs;
  const double armor_yaw = center_yaw + angles[panel_id];
  obs.x = center_x + cfg.outpost.radius * std::cos(armor_yaw);
  obs.y = center_y + cfg.outpost.radius * std::sin(armor_yaw);
  obs.z = center_z + offsets[panel_id];
  obs.yaw = armor_yaw;
  obs.timestamp = ts;
  return obs;
}

UnifiedConfig makeTestConfig() {
  UnifiedConfig cfg = UnifiedConfig::create_default();

  cfg.outpost.radius = 0.28;
  cfg.outpost.z_offset_0 = 0.10;
  cfg.outpost.z_offset_1 = 0.0;
  cfg.outpost.z_offset_2 = -0.10;
  cfg.outpost.panel_angle_step = 2.0 * M_PI / 3.0;

  cfg.outpost.weight_yaw = 1.0;
  cfg.outpost.weight_z_state = 4.0;
  cfg.outpost.weight_z_history = 1.5;
  cfg.outpost.weight_xy_residual = 3.0;
  cfg.outpost.weight_switch_penalty = 0.03;

  cfg.outpost.binding_transition_confirm_frames = 2;
  cfg.outpost.binding_same_panel_yaw_gate = 0.35;
  cfg.outpost.binding_same_panel_z_gate = 0.08;
  cfg.outpost.binding_same_panel_xy_gate = 0.20;
  cfg.outpost.binding_min_candidate_prob = 0.30;
  cfg.outpost.binding_min_candidate_margin = 0.08;
  cfg.outpost.binding_switch_strong_score = 0.30;
  cfg.outpost.binding_period_min_spin_rate = 0.0;
  cfg.outpost.binding_period_weight = 0.50;
  cfg.outpost.binding_period_update_min_confidence = 0.40;
  cfg.outpost.binding_period_update_min_jump = 0.01;
  cfg.outpost.binding_dz_ema_alpha = 0.20;

  return cfg;
}

Eigen::Vector3d armorPositionFromMessage(const Eigen::Vector3d &center,
                                         double yaw,
                                         const geometry_msgs::msg::Pose &offset) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double x = center.x() + offset.position.x * c - offset.position.y * s;
  const double y = center.y() + offset.position.x * s + offset.position.y * c;
  const double z = center.z() + offset.position.z;
  return Eigen::Vector3d(x, y, z);
}

}  // namespace

TEST(OutpostBinding, PublishesSemanticAndCandidateDiagnostics) {
  auto cfg = makeTestConfig();
  OutpostArmorTracker tracker(cfg, 0.05, false);

  const auto init_obs = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.2, 0, 0.00);
  tracker.initialize({init_obs});

  const auto obs_same = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.2, 0, 0.05);
  ASSERT_TRUE(tracker.update({obs_same}));

  const auto &snap = tracker.debug_snapshot();
  EXPECT_TRUE(snap.valid);
  EXPECT_EQ(snap.runtime_panel_id, 0);
  EXPECT_EQ(snap.bound_height_label, 0);
  EXPECT_GE(snap.candidate_panel_id, 0);
  EXPECT_LE(snap.candidate_panel_id, 2);
  EXPECT_TRUE(std::isfinite(snap.candidate_prob));
  EXPECT_TRUE(std::isfinite(snap.candidate_margin));
  EXPECT_TRUE(std::isfinite(snap.selected_xy_residual));
}

TEST(OutpostBinding, PeriodicModelUpdateBlockedByMinJumpGate) {
  auto cfg = makeTestConfig();
  cfg.outpost.binding_period_update_min_jump = 0.30;  // higher than expected jumps
  OutpostArmorTracker tracker(cfg, 0.05, false);

  const auto init_obs = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 0, 0.00);
  tracker.initialize({init_obs});

  const auto obs_same = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 0, 0.05);
  ASSERT_TRUE(tracker.update({obs_same}));

  const auto &snap = tracker.debug_snapshot();
  EXPECT_EQ(snap.period_update_applied, 0);
  EXPECT_TRUE(std::isnan(snap.dz_small_est));
  EXPECT_TRUE(std::isnan(snap.dz_large_est));
}

TEST(OutpostBinding, CandidateGateRejectsWeakSwitch) {
  auto cfg = makeTestConfig();
  cfg.outpost.softmax_temperature = 10.0;        // flatten posterior
  cfg.outpost.binding_min_candidate_prob = 0.95; // intentionally strict
  cfg.outpost.binding_min_candidate_margin = 0.50;
  OutpostArmorTracker tracker(cfg, 0.05, false);

  const auto init_obs = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 0, 0.00);
  tracker.initialize({init_obs});

  const auto obs_switch = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 2, 0.05);
  ASSERT_TRUE(tracker.update({obs_switch}));

  const auto &snap = tracker.debug_snapshot();
  EXPECT_EQ(snap.runtime_panel_id, 0);
  EXPECT_EQ(snap.switch_event, 0);
  EXPECT_TRUE(snap.switch_reason == 2 || snap.switch_reason == 3);
}

TEST(OutpostBinding, ConfidentEvidenceConfirmsSwitchAndSemantic) {
  auto cfg = makeTestConfig();
  cfg.outpost.softmax_temperature = 0.30;
  cfg.outpost.weight_switch_penalty = 0.0;
  cfg.outpost.binding_period_weight = 0.0;
  cfg.outpost.binding_transition_confirm_frames = 1;
  cfg.outpost.binding_min_candidate_prob = 0.05;
  cfg.outpost.binding_min_candidate_margin = 0.01;
  cfg.outpost.binding_switch_strong_score = 0.05;
  OutpostArmorTracker tracker(cfg, 0.05, false);

  const auto init_obs = makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 0, 0.00);
  tracker.initialize({init_obs});

  bool switched = false;
  OutpostArmorTracker::DebugSnapshot snap;
  for (int i = 1; i <= 6; ++i) {
    const auto obs =
        makeOutpostObservation(cfg, 0.0, 0.0, 0.0, 0.0, 2, 0.05 * i);
    ASSERT_TRUE(tracker.update({obs}));
    snap = tracker.debug_snapshot();
    if (snap.switch_event == 1) {
      switched = true;
      break;
    }
  }

  EXPECT_TRUE(switched);
  EXPECT_EQ(snap.runtime_panel_id, 2);
  EXPECT_EQ(snap.bound_height_label, 2);
  EXPECT_TRUE(snap.switch_event == 1 || snap.switch_reason == 1);
}

TEST(OutpostBinding, StructuredYawAndOffsetsMatchObservedArmorPosition) {
  auto cfg = makeTestConfig();
  cfg.outpost.stable_frames = 1;
  cfg.outpost.entropy_exit = 1.0;
  cfg.outpost.max_prob_exit = 0.0;

  OutpostArmorTracker tracker(cfg, 0.05, false);

  constexpr double cx = 0.6;
  constexpr double cy = -0.4;
  constexpr double cz = 0.2;
  constexpr double cyaw = 0.35;

  const auto init_obs = makeOutpostObservation(cfg, cx, cy, cz, cyaw, 0, 0.00);
  tracker.initialize({init_obs});

  const auto obs_same =
      makeOutpostObservation(cfg, cx, cy, cz, cyaw, 0, 0.05);
  ASSERT_TRUE(tracker.update({obs_same}));
  ASSERT_FALSE(tracker.is_ambiguous_single_mode());

  const auto offsets = tracker.build_armors_offset_for_message();
  ASSERT_EQ(offsets.size(), 3u);

  const auto snap = tracker.debug_snapshot();
  ASSERT_GE(snap.runtime_panel_id, 0);
  ASSERT_LE(snap.runtime_panel_id, 2);

  const Eigen::Vector3d center = tracker.get_center_position();
  const double yaw = tracker.get_yaw();
  const Eigen::Vector3d reconstructed =
      armorPositionFromMessage(center, yaw, offsets[snap.runtime_panel_id]);

  EXPECT_NEAR(reconstructed.x(), obs_same.x, 0.05);
  EXPECT_NEAR(reconstructed.y(), obs_same.y, 0.05);
  EXPECT_NEAR(reconstructed.z(), obs_same.z, 0.05);
}
