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

#include "armor_tracker/track_prediction_manager.hpp"
#include "armor_tracker/armor_types.hpp"

using fyt::auto_aim::TrackedArmorState;
using fyt::auto_aim::TrackingState;
using fyt::auto_aim::ArmorSourceType;
using fyt::auto_aim::PredictionWindowConfig;
using fyt::auto_aim::TrackPredictionManager;

static builtin_interfaces::msg::Time make_time(uint32_t sec, uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

static TrackedArmorState make_state(
  int track_id, const std::string & armor_id, double x = 0.0,
  double vx = 1.0)
{
  TrackedArmorState s;
  s.track_id = track_id;
  s.armor_id = armor_id;
  s.armor_type = "small";
  s.position = Eigen::Vector3d(x, 0.0, 1.0);
  s.velocity = Eigen::Vector3d(vx, 0.0, 0.0);
  s.yaw = 0.0;
  s.yaw_velocity = 0.0;
  s.tracking_state = TrackingState::TRACKING;
  s.source_type = ArmorSourceType::DETECT;
  s.confidence = 1.0;
  s.tracking_count = 1;
  s.lost_count = 0;
  s.time_since_update = 0;
  s.last_detected_time = make_time(0);
  return s;
}

TEST(TrackPredictionManagerTest, BasicPrediction) {
  PredictionWindowConfig cfg;
  cfg.prediction_steps = 3;
  cfg.predict_interval = 1;
  cfg.dt = 0.1;

  TrackPredictionManager mgr(cfg);

  auto t = make_state(1, "1", 0.0, 1.0);

  mgr.generatePredictions({t}, make_time(1));
  auto w = mgr.getPredictionWindow(1);
  EXPECT_EQ(w.predictions.size(), 3u);
  EXPECT_EQ(w.track_id, 1);
  EXPECT_EQ(w.armor_id, "1");

  // Expected positions: x = 0 + vx * dt * step
  for (uint32_t i = 0; i < w.predictions.size(); ++i) {
    double expected_x = 1.0 * cfg.dt * (i + 1);
    EXPECT_NEAR(w.predictions[i].position.x, expected_x, 1e-6);
  }
}

TEST(TrackPredictionManagerTest, PredictInterval) {
  PredictionWindowConfig cfg;
  cfg.prediction_steps = 2;
  cfg.predict_interval = 2; // only every 2nd call
  cfg.dt = 0.05;

  TrackPredictionManager mgr(cfg);
  auto t = make_state(2, "2", 0.0, 1.0);

  // first call should not generate
  mgr.generatePredictions({t}, make_time(1));
  auto w1 = mgr.getPredictionWindow(2);
  EXPECT_EQ(w1.predictions.size(), 0u);

  // second call should generate
  mgr.generatePredictions({t}, make_time(2));
  auto w2 = mgr.getPredictionWindow(2);
  EXPECT_EQ(w2.predictions.size(), 2u);
}
