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

#include "armor_tracker/track_history_manager.hpp"
#include "armor_tracker/armor_types.hpp"

using fyt::auto_aim::TrackedArmorState;
using fyt::auto_aim::TrackingState;
using fyt::auto_aim::ArmorSourceType;
using fyt::auto_aim::HistoryWindowConfig;
using fyt::auto_aim::TrackHistoryManager;

static builtin_interfaces::msg::Time make_time(uint32_t sec, uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

static TrackedArmorState make_state(int track_id, const std::string & armor_id, double x = 0.0)
{
  TrackedArmorState s;
  s.track_id = track_id;
  s.armor_id = armor_id;
  s.armor_type = "small";
  s.position = Eigen::Vector3d(x, 0.0, 1.0);
  s.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
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

TEST(TrackHistoryManagerTest, BasicRecordAndSizeLimit) {
  HistoryWindowConfig cfg;
  cfg.max_window_size = 3;
  cfg.record_interval = 1;

  TrackHistoryManager mgr(cfg);

  // Single track scenario
  auto t = make_state(1, "1", 0.0);

  // update 5 times
  for (int i = 0; i < 5; ++i) {
    mgr.update({t}, make_time(i));
  }

  auto window = mgr.getHistoryWindow(1);
  // max window size is 3, so only last 3 entries should exist
  EXPECT_EQ(window.history.size(), 3u);
  EXPECT_EQ(window.current_iteration, 5u);

  // test removal
  mgr.removeTrack(1);
  auto empty = mgr.getHistoryWindow(1);
  EXPECT_EQ(empty.history.size(), 0u);
}

TEST(TrackHistoryManagerTest, RecordInterval) {
  HistoryWindowConfig cfg;
  cfg.max_window_size = 10;
  cfg.record_interval = 2; // only every second call should be recorded

  TrackHistoryManager mgr(cfg);

  auto t = make_state(5, "5", 0.0);

  for (int i = 0; i < 6; ++i) {
    mgr.update({t}, make_time(i));
  }

  // 6 iterations, record_interval=2 => should record 3 entries
  auto w = mgr.getHistoryWindow(5);
  EXPECT_EQ(w.history.size(), 3u);
  EXPECT_EQ(w.current_iteration, 6u);
}

TEST(TrackHistoryManagerTest, Clear) {
  HistoryWindowConfig cfg;
  cfg.max_window_size = 5;
  cfg.record_interval = 1;
  TrackHistoryManager mgr(cfg);

  auto t1 = make_state(1, "1", 0.0);
  auto t2 = make_state(2, "2", 10.0);

  mgr.update({t1, t2}, make_time(1));
  auto all = mgr.getAllHistoryWindows();
  EXPECT_EQ(all.windows.size(), 2u);

  mgr.clear();
  auto all2 = mgr.getAllHistoryWindows();
  EXPECT_EQ(all2.windows.size(), 0u);
  EXPECT_EQ(mgr.getCurrentIteration(), 0u);
}
