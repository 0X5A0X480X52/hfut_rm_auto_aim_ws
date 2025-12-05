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

/**
 * @file test_armor_grouper.cpp
 * @brief Unit tests for ArmorGrouper class
 */

#include <gtest/gtest.h>
#include "robot_pose_estimator/armor_grouper.hpp"
#include "robot_pose_estimator/robot_types.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"

using fyt::auto_aim::ArmorGrouper;
using fyt::auto_aim::ArmorState;

/**
 * @brief Create a test TrackedArmor message
 */
static rm_interfaces::msg::TrackedArmor createTestTrackedArmor(
    const std::string& armor_id,
    const std::string& armor_type,
    double x, double y, double z,
    int track_id,
    uint8_t tracking_state = rm_interfaces::msg::TrackedArmor::TRACKING)
{
  rm_interfaces::msg::TrackedArmor armor;
  armor.armor_id = armor_id;
  armor.armor_type = armor_type;
  armor.track_id = track_id;
  armor.tracking_state = tracking_state;
  armor.confidence = 1.0;
  armor.position.x = x;
  armor.position.y = y;
  armor.position.z = z;
  armor.yaw = 0.0;
  armor.source_type = rm_interfaces::msg::TrackedArmor::SOURCE_DETECT;
  return armor;
}

// ============================================================================
// ArmorGrouper Tests
// ============================================================================

class ArmorGrouperTest : public ::testing::Test {
protected:
  void SetUp() override {
    grouper_ = std::make_unique<ArmorGrouper>();
    // Valid tracking states: DETECTING and TRACKING
    valid_states_ = {
      rm_interfaces::msg::TrackedArmor::DETECTING,
      rm_interfaces::msg::TrackedArmor::TRACKING
    };
  }

  std::unique_ptr<ArmorGrouper> grouper_;
  std::vector<uint8_t> valid_states_;
};

TEST_F(ArmorGrouperTest, GroupSingleRobot) {
  rm_interfaces::msg::TrackedArmors tracked_armors;
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.0, 0.0, 1.0, 1));
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.0, 0.5, 1.0, 2));
  
  auto groups = grouper_->groupArmors(tracked_armors, valid_states_);
  
  EXPECT_EQ(groups.size(), 1u);
  EXPECT_TRUE(groups.find("1") != groups.end());
  EXPECT_EQ(groups["1"].size(), 2u);
}

TEST_F(ArmorGrouperTest, GroupMultipleRobots) {
  rm_interfaces::msg::TrackedArmors tracked_armors;
  // Robot 1
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.0, 0.0, 1.0, 1));
  // Robot 2 (hero)
  tracked_armors.armors.push_back(createTestTrackedArmor("2", "large", 5.0, 1.0, 1.2, 2));
  // Robot 3 (balance)
  tracked_armors.armors.push_back(createTestTrackedArmor("3", "large", 7.0, 0.0, 0.8, 3));
  
  auto groups = grouper_->groupArmors(tracked_armors, valid_states_);
  
  EXPECT_EQ(groups.size(), 3u);
  EXPECT_TRUE(groups.find("1") != groups.end());
  EXPECT_TRUE(groups.find("2") != groups.end());
  EXPECT_TRUE(groups.find("3") != groups.end());
}

TEST_F(ArmorGrouperTest, GroupEmptyInput) {
  rm_interfaces::msg::TrackedArmors tracked_armors;
  
  auto groups = grouper_->groupArmors(tracked_armors, valid_states_);
  
  EXPECT_EQ(groups.size(), 0u);
}

TEST_F(ArmorGrouperTest, MultipleArmorsPerRobot) {
  rm_interfaces::msg::TrackedArmors tracked_armors;
  // Standard robot 1 with multiple armor detections
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.0, 0.0, 1.0, 1));
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.0, 0.26, 1.0, 2));
  tracked_armors.armors.push_back(createTestTrackedArmor("1", "small", 3.26, 0.0, 1.0, 3));
  
  auto groups = grouper_->groupArmors(tracked_armors, valid_states_);
  
  EXPECT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups["1"].size(), 3u);
}

TEST_F(ArmorGrouperTest, TrackedArmorToState) {
  auto msg = createTestTrackedArmor("1", "small", 3.0, 1.0, 0.5, 42);
  
  auto state = grouper_->trackedArmorToState(msg);
  
  EXPECT_EQ(state.armor_id, "1");
  EXPECT_EQ(state.armor_type, "small");
  EXPECT_NEAR(state.position.x(), 3.0, 0.01);
  EXPECT_NEAR(state.position.y(), 1.0, 0.01);
  EXPECT_NEAR(state.position.z(), 0.5, 0.01);
  EXPECT_EQ(state.track_id, 42);
}

TEST_F(ArmorGrouperTest, GetRobotIdFromArmorId) {
  // Normal armor IDs
  EXPECT_EQ(grouper_->getRobotIdFromArmorId("1"), "1");
  EXPECT_EQ(grouper_->getRobotIdFromArmorId("2"), "2");
  EXPECT_EQ(grouper_->getRobotIdFromArmorId("3"), "3");
  
  // Special IDs
  EXPECT_EQ(grouper_->getRobotIdFromArmorId("outpost"), "outpost");
  EXPECT_EQ(grouper_->getRobotIdFromArmorId("base"), "base");
}

TEST_F(ArmorGrouperTest, IsValidState) {
  using TA = rm_interfaces::msg::TrackedArmor;
  EXPECT_TRUE(grouper_->isValidState(TA::DETECTING, valid_states_));
  EXPECT_TRUE(grouper_->isValidState(TA::TRACKING, valid_states_));
  EXPECT_FALSE(grouper_->isValidState(TA::LOST, valid_states_));
  EXPECT_FALSE(grouper_->isValidState(TA::TEMP_LOST, valid_states_));
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
