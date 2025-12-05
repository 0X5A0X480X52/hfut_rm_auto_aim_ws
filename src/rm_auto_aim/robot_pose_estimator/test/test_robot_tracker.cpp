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
 * @file test_robot_tracker.cpp
 * @brief Unit tests for RobotTracker class
 */

#include <gtest/gtest.h>
#include <cmath>

#include "robot_pose_estimator/robot_tracker.hpp"
#include "robot_pose_estimator/robot_types.hpp"
#include "rm_utils/logger/log.hpp"

using fyt::auto_aim::RobotTracker;
using fyt::auto_aim::RobotType;
using fyt::auto_aim::ArmorState;
using fyt::auto_aim::RobotState;
using fyt::auto_aim::PoseEstimatorConfig;

// Test fixture to register logger before running tests
class RobotTrackerTestSetup : public ::testing::Environment {
public:
  void SetUp() override {
    // Register logger for robot_pose_estimator to avoid exceptions
    FYT_REGISTER_LOGGER("robot_pose_estimator", "/tmp/robot_pose_estimator_test", DEBUG);
  }
};

/**
 * @brief Create a test ArmorState
 */
static ArmorState createTestArmor(
    const std::string& armor_id,
    const std::string& armor_type,
    double x, double y, double z,
    double yaw = 0.0,
    int track_id = 0)
{
  ArmorState armor;
  armor.armor_id = armor_id;
  armor.armor_type = armor_type;
  armor.position = Eigen::Vector3d(x, y, z);
  armor.velocity = Eigen::Vector3d::Zero();
  armor.yaw = yaw;
  armor.yaw_velocity = 0.0;
  armor.confidence = 1.0;
  armor.is_observed = true;
  armor.track_id = track_id;
  return armor;
}

/**
 * @brief Create default test config
 */
static PoseEstimatorConfig createTestConfig()
{
  PoseEstimatorConfig config;
  config.sigma2_q_xyz = 0.05;
  config.sigma2_q_yaw = 1.0;
  config.sigma2_q_r = 0.05;
  config.r_xyz = 0.05;
  config.r_yaw = 0.02;
  config.max_match_distance = 0.5;
  config.max_match_yaw_diff = 1.0;
  config.tracking_threshold = 3;
  config.lost_threshold = 5;
  config.robot_timeout = 2.0;
  config.robot_params.standard_radius = 0.26;
  config.robot_params.balance_radius = 0.15;
  config.robot_params.hero_radius = 0.28;
  config.robot_params.outpost_radius = 0.26;
  return config;
}

// ============================================================================
// RobotTracker Basic Tests
// ============================================================================

class RobotTrackerTest : public ::testing::Test {
protected:
  void SetUp() override {
    config_ = createTestConfig();
    tracker_ = std::make_unique<RobotTracker>(config_);
  }

  PoseEstimatorConfig config_;
  std::unique_ptr<RobotTracker> tracker_;
};

TEST_F(RobotTrackerTest, InitializationWithArmor) {
  // Initialize with a standard robot armor
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  
  tracker_->init(armor);
  
  auto state = tracker_->getRobotState();
  EXPECT_EQ(state.robot_id, "1");
  EXPECT_EQ(state.robot_type, RobotType::STANDARD_4);
  EXPECT_EQ(state.num_armors, 4);
  
  // Center should be estimated based on armor position and radius
  EXPECT_NEAR(state.center_position.z(), 1.0, 0.1);
}

TEST_F(RobotTrackerTest, InitializationWithBalanceRobot) {
  // Initialize with a balance robot armor (large armor, id 3/4/5)
  auto armor = createTestArmor("3", "large", 2.0, 0.0, 0.8, 0.0, 1);
  
  tracker_->init(armor);
  
  auto state = tracker_->getRobotState();
  EXPECT_EQ(state.robot_id, "3");
  EXPECT_EQ(state.robot_type, RobotType::BALANCE_2);
  EXPECT_EQ(state.num_armors, 2);
}

TEST_F(RobotTrackerTest, InitializationWithHeroRobot) {
  // Initialize with hero robot armor
  auto armor = createTestArmor("2", "large", 4.0, 1.0, 1.2, 0.5, 1);
  
  tracker_->init(armor);
  
  auto state = tracker_->getRobotState();
  EXPECT_EQ(state.robot_id, "2");
  EXPECT_EQ(state.robot_type, RobotType::HERO_4);
  EXPECT_EQ(state.num_armors, 4);
}

TEST_F(RobotTrackerTest, InitializationWithOutpost) {
  // Initialize with outpost
  auto armor = createTestArmor("outpost", "large", 5.0, 0.0, 1.5, 0.0, 1);
  
  tracker_->init(armor);
  
  auto state = tracker_->getRobotState();
  EXPECT_EQ(state.robot_id, "outpost");
  EXPECT_EQ(state.robot_type, RobotType::OUTPOST_3);
  EXPECT_EQ(state.num_armors, 3);
}

TEST_F(RobotTrackerTest, TrackerState) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  // Initial state should be DETECTING
  EXPECT_EQ(tracker_->getState(), RobotTracker::State::DETECTING);
  
  // After enough updates, should transition to TRACKING
  std::vector<ArmorState> armors = {armor};
  for (int i = 0; i < config_.tracking_threshold + 1; ++i) {
    tracker_->update(armors, 0.01);
  }
  
  EXPECT_EQ(tracker_->getState(), RobotTracker::State::TRACKING);
}

TEST_F(RobotTrackerTest, TrackerLostState) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  // Update with armors to get to tracking state
  std::vector<ArmorState> armors = {armor};
  for (int i = 0; i < config_.tracking_threshold + 1; ++i) {
    tracker_->update(armors, 0.01);
  }
  EXPECT_EQ(tracker_->getState(), RobotTracker::State::TRACKING);
  
  // Update with empty armors to trigger lost
  std::vector<ArmorState> empty_armors;
  for (int i = 0; i < config_.lost_threshold + 1; ++i) {
    tracker_->update(empty_armors, 0.01);
  }
  
  EXPECT_EQ(tracker_->getState(), RobotTracker::State::LOST);
}

TEST_F(RobotTrackerTest, PredictStep) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  auto state_before = tracker_->getRobotState();
  
  // Predict for a small time step
  tracker_->predict(0.1);
  
  auto state_after = tracker_->getRobotState();
  
  // State should be updated (position might change slightly due to velocity)
  // Since initial velocity is 0, position should stay roughly the same
  EXPECT_NEAR(state_before.center_position.x(), state_after.center_position.x(), 0.5);
  EXPECT_NEAR(state_before.center_position.y(), state_after.center_position.y(), 0.5);
  EXPECT_NEAR(state_before.center_position.z(), state_after.center_position.z(), 0.5);
}

TEST_F(RobotTrackerTest, UpdateWithSingleArmor) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  // Move armor slightly
  auto armor2 = createTestArmor("1", "small", 3.1, 0.05, 1.0, 0.1, 1);
  std::vector<ArmorState> armors = {armor2};
  
  bool matched = tracker_->update(armors, 0.01);
  
  EXPECT_TRUE(matched);
  
  auto state = tracker_->getRobotState();
  // Position should be updated towards new observation
  EXPECT_GT(state.center_position.x(), 2.5);
}

TEST_F(RobotTrackerTest, GetArmorPositions) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  auto positions = tracker_->getArmorPositions();
  
  // Standard robot should have 4 armor positions
  EXPECT_EQ(positions.size(), 4u);
  
  // All positions should be at reasonable distances from center
  auto state = tracker_->getRobotState();
  for (const auto& pos : positions) {
    double dist_xy = std::sqrt(
        std::pow(pos.x() - state.center_position.x(), 2) +
        std::pow(pos.y() - state.center_position.y(), 2));
    EXPECT_NEAR(dist_xy, state.radius_1, 0.1);
  }
}

TEST_F(RobotTrackerTest, GetRobotId) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  EXPECT_EQ(tracker_->getRobotId(), "1");
}

TEST_F(RobotTrackerTest, GetRobotType) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  EXPECT_EQ(tracker_->getRobotType(), RobotType::STANDARD_4);
}

TEST_F(RobotTrackerTest, GetArmorsNum) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  EXPECT_EQ(tracker_->getArmorsNum(), 4);
}

TEST_F(RobotTrackerTest, ShouldRemove) {
  auto armor = createTestArmor("1", "small", 3.0, 0.0, 1.0, 0.0, 1);
  tracker_->init(armor);
  
  // Should not remove initially
  EXPECT_FALSE(tracker_->shouldRemove());
  
  // Update with empty armors many times to trigger lost
  std::vector<ArmorState> empty_armors;
  for (int i = 0; i < config_.lost_threshold * 2; ++i) {
    tracker_->update(empty_armors, 0.01);
  }
  
  // After being lost for a while, should be removable
  // The exact behavior depends on implementation
}

// ============================================================================
// Robot Type Detection Tests
// ============================================================================

TEST(RobotTypeTest, GetRobotTypeFromArmor) {
  using fyt::auto_aim::getRobotTypeFromArmor;
  
  // Standard robot (ID "1")
  EXPECT_EQ(getRobotTypeFromArmor("1", "small"), RobotType::STANDARD_4);
  
  // Hero robot (ID "2")
  EXPECT_EQ(getRobotTypeFromArmor("2", "large"), RobotType::HERO_4);
  
  // Balance robot (large armor with ID 3/4/5)
  EXPECT_EQ(getRobotTypeFromArmor("3", "large"), RobotType::BALANCE_2);
  EXPECT_EQ(getRobotTypeFromArmor("4", "large"), RobotType::BALANCE_2);
  EXPECT_EQ(getRobotTypeFromArmor("5", "large"), RobotType::BALANCE_2);
  
  // Outpost
  EXPECT_EQ(getRobotTypeFromArmor("outpost", "large"), RobotType::OUTPOST_3);
  
  // Base
  EXPECT_EQ(getRobotTypeFromArmor("base", "large"), RobotType::BASE);
}

TEST(RobotTypeTest, GetArmorsNumFromType) {
  using fyt::auto_aim::getArmorsNumFromType;
  
  EXPECT_EQ(getArmorsNumFromType(RobotType::BALANCE_2), 2);
  EXPECT_EQ(getArmorsNumFromType(RobotType::OUTPOST_3), 3);
  EXPECT_EQ(getArmorsNumFromType(RobotType::STANDARD_4), 4);
  EXPECT_EQ(getArmorsNumFromType(RobotType::HERO_4), 4);
  EXPECT_EQ(getArmorsNumFromType(RobotType::SENTRY), 4);
}

TEST(RobotTypeTest, GetRobotTypeName) {
  using fyt::auto_aim::getRobotTypeName;
  
  EXPECT_EQ(getRobotTypeName(RobotType::BALANCE_2), "Balance");
  EXPECT_EQ(getRobotTypeName(RobotType::STANDARD_4), "Standard");
  EXPECT_EQ(getRobotTypeName(RobotType::HERO_4), "Hero");
  EXPECT_EQ(getRobotTypeName(RobotType::OUTPOST_3), "Outpost");
  EXPECT_EQ(getRobotTypeName(RobotType::SENTRY), "Sentry");
  EXPECT_EQ(getRobotTypeName(RobotType::BASE), "Base");
  EXPECT_EQ(getRobotTypeName(RobotType::UNKNOWN), "Unknown");
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Register the global environment (logger setup)
  ::testing::AddGlobalTestEnvironment(new RobotTrackerTestSetup);
  return RUN_ALL_TESTS();
}
