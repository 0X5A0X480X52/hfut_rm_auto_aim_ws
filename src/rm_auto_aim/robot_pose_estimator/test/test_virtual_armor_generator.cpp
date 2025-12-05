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
 * @file test_virtual_armor_generator.cpp
 * @brief Unit tests for VirtualArmorGenerator class
 */

#include <gtest/gtest.h>
#include <cmath>

#include "robot_pose_estimator/virtual_armor_generator.hpp"
#include "robot_pose_estimator/robot_types.hpp"

using fyt::auto_aim::VirtualArmorGenerator;
using fyt::auto_aim::RobotState;
using fyt::auto_aim::RobotType;

/**
 * @brief Create test robot state
 */
static RobotState createTestRobotState(
    const std::string& robot_id,
    RobotType robot_type,
    double x, double y, double z,
    double yaw = 0.0,
    double radius = 0.26)
{
  RobotState state;
  state.robot_id = robot_id;
  state.robot_type = robot_type;
  state.center_position = Eigen::Vector3d(x, y, z);
  state.center_velocity = Eigen::Vector3d::Zero();
  state.yaw = yaw;
  state.yaw_velocity = 0.0;
  state.radius_1 = radius;
  state.radius_2 = radius;
  state.d_zc = 0.0;
  state.is_tracking = true;
  state.confidence = 1.0;
  
  switch (robot_type) {
    case RobotType::BALANCE_2:
      state.num_armors = 2;
      break;
    case RobotType::OUTPOST_3:
      state.num_armors = 3;
      break;
    default:
      state.num_armors = 4;
      break;
  }
  
  return state;
}

// ============================================================================
// VirtualArmorGenerator Tests
// ============================================================================

class VirtualArmorGeneratorTest : public ::testing::Test {
protected:
  void SetUp() override {
    generator_ = std::make_unique<VirtualArmorGenerator>();
  }

  std::unique_ptr<VirtualArmorGenerator> generator_;
};

TEST_F(VirtualArmorGeneratorTest, GenerateForStandardRobot) {
  auto robot_state = createTestRobotState("1", RobotType::STANDARD_4, 3.0, 0.0, 1.0);
  
  auto armors = generator_->generateArmors(robot_state);
  
  // Standard robot should generate 4 armors
  EXPECT_EQ(armors.size(), 4u);
  
  // Check armor properties
  for (const auto& armor : armors) {
    EXPECT_EQ(armor.number, "1");
    EXPECT_EQ(armor.type, "small");
  }
}

TEST_F(VirtualArmorGeneratorTest, GenerateForBalanceRobot) {
  auto robot_state = createTestRobotState("3", RobotType::BALANCE_2, 2.0, 0.0, 0.8, 0.0, 0.15);
  
  auto armors = generator_->generateArmors(robot_state);
  
  // Balance robot should generate 2 armors
  EXPECT_EQ(armors.size(), 2u);
  
  // Check armor type (large for balance)
  for (const auto& armor : armors) {
    EXPECT_EQ(armor.number, "3");
    EXPECT_EQ(armor.type, "large");
  }
}

TEST_F(VirtualArmorGeneratorTest, GenerateForHeroRobot) {
  auto robot_state = createTestRobotState("2", RobotType::HERO_4, 4.0, 1.0, 1.2, 0.5, 0.28);
  
  auto armors = generator_->generateArmors(robot_state);
  
  // Hero should generate 4 armors
  EXPECT_EQ(armors.size(), 4u);
  
  for (const auto& armor : armors) {
    EXPECT_EQ(armor.number, "2");
    EXPECT_EQ(armor.type, "large");
  }
}

TEST_F(VirtualArmorGeneratorTest, GenerateForOutpost) {
  auto robot_state = createTestRobotState("outpost", RobotType::OUTPOST_3, 5.0, 0.0, 1.5, 0.0, 0.26);
  
  auto armors = generator_->generateArmors(robot_state);
  
  // Outpost should generate 3 armors
  EXPECT_EQ(armors.size(), 3u);
  
  for (const auto& armor : armors) {
    EXPECT_EQ(armor.number, "outpost");
  }
}

TEST_F(VirtualArmorGeneratorTest, ArmorPositionsFormCircle) {
  auto robot_state = createTestRobotState("1", RobotType::STANDARD_4, 3.0, 0.0, 1.0, 0.0, 0.26);
  
  auto armors = generator_->generateArmors(robot_state);
  
  // All armors should be at approximately the same radius from center
  for (const auto& armor : armors) {
    double dx = armor.pose.position.x - robot_state.center_position.x();
    double dy = armor.pose.position.y - robot_state.center_position.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    
    EXPECT_NEAR(dist, robot_state.radius_1, 0.05);
  }
}

TEST_F(VirtualArmorGeneratorTest, ArmorPositionsWithYaw) {
  // Test with non-zero yaw
  double yaw = M_PI / 4.0;  // 45 degrees
  auto robot_state = createTestRobotState("1", RobotType::STANDARD_4, 0.0, 0.0, 1.0, yaw, 0.26);
  
  auto armors = generator_->generateArmors(robot_state);
  
  EXPECT_EQ(armors.size(), 4u);
  
  // First armor should be at yaw angle from center
  double expected_x = robot_state.radius_1 * std::cos(yaw);
  double expected_y = robot_state.radius_1 * std::sin(yaw);
  
  // Find the armor closest to expected position
  bool found = false;
  for (const auto& armor : armors) {
    double dx = armor.pose.position.x - expected_x;
    double dy = armor.pose.position.y - expected_y;
    if (std::sqrt(dx * dx + dy * dy) < 0.1) {
      found = true;
      break;
    }
  }
  
  EXPECT_TRUE(found);
}

TEST_F(VirtualArmorGeneratorTest, CalculateArmorPositions) {
  Eigen::Vector3d center(3.0, 0.0, 1.0);
  double yaw = 0.0;
  double r1 = 0.26;
  double r2 = 0.26;
  double d_zc = 0.0;
  double d_za = 0.0;
  int armors_num = 4;
  
  auto positions = generator_->calculateArmorPositions(center, yaw, r1, r2, d_zc, d_za, armors_num);
  
  EXPECT_EQ(positions.size(), 4u);
  
  // Check positions are at correct radius
  for (const auto& pos : positions) {
    double dist_xy = std::sqrt(
        std::pow(pos.x() - center.x(), 2) + 
        std::pow(pos.y() - center.y(), 2));
    EXPECT_NEAR(dist_xy, r1, 0.05);
  }
}

TEST_F(VirtualArmorGeneratorTest, CalculateArmorYaw) {
  // Test yaw calculation for 4-armor robot
  double base_yaw = 0.0;
  
  double yaw0 = generator_->calculateArmorYaw(base_yaw, 0, 4);
  double yaw1 = generator_->calculateArmorYaw(base_yaw, 1, 4);
  double yaw2 = generator_->calculateArmorYaw(base_yaw, 2, 4);
  double yaw3 = generator_->calculateArmorYaw(base_yaw, 3, 4);
  
  // Armors should be 90 degrees (PI/2) apart
  EXPECT_NEAR(std::abs(yaw1 - yaw0), M_PI / 2.0, 0.01);
  EXPECT_NEAR(std::abs(yaw2 - yaw1), M_PI / 2.0, 0.01);
  EXPECT_NEAR(std::abs(yaw3 - yaw2), M_PI / 2.0, 0.01);
}

TEST_F(VirtualArmorGeneratorTest, CreateArmorMsg) {
  Eigen::Vector3d position(3.0, 1.0, 0.5);
  double yaw = 0.5;
  std::string armor_id = "1";
  std::string armor_type = "small";
  
  auto msg = generator_->createArmorMsg(position, yaw, armor_id, armor_type);
  
  EXPECT_EQ(msg.number, "1");
  EXPECT_EQ(msg.type, "small");
  EXPECT_NEAR(msg.pose.position.x, 3.0, 0.01);
  EXPECT_NEAR(msg.pose.position.y, 1.0, 0.01);
  EXPECT_NEAR(msg.pose.position.z, 0.5, 0.01);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
