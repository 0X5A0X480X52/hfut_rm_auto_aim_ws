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

#ifndef TARGET_SELECTOR__SELECTION_STRATEGY_HPP_
#define TARGET_SELECTOR__SELECTION_STRATEGY_HPP_

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rm_interfaces/msg/tracked_robot.hpp"
#include "rm_interfaces/msg/tracked_robots.hpp"

namespace fyt::auto_aim {

/**
 * @brief Selection result containing the selected robot and additional info
 */
struct SelectionResult {
  std::string robot_id;          // Selected robot ID
  double confidence;             // Selection confidence [0, 1]
  double yaw_deviation;          // Yaw deviation from reference direction (rad)
  double distance;               // Distance to robot center (m)
  
  SelectionResult() 
    : robot_id(""), confidence(0.0), yaw_deviation(0.0), distance(0.0) {}
  
  SelectionResult(const std::string& id, double conf, double yaw_dev, double dist)
    : robot_id(id), confidence(conf), yaw_deviation(yaw_dev), distance(dist) {}
  
  bool isValid() const { return !robot_id.empty(); }
};

/**
 * @brief Configuration for target selection
 */
struct SelectionConfig {
  double reference_yaw;          // Reference yaw direction for selection (rad)
  double max_yaw_deviation;      // Maximum allowed yaw deviation (rad)
  double max_distance;           // Maximum allowed distance (m)
  double min_confidence;         // Minimum robot confidence required
  std::string current_target_id; // Currently locked target ID (for hysteresis)
  double hysteresis_threshold;   // Threshold for switching targets (ratio)
  std::vector<std::string> priority_robot_ids;  // Robot IDs sorted by high->low priority
  int sticky_lock_frames;        // Frames needed to lock a preferred target id
  int sticky_lost_frames;        // Frames needed to clear preferred target id
  
  SelectionConfig()
    : reference_yaw(0.0)
    , max_yaw_deviation(M_PI)
    , max_distance(10.0)
    , min_confidence(0.3)
    , current_target_id("")
    , hysteresis_threshold(0.1)
    , priority_robot_ids()
    , sticky_lock_frames(3)
    , sticky_lost_frames(3) {}
};

/**
 * @brief Abstract base class for target selection strategies
 * 
 * This class defines the interface for all target selection strategies.
 * Subclasses should implement the selectTarget() method with their specific
 * selection logic.
 */
class SelectionStrategy {
public:
  using TrackedRobot = rm_interfaces::msg::TrackedRobot;
  using TrackedRobots = rm_interfaces::msg::TrackedRobots;

  SelectionStrategy() = default;
  virtual ~SelectionStrategy() = default;

  // Disable copy
  SelectionStrategy(const SelectionStrategy&) = delete;
  SelectionStrategy& operator=(const SelectionStrategy&) = delete;

  // Enable move
  SelectionStrategy(SelectionStrategy&&) = default;
  SelectionStrategy& operator=(SelectionStrategy&&) = default;

  /**
   * @brief Select target robot from a list of tracked robots
   * 
   * @param robots List of tracked robots
   * @param config Selection configuration
   * @return std::optional<SelectionResult> Selected robot, or std::nullopt if none suitable
   */
  virtual std::optional<SelectionResult> selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) = 0;

  /**
   * @brief Get the name of this selection strategy
   * @return Strategy name string
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Get a description of this selection strategy
   * @return Strategy description string
   */
  virtual std::string getDescription() const = 0;

protected:
  /**
   * @brief Filter robots based on basic criteria
   * 
   * @param robots All tracked robots
   * @param config Selection configuration
   * @return Filtered list of candidate robots
   */
  std::vector<const TrackedRobot*> filterCandidates(
    const TrackedRobots& robots,
    const SelectionConfig& config) const;

  /**
   * @brief Calculate yaw angle to a robot's center position
   * 
   * @param robot Target robot
   * @return Yaw angle (rad)
   */
  double calculateYawToRobot(const TrackedRobot& robot) const;

  /**
   * @brief Calculate distance to a robot's center position
   * 
   * @param robot Target robot
   * @return Distance (m)
   */
  double calculateDistanceToRobot(const TrackedRobot& robot) const;

  /**
   * @brief Calculate yaw deviation from reference direction
   * 
   * @param robot Target robot
   * @param reference_yaw Reference yaw direction (rad)
   * @return Yaw deviation (rad), always positive
   */
  double calculateYawDeviation(
    const TrackedRobot& robot,
    double reference_yaw) const;

  /**
   * @brief Normalize angle to [-pi, pi]
   * 
   * @param angle Input angle (rad)
   * @return Normalized angle (rad)
   */
  double normalizeAngle(double angle) const;
};

// Smart pointer type for strategies
using SelectionStrategyPtr = std::unique_ptr<SelectionStrategy>;

}  // namespace fyt::auto_aim

#endif  // TARGET_SELECTOR__SELECTION_STRATEGY_HPP_
