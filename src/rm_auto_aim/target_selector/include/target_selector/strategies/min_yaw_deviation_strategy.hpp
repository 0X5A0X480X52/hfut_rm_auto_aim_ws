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

#ifndef TARGET_SELECTOR__STRATEGIES__MIN_YAW_DEVIATION_STRATEGY_HPP_
#define TARGET_SELECTOR__STRATEGIES__MIN_YAW_DEVIATION_STRATEGY_HPP_

#include "target_selector/selection_strategy.hpp"

namespace fyt::auto_aim {

/**
 * @brief Target selection strategy based on minimum yaw deviation
 * 
 * This strategy selects the robot whose center position has the smallest
 * yaw angle deviation from a reference direction (typically the current
 * gimbal yaw or a fixed forward direction).
 * 
 * Selection criteria:
 * 1. Filter robots by confidence, distance, and max yaw deviation
 * 2. If current target is still valid and within hysteresis, keep it
 * 3. Otherwise, select the robot with minimum yaw deviation
 * 
 * This is useful for:
 * - Minimizing gimbal rotation needed to lock on target
 * - Maintaining target lock stability (hysteresis)
 * - Quick acquisition of nearest angular target
 */
class MinYawDeviationStrategy : public SelectionStrategy {
public:
  MinYawDeviationStrategy() = default;
  ~MinYawDeviationStrategy() override = default;

  /**
   * @brief Select target with minimum yaw deviation from reference
   * 
   * @param robots List of tracked robots
   * @param config Selection configuration (reference_yaw is the key parameter)
   * @return Selected robot or std::nullopt
   */
  std::optional<SelectionResult> selectTarget(
    const TrackedRobots& robots,
    const SelectionConfig& config) override;

  std::string getName() const override {
    return "MinYawDeviation";
  }

  std::string getDescription() const override {
    return "Selects the robot with minimum yaw deviation from reference direction";
  }

private:
  /**
   * @brief Find the robot with minimum yaw deviation
   * 
   * @param candidates Filtered candidate robots
   * @param reference_yaw Reference yaw direction
   * @return Robot with minimum yaw deviation
   */
  const TrackedRobot* findMinYawDeviationRobot(
    const std::vector<const TrackedRobot*>& candidates,
    double reference_yaw) const;

  /**
   * @brief Check if current target should be kept (hysteresis)
   * 
   * @param candidates Filtered candidates
   * @param current_target_id Currently locked target ID
   * @param best_candidate Best new candidate
   * @param reference_yaw Reference yaw
   * @param hysteresis_threshold Threshold for switching
   * @return true if should keep current target
   */
  bool shouldKeepCurrentTarget(
    const std::vector<const TrackedRobot*>& candidates,
    const std::string& current_target_id,
    const TrackedRobot* best_candidate,
    double reference_yaw,
    double hysteresis_threshold) const;

  /**
   * @brief Find robot by ID in candidates list
   */
  const TrackedRobot* findRobotById(
    const std::vector<const TrackedRobot*>& candidates,
    const std::string& robot_id) const;
};

}  // namespace fyt::auto_aim

#endif  // TARGET_SELECTOR__STRATEGIES__MIN_YAW_DEVIATION_STRATEGY_HPP_
