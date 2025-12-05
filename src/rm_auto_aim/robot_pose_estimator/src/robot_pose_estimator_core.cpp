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

#include "robot_pose_estimator/robot_pose_estimator_core.hpp"

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

RobotPoseEstimatorCore::RobotPoseEstimatorCore(const PoseEstimatorConfig& config)
  : config_(config) {
  // 默认有效状态: DETECTING(1), TRACKING(2), TEMP_LOST(3)
  // 对应 TrackedArmor 消息中的状态常量
  valid_states_ = {1, 2, 3};  // DETECTING, TRACKING, TEMP_LOST
}

void RobotPoseEstimatorCore::update(
    const rm_interfaces::msg::TrackedArmors& tracked_armors, double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 1. 将装甲板按机器人ID分组
  auto armor_groups = grouper_.groupArmors(tracked_armors, valid_states_);
  
  // 2. 对每个机器人执行更新
  for (const auto& [robot_id, armors] : armor_groups) {
    processRobot(robot_id, armors, dt);
  }
  
  // 3. 对未观测到的机器人执行预测
  for (auto& [robot_id, tracker] : trackers_) {
    if (armor_groups.find(robot_id) == armor_groups.end()) {
      tracker->predict(dt);
    }
  }
  
  // 4. 清理丢失的跟踪器
  pruneTrackers();
}

void RobotPoseEstimatorCore::predict(double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [robot_id, tracker] : trackers_) {
    tracker->predict(dt);
  }
  
  pruneTrackers();
}

void RobotPoseEstimatorCore::processRobot(
    const std::string& robot_id,
    const std::vector<ArmorState>& armors,
    double dt) {
  
  auto it = trackers_.find(robot_id);
  
  if (it == trackers_.end()) {
    // 新机器人，创建跟踪器
    if (!armors.empty()) {
      auto tracker = std::make_unique<RobotTracker>(config_);
      tracker->init(armors[0]);
      
      // 如果有多个装甲板，执行一次更新
      if (armors.size() > 1) {
        tracker->update(armors, dt);
      }
      
      trackers_[robot_id] = std::move(tracker);
      FYT_INFO("robot_pose_estimator", "Created tracker for robot {}", robot_id);
    }
  } else {
    // 已有跟踪器，执行更新
    it->second->update(armors, dt);
  }
}

void RobotPoseEstimatorCore::pruneTrackers() {
  for (auto it = trackers_.begin(); it != trackers_.end();) {
    if (it->second->shouldRemove()) {
      FYT_INFO("robot_pose_estimator", "Removed tracker for robot {}", it->first);
      it = trackers_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<RobotState> RobotPoseEstimatorCore::getRobotStates() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<RobotState> states;
  states.reserve(trackers_.size());
  
  for (const auto& [robot_id, tracker] : trackers_) {
    states.push_back(tracker->getRobotState());
  }
  
  return states;
}

std::vector<RobotState> RobotPoseEstimatorCore::getTrackingRobots() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<RobotState> states;
  
  for (const auto& [robot_id, tracker] : trackers_) {
    auto state = tracker->getState();
    if (state == RobotTracker::State::TRACKING || 
        state == RobotTracker::State::TEMP_LOST ||
        state == RobotTracker::State::DETECTING) {
      states.push_back(tracker->getRobotState());
    }
  }
  
  return states;
}

std::vector<rm_interfaces::msg::Armor> RobotPoseEstimatorCore::generateVirtualArmors() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<rm_interfaces::msg::Armor> all_armors;
  
  for (const auto& [robot_id, tracker] : trackers_) {
    auto state = tracker->getRobotState();
    auto armors = virtual_generator_.generateArmors(state);
    all_armors.insert(all_armors.end(), armors.begin(), armors.end());
  }
  
  return all_armors;
}

void RobotPoseEstimatorCore::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.clear();
  FYT_INFO("robot_pose_estimator", "Estimator reset");
}

void RobotPoseEstimatorCore::updateConfig(const PoseEstimatorConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  FYT_INFO("robot_pose_estimator", "Config updated");
}

}  // namespace fyt::auto_aim
