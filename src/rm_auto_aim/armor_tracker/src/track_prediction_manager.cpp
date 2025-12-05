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

#include "armor_tracker/track_prediction_manager.hpp"

#include <cmath>

namespace fyt::auto_aim {

TrackPredictionManager::TrackPredictionManager(const PredictionWindowConfig& config)
  : config_(config)
  , current_iteration_(0)
{
}

void TrackPredictionManager::generatePredictions(
  const std::vector<TrackedArmorState>& tracks,
  const builtin_interfaces::msg::Time& timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  current_iteration_++;
  
  // 检查是否需要生成预测（根据预测间隔）
  if (config_.predict_interval > 0 && 
      current_iteration_ % config_.predict_interval != 0) {
    return;
  }
  
  // 清除旧的预测
  prediction_map_.clear();
  
  // 为每个跟踪对象生成预测
  for (const auto& track : tracks) {
    // 只为活跃的跟踪生成预测
    if (track.tracking_state == TrackingState::LOST) {
      continue;
    }
    
    rm_interfaces::msg::TrackPredictionWindow window;
    window.track_id = track.track_id;
    window.armor_id = track.armor_id;
    window.armor_type = track.armor_type;
    window.prediction_steps = config_.prediction_steps;
    window.predict_interval = config_.predict_interval;
    window.current_iteration = current_iteration_;
    
    // 生成预测状态序列
    window.predictions = predictTrack(track, timestamp);
    
    prediction_map_[track.track_id] = window;
  }
}

rm_interfaces::msg::TrackPredictionWindow TrackPredictionManager::getPredictionWindow(
  int track_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = prediction_map_.find(track_id);
  if (it != prediction_map_.end()) {
    return it->second;
  }
  
  // 返回空窗口
  rm_interfaces::msg::TrackPredictionWindow empty;
  empty.track_id = track_id;
  empty.prediction_steps = config_.prediction_steps;
  empty.predict_interval = config_.predict_interval;
  empty.current_iteration = current_iteration_;
  return empty;
}

rm_interfaces::msg::TrackPredictionWindows TrackPredictionManager::getAllPredictionWindows() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  rm_interfaces::msg::TrackPredictionWindows windows;
  windows.windows.reserve(prediction_map_.size());
  
  for (const auto& [track_id, window] : prediction_map_) {
    windows.windows.push_back(window);
  }
  
  return windows;
}

void TrackPredictionManager::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  prediction_map_.clear();
  current_iteration_ = 0;
}

void TrackPredictionManager::updateConfig(const PredictionWindowConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

std::vector<rm_interfaces::msg::TrackWindowState> TrackPredictionManager::predictTrack(
  const TrackedArmorState& current_state,
  const builtin_interfaces::msg::Time& timestamp)
{
  std::vector<rm_interfaces::msg::TrackWindowState> predictions;
  predictions.reserve(config_.prediction_steps);
  
  // 当前状态
  Eigen::Vector3d pos = current_state.position;
  Eigen::Vector3d vel = current_state.velocity;
  double yaw = current_state.yaw;
  double yaw_vel = current_state.yaw_velocity;
  
  // 基础时间戳（秒和纳秒）
  int32_t base_sec = timestamp.sec;
  uint32_t base_nanosec = timestamp.nanosec;
  
  // 生成预测序列
  for (uint32_t step = 1; step <= config_.prediction_steps; ++step) {
    double dt = config_.dt * step;
    
    // 预测位置和 yaw
    Eigen::Vector3d pred_pos = predictPositionCV(pos, vel, dt);
    double pred_yaw = predictYawCV(yaw, yaw_vel, dt);
    
    // 创建预测状态消息
    rm_interfaces::msg::TrackWindowState state;
    state.iteration = current_iteration_ + step;
    
    // 计算预测时间戳
    double total_nanosec = base_nanosec + dt * 1e9;
    int32_t extra_sec = static_cast<int32_t>(total_nanosec / 1e9);
    state.timestamp.sec = base_sec + extra_sec;
    state.timestamp.nanosec = static_cast<uint32_t>(
      std::fmod(total_nanosec, 1e9));
    
    state.position.x = pred_pos.x();
    state.position.y = pred_pos.y();
    state.position.z = pred_pos.z();
    
    // 速度保持不变（匀速模型）
    state.velocity.x = vel.x();
    state.velocity.y = vel.y();
    state.velocity.z = vel.z();
    
    state.yaw = pred_yaw;
    state.yaw_velocity = yaw_vel;
    
    // 置信度随预测步数衰减
    state.confidence = current_state.confidence * std::exp(-0.02 * step);
    
    // 来源类型为预测
    state.source_type = static_cast<uint8_t>(ArmorSourceType::PREDICT);
    
    // 跟踪状态保持（如果原本是 TRACKING，预测状态视为 TEMP_LOST）
    if (current_state.tracking_state == TrackingState::TRACKING) {
      state.tracking_state = static_cast<uint8_t>(TrackingState::TRACKING);
    } else {
      state.tracking_state = static_cast<uint8_t>(current_state.tracking_state);
    }
    
    predictions.push_back(state);
  }
  
  return predictions;
}

Eigen::Vector3d TrackPredictionManager::predictPositionCV(
  const Eigen::Vector3d& pos,
  const Eigen::Vector3d& vel,
  double dt)
{
  // 匀速模型: x(t) = x(0) + v * t
  return pos + vel * dt;
}

double TrackPredictionManager::predictYawCV(double yaw, double yaw_vel, double dt)
{
  // 匀速角度模型
  double pred_yaw = yaw + yaw_vel * dt;
  
  // 归一化到 [-pi, pi]
  while (pred_yaw > M_PI) pred_yaw -= 2.0 * M_PI;
  while (pred_yaw < -M_PI) pred_yaw += 2.0 * M_PI;
  
  return pred_yaw;
}

}  // namespace fyt::auto_aim
