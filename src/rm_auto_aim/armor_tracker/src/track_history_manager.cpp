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

#include "armor_tracker/track_history_manager.hpp"

#include <algorithm>
#include <set>

namespace fyt::auto_aim {

TrackHistoryManager::TrackHistoryManager(const HistoryWindowConfig& config)
  : config_(config)
  , current_iteration_(0)
{
}

void TrackHistoryManager::update(
  const std::vector<TrackedArmorState>& tracks,
  const builtin_interfaces::msg::Time& timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  current_iteration_++;
  
  // 检查是否需要记录（根据记录间隔）
  if (config_.record_interval > 0 && 
      current_iteration_ % config_.record_interval != 0) {
    return;
  }
  
  // 更新每个跟踪对象的历史
  for (const auto& track : tracks) {
    // 创建历史条目
    TrackHistoryEntry entry;
    entry.iteration = current_iteration_;
    entry.timestamp = timestamp;
    entry.state = track;
    
    // 获取或创建该 track_id 的历史队列
    auto& history = history_map_[track.track_id];
    
    // 添加到队列末尾
    history.push_back(entry);
    
    // 如果超过最大容量，移除最旧的
    while (history.size() > config_.max_window_size) {
      history.pop_front();
    }
    
    // 更新 track 信息
    track_info_map_[track.track_id] = {track.armor_id, track.armor_type};
  }
  
  // 清理已丢失的跟踪对象
  cleanupLostTracks(tracks);
}

rm_interfaces::msg::TrackHistoryWindow TrackHistoryManager::getHistoryWindow(
  int track_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  rm_interfaces::msg::TrackHistoryWindow window;
  window.track_id = track_id;
  window.max_window_size = config_.max_window_size;
  window.record_interval = config_.record_interval;
  window.current_iteration = current_iteration_;
  
  auto it = history_map_.find(track_id);
  if (it != history_map_.end()) {
    // 填充 armor 信息
    auto info_it = track_info_map_.find(track_id);
    if (info_it != track_info_map_.end()) {
      window.armor_id = info_it->second.first;
      window.armor_type = info_it->second.second;
    }
    
    // 填充历史状态
    window.history.reserve(it->second.size());
    for (const auto& entry : it->second) {
      window.history.push_back(stateToMessage(entry));
    }
  }
  
  return window;
}

rm_interfaces::msg::TrackHistoryWindows TrackHistoryManager::getAllHistoryWindows() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  rm_interfaces::msg::TrackHistoryWindows windows;
  windows.windows.reserve(history_map_.size());
  
  for (const auto& [track_id, history] : history_map_) {
    rm_interfaces::msg::TrackHistoryWindow window;
    window.track_id = track_id;
    window.max_window_size = config_.max_window_size;
    window.record_interval = config_.record_interval;
    window.current_iteration = current_iteration_;
    
    // 填充 armor 信息
    auto info_it = track_info_map_.find(track_id);
    if (info_it != track_info_map_.end()) {
      window.armor_id = info_it->second.first;
      window.armor_type = info_it->second.second;
    }
    
    // 填充历史状态
    window.history.reserve(history.size());
    for (const auto& entry : history) {
      window.history.push_back(stateToMessage(entry));
    }
    
    windows.windows.push_back(window);
  }
  
  return windows;
}

void TrackHistoryManager::removeTrack(int track_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_map_.erase(track_id);
  track_info_map_.erase(track_id);
}

void TrackHistoryManager::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_map_.clear();
  track_info_map_.clear();
  current_iteration_ = 0;
}

void TrackHistoryManager::updateConfig(const HistoryWindowConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  
  // 如果新的窗口大小更小，需要裁剪现有历史
  for (auto& [track_id, history] : history_map_) {
    while (history.size() > config_.max_window_size) {
      history.pop_front();
    }
  }
}

rm_interfaces::msg::TrackWindowState TrackHistoryManager::stateToMessage(
  const TrackHistoryEntry& entry) const
{
  rm_interfaces::msg::TrackWindowState msg;
  
  msg.iteration = entry.iteration;
  msg.timestamp = entry.timestamp;
  
  msg.position.x = entry.state.position.x();
  msg.position.y = entry.state.position.y();
  msg.position.z = entry.state.position.z();
  
  msg.velocity.x = entry.state.velocity.x();
  msg.velocity.y = entry.state.velocity.y();
  msg.velocity.z = entry.state.velocity.z();
  
  msg.yaw = entry.state.yaw;
  msg.yaw_velocity = entry.state.yaw_velocity;
  
  msg.confidence = entry.state.confidence;
  msg.source_type = static_cast<uint8_t>(entry.state.source_type);
  msg.tracking_state = static_cast<uint8_t>(entry.state.tracking_state);
  
  return msg;
}

void TrackHistoryManager::cleanupLostTracks(
  const std::vector<TrackedArmorState>& current_tracks)
{
  // 收集当前活跃的 track_id
  std::set<int> active_ids;
  for (const auto& track : current_tracks) {
    active_ids.insert(track.track_id);
  }
  
  // 移除不在活跃列表中且状态为 LOST 的跟踪
  // 注意：这里保留一段时间的历史，即使跟踪丢失
  // 可以通过配置项控制是否立即清理
  // 当前实现：只有当跟踪完全从跟踪器中消失时才清理
  
  std::vector<int> to_remove;
  for (const auto& [track_id, _] : history_map_) {
    if (active_ids.find(track_id) == active_ids.end()) {
      to_remove.push_back(track_id);
    }
  }
  
  for (int id : to_remove) {
    history_map_.erase(id);
    track_info_map_.erase(id);
  }
}

}  // namespace fyt::auto_aim
