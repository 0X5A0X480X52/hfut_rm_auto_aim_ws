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

#ifndef ARMOR_TRACKER__TRACK_HISTORY_MANAGER_HPP_
#define ARMOR_TRACKER__TRACK_HISTORY_MANAGER_HPP_

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "armor_tracker/armor_types.hpp"
#include "rm_interfaces/msg/track_history_window.hpp"
#include "rm_interfaces/msg/track_history_windows.hpp"
#include "rm_interfaces/msg/track_window_state.hpp"

namespace fyt::auto_aim {

/**
 * @brief 历史窗口配置
 */
struct HistoryWindowConfig {
  uint32_t max_window_size = 100;   ///< 窗口最大容量
  uint32_t record_interval = 1;     ///< 记录间隔 (每多少次迭代记录一次)
};

/**
 * @brief 单个跟踪对象的历史状态
 */
struct TrackHistoryEntry {
  uint64_t iteration;               ///< 迭代序号
  builtin_interfaces::msg::Time timestamp;
  TrackedArmorState state;
};

/**
 * @brief 跟踪历史窗口管理器
 * 
 * 为每个跟踪对象维护一个滑动窗口队列，记录最近一段时间内的状态。
 * 通过 track_id 唯一标识每个跟踪对象的历史状态。
 */
class TrackHistoryManager {
public:
  /**
   * @brief 构造函数
   * @param config 历史窗口配置
   */
  explicit TrackHistoryManager(const HistoryWindowConfig& config = HistoryWindowConfig());
  
  /**
   * @brief 析构函数
   */
  ~TrackHistoryManager() = default;
  
  /**
   * @brief 更新历史记录
   * @param tracks 当前所有跟踪结果
   * @param timestamp 当前时间戳
   */
  void update(const std::vector<TrackedArmorState>& tracks,
              const builtin_interfaces::msg::Time& timestamp);
  
  /**
   * @brief 获取指定跟踪对象的历史窗口
   * @param track_id 跟踪ID
   * @return 历史窗口消息
   */
  rm_interfaces::msg::TrackHistoryWindow getHistoryWindow(int track_id) const;
  
  /**
   * @brief 获取所有跟踪对象的历史窗口
   * @return 所有历史窗口消息
   */
  rm_interfaces::msg::TrackHistoryWindows getAllHistoryWindows() const;
  
  /**
   * @brief 移除指定跟踪对象的历史
   * @param track_id 跟踪ID
   */
  void removeTrack(int track_id);
  
  /**
   * @brief 清除所有历史记录
   */
  void clear();
  
  /**
   * @brief 更新配置
   * @param config 新配置
   */
  void updateConfig(const HistoryWindowConfig& config);
  
  /**
   * @brief 获取当前配置
   */
  const HistoryWindowConfig& getConfig() const { return config_; }
  
  /**
   * @brief 获取当前迭代序号
   */
  uint64_t getCurrentIteration() const { return current_iteration_; }

private:
  /**
   * @brief 将内部状态转换为消息格式
   */
  rm_interfaces::msg::TrackWindowState stateToMessage(
    const TrackHistoryEntry& entry) const;
  
  /**
   * @brief 清理已丢失的跟踪对象
   */
  void cleanupLostTracks(const std::vector<TrackedArmorState>& current_tracks);

private:
  HistoryWindowConfig config_;
  
  // track_id -> 历史状态队列
  std::map<int, std::deque<TrackHistoryEntry>> history_map_;
  
  // track_id -> (armor_id, armor_type)
  std::map<int, std::pair<std::string, std::string>> track_info_map_;
  
  // 当前迭代序号
  uint64_t current_iteration_;
  
  // 线程安全
  mutable std::mutex mutex_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__TRACK_HISTORY_MANAGER_HPP_
