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

#ifndef ARMOR_TRACKER__TRACK_PREDICTION_MANAGER_HPP_
#define ARMOR_TRACKER__TRACK_PREDICTION_MANAGER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Dense>

#include "armor_tracker/armor_types.hpp"
#include "rm_interfaces/msg/track_prediction_window.hpp"
#include "rm_interfaces/msg/track_prediction_windows.hpp"
#include "rm_interfaces/msg/track_window_state.hpp"

namespace fyt::auto_aim {

/**
 * @brief 预测窗口配置
 */
struct PredictionWindowConfig {
  uint32_t prediction_steps = 30;    ///< 预测步数
  uint32_t predict_interval = 1;     ///< 预测间隔 (每多少次迭代记录一次)
  double dt = 0.01;                  ///< 预测时间步长 (秒)
};

/**
 * @brief 跟踪预测窗口管理器
 * 
 * 利用状态向量预测每个跟踪对象未来 N 步的状态。
 * 通过 track_id 唯一标识每个跟踪对象的预测状态。
 */
class TrackPredictionManager {
public:
  /**
   * @brief 构造函数
   * @param config 预测窗口配置
   */
  explicit TrackPredictionManager(const PredictionWindowConfig& config = PredictionWindowConfig());
  
  /**
   * @brief 析构函数
   */
  ~TrackPredictionManager() = default;
  
  /**
   * @brief 生成预测窗口
   * 
   * 根据当前跟踪状态和状态向量，预测未来 N 步的状态。
   * 使用简化的运动学模型进行预测（匀速/匀加速）。
   * 
   * @param tracks 当前所有跟踪结果
   * @param timestamp 当前时间戳
   */
  void generatePredictions(const std::vector<TrackedArmorState>& tracks,
                           const builtin_interfaces::msg::Time& timestamp);
  
  /**
   * @brief 获取指定跟踪对象的预测窗口
   * @param track_id 跟踪ID
   * @return 预测窗口消息
   */
  rm_interfaces::msg::TrackPredictionWindow getPredictionWindow(int track_id) const;
  
  /**
   * @brief 获取所有跟踪对象的预测窗口
   * @return 所有预测窗口消息
   */
  rm_interfaces::msg::TrackPredictionWindows getAllPredictionWindows() const;
  
  /**
   * @brief 清除所有预测
   */
  void clear();
  
  /**
   * @brief 更新配置
   * @param config 新配置
   */
  void updateConfig(const PredictionWindowConfig& config);
  
  /**
   * @brief 获取当前配置
   */
  const PredictionWindowConfig& getConfig() const { return config_; }
  
  /**
   * @brief 获取当前迭代序号
   */
  uint64_t getCurrentIteration() const { return current_iteration_; }

private:
  /**
   * @brief 预测单个对象的未来状态
   * @param current_state 当前状态
   * @param timestamp 当前时间戳
   * @return 预测状态列表
   */
  std::vector<rm_interfaces::msg::TrackWindowState> predictTrack(
    const TrackedArmorState& current_state,
    const builtin_interfaces::msg::Time& timestamp);
  
  /**
   * @brief 使用匀速模型预测位置
   */
  Eigen::Vector3d predictPositionCV(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    double dt);
  
  /**
   * @brief 使用匀速模型预测 yaw
   */
  double predictYawCV(double yaw, double yaw_vel, double dt);

private:
  PredictionWindowConfig config_;
  
  // track_id -> 预测窗口消息
  std::map<int, rm_interfaces::msg::TrackPredictionWindow> prediction_map_;
  
  // 当前迭代序号
  uint64_t current_iteration_;
  
  // 线程安全
  mutable std::mutex mutex_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__TRACK_PREDICTION_MANAGER_HPP_
