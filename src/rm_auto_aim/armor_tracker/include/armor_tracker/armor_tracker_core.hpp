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

#ifndef ARMOR_TRACKER__ARMOR_TRACKER_CORE_HPP_
#define ARMOR_TRACKER__ARMOR_TRACKER_CORE_HPP_

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <functional>
#include <utility>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "muit_obj_tracker/tracker/tracker_manager.hpp"
#include "muit_obj_tracker/tracker/point_tracker.hpp"
#include "muit_obj_tracker/utils/types.hpp"
#include "models/model_config_loader.h"

#include "armor_tracker/armor_types.hpp"
#include "armor_tracker/strategies/tracking_strategy_manager.hpp"

namespace fyt::auto_aim {

/**
 * @brief 装甲板3D跟踪器核心类
 * 
 * 基于 muit_obj_tracker 实现3D装甲板跟踪，支持：
 * - 多目标跟踪（匈牙利算法数据关联）
 * - 异步预测和更新
 * - 多来源数据融合（检测/估计/预测）
 * - 策略模式处理不同来源
 */
class ArmorTrackerCore {
public:
  using Detection = muit_obj_tracker::Detection;
  using TrackResult = muit_obj_tracker::TrackResult;
  using TrackerType = muit_obj_tracker::TrackerType;
  
  /**
   * @brief 构造函数
   * @param config 跟踪器配置
   */
  explicit ArmorTrackerCore(const TrackerConfig& config);
  
  /**
   * @brief 析构函数
   */
  ~ArmorTrackerCore() = default;
  
  /**
   * @brief 执行预测步骤
   * 应在定时器中调用，用于异步更新
   */
  void predict();
  
  /**
   * @brief 使用检测结果更新跟踪器
   * @param observations 装甲板观测列表
   */
  void update(const std::vector<ArmorObservation>& observations);
  
  /**
   * @brief 仅执行预测更新（无观测）
   * 用于在没有检测结果时维持跟踪
   */
  void predictUpdate();
  
  /**
   * @brief 获取所有跟踪结果
   * @return 跟踪结果列表
   */
  std::vector<TrackedArmorState> getTracks() const;
  
  /**
   * @brief 获取指定状态的跟踪结果
   * @param state 跟踪状态
   * @return 符合条件的跟踪结果列表
   */
  std::vector<TrackedArmorState> getTracksByState(TrackingState state) const;
  
  /**
   * @brief 重置跟踪器
   */
  void reset();
  
  /**
   * @brief 设置跟踪策略管理器
   * @param manager 策略管理器
   */
  void setStrategyManager(std::shared_ptr<TrackingStrategyManager> manager);
  
  /**
   * @brief 获取配置
   */
  const TrackerConfig& getConfig() const;
  
  /**
   * @brief 更新配置
   * @param config 新配置
   */
  void updateConfig(const TrackerConfig& config);

private:
  /**
   * @brief 将 ArmorObservation 转换为 Detection
   * @param obs 装甲板观测
   * @return muit_obj_tracker 的 Detection 结构
   */
  Detection observationToDetection(const ArmorObservation& obs) const;
  
  /**
   * @brief 将 TrackResult 转换为 TrackedArmorState
   * @param result 跟踪结果
   * @param armor_id 装甲板ID
   * @param armor_type 装甲板类型
   * @return 装甲板跟踪状态
   */
  TrackedArmorState trackResultToState(
    const TrackResult& result,
    const std::string& armor_id,
    const std::string& armor_type) const;
  
  /**
   * @brief 根据 track_id 查找对应的装甲板信息
   */
  std::pair<std::string, std::string> findArmorInfo(int track_id) const;
  
  /**
   * @brief 计算跟踪状态
   */
  TrackingState computeTrackingState(const TrackResult& result) const;
  
  /**
   * @brief 应用策略处理
   */
  void applyStrategy(
    TrackedArmorState& state,
    ArmorSourceType source_type,
    const ArmorObservation* observation = nullptr);

private:
  TrackerConfig config_;
  
  // muit_obj_tracker 实例
  std::unique_ptr<muit_obj_tracker::ITracker> tracker_;
  
  // 策略管理器
  std::shared_ptr<TrackingStrategyManager> strategy_manager_;
  
  // 装甲板信息映射: track_id -> (armor_id, armor_type)
  std::map<int, std::pair<std::string, std::string>> armor_info_map_;
  
  // yaw信息映射: track_id -> (yaw, yaw_velocity)
  // 用于保存观测的yaw值，因为卡尔曼滤波状态向量不包含yaw
  std::map<int, std::pair<double, double>> yaw_info_map_;
  
  // 来源类型映射: track_id -> ArmorSourceType
  std::map<int, ArmorSourceType> source_type_map_;
  
  // 最后更新的来源类型
  ArmorSourceType last_source_type_;
  
  // 线程安全
  mutable std::mutex mutex_;
  
  // 是否已初始化
  bool initialized_;
};

// Implementations moved to corresponding source file.

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__ARMOR_TRACKER_CORE_HPP_
