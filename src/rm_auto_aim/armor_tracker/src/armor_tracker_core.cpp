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

#include "armor_tracker/armor_tracker_core.hpp"

#include <algorithm>

namespace fyt::auto_aim {

ArmorTrackerCore::ArmorTrackerCore(const TrackerConfig& config)
  : config_(config)
  , strategy_manager_(std::make_shared<TrackingStrategyManager>())
  , last_source_type_(ArmorSourceType::PREDICT)
  , initialized_(false)
{
  // 加载模型配置
  ModelConfig model_config;
  if (!config_.model_config_file.empty()) {
    try {
      model_config = ModelConfigLoader::loadFromYaml(config_.model_config_file);
    } catch (const std::exception& e) {
      // 使用默认配置
    }
  }

  // 创建 PointTracker（适用于3D点跟踪）
  tracker_ = std::make_unique<muit_obj_tracker::PointTracker>(
    config_.lost_threshold,      // max_age
    config_.tracking_threshold,  // min_hits
    config_.max_match_distance,  // distance_threshold
    config_.model_name,
    model_config
  );

  initialized_ = true;
}

void ArmorTrackerCore::predict() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (tracker_) {
    tracker_->predict();
  }
}

void ArmorTrackerCore::setStrategyManager(std::shared_ptr<TrackingStrategyManager> manager) {
  std::lock_guard<std::mutex> lock(mutex_);
  strategy_manager_ = manager;
}

const TrackerConfig& ArmorTrackerCore::getConfig() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void ArmorTrackerCore::update(const std::vector<ArmorObservation>& observations) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!tracker_ || observations.empty()) {
    return;
  }

  // 转换观测数据
  std::vector<Detection> detections;
  detections.reserve(observations.size());

  for (const auto& obs : observations) {
    auto strategy = strategy_manager_->getStrategy(obs.source);
    if (strategy) {
      // 预处理
      ArmorObservation processed_obs = strategy->preprocess(obs);
      detections.push_back(observationToDetection(processed_obs));
    } else {
      detections.push_back(observationToDetection(obs));
    }
  }

  // 更新跟踪器
  tracker_->update(detections);

  // 更新装甲板信息映射
  auto tracks = tracker_->getTracks();
  for (size_t i = 0; i < tracks.size() && i < observations.size(); ++i) {
    armor_info_map_[tracks[i].track_id] = {
      observations[i].armor_id,
      observations[i].armor_type
    };
    source_type_map_[tracks[i].track_id] = observations[i].source;
  }

  // 记录最后的来源类型
  if (!observations.empty()) {
    last_source_type_ = observations.front().source;
  }
}

void ArmorTrackerCore::predictUpdate() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (tracker_) {
    // 仅执行预测，不提供观测
    tracker_->predict();

    // 更新所有跟踪为预测来源
    for (auto& [track_id, source] : source_type_map_) {
      source = ArmorSourceType::PREDICT;
    }
    last_source_type_ = ArmorSourceType::PREDICT;
  }
}

std::vector<TrackedArmorState> ArmorTrackerCore::getTracks() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<TrackedArmorState> results;

  if (!tracker_) {
    return results;
  }

  auto tracks = tracker_->getTracks();
  results.reserve(tracks.size());

  for (const auto& track : tracks) {
    auto [armor_id, armor_type] = findArmorInfo(track.track_id);
    auto state = trackResultToState(track, armor_id, armor_type);

    // 应用策略后处理
    auto source_it = source_type_map_.find(track.track_id);
    ArmorSourceType source = (source_it != source_type_map_.end())
                              ? source_it->second
                              : ArmorSourceType::PREDICT;

    auto strategy = strategy_manager_->getStrategy(source);
    if (strategy) {
      strategy->postprocess(state);
    }

    results.push_back(state);
  }

  return results;
}

std::vector<TrackedArmorState> ArmorTrackerCore::getTracksByState(TrackingState state) const {
  auto all_tracks = getTracks();
  std::vector<TrackedArmorState> filtered;

  std::copy_if(all_tracks.begin(), all_tracks.end(), std::back_inserter(filtered),
    [state](const TrackedArmorState& s) { return s.tracking_state == state; });

  return filtered;
}

void ArmorTrackerCore::reset() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (tracker_) {
    tracker_->reset();
  }
  armor_info_map_.clear();
  source_type_map_.clear();
  last_source_type_ = ArmorSourceType::PREDICT;
}

void ArmorTrackerCore::updateConfig(const TrackerConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;

  // 重新创建跟踪器
  ModelConfig model_config;
  if (!config_.model_config_file.empty()) {
    try {
      model_config = ModelConfigLoader::loadFromYaml(config_.model_config_file);
    } catch (const std::exception& e) {
      // 使用默认配置
    }
  }

  tracker_ = std::make_unique<muit_obj_tracker::PointTracker>(
    config_.lost_threshold,
    config_.tracking_threshold,
    config_.max_match_distance,
    config_.model_name,
    model_config
  );

  armor_info_map_.clear();
  source_type_map_.clear();
}

ArmorTrackerCore::Detection ArmorTrackerCore::observationToDetection(
  const ArmorObservation& obs) const {
  Detection det;

  // 使用装甲板ID的哈希作为检测ID
  det.id = std::hash<std::string>{}(obs.armor_id) % 1000;
  det.confidence = obs.confidence;

  // 将3D位置映射到bbox（用于内部数据关联）
  // 这里使用简化的映射：将 x, y, z 编码到 bbox 中
  // PointTracker 会从 bbox 中心提取位置
  det.bbox = cv::Rect(
    static_cast<int>(obs.position.x() * 1000),  // x -> bbox.x
    static_cast<int>(obs.position.y() * 1000),  // y -> bbox.y
    static_cast<int>(obs.position.z() * 1000),  // z -> bbox.width (用于存储)
    static_cast<int>(obs.yaw * 1000)            // yaw -> bbox.height (用于存储)
  );

  return det;
}

TrackedArmorState ArmorTrackerCore::trackResultToState(
  const TrackResult& result,
  const std::string& armor_id,
  const std::string& armor_type) const {

  TrackedArmorState state;
  state.track_id = result.track_id;
  state.armor_id = armor_id;
  state.armor_type = armor_type;

  // 从状态向量提取位置和速度
  // 根据模型配置，状态向量格式可能不同
  // 假设使用 3D CV_KF: [x, vx, y, vy, z, vz] 或类似格式
  const auto& s = result.state;

  if (s.size() >= 6) {
    // 3D 位置和速度
    state.position = Eigen::Vector3d(s(0), s(2), s(4));
    state.velocity = Eigen::Vector3d(s(1), s(3), s(5));

    // 如果状态向量包含 yaw
    if (s.size() >= 8) {
      state.yaw = s(6);
      state.yaw_velocity = s(7);
    } else {
      // 从 bbox 恢复 yaw
      state.yaw = result.bbox.height / 1000.0;
      state.yaw_velocity = 0.0;
    }
  } else if (s.size() >= 4) {
    // 2D 模式，从 bbox 恢复 z
    state.position = Eigen::Vector3d(s(0), s(2), result.bbox.width / 1000.0);
    state.velocity = Eigen::Vector3d(s(1), s(3), 0.0);
    state.yaw = result.bbox.height / 1000.0;
    state.yaw_velocity = 0.0;
  }

  // 计算跟踪状态
  state.tracking_state = computeTrackingState(result);

  // 设置来源类型（默认为预测）
  auto source_it = source_type_map_.find(result.track_id);
  state.source_type = (source_it != source_type_map_.end()) 
                       ? source_it->second 
                       : ArmorSourceType::PREDICT;

  // 置信度
  state.confidence = result.is_active ? 1.0 : 0.5;

  return state;
}

std::pair<std::string, std::string> ArmorTrackerCore::findArmorInfo(int track_id) const {
  auto it = armor_info_map_.find(track_id);
  if (it != armor_info_map_.end()) {
    return it->second;
  }
  return {"unknown", "unknown"};
}

TrackingState ArmorTrackerCore::computeTrackingState(const TrackResult& result) const {
  if (!result.is_active) {
    return TrackingState::LOST;
  }

  // 根据 muit_obj_tracker 的逻辑判断状态
  // PointTracker 使用 time_since_update 和 hits 来管理轨迹
  // 这里需要从 TrackResult 推断状态

  // 简化的状态推断：
  // - is_active = true 且在返回列表中 -> TRACKING 或 DETECTING
  // 更精确的状态需要访问内部 Track 结构

  return TrackingState::TRACKING;
}

void ArmorTrackerCore::applyStrategy(
  TrackedArmorState& state,
  ArmorSourceType source_type,
  const ArmorObservation* observation) {

  auto strategy = strategy_manager_->getStrategy(source_type);
  if (strategy) {
    strategy->postprocess(state);
  }
}

} // namespace fyt::auto_aim
