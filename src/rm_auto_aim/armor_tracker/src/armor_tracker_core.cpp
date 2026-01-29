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
#include <limits>
#include "rm_utils/logger/log.hpp"

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
      // 使用默认配置，但需要根据模型类型设置正确的维度
      FYT_WARN("armor_tracker", "Failed to load model config from {}, using default", config_.model_config_file);
    }
  }
  
  // 根据模型名称设置正确的默认配置
  if (config_.model_name == "CV_KF") {
    // CV_KF: 3D跟踪，状态维度6 (x,vx,y,vy,z,vz)，观测维度3 (x,y,z)
    model_config.Dim = 3;
    model_config.T = 0.01;  // 100Hz
    model_config.R = Eigen::MatrixXd::Identity(3, 3) * 0.01;  // 观测噪声
    model_config.X_0 = Eigen::VectorXd::Zero(6);  // 初始状态
  } else if (config_.model_name == "CA_KF" || config_.model_name == "CS_KF" || config_.model_name == "Singer_KF") {
    // 这些模型: 3D跟踪，状态维度9 (x,vx,ax,y,vy,ay,z,vz,az)，观测维度3 (x,y,z)
    model_config.Dim = 3;
    model_config.T = 0.01;
    model_config.R = Eigen::MatrixXd::Identity(3, 3) * 0.01;
    model_config.X_0 = Eigen::VectorXd::Zero(9);
  } else if (config_.model_name == "CTRV_EKF") {
    // CTRV_EKF: 状态维度5 (x,y,v,theta,omega)，观测维度2 (x,y)
    model_config.Dim = 1;  // CTRV是2D模型
    model_config.T = 0.01;
    model_config.R = Eigen::MatrixXd::Identity(2, 2) * 0.01;
    model_config.X_0 = Eigen::VectorXd::Zero(5);
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
    FYT_DEBUG("armor_tracker", "[update] Early return: tracker={}, observations.size()={}", 
              (tracker_ ? "valid" : "null"), observations.size());
    return;
  }

  // 转换观测数据
  std::vector<Detection> detections;
  detections.reserve(observations.size());

  FYT_DEBUG("armor_tracker", "[update] Processing {} observations", observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& obs = observations[i];
    FYT_DEBUG("armor_tracker", "  Obs[{}]: armor_id='{}', pos=({:.3f}, {:.3f}, {:.3f})",
             i, obs.armor_id, obs.position.x(), obs.position.y(), obs.position.z());
    
    auto strategy = strategy_manager_->getStrategy(obs.source);
    if (strategy) {
      // 预处理
      ArmorObservation processed_obs = strategy->preprocess(obs);
      detections.push_back(observationToDetection(processed_obs));
    } else {
      detections.push_back(observationToDetection(obs));
    }
  }

  // 打印更新前的跟踪器状态
  auto tracks_before = tracker_->getTracks();
  FYT_DEBUG("armor_tracker", "[update] Before: {} tracks, {} detections", 
            tracks_before.size(), detections.size());

  // 更新跟踪器
  tracker_->update(detections);

  // 更新装甲板信息映射
  // 只对刚刚被更新的track（time_since_update == 0）更新armor_id映射
  // 这样可以确保只有被匈牙利算法实际匹配到的track才会更新其armor_id
  auto tracks = tracker_->getTracks();
  FYT_DEBUG("armor_tracker", "[update] After: {} tracks", tracks.size());
  
  for (size_t i = 0; i < tracks.size(); ++i) {
    int track_id = tracks[i].track_id;
    
    // 只有刚被更新的track才需要更新armor_id映射
    if (tracks[i].time_since_update != 0) {
      // 跳过未被更新的track，保留其原有的armor_id
      auto [old_id, old_type] = findArmorInfo(track_id);
      FYT_DEBUG("armor_tracker", "  Track[{}] not updated (tsu={}), keep id='{}'",
               track_id, tracks[i].time_since_update, old_id);
      continue;
    }
    
    // 从 track state 中提取位置（适配不同模型）
    Eigen::Vector3d track_pos;
    const auto& state = tracks[i].state;
    if (state.size() >= 9) {
      // CA_KF/CS_KF/Singer_KF: [x, vx, ax, y, vy, ay, z, vz, az]
      track_pos = Eigen::Vector3d(state(0), state(3), state(6));
    } else if (state.size() >= 6) {
      // CV_KF: [x, vx, y, vy, z, vz]
      track_pos = Eigen::Vector3d(state(0), state(2), state(4));
    } else if (state.size() >= 5) {
      // CTRV_EKF: [x, y, v, theta, omega]
      track_pos = Eigen::Vector3d(state(0), state(1), 0);
    } else {
      track_pos = Eigen::Vector3d::Zero();
    }
    
    // 找到最近的observation
    double min_dist = std::numeric_limits<double>::max();
    size_t best_obs_idx = 0;
    for (size_t j = 0; j < observations.size(); ++j) {
      double dist = (track_pos - observations[j].position).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_obs_idx = j;
      }
    }
    
    // 更新映射（只有距离在合理范围内）
    if (min_dist < config_.max_match_distance * 2.0 && !observations.empty()) {
      armor_info_map_[track_id] = {
        observations[best_obs_idx].armor_id,
        observations[best_obs_idx].armor_type
      };
      source_type_map_[track_id] = observations[best_obs_idx].source;
      
      // 保存观测的yaw值（用于预测时使用，因为卡尔曼状态不包含yaw）
      yaw_info_map_[track_id] = {
        observations[best_obs_idx].yaw,
        0.0  // yaw_velocity 暂时设为0，后续可通过差分估计
      };
      
      FYT_DEBUG("armor_tracker", "  Track[{}] updated -> Obs[{}] (id='{}', yaw={:.3f}, dist={:.4f})",
               track_id, best_obs_idx, observations[best_obs_idx].armor_id, 
               observations[best_obs_idx].yaw, min_dist);
    } else {
      auto [old_id, old_type] = findArmorInfo(track_id);
      FYT_DEBUG("armor_tracker", "  Track[{}] no match (dist={:.4f}), keep id='{}'",
               track_id, min_dist, old_id);
    }
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
  yaw_info_map_.clear();
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

  // 直接使用米作为单位，不进行缩放
  det.position = obs.position;  // 米
  det.yaw = obs.yaw;            // 弧度

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
  // 注意：数据直接以米为单位，不需要缩放
  const auto& s = result.state;

  if (s.size() >= 9) {
    // 3D CA_KF/CS_KF/Singer_KF: [x, vx, ax, y, vy, ay, z, vz, az]
    // 位置索引: 0,3,6  速度索引: 1,4,7
    state.position = Eigen::Vector3d(s(0), s(3), s(6));
    state.velocity = Eigen::Vector3d(s(1), s(4), s(7));

  } else if (s.size() >= 6) {
    // 3D CV_KF: [x, vx, y, vy, z, vz]
    // 位置索引: 0,2,4  速度索引: 1,3,5
    state.position = Eigen::Vector3d(s(0), s(2), s(4));
    state.velocity = Eigen::Vector3d(s(1), s(3), s(5));

  } else if (s.size() >= 4) {
    // 2D 模式
    state.position = Eigen::Vector3d(s(0), s(2), 0.0);
    state.velocity = Eigen::Vector3d(s(1), s(3), 0.0);
  }
  
  // 从yaw_info_map_获取观测的yaw值（卡尔曼状态不包含yaw）
  auto yaw_it = yaw_info_map_.find(result.track_id);
  if (yaw_it != yaw_info_map_.end()) {
    state.yaw = yaw_it->second.first;
    state.yaw_velocity = yaw_it->second.second;
  } else {
    // 如果没有保存的yaw，使用装甲板正对相机的假设作为fallback
    // 装甲板的yaw定义：装甲板法向量方向（从PnP/BA得到）
    // 当装甲板正对相机时，其法向量指向相机（即相反于视角方向）
    // 视角方向 = atan2(y, x)，装甲板正对相机时 yaw = 视角方向 + π
    double view_angle = std::atan2(state.position.y(), state.position.x());
    state.yaw = view_angle + M_PI;
    // 归一化到 [-π, π]
    while (state.yaw > M_PI) state.yaw -= 2 * M_PI;
    while (state.yaw < -M_PI) state.yaw += 2 * M_PI;
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
