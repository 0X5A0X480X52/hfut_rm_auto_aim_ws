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

#ifndef ARMOR_TRACKER__ARMOR_TYPES_HPP_
#define ARMOR_TRACKER__ARMOR_TYPES_HPP_

#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace fyt::auto_aim {

/**
 * @brief 装甲板观测来源类型
 * 用于标识当前状态估计的数据来源
 */
enum class ArmorSourceType : uint8_t {
  DETECT = 0,    ///< 融合了上游检测节点的位置估计
  ESTIMATE = 1,  ///< 融合了robot_pose_estimator发布的估计装甲板位置
  PREDICT = 2    ///< 无观测输入，仅通过模型预测的位置估计
};

/**
 * @brief 装甲板跟踪状态
 * 与 muit_obj_tracker 的状态机兼容
 */
enum class TrackingState : uint8_t {
  LOST = 0,       ///< 跟踪丢失，即将被移除
  DETECTING = 1,  ///< 检测中，尚未确认跟踪
  TRACKING = 2,   ///< 正在跟踪
  TEMP_LOST = 3   ///< 临时丢失，使用预测维持
};

/**
 * @brief 装甲板观测数据
 * 用于从不同来源接收装甲板检测结果
 */
struct ArmorObservation {
  std::string armor_id;        ///< 装甲板ID: "1"~"5", "outpost", "base"
  std::string armor_type;      ///< 装甲板类型: "small", "large"
  
  Eigen::Vector3d position;    ///< 3D位置
  double yaw;                  ///< yaw角
  float confidence;            ///< 检测置信度
  
  ArmorSourceType source;      ///< 观测来源
  builtin_interfaces::msg::Time timestamp;  ///< 时间戳
  
  ArmorObservation();
};

/**
 * @brief 装甲板跟踪结果
 * 包含完整的状态估计信息
 */
struct TrackedArmorState {
  int track_id;                ///< 跟踪ID (由跟踪器分配)
  std::string armor_id;        ///< 装甲板ID
  std::string armor_type;      ///< 装甲板类型
  
  // 状态估计
  Eigen::Vector3d position;    ///< 位置 [x, y, z]
  Eigen::Vector3d velocity;    ///< 速度 [vx, vy, vz]
  double yaw;                  ///< yaw角
  double yaw_velocity;         ///< yaw角速度
  
  // 跟踪状态
  TrackingState tracking_state;  ///< 跟踪状态
  ArmorSourceType source_type;   ///< 当前状态的来源类型
  double confidence;             ///< 置信度 [0, 1]
  
  // 统计信息
  int tracking_count;          ///< 连续跟踪帧数
  int lost_count;              ///< 连续丢失帧数
  int time_since_update;       ///< 自上次更新后的帧数
  
  builtin_interfaces::msg::Time last_detected_time;  ///< 最后检测时间
  
  TrackedArmorState();
};

/**
 * @brief 跟踪器配置参数
 */
struct TrackerConfig {
  // 数据关联参数
  double max_match_distance = 0.5;    ///< 最大匹配距离(米)
  double max_match_yaw_diff = 1.0;    ///< 最大匹配yaw角差(弧度)
  
  // 状态机参数
  int tracking_threshold = 3;         ///< 进入TRACKING状态的最小匹配次数
  int lost_threshold = 30;            ///< 进入LOST状态的最大丢失帧数
  int max_trackers = 20;              ///< 最大跟踪器数量
  
  // 异步更新参数
  double predict_rate = 100.0;        ///< 预测更新频率(Hz)
  double max_dt = 0.1;                ///< 最大预测时间间隔(秒)
  
  // 模型配置
  std::string model_name = "CV_KF";   ///< 使用的滤波模型名称
  std::string model_config_file = ""; ///< 模型配置文件路径
};

/**
 * @brief 将 ArmorSourceType 转换为字符串
 */
std::string sourceTypeToString(ArmorSourceType type);

/**
 * @brief 将 TrackingState 转换为字符串
 */
std::string trackingStateToString(TrackingState state);

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__ARMOR_TYPES_HPP_
