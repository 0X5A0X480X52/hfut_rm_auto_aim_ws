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

#ifndef ROBOT_POSE_ESTIMATOR__ROBOT_TYPES_HPP_
#define ROBOT_POSE_ESTIMATOR__ROBOT_TYPES_HPP_

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace fyt::auto_aim {

/**
 * @brief 机器人类型枚举
 */
enum class RobotType : uint8_t {
  BALANCE_2 = 0,    ///< 平衡步兵: 2装甲板
  STANDARD_4 = 1,   ///< 标准机器人: 4装甲板
  HERO_4 = 2,       ///< 英雄: 4装甲板
  OUTPOST_3 = 3,    ///< 前哨站: 3装甲板
  SENTRY = 4,       ///< 哨兵
  BASE = 5,         ///< 基地
  UNKNOWN = 255     ///< 未知类型
};

/**
 * @brief 装甲板数量枚举
 */
enum class ArmorsNum : int {
  BALANCE_2 = 2,    ///< 平衡步兵: 2装甲板
  OUTPOST_3 = 3,    ///< 前哨站: 3装甲板
  NORMAL_4 = 4      ///< 标准/英雄: 4装甲板
};

/**
 * @brief 机器人配置参数
 */
struct RobotConfig {
  RobotType type = RobotType::UNKNOWN;
  int num_armors = 4;
  double radius = 0.26;         ///< 默认旋转半径
  double height_offset = 0.0;   ///< 中心高度偏移
  
  // 获取装甲板角度分布
  std::vector<double> getArmorAngles() const {
    std::vector<double> angles;
    double step = 2.0 * M_PI / num_armors;
    for (int i = 0; i < num_armors; ++i) {
      angles.push_back(i * step);
    }
    return angles;
  }
};

/**
 * @brief 单个装甲板的状态信息
 */
struct ArmorState {
  std::string armor_id;        ///< 装甲板ID
  std::string armor_type;      ///< 装甲板类型 (small/large)
  Eigen::Vector3d position;    ///< 3D位置
  Eigen::Vector3d velocity;    ///< 3D速度
  double yaw;                  ///< yaw角
  double yaw_velocity;         ///< yaw角速度
  double confidence;           ///< 置信度
  bool is_observed;            ///< 是否被观测到
  int track_id;                ///< 跟踪ID
  
  ArmorState() 
    : position(Eigen::Vector3d::Zero())
    , velocity(Eigen::Vector3d::Zero())
    , yaw(0.0)
    , yaw_velocity(0.0)
    , confidence(0.0)
    , is_observed(false)
    , track_id(-1) {}
};

/**
 * @brief 机器人状态估计结果
 */
struct RobotState {
  std::string robot_id;                    ///< 机器人ID
  RobotType robot_type = RobotType::UNKNOWN;  ///< 机器人类型
  int num_armors = 0;                      ///< 装甲板数量
  
  // 中心位置和速度
  Eigen::Vector3d center_position;         ///< 机器人中心位置
  Eigen::Vector3d center_velocity;         ///< 机器人中心速度
  
  // 姿态信息
  double yaw = 0.0;                        ///< yaw角
  double yaw_velocity = 0.0;               ///< yaw角速度
  
  // 几何参数
  double radius_1 = 0.26;                  ///< 主半径
  double radius_2 = 0.26;                  ///< 副半径 (4装甲板时可能不同)
  double d_zc = 0.0;                       ///< 中心z偏移
  double d_za = 0.0;                       ///< 装甲板z偏移差
  
  // 装甲板状态
  std::vector<ArmorState> armor_states;    ///< 所有装甲板状态
  std::vector<std::string> bound_armor_ids; ///< 绑定的装甲板跟踪ID
  
  // 估计信息
  double confidence = 0.0;                 ///< 估计置信度
  bool is_tracking = false;                ///< 是否正在跟踪
  
  RobotState() 
    : center_position(Eigen::Vector3d::Zero())
    , center_velocity(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief 姿态估计器配置
 */
struct PoseEstimatorConfig {
  // EKF参数
  double sigma2_q_xyz = 0.05;              ///< 位置过程噪声
  double sigma2_q_yaw = 1.0;               ///< yaw过程噪声
  double sigma2_q_r = 0.05;                ///< 半径过程噪声
  double r_xyz = 0.05;                     ///< 位置观测噪声
  double r_yaw = 0.02;                     ///< yaw观测噪声
  
  // 关联参数
  double max_match_distance = 0.5;         ///< 最大匹配距离
  double max_match_yaw_diff = 1.0;         ///< 最大yaw差
  
  // 跟踪参数
  int tracking_threshold = 5;              ///< 进入跟踪状态的帧数
  int lost_threshold = 100;                ///< 丢失阈值 (帧数)
  double robot_timeout = 2.0;              ///< 机器人超时时间
  
  // 机器人配置
  struct RobotParams {
    double balance_radius = 0.15;
    double standard_radius = 0.23;
    double hero_radius = 0.28;
    double outpost_radius = 0.26;
  } robot_params;
  
  // 虚拟装甲板生成
  bool virtual_armor_generation = true;    ///< 是否生成虚拟装甲板
};

/**
 * @brief 根据装甲板ID和类型获取机器人类型
 */
inline RobotType getRobotTypeFromArmor(const std::string& armor_id, const std::string& armor_type) {
  if (armor_id == "outpost") {
    return RobotType::OUTPOST_3;
  } else if (armor_id == "base") {
    return RobotType::BASE;
  } else if (armor_type == "large" && 
             (armor_id == "3" || armor_id == "4" || armor_id == "5")) {
    return RobotType::BALANCE_2;
  } else if (armor_id == "1") {
    return RobotType::STANDARD_4;
  } else if (armor_id == "2") {
    return RobotType::HERO_4;
  } else {
    return RobotType::STANDARD_4;  // 默认为标准4装甲板
  }
}

/**
 * @brief 获取机器人装甲板数量
 */
inline int getArmorsNumFromType(RobotType type) {
  switch (type) {
    case RobotType::BALANCE_2:
      return 2;
    case RobotType::OUTPOST_3:
      return 3;
    case RobotType::STANDARD_4:
    case RobotType::HERO_4:
    case RobotType::SENTRY:
    case RobotType::BASE:
    default:
      return 4;
  }
}

/**
 * @brief 获取机器人类型名称
 */
inline std::string getRobotTypeName(RobotType type) {
  switch (type) {
    case RobotType::BALANCE_2:
      return "Balance";
    case RobotType::STANDARD_4:
      return "Standard";
    case RobotType::HERO_4:
      return "Hero";
    case RobotType::OUTPOST_3:
      return "Outpost";
    case RobotType::SENTRY:
      return "Sentry";
    case RobotType::BASE:
      return "Base";
    default:
      return "Unknown";
  }
}

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__ROBOT_TYPES_HPP_
