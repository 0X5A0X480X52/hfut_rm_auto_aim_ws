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

#ifndef ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_CORE_HPP_
#define ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_CORE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "robot_pose_estimator/robot_types.hpp"
#include "robot_pose_estimator/robot_tracker.hpp"
#include "robot_pose_estimator/armor_grouper.hpp"
#include "robot_pose_estimator/virtual_armor_generator.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"

namespace fyt::auto_aim {

/**
 * @brief 机器人姿态估计器核心类
 * 
 * 管理多个机器人跟踪器，提供:
 * - 装甲板分组
 * - 机器人状态估计
 * - 虚拟装甲板生成
 */
class RobotPoseEstimatorCore {
public:
  /**
   * @brief 构造函数
   * @param config 配置参数
   */
  explicit RobotPoseEstimatorCore(const PoseEstimatorConfig& config);
  
  /**
   * @brief 析构函数
   */
  ~RobotPoseEstimatorCore() = default;

  /**
   * @brief 更新姿态估计
   * @param tracked_armors 跟踪的装甲板
   * @param dt 时间间隔
   */
  void update(const rm_interfaces::msg::TrackedArmors& tracked_armors, double dt);

  /**
   * @brief 仅执行预测步骤
   * @param dt 时间间隔
   */
  void predict(double dt);

  /**
   * @brief 获取所有机器人状态
   * @return 机器人状态列表
   */
  std::vector<RobotState> getRobotStates() const;

  /**
   * @brief 获取正在跟踪的机器人状态
   * @return 正在跟踪的机器人状态列表
   */
  std::vector<RobotState> getTrackingRobots() const;

  /**
   * @brief 生成所有机器人的虚拟装甲板
   * @return 虚拟装甲板消息列表
   */
  std::vector<rm_interfaces::msg::Armor> generateVirtualArmors() const;

  /**
   * @brief 重置估计器
   */
  void reset();

  /**
   * @brief 更新配置
   * @param config 新配置
   */
  void updateConfig(const PoseEstimatorConfig& config);

  /**
   * @brief 获取当前配置
   */
  const PoseEstimatorConfig& getConfig() const { return config_; }

private:
  /**
   * @brief 处理新检测到的机器人
   * @param robot_id 机器人ID
   * @param armors 该机器人的装甲板
   * @param dt 时间间隔
   */
  void processRobot(const std::string& robot_id, 
                    const std::vector<ArmorState>& armors,
                    double dt);

  /**
   * @brief 清理丢失的跟踪器
   */
  void pruneTrackers();

private:
  // 配置
  PoseEstimatorConfig config_;
  
  // 组件
  ArmorGrouper grouper_;
  VirtualArmorGenerator virtual_generator_;
  
  // 机器人跟踪器 (robot_id -> tracker)
  std::map<std::string, std::unique_ptr<RobotTracker>> trackers_;
  
  // 有效的跟踪状态
  std::vector<uint8_t> valid_states_;
  
  // 线程安全
  mutable std::mutex mutex_;
};

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_CORE_HPP_
