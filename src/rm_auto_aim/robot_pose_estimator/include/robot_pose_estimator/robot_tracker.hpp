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

#ifndef ROBOT_POSE_ESTIMATOR__ROBOT_TRACKER_HPP_
#define ROBOT_POSE_ESTIMATOR__ROBOT_TRACKER_HPP_

#include <memory>
#include <string>
#include <deque>

#include <Eigen/Dense>
#include <angles/angles.h>

#include "robot_pose_estimator/robot_types.hpp"

namespace fyt::auto_aim {

/**
 * @brief 机器人状态维度
 * 状态向量: [xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc]
 */
constexpr int ROBOT_STATE_DIM = 10;

/**
 * @brief 观测维度
 * 观测向量: [xa, ya, za, yaw_a]
 */
constexpr int ROBOT_MEAS_DIM = 4;

/**
 * @brief 单个机器人跟踪器
 * 
 * 基于 armor_solver 中的 Tracker 类实现，使用 EKF 估计机器人状态
 */
class RobotTracker {
public:
  /**
   * @brief 跟踪器状态
   */
  enum class State {
    LOST,        ///< 丢失
    DETECTING,   ///< 检测中
    TRACKING,    ///< 跟踪中
    TEMP_LOST    ///< 临时丢失
  };

  /**
   * @brief 构造函数
   * @param config 配置参数
   */
  explicit RobotTracker(const PoseEstimatorConfig& config);
  
  /**
   * @brief 析构函数
   */
  ~RobotTracker() = default;

  /**
   * @brief 使用第一个装甲板初始化跟踪器
   * @param armor 初始化用的装甲板状态
   */
  void init(const ArmorState& armor);

  /**
   * @brief 更新跟踪器
   * @param armors 当前帧检测到的同一机器人的所有装甲板
   * @param dt 时间间隔
   * @return 是否成功匹配
   */
  bool update(const std::vector<ArmorState>& armors, double dt);

  /**
   * @brief 仅执行预测步骤
   * @param dt 时间间隔
   */
  void predict(double dt);

  /**
   * @brief 获取机器人状态
   * @return 机器人状态
   */
  RobotState getRobotState() const;

  /**
   * @brief 获取所有装甲板的估计位置
   * @return 装甲板位置列表
   */
  std::vector<Eigen::Vector3d> getArmorPositions() const;

  /**
   * @brief 设置绑定的装甲板ID列表
   * @param armor_ids 装甲板ID列表
   */
  void setBoundArmorIds(const std::vector<std::string>& armor_ids);

  /**
   * @brief 设置当前绑定的 track 数量（来自 armor_tracker）
   * @param count 绑定的 track 数量
   */
  void setBoundTrackCount(int count);

  /**
   * @brief 获取跟踪器状态
   */
  State getState() const { return state_; }

  /**
   * @brief 获取机器人ID
   */
  std::string getRobotId() const { return robot_id_; }

  /**
   * @brief 获取机器人类型
   */
  RobotType getRobotType() const { return robot_type_; }

  /**
   * @brief 获取装甲板数量
   */
  int getArmorsNum() const { return num_armors_; }

  /**
   * @brief 检查是否应该移除该跟踪器
   */
  bool shouldRemove() const;

  /**
   * @brief 获取置信度
   */
  double getConfidence() const;

private:
  /**
   * @brief 初始化EKF
   * @param armor 初始化用的装甲板状态
   */
  void initEKF(const ArmorState& armor);

  /**
   * @brief 处理装甲板跳变（旋转导致的观测装甲板切换）
   * @param armor 当前观测到的装甲板
   */
  void handleArmorJump(const ArmorState& armor);

  /**
   * @brief 从状态向量计算装甲板位置
   * @param state 状态向量
   * @return 装甲板位置
   */
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& state) const;

  /**
   * @brief 找到最佳匹配的装甲板
   * @param armors 候选装甲板列表
   * @return 最佳匹配的索引，-1表示无匹配
   */
  int findBestMatch(const std::vector<ArmorState>& armors);

  /**
   * @brief 根据装甲板信息确定机器人类型
   * @param armor 装甲板状态
   */
  void determineRobotType(const ArmorState& armor);

  /**
   * @brief 更新状态机
   * @param matched 是否匹配成功
   */
  void updateStateMachine(bool matched);

  /**
   * @brief 预测步骤（EKF）
   * @param dt 时间间隔
   */
  void predictStep(double dt);

  /**
   * @brief 更新步骤（EKF）
   * @param z 观测向量
   */
  void updateStep(const Eigen::VectorXd& z);

private:
  // 配置
  PoseEstimatorConfig config_;
  
  // 状态
  State state_ = State::LOST;
  std::string robot_id_;
  RobotType robot_type_ = RobotType::UNKNOWN;
  int num_armors_ = 4;
  
  // EKF状态
  Eigen::VectorXd target_state_;
  Eigen::VectorXd measurement_;
  
  // 协方差矩阵
  Eigen::MatrixXd P_pri_;   ///< 先验协方差
  Eigen::MatrixXd P_post_;  ///< 后验协方差
  
  // 几何参数
  double d_za_ = 0.0;        ///< 装甲板z偏移差
  double another_r_ = 0.26;  ///< 另一对装甲板的半径
  double d_zc_ = 0.0;        ///< 中心z偏移
  
  // 跟踪统计
  int detect_count_ = 0;
  int lost_count_ = 0;
  double last_yaw_ = 0.0;
  
  // 绑定的装甲板ID列表
  std::vector<std::string> bound_armor_ids_;
  
  // 来自 armor_tracker 的绑定 track 数量（用于置信度计算）
  int bound_track_count_ = 0;
  
  // 最后匹配的装甲板
  ArmorState last_armor_;
  
  // 滑动窗口用于平滑置信度（保存最近若干帧的观测置信度）
  mutable std::deque<double> confidence_window_;
  mutable size_t confidence_window_size_ = 20;  // 默认窗口大小
};

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__ROBOT_TRACKER_HPP_
