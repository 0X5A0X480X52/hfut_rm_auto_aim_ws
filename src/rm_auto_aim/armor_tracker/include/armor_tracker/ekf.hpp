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

#ifndef ARMOR_TRACKER__EKF_HPP_
#define ARMOR_TRACKER__EKF_HPP_

#include <Eigen/Dense>
#include <functional>

namespace fyt::auto_aim {

/**
 * @brief 装甲板跟踪用的扩展卡尔曼滤波器
 * 
 * 状态向量: [x, vx, y, vy, z, vz, yaw, vyaw]
 * 观测向量: [x, y, z, yaw]
 */
class ArmorEKF {
public:
  // 状态维度
  static constexpr int STATE_DIM = 8;
  // 观测维度
  static constexpr int MEASUREMENT_DIM = 4;

  using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
  using MeasurementVector = Eigen::Matrix<double, MEASUREMENT_DIM, 1>;
  using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  using MeasurementMatrix = Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM>;
  using MeasurementStateMatrix = Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM>;

  /**
   * @brief 构造函数
   * @param process_noise_cov 过程噪声协方差
   * @param measurement_noise_cov 测量噪声协方差
   */
  ArmorEKF(
    const StateMatrix& process_noise_cov,
    const MeasurementMatrix& measurement_noise_cov
  );

  /**
   * @brief 初始化滤波器
   * @param initial_state 初始状态
   */
  void init(const StateVector& initial_state);

  /**
   * @brief 预测步骤
   * @param dt 时间间隔
   */
  void predict(double dt);

  /**
   * @brief 更新步骤
   * @param measurement 观测值
   */
  void update(const MeasurementVector& measurement);

  /**
   * @brief 获取当前状态估计
   */
  StateVector getState() const { return state_; }

  /**
   * @brief 获取位置 [x, y, z]
   */
  Eigen::Vector3d getPosition() const;

  /**
   * @brief 获取速度 [vx, vy, vz]
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * @brief 获取yaw角
   */
  double getYaw() const { return state_(6); }

  /**
   * @brief 获取yaw角速度
   */
  double getYawVelocity() const { return state_(7); }

  /**
   * @brief 设置状态
   */
  void setState(const StateVector& state) { state_ = state; }

private:
  StateVector state_;                    // 状态向量
  StateMatrix covariance_;               // 协方差矩阵
  StateMatrix process_noise_cov_;        // 过程噪声协方差
  MeasurementMatrix measurement_noise_cov_;  // 测量噪声协方差

  /**
   * @brief 状态转移函数
   * @param state 当前状态
   * @param dt 时间间隔
   * @return 预测状态
   */
  StateVector stateTransition(const StateVector& state, double dt) const;

  /**
   * @brief 状态转移雅可比矩阵
   * @param dt 时间间隔
   * @return 雅可比矩阵
   */
  StateMatrix stateTransitionJacobian(double dt) const;

  /**
   * @brief 观测函数
   * @param state 状态
   * @return 观测值
   */
  MeasurementVector observationFunction(const StateVector& state) const;

  /**
   * @brief 观测雅可比矩阵
   * @return 雅可比矩阵
   */
  MeasurementStateMatrix observationJacobian() const;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__EKF_HPP_
