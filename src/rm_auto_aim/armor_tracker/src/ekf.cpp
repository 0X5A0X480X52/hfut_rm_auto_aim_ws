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

#include "armor_tracker/ekf.hpp"
#include <cmath>

namespace fyt::auto_aim {

ArmorEKF::ArmorEKF(
  const StateMatrix& process_noise_cov,
  const MeasurementMatrix& measurement_noise_cov)
: process_noise_cov_(process_noise_cov)
, measurement_noise_cov_(measurement_noise_cov)
{
  state_.setZero();
  covariance_.setIdentity();
}

void ArmorEKF::init(const StateVector& initial_state) {
  state_ = initial_state;
  covariance_.setIdentity();
  covariance_ *= 10.0;  // 初始不确定性
}

void ArmorEKF::predict(double dt) {
  // 状态预测
  StateVector predicted_state = stateTransition(state_, dt);
  
  // 雅可比矩阵
  StateMatrix F = stateTransitionJacobian(dt);
  
  // 协方差预测
  StateMatrix Q = process_noise_cov_;
  // 根据dt调整过程噪声
  Q *= dt;
  
  covariance_ = F * covariance_ * F.transpose() + Q;
  state_ = predicted_state;
}

void ArmorEKF::update(const MeasurementVector& measurement) {
  // 观测预测
  MeasurementVector predicted_measurement = observationFunction(state_);
  
  // 观测雅可比矩阵
  MeasurementStateMatrix H = observationJacobian();
  
  // 残差
  MeasurementVector innovation = measurement - predicted_measurement;
  
  // 归一化yaw角差到 [-pi, pi]
  innovation(3) = std::atan2(std::sin(innovation(3)), std::cos(innovation(3)));
  
  // 创新协方差
  MeasurementMatrix S = H * covariance_ * H.transpose() + measurement_noise_cov_;
  
  // 卡尔曼增益
  Eigen::Matrix<double, STATE_DIM, MEASUREMENT_DIM> K = 
    covariance_ * H.transpose() * S.inverse();
  
  // 状态更新
  state_ = state_ + K * innovation;
  
  // 协方差更新
  StateMatrix I = StateMatrix::Identity();
  covariance_ = (I - K * H) * covariance_;
}

Eigen::Vector3d ArmorEKF::getPosition() const {
  return Eigen::Vector3d(state_(0), state_(2), state_(4));
}

Eigen::Vector3d ArmorEKF::getVelocity() const {
  return Eigen::Vector3d(state_(1), state_(3), state_(5));
}

ArmorEKF::StateVector ArmorEKF::stateTransition(
  const StateVector& state, double dt) const {
  StateVector next_state;
  
  // 位置更新: x = x + vx * dt
  next_state(0) = state(0) + state(1) * dt;
  next_state(1) = state(1);  // 速度保持不变 (匀速模型)
  
  next_state(2) = state(2) + state(3) * dt;
  next_state(3) = state(3);
  
  next_state(4) = state(4) + state(5) * dt;
  next_state(5) = state(5);
  
  // yaw角更新
  next_state(6) = state(6) + state(7) * dt;
  next_state(7) = state(7);
  
  return next_state;
}

ArmorEKF::StateMatrix ArmorEKF::stateTransitionJacobian(double dt) const {
  StateMatrix F = StateMatrix::Identity();
  
  // 位置对速度的偏导数
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;
  
  return F;
}

ArmorEKF::MeasurementVector ArmorEKF::observationFunction(
  const StateVector& state) const {
  MeasurementVector z;
  z(0) = state(0);  // x
  z(1) = state(2);  // y
  z(2) = state(4);  // z
  z(3) = state(6);  // yaw
  return z;
}

ArmorEKF::MeasurementStateMatrix ArmorEKF::observationJacobian() const {
  MeasurementStateMatrix H = MeasurementStateMatrix::Zero();
  
  H(0, 0) = 1.0;  // x
  H(1, 2) = 1.0;  // y
  H(2, 4) = 1.0;  // z
  H(3, 6) = 1.0;  // yaw
  
  return H;
}

}  // namespace fyt::auto_aim
