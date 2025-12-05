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

#ifndef ROBOT_POSE_ESTIMATOR__MOTION_MODEL_HPP_
#define ROBOT_POSE_ESTIMATOR__MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::auto_aim {

/**
 * @brief 运动模型类型
 */
enum class MotionModel {
  CONSTANT_VELOCITY = 0,  ///< 匀速模型
  CONSTANT_ROTATION = 1,  ///< 匀转模型
  CONSTANT_VEL_ROT = 2    ///< 匀速+匀转模型
};

/**
 * @brief 机器人状态维度
 * 状态向量: [xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc]
 * - xc, yc, zc: 机器人中心位置
 * - vxc, vyc, vzc: 机器人中心速度
 * - yaw: 机器人yaw角
 * - v_yaw: yaw角速度
 * - r: 旋转半径
 * - d_zc: z方向偏移
 */
constexpr int ROBOT_STATE_DIM = 10;

/**
 * @brief 观测维度
 * 观测向量: [xa, ya, za, yaw_a]
 * - xa, ya, za: 装甲板位置
 * - yaw_a: 装甲板yaw角
 */
constexpr int ROBOT_MEAS_DIM = 4;

/**
 * @brief 预测函数
 * 基于运动模型预测下一时刻状态
 */
struct RobotPredict {
  explicit RobotPredict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
    : dt_(dt), model_(model) {}

  template <typename T>
  void operator()(const T x0[ROBOT_STATE_DIM], T x1[ROBOT_STATE_DIM]) {
    for (int i = 0; i < ROBOT_STATE_DIM; ++i) {
      x1[i] = x0[i];
    }

    // 位置预测
    if (model_ == MotionModel::CONSTANT_VEL_ROT || model_ == MotionModel::CONSTANT_VELOCITY) {
      // 线速度预测
      x1[0] += x0[1] * dt_;  // xc += vxc * dt
      x1[2] += x0[3] * dt_;  // yc += vyc * dt
      x1[4] += x0[5] * dt_;  // zc += vzc * dt
    } else {
      // 无速度
      x1[1] = T(0);
      x1[3] = T(0);
      x1[5] = T(0);
    }

    // yaw预测
    if (model_ == MotionModel::CONSTANT_VEL_ROT || model_ == MotionModel::CONSTANT_ROTATION) {
      // 角速度预测
      x1[6] += x0[7] * dt_;  // yaw += v_yaw * dt
    } else {
      // 无旋转
      x1[7] = T(0);
    }
  }

  double dt_;
  MotionModel model_;
};

/**
 * @brief 观测函数
 * 从状态计算观测值（装甲板位置）
 */
struct RobotMeasure {
  template <typename T>
  void operator()(const T x[ROBOT_STATE_DIM], T z[ROBOT_MEAS_DIM]) {
    // xa = xc - r * cos(yaw)
    z[0] = x[0] - ceres::cos(x[6]) * x[8];
    // ya = yc - r * sin(yaw)
    z[1] = x[2] - ceres::sin(x[6]) * x[8];
    // za = zc + d_zc
    z[2] = x[4] + x[9];
    // yaw_a = yaw
    z[3] = x[6];
  }
};

/**
 * @brief 机器人状态EKF类型定义
 */
using RobotStateEKF = ExtendedKalmanFilter<ROBOT_STATE_DIM, ROBOT_MEAS_DIM, RobotPredict, RobotMeasure>;

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__MOTION_MODEL_HPP_
