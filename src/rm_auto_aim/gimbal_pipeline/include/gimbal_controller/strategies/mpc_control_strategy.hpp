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

#ifndef GIMBAL_CONTROLLER__STRATEGIES__MPC_CONTROL_STRATEGY_HPP_
#define GIMBAL_CONTROLLER__STRATEGIES__MPC_CONTROL_STRATEGY_HPP_

#include <Eigen/Dense>

#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/mpc/gimbal_dynamics_model.hpp"
#include "gimbal_controller/mpc/qp_solver.hpp"
#include "gimbal_controller/mpc/mpc_reference_generator.hpp"

namespace gimbal_controller
{

/**
 * @brief MPC 控制策略
 *
 * 基于模型预测控制的云台控制策略:
 *   1. 差分估计云台角速度 (无下位机反馈)
 *   2. 调用 MpcReferenceGenerator 生成 N 步参考轨迹
 *   3. 构建 QP (含延迟补偿) 并用 qpOASES 求解
 *   4. 提取首步控制量推算目标 yaw/pitch
 *   5. 调用 FireAdvisor 判断开火
 */
class MpcControlStrategy : public GimbalControlStrategy
{
public:
  MpcControlStrategy();
  ~MpcControlStrategy() override = default;

  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext & context) override;

  std::string getName() const override { return "MpcControlStrategy"; }

  /**
   * @brief 设置 MPC 参数
   * @param N 预测步数
   * @param dt 时间步长 (秒)
   * @param control_delay_s 控制延迟 (秒)
   * @param max_accel 最大允许角加速度 (rad/s²)
   * @param q_yaw / q_pitch 位置跟踪权重
   * @param q_yaw_vel / q_pitch_vel 速度跟踪权重
   * @param r_yaw / r_pitch 控制量权重
   * @param s_yaw / s_pitch 控制平滑权重
   */
  void setMpcParameters(
    int N, double dt, double control_delay_s, double max_accel,
    double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel,
    double r_yaw, double r_pitch, double s_yaw, double s_pitch);

  /**
   * @brief 设置延时补偿参数
   * @param enable 是否启用延时补偿版本的参考轨迹生成
   * @param prediction_delay_s 额外预测延迟 (秒)
   * @param flight_time_iters 飞行时间迭代次数
   * @param max_processing_delay_s 最大允许的 processing_delay 上限 (秒)，超出则被截断
   */
  void setDelayCompensation(
    bool enable, double prediction_delay_s, int flight_time_iters,
    double max_processing_delay_s);

  /**
   * @brief 设置 MPC 机动自适应权重衰减参数
   *
   * 启用后，每帧通过对 UKF center_velocity 做时间戳感知差分计算机动因子 alpha，
   * 用 alpha 衰减远期 Q 权重并放大 R 正则项，在机动时减少控制量。
   * 禁用时 (enable=false) 与原实现完全一致。
   *
   * @param enable   是否启用
   * @param a_max    差分速度归一化上限 (m/s²)
   * @param eta      alpha EMA 平滑系数 (0.1~0.3)
   * @param tau      Q 衰减时间常数 (步数尺度)
   * @param r_scale  R 放大系数
   */
  void setManeuverAdaptParameters(
    bool enable, double a_max, double eta, double tau, double r_scale);

  /**
   * @brief 在 setComponents() 之后调用, 将组件注入到 MpcReferenceGenerator
   */
  void initReferenceGenerator();

private:
  // MPC 核心模块
  mpc::GimbalDynamicsModel dynamics_model_;
  mpc::QPSolver qp_solver_;
  mpc::MpcReferenceGenerator ref_generator_;

  // 缓存的 QP 结构 (仅在参数变更时重建)
  Eigen::MatrixXd A_pred_;
  Eigen::MatrixXd B_ctrl_;    // 含延迟的 B_d (或无延迟时为 B_pred)
  Eigen::MatrixXd D_;
  Eigen::MatrixXd Q_blk_;
  Eigen::MatrixXd R_blk_;
  Eigen::MatrixXd S_blk_;
  bool matrices_dirty_{true};

  void rebuildMatrices();

  // MPC 参数
  int N_{20};
  double dt_{0.01};
  double control_delay_s_{0.0};
  double max_accel_{30.0};

  // 权重参数
  double q_yaw_{100.0};
  double q_pitch_{100.0};
  double q_yaw_vel_{10.0};
  double q_pitch_vel_{10.0};
  double r_yaw_{0.01};
  double r_pitch_{0.01};
  double s_yaw_{5.0};
  double s_pitch_{5.0};

  // 角速度差分估计状态
  double prev_yaw_{0.0};
  double prev_pitch_{0.0};
  bool has_prev_state_{false};

  // 延时补偿参数
  bool enable_delay_compensation_{false};
  double prediction_delay_s_{0.0};
  int flight_time_iters_{2};
  double max_processing_delay_s_{0.5};  // processing_delay 上限 (秒)

  // 上一步求解结果 (warmstart)
  Eigen::VectorXd U_prev_;
  // 机动自适应权重衰减参数
  bool enable_maneuver_adapt_{false};
  double a_max_{3.0};             // 差分速度归一化上限 (m/s²)
  double eta_{0.2};               // EMA 平滑系数
  double tau_{10.0};              // Q 衰减时间常数 (步数)
  double r_scale_maneuver_{10.0}; // R 放大系数

  // 机动 alpha EMA 状态
  double alpha_ema_{0.0};

  // 目标速度差分历史（时间戳感知）
  Eigen::Vector3d prev_target_velocity_{Eigen::Vector3d::Zero()};
  rclcpp::Time prev_target_stamp_{0, 0, RCL_ROS_TIME};
  bool has_prev_velocity_{false};
  /**
   * @brief QP 求解失败时回退到直接瞄准参考轨迹首步
   */
  rm_interfaces::msg::GimbalCmd fallbackDirectAim(
    const GimbalControlContext & context,
    const Eigen::VectorXd & X_ref);
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__STRATEGIES__MPC_CONTROL_STRATEGY_HPP_
