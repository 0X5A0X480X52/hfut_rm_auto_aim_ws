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

#ifndef GIMBAL_CONTROLLER__MPC__GIMBAL_DYNAMICS_MODEL_HPP_
#define GIMBAL_CONTROLLER__MPC__GIMBAL_DYNAMICS_MODEL_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace gimbal_controller
{
namespace mpc
{

/**
 * @brief 云台双轴二阶离散动力学模型
 *
 * 状态向量: x = [yaw, pitch, yaw_dot, pitch_dot]^T  (4×1)
 * 控制输入: u = [yaw_ddot, pitch_ddot]^T             (2×1)
 *
 * 离散化状态转移方程:
 *   x[k+1] = A * x[k] + B * u[k]
 *
 * 其中:
 *   A = [[1, 0, dt, 0 ],
 *        [0, 1, 0,  dt],
 *        [0, 0, 1,  0 ],
 *        [0, 0, 0,  1 ]]
 *
 *   B = [[0.5*dt², 0      ],
 *        [0,       0.5*dt²],
 *        [dt,      0      ],
 *        [0,       dt     ]]
 */
class GimbalDynamicsModel
{
public:
  static constexpr int STATE_DIM = 4;
  static constexpr int CONTROL_DIM = 2;

  using StateMatrix = Eigen::Matrix4d;
  using InputMatrix = Eigen::Matrix<double, 4, 2>;
  using StateVector = Eigen::Vector4d;
  using ControlVector = Eigen::Vector2d;

  explicit GimbalDynamicsModel(double dt = 0.01)
  : dt_(dt)
  {
    buildMatrices();
  }

  void setDt(double dt)
  {
    dt_ = dt;
    buildMatrices();
  }

  double dt() const { return dt_; }
  const StateMatrix & A() const { return A_; }
  const InputMatrix & B() const { return B_; }

  /**
   * @brief 单步状态预测
   */
  StateVector predict(const StateVector & x, const ControlVector & u) const
  {
    return A_ * x + B_ * u;
  }

  /**
   * @brief 构建 N 步预测矩阵 (无延迟)
   *
   * X_pred = A_pred * x0 + B_pred * U
   *
   * A_pred: (4N × 4),  各行为 A^1, A^2, ..., A^N
   * B_pred: (4N × 2N), 下三角 block Toeplitz 矩阵
   *
   * @param N 预测步数
   * @param[out] A_pred 预测状态矩阵
   * @param[out] B_pred 预测控制矩阵
   */
  void buildPredictionMatrices(
    int N,
    Eigen::MatrixXd & A_pred,
    Eigen::MatrixXd & B_pred) const
  {
    const int nx = STATE_DIM;
    const int nu = CONTROL_DIM;

    A_pred.resize(nx * N, nx);
    B_pred.resize(nx * N, nu * N);
    A_pred.setZero();
    B_pred.setZero();

    // A_powers[i] = A^(i+1)
    StateMatrix A_power = A_;
    for (int i = 0; i < N; ++i) {
      A_pred.block(i * nx, 0, nx, nx) = A_power;

      // B_pred row-block i, col-block j: A^(i-j) * B  for j <= i
      for (int j = 0; j <= i; ++j) {
        int exp = i - j;  // exponent: A^exp * B
        if (exp == 0) {
          B_pred.block(i * nx, j * nu, nx, nu) = B_;
        } else {
          // Compute A^exp — 使用递推避免重复乘法
          StateMatrix A_exp = StateMatrix::Identity();
          for (int p = 0; p < exp; ++p) {
            A_exp = A_exp * A_;
          }
          B_pred.block(i * nx, j * nu, nx, nu) = A_exp * B_;
        }
      }

      A_power = A_power * A_;
    }
  }

  /**
   * @brief 构建延迟映射的控制矩阵 B_d
   *
   * 考虑控制延迟 tau 秒 (d = round(tau/dt) 步):
   *   u_exec[t] = u[t - d]
   *
   * B_d 的构建: 前 d 步控制输入尚未生效，对应列块为 0
   * 第 d+1 步开始生效: B_d(i, j) = A^(i-j-d) * B  当 j+d <= i 时
   *
   * @param N 预测步数
   * @param delay_s 延迟时间 (秒)
   * @return B_d (4N × 2N)
   */
  Eigen::MatrixXd buildDelayedB(int N, double delay_s) const
  {
    const int nx = STATE_DIM;
    const int nu = CONTROL_DIM;
    const int d = static_cast<int>(std::round(delay_s / dt_));

    Eigen::MatrixXd B_d(nx * N, nu * N);
    B_d.setZero();

    // 预计算 A^k * B, k = 0, 1, ..., N-1
    std::vector<Eigen::Matrix<double, 4, 2>> AkB(N);
    AkB[0] = B_;
    StateMatrix A_power = A_;
    for (int k = 1; k < N; ++k) {
      AkB[k] = A_power * B_;
      A_power = A_power * A_;
    }

    for (int i = 0; i < N; ++i) {
      for (int j = 0; j <= i; ++j) {
        int effective_step = i - j;
        // 控制 u[j] 经过 d 步延迟后才生效
        // 等价于: u[j] 在时刻 j+d 开始影响状态
        // 对行 i: 需要 j + d <= i, 即 effective_step >= d
        if (effective_step >= d) {
          int exp = effective_step - d;
          B_d.block(i * nx, j * nu, nx, nu) = AkB[exp];
        }
      }
    }

    return B_d;
  }

  /**
   * @brief 构建双轴 block-diagonal 差分矩阵 D
   *
   * D * U = [u[0] - u_prev, u[1] - u[0], ..., u[N-1] - u[N-2]]
   * (此处假设 u_prev = 0，首行为 u[0] 自身)
   *
   * 对于双轴控制 (nu=2):
   *   D 是一个 (2N × 2N) block-diagonal 矩阵，
   *   每个 2×2 block 对应 [yaw_ddot, pitch_ddot] 的差分
   *
   * @param N 预测步数
   * @return D (2N × 2N)
   */
  static Eigen::MatrixXd buildDifferenceMatrix(int N)
  {
    const int nu = CONTROL_DIM;
    Eigen::MatrixXd D(nu * N, nu * N);
    D.setZero();

    // 第一行块: I (u[0] - 0)
    D.block(0, 0, nu, nu) = Eigen::Matrix2d::Identity();

    // 后续行块: u[k] - u[k-1]
    for (int k = 1; k < N; ++k) {
      D.block(k * nu, k * nu, nu, nu) = Eigen::Matrix2d::Identity();
      D.block(k * nu, (k - 1) * nu, nu, nu) = -Eigen::Matrix2d::Identity();
    }

    return D;
  }

  /**
   * @brief 构建 QP 的 Hessian 和线性项
   *
   * J = (X - X_ref)^T Q_blk (X - X_ref) + U^T R_blk U + (D*U)^T S_blk (D*U)
   *   其中 X = A_pred * x0 + B_ctrl * U
   *
   * H = 2 * (B_ctrl^T Q_blk B_ctrl + R_blk + D^T S_blk D)
   * f = 2 * B_ctrl^T Q_blk (A_pred * x0 - X_ref_flat)
   *
   * @param A_pred (4N × 4) 预测状态传播矩阵
   * @param B_ctrl (4N × 2N) 控制矩阵 (可以是 B_pred 或 B_d)
   * @param D      (2N × 2N) 差分矩阵
   * @param Q_blk  (4N × 4N) block-diagonal 跟踪权重
   * @param R_blk  (2N × 2N) diagonal 控制权重
   * @param S_blk  (2N × 2N) diagonal 平滑权重
   * @param x0     (4 × 1) 当前云台状态
   * @param X_ref  (4N × 1) 参考轨迹 (flatten)
   * @param[out] H (2N × 2N) Hessian
   * @param[out] f (2N × 1) 线性项
   */
  static void buildQP(
    const Eigen::MatrixXd & A_pred,
    const Eigen::MatrixXd & B_ctrl,
    const Eigen::MatrixXd & D,
    const Eigen::MatrixXd & Q_blk,
    const Eigen::MatrixXd & R_blk,
    const Eigen::MatrixXd & S_blk,
    const StateVector & x0,
    const Eigen::VectorXd & X_ref,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & f)
  {
    // H = 2 * (B^T Q B + R + D^T S D)
    H = 2.0 * (B_ctrl.transpose() * Q_blk * B_ctrl + R_blk + D.transpose() * S_blk * D);

    // 确保对称性 (数值精度)
    H = 0.5 * (H + H.transpose());

    // f = 2 * B^T Q (A_pred * x0 - X_ref)
    Eigen::VectorXd error = A_pred * x0 - X_ref;
    f = 2.0 * B_ctrl.transpose() * Q_blk * error;
  }

  /**
   * @brief 构建 block-diagonal 权重矩阵 Q
   * @param N 预测步数
   * @param q_yaw yaw 位置误差权重
   * @param q_pitch pitch 位置误差权重
   * @param q_yaw_vel yaw 速度误差权重
   * @param q_pitch_vel pitch 速度误差权重
   * @return Q (4N × 4N)
   */
  static Eigen::MatrixXd buildWeightQ(
    int N, double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel)
  {
    const int nx = STATE_DIM;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
    Eigen::Vector4d diag_q(q_yaw, q_pitch, q_yaw_vel, q_pitch_vel);
    for (int k = 0; k < N; ++k) {
      Q.block(k * nx, k * nx, nx, nx) = diag_q.asDiagonal();
    }
    return Q;
  }

  /**
   * @brief 构建 diagonal 控制权重矩阵 R
   */
  static Eigen::MatrixXd buildWeightR(int N, double r_yaw, double r_pitch)
  {
    const int nu = CONTROL_DIM;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu * N, nu * N);
    Eigen::Vector2d diag_r(r_yaw, r_pitch);
    for (int k = 0; k < N; ++k) {
      R.block(k * nu, k * nu, nu, nu) = diag_r.asDiagonal();
    }
    return R;
  }

  /**
   * @brief 构建 diagonal 平滑权重矩阵 S
   */
  static Eigen::MatrixXd buildWeightS(int N, double s_yaw, double s_pitch)
  {
    return buildWeightR(N, s_yaw, s_pitch);  // same structure
  }

private:
  void buildMatrices()
  {
    double dt = dt_;
    double dt2 = 0.5 * dt * dt;

    A_ << 1.0, 0.0, dt,  0.0,
          0.0, 1.0, 0.0, dt,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    B_ << dt2, 0.0,
          0.0, dt2,
          dt,  0.0,
          0.0, dt;
  }

  double dt_;
  StateMatrix A_;
  InputMatrix B_;
};

}  // namespace mpc
}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__MPC__GIMBAL_DYNAMICS_MODEL_HPP_
