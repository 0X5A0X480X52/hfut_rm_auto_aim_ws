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
#include <algorithm>
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

  /**
   * @brief 计算状态 RMS 的安全倒平方缩放系数
   *
   * scale = 1 / max(rms_i^2, rms_epsilon)
   */
  static Eigen::Vector4d computeStateInvRms2Scale(
    const Eigen::Vector4d & state_rms, double rms_epsilon)
  {
    const double eps = std::max(rms_epsilon, 1e-12);
    Eigen::Vector4d scale;
    for (int i = 0; i < STATE_DIM; ++i) {
      const double v = std::abs(state_rms(i));
      const double denom = std::max(v * v, eps);
      scale(i) = 1.0 / denom;
    }
    return scale;
  }

  /**
   * @brief 计算控制 RMS 的安全倒平方缩放系数
   *
   * scale = 1 / max(rms_i^2, rms_epsilon)
   */
  static Eigen::Vector2d computeControlInvRms2Scale(
    const Eigen::Vector2d & control_rms, double rms_epsilon)
  {
    const double eps = std::max(rms_epsilon, 1e-12);
    Eigen::Vector2d scale;
    for (int i = 0; i < CONTROL_DIM; ++i) {
      const double v = std::abs(control_rms(i));
      const double denom = std::max(v * v, eps);
      scale(i) = 1.0 / denom;
    }
    return scale;
  }

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
    return buildWeightQ(
      N, q_yaw, q_pitch, q_yaw_vel, q_pitch_vel,
      Eigen::Vector4d::Ones(), 1e-12);
  }

  /**
   * @brief 构建带 RMS 归一化的 block-diagonal 权重矩阵 Q
   *
   * Q_diag = diag(q) .* (1 / rms(state)^2)
   */
  static Eigen::MatrixXd buildWeightQ(
    int N, double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel,
    const Eigen::Vector4d & state_rms, double rms_epsilon)
  {
    const int nx = STATE_DIM;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
    Eigen::Vector4d diag_q(q_yaw, q_pitch, q_yaw_vel, q_pitch_vel);
    const Eigen::Vector4d inv_rms2 = computeStateInvRms2Scale(state_rms, rms_epsilon);
    const Eigen::Vector4d diag_q_norm = diag_q.cwiseProduct(inv_rms2);
    for (int k = 0; k < N; ++k) {
      Q.block(k * nx, k * nx, nx, nx) = diag_q_norm.asDiagonal();
    }
    return Q;
  }

  /**
   * @brief 对 block-diagonal Q 的每步子块应用缩放因子
   * @param Q  (4N × 4N) block-diagonal 权重矩阵
   * @param w  (N) 每步缩放因子
   */
  static void scaleBlockDiagonalQ(Eigen::MatrixXd & Q, const Eigen::VectorXd & w)
  {
    const int nx = STATE_DIM;
    const int N = static_cast<int>(w.size());
    for (int k = 0; k < N; ++k) {
      Q.block(k * nx, k * nx, nx, nx) *= w(k);
    }
  }

  /**
   * @brief 构建 diagonal 控制权重矩阵 R
   */
  static Eigen::MatrixXd buildWeightR(int N, double r_yaw, double r_pitch)
  {
    return buildWeightR(N, r_yaw, r_pitch, Eigen::Vector2d::Ones(), 1e-12);
  }

  /**
   * @brief 构建带 RMS 归一化的 diagonal 控制权重矩阵 R
   *
   * R_diag = diag(r) .* (1 / rms(control)^2)
   */
  static Eigen::MatrixXd buildWeightR(
    int N, double r_yaw, double r_pitch,
    const Eigen::Vector2d & control_rms, double rms_epsilon)
  {
    const int nu = CONTROL_DIM;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu * N, nu * N);
    Eigen::Vector2d diag_r(r_yaw, r_pitch);
    const Eigen::Vector2d inv_rms2 = computeControlInvRms2Scale(control_rms, rms_epsilon);
    const Eigen::Vector2d diag_r_norm = diag_r.cwiseProduct(inv_rms2);
    for (int k = 0; k < N; ++k) {
      R.block(k * nu, k * nu, nu, nu) = diag_r_norm.asDiagonal();
    }
    return R;
  }

  /**
   * @brief 构建 diagonal 平滑权重矩阵 S
   */
  static Eigen::MatrixXd buildWeightS(int N, double s_yaw, double s_pitch)
  {
    return buildWeightS(N, s_yaw, s_pitch, Eigen::Vector2d::Ones(), 1e-12);
  }

  /**
   * @brief 构建带 RMS 归一化的 diagonal 平滑权重矩阵 S
   *
   * S_diag = diag(s) .* (1 / rms(delta_u)^2)
   */
  static Eigen::MatrixXd buildWeightS(
    int N, double s_yaw, double s_pitch,
    const Eigen::Vector2d & delta_u_rms, double rms_epsilon)
  {
    return buildWeightR(N, s_yaw, s_pitch, delta_u_rms, rms_epsilon);  // same structure
  }

  /**
   * @brief 构建机动自适应的 block-diagonal 跟踪权重矩阵 Q_eff
   *
   * 每步 k 的权重按机动因子 alpha 衰减远期权重：
   *   Q_k = diag(q) * (1 - alpha * (1 - exp(-k / tau)))
   *
   * - k=0: Q_0 = diag(q)（首步无衰减，保证近期精度）
   * - k→∞: Q_k → diag(q) * (1 - alpha)（远期权重降低，减少超调追踪）
   * - alpha=0: Q_k = diag(q)（退化为标准 buildWeightQ，无自适应）
   *
   * @param N           预测步数
   * @param q_yaw       yaw 位置误差权重
   * @param q_pitch     pitch 位置误差权重
   * @param q_yaw_vel   yaw 速度误差权重
   * @param q_pitch_vel pitch 速度误差权重
   * @param alpha       机动因子 [0, 1]，由速度差分计算后 EMA 平滑得到
   * @param tau         衰减时间常数（步数尺度，>0），控制远期衰减速度
   * @return Q_eff (4N × 4N)
   */
  static Eigen::MatrixXd buildAdaptiveWeightQ(
    int N, double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel,
    double alpha, double tau)
  {
    return buildAdaptiveWeightQ(
      N, q_yaw, q_pitch, q_yaw_vel, q_pitch_vel,
      alpha, tau, Eigen::Vector4d::Ones(), 1e-12);
  }

  /**
   * @brief 构建带 RMS 归一化的机动自适应 block-diagonal 跟踪权重矩阵 Q_eff
   */
  static Eigen::MatrixXd buildAdaptiveWeightQ(
    int N, double q_yaw, double q_pitch, double q_yaw_vel, double q_pitch_vel,
    double alpha, double tau,
    const Eigen::Vector4d & state_rms, double rms_epsilon)
  {
    const int nx = STATE_DIM;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
    Eigen::Vector4d diag_q(q_yaw, q_pitch, q_yaw_vel, q_pitch_vel);
    const Eigen::Vector4d inv_rms2 = computeStateInvRms2Scale(state_rms, rms_epsilon);
    const Eigen::Vector4d diag_q_norm = diag_q.cwiseProduct(inv_rms2);
    for (int k = 0; k < N; ++k) {
      double decay = 1.0 - std::exp(-static_cast<double>(k) / tau);
      double scale = 1.0 - alpha * decay;
      Q.block(k * nx, k * nx, nx, nx) = (scale * diag_q_norm).asDiagonal();
    }
    return Q;
  }

  /**
   * @brief 构建机动自适应的 diagonal 控制权重矩阵 R_eff
   *
   * 全步统一放大：R_eff = R * (1 + alpha * r_scale)
   *
   * - alpha=0: R_eff = R（退化为标准 buildWeightR，无自适应）
   * - alpha=1: R_eff = R * (1 + r_scale)（最大控制抑制）
   *
   * @param N       预测步数
   * @param r_yaw   yaw 控制权重（基准值）
   * @param r_pitch pitch 控制权重（基准值）
   * @param alpha   机动因子 [0, 1]
   * @param r_scale R 放大系数（>=0）
   * @return R_eff (2N × 2N)
   */
  static Eigen::MatrixXd buildAdaptiveWeightR(
    int N, double r_yaw, double r_pitch, double alpha, double r_scale)
  {
    return buildAdaptiveWeightR(
      N, r_yaw, r_pitch, alpha, r_scale,
      Eigen::Vector2d::Ones(), 1e-12);
  }

  /**
   * @brief 构建带 RMS 归一化的机动自适应 diagonal 控制权重矩阵 R_eff
   */
  static Eigen::MatrixXd buildAdaptiveWeightR(
    int N, double r_yaw, double r_pitch, double alpha, double r_scale,
    const Eigen::Vector2d & control_rms, double rms_epsilon)
  {
    double scale = 1.0 + alpha * r_scale;
    return buildWeightR(N, r_yaw * scale, r_pitch * scale, control_rms, rms_epsilon);
  }

  /**
   * @brief 构建 FOV 软约束扩展 QP
   *
   * 将原始 QP (决策变量 U ∈ R^{2N}) 扩展为带 slack 变量的 QP
   * (决策变量 Z = [U; s] ∈ R^{2N + 2K}, K 为约束步数):
   *
   *   min  0.5 Z^T H_ext Z + f_ext^T Z
   *   s.t. lb_ext <= Z <= ub_ext       (box 约束)
   *        lbA    <= A_con Z <= ubA    (FOV 线性约束)
   *
   * FOV 约束: 对每个约束步 k (k = 0..K-1):
   *   e_yaw_k   = Sel_yaw_k (A_pred x0 + B_ctrl U) - X_ref_yaw_k
   *   e_pitch_k = Sel_pitch_k (A_pred x0 + B_ctrl U) - X_ref_pitch_k
   *
   *   e_yaw_k   - s_yaw_k   <= fov_yaw - margin     (上界)
   *   -e_yaw_k  - s_yaw_k   <= fov_yaw - margin     (下界, 等价 e >= -(fov-margin)+s)
   *   e_pitch_k - s_pitch_k <= fov_pitch - margin
   *   -e_pitch_k - s_pitch_k <= fov_pitch - margin
   *
   * s >= 0, slack 惩罚: w_slack * sum(s_k^2)
   *
   * @param H_orig         原始 Hessian (2N × 2N)
   * @param f_orig         原始梯度 (2N)
   * @param A_pred         预测状态矩阵 (4N × 4)
   * @param B_ctrl         控制矩阵 (4N × 2N)
   * @param x0             当前状态 (4)
   * @param X_ref          参考轨迹 (4N)
   * @param N              总预测步数
   * @param K              约束步数 (K <= N, 仅对前 K 步施加 FOV 约束)
   * @param fov_half_yaw   camera yaw 视场半角 (rad)
   * @param fov_half_pitch camera pitch 视场半角 (rad)
   * @param margin         安全裕度 (rad)
   * @param slack_weight   slack 惩罚权重
   * @param[out] H_ext     扩展 Hessian (2N+2K × 2N+2K)
   * @param[out] f_ext     扩展梯度 (2N+2K)
   * @param[out] A_con     约束矩阵 (4K × 2N+2K)
   * @param[out] lbA       约束下界 (4K)
   * @param[out] ubA       约束上界 (4K)
   */
  static void buildFovSoftConstraintQP(
    const Eigen::MatrixXd & H_orig,
    const Eigen::VectorXd & f_orig,
    const Eigen::MatrixXd & A_pred,
    const Eigen::MatrixXd & B_ctrl,
    const StateVector & x0,
    const Eigen::VectorXd & X_ref,
    int N, int K,
    double fov_half_yaw,
    double fov_half_pitch,
    double margin,
    double slack_weight,
    Eigen::MatrixXd & H_ext,
    Eigen::VectorXd & f_ext,
    Eigen::MatrixXd & A_con,
    Eigen::VectorXd & lbA,
    Eigen::VectorXd & ubA)
  {
    const int nx = STATE_DIM;
    const int nu = CONTROL_DIM;
    const int n_u = nu * N;        // 原始控制变量维度
    const int n_s = nu * K;        // slack 变量维度 (2 per step: yaw + pitch)
    const int n_z = n_u + n_s;     // 扩展决策变量 Z = [U; s]
    const int n_con = 2 * nu * K;  // 约束数: 4K (K步 × 2轴 × 上下界)

    // ── 1. 扩展 Hessian: [[H, 0]; [0, 2*w*I]] ──
    H_ext = Eigen::MatrixXd::Zero(n_z, n_z);
    H_ext.topLeftCorner(n_u, n_u) = H_orig;
    for (int i = 0; i < n_s; ++i) {
      H_ext(n_u + i, n_u + i) = 2.0 * slack_weight;
    }

    // ── 2. 扩展梯度: [f; 0] ──
    f_ext = Eigen::VectorXd::Zero(n_z);
    f_ext.head(n_u) = f_orig;

    // ── 3. 预计算预测误差的自由响应部分 ──
    // X_free = A_pred * x0, X_forced = B_ctrl * U
    // e_k = (X_free + X_forced)_k - X_ref_k (分 yaw/pitch)
    Eigen::VectorXd X_free = A_pred * x0;

    // ── 4. 构建约束矩阵 A_con 和边界 ──
    // 约束排列: [yaw_upper_0, yaw_lower_0, pitch_upper_0, pitch_lower_0,
    //            yaw_upper_1, yaw_lower_1, pitch_upper_1, pitch_lower_1, ...]
    A_con = Eigen::MatrixXd::Zero(n_con, n_z);
    ubA = Eigen::VectorXd::Zero(n_con);
    lbA = Eigen::VectorXd::Constant(n_con, -1e20);  // 单侧约束

    double bound_yaw = fov_half_yaw - margin;
    double bound_pitch = fov_half_pitch - margin;
    if (bound_yaw < 0.0) bound_yaw = 0.0;
    if (bound_pitch < 0.0) bound_pitch = 0.0;

    for (int k = 0; k < K; ++k) {
      // 从 B_ctrl 提取第 k 步的 yaw 行 (行索引 k*nx + 0) 和 pitch 行 (行索引 k*nx + 1)
      Eigen::RowVectorXd B_yaw_k   = B_ctrl.row(k * nx + 0);   // (1 × 2N)
      Eigen::RowVectorXd B_pitch_k = B_ctrl.row(k * nx + 1);   // (1 × 2N)

      // 自由响应分量
      double free_yaw_k   = X_free(k * nx + 0) - X_ref(k * nx + 0);
      double free_pitch_k = X_free(k * nx + 1) - X_ref(k * nx + 1);

      int row_base = k * 4;  // 每步 4 个约束
      int s_yaw_idx   = n_u + k * nu + 0;  // slack yaw 在 Z 中的索引
      int s_pitch_idx = n_u + k * nu + 1;  // slack pitch 在 Z 中的索引

      // yaw 上界: B_yaw_k * U - s_yaw_k <= bound_yaw - free_yaw_k
      A_con.block(row_base + 0, 0, 1, n_u) = B_yaw_k;
      A_con(row_base + 0, s_yaw_idx) = -1.0;
      ubA(row_base + 0) = bound_yaw - free_yaw_k;

      // yaw 下界: -B_yaw_k * U - s_yaw_k <= bound_yaw + free_yaw_k
      A_con.block(row_base + 1, 0, 1, n_u) = -B_yaw_k;
      A_con(row_base + 1, s_yaw_idx) = -1.0;
      ubA(row_base + 1) = bound_yaw + free_yaw_k;

      // pitch 上界: B_pitch_k * U - s_pitch_k <= bound_pitch - free_pitch_k
      A_con.block(row_base + 2, 0, 1, n_u) = B_pitch_k;
      A_con(row_base + 2, s_pitch_idx) = -1.0;
      ubA(row_base + 2) = bound_pitch - free_pitch_k;

      // pitch 下界: -B_pitch_k * U - s_pitch_k <= bound_pitch + free_pitch_k
      A_con.block(row_base + 3, 0, 1, n_u) = -B_pitch_k;
      A_con(row_base + 3, s_pitch_idx) = -1.0;
      ubA(row_base + 3) = bound_pitch + free_pitch_k;
    }
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
