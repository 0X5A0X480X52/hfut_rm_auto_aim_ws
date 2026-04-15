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

#ifndef GIMBAL_CONTROLLER__MPC__QP_SOLVER_HPP_
#define GIMBAL_CONTROLLER__MPC__QP_SOLVER_HPP_

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <algorithm>
#include <memory>
#include <vector>

namespace gimbal_controller
{
namespace mpc
{

/**
 * @brief QP 求解结果
 */
struct QPResult
{
  Eigen::VectorXd U;          // 最优控制序列
  bool success{false};        // 是否成功
  int num_iterations{0};      // 迭代次数
  double cost{0.0};           // 目标函数值
  int active_bound_size{0};   // 近似活跃 box 约束数量
  int active_linear_size{0};  // 近似活跃线性约束数量
  int active_set_size{0};     // 近似活跃约束总数
};

/**
 * @brief qpOASES 封装
 *
 * 求解标准 QP:
 *   min  0.5 * U^T H U + f^T U
 *   s.t. lb  <= U       <= ub
 *        lbA <= A_con*U <= ubA
 *
 * 支持热启动 (hotstart): 首次调用 init(), 后续调用 hotstart()
 */
class QPSolver
{
public:
  QPSolver() = default;
  ~QPSolver() = default;

  /**
   * @brief 初始化求解器参数
   * @param max_iter 最大迭代次数
   * @param cpu_time_limit 最大CPU时间 (秒), 0 表示无限制
   */
  void init(int max_iter = 200, double cpu_time_limit = 0.005)
  {
    max_iter_ = max_iter;
    cpu_time_limit_ = cpu_time_limit;
  }

  /**
   * @brief 求解 QP 问题
   *
   * @param H Hessian 矩阵 (n_vars × n_vars), 必须对称正定
   * @param f 线性项 (n_vars × 1)
   * @param lb 变量下界 (n_vars × 1)
   * @param ub 变量上界 (n_vars × 1)
   * @param A_con 约束矩阵 (n_constraints × n_vars), 可为空
   * @param lbA 约束下界 (n_constraints × 1), 可为空
   * @param ubA 约束上界 (n_constraints × 1), 可为空
   * @param U_warm 热启动初值 (n_vars × 1), 可为空
   * @return QPResult
   */
  QPResult solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub,
    const Eigen::MatrixXd & A_con = Eigen::MatrixXd(),
    const Eigen::VectorXd & lbA = Eigen::VectorXd(),
    const Eigen::VectorXd & ubA = Eigen::VectorXd(),
    const Eigen::VectorXd & U_warm = Eigen::VectorXd())
  {
    QPResult result;
    const int n_vars = static_cast<int>(H.rows());
    const int n_constraints = static_cast<int>(A_con.rows());

    // qpOASES 使用 row-major 格式
    // Eigen 默认 col-major，需要转换
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_row = H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_row;
    if (n_constraints > 0) {
      A_row = A_con;
    }

    bool size_changed = (n_vars != prev_n_vars_ || n_constraints != prev_n_constraints_);
    bool matrix_changed = false;

    if (!size_changed && qp_initialized_) {
      matrix_changed = hasMatrixChanged(H_row, A_row, n_constraints);
    }

    if (size_changed || !qp_initialized_ || matrix_changed) {
      // 首次调用或问题尺寸变化: 重新创建 QProblem
      if (n_constraints > 0) {
        qp_ = std::make_unique<qpOASES::QProblem>(n_vars, n_constraints);
      } else {
        qp_bound_only_ = std::make_unique<qpOASES::QProblemB>(n_vars);
      }
      qp_initialized_ = false;
      prev_n_vars_ = n_vars;
      prev_n_constraints_ = n_constraints;
    }

    qpOASES::int_t nWSR = max_iter_;
    qpOASES::real_t cpu_time = cpu_time_limit_;
    qpOASES::real_t * cpu_time_ptr = (cpu_time_limit_ > 0) ? &cpu_time : nullptr;

    qpOASES::returnValue ret;

    if (n_constraints > 0) {
      // 设置选项
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;
      qp_->setOptions(options);

      if (!qp_initialized_) {
        ret = qp_->init(
          H_row.data(), f.data(),
          A_row.data(),
          lb.data(), ub.data(),
          lbA.data(), ubA.data(),
          nWSR, cpu_time_ptr);

        if (ret == qpOASES::SUCCESSFUL_RETURN) {
          qp_initialized_ = true;
          cacheMatrices(H_row, A_row, n_constraints);
        }
      } else {
        // 热启动: hotstart 只接受 gradient + bounds 更新
        ret = qp_->hotstart(
          f.data(),
          lb.data(), ub.data(),
          lbA.data(), ubA.data(),
          nWSR, cpu_time_ptr);

        if (ret != qpOASES::SUCCESSFUL_RETURN) {
          // hotstart 失败，重新 init
          qp_ = std::make_unique<qpOASES::QProblem>(n_vars, n_constraints);
          qp_->setOptions(options);
          qp_initialized_ = false;
          nWSR = max_iter_;
          cpu_time = cpu_time_limit_;
          cpu_time_ptr = (cpu_time_limit_ > 0) ? &cpu_time : nullptr;
          ret = qp_->init(
            H_row.data(), f.data(),
            A_row.data(),
            lb.data(), ub.data(),
            lbA.data(), ubA.data(),
            nWSR, cpu_time_ptr);
          if (ret == qpOASES::SUCCESSFUL_RETURN) {
            qp_initialized_ = true;
            cacheMatrices(H_row, A_row, n_constraints);
          }
        }
      }

      if (ret == qpOASES::SUCCESSFUL_RETURN) {
        result.U.resize(n_vars);
        qp_->getPrimalSolution(result.U.data());
        result.success = true;
        result.num_iterations = static_cast<int>(nWSR);
        result.cost = qp_->getObjVal();
        // hotstart 路径矩阵未变，无需更新缓存；init 路径已更新。
      }
    } else {
      // 仅 box 约束
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;
      qp_bound_only_->setOptions(options);

      if (!qp_initialized_) {
        ret = qp_bound_only_->init(
          H_row.data(), f.data(),
          lb.data(), ub.data(),
          nWSR, cpu_time_ptr);

        if (ret == qpOASES::SUCCESSFUL_RETURN) {
          qp_initialized_ = true;
          cacheMatrices(H_row, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(), n_constraints);
        }
      } else {
        ret = qp_bound_only_->hotstart(
          f.data(),
          lb.data(), ub.data(),
          nWSR, cpu_time_ptr);

        if (ret != qpOASES::SUCCESSFUL_RETURN) {
          qp_bound_only_ = std::make_unique<qpOASES::QProblemB>(n_vars);
          qp_bound_only_->setOptions(options);
          qp_initialized_ = false;
          nWSR = max_iter_;
          cpu_time = cpu_time_limit_;
          cpu_time_ptr = (cpu_time_limit_ > 0) ? &cpu_time : nullptr;
          ret = qp_bound_only_->init(
            H_row.data(), f.data(),
            lb.data(), ub.data(),
            nWSR, cpu_time_ptr);
          if (ret == qpOASES::SUCCESSFUL_RETURN) {
            qp_initialized_ = true;
            cacheMatrices(H_row, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(), n_constraints);
          }
        }
      }

      if (ret == qpOASES::SUCCESSFUL_RETURN) {
        result.U.resize(n_vars);
        qp_bound_only_->getPrimalSolution(result.U.data());
        result.success = true;
        result.num_iterations = static_cast<int>(nWSR);
        result.cost = qp_bound_only_->getObjVal();
        // hotstart 路径矩阵未变，无需更新缓存；init 路径已更新。
      }
    }

    return result;
  }

  /**
   * @brief 重置求解器状态 (下次调用将重新 init)
   */
  void reset()
  {
    qp_initialized_ = false;
    qp_.reset();
    qp_bound_only_.reset();
    prev_H_.resize(0, 0);
    prev_A_.resize(0, 0);
  }

private:
  using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  bool hasMatrixChanged(
    const RowMajorMatrix & H,
    const RowMajorMatrix & A,
    int n_constraints) const
  {
    if (prev_H_.rows() != H.rows() || prev_H_.cols() != H.cols()) {
      return true;
    }

    const double h_scale = std::max(1.0, H.cwiseAbs().maxCoeff());
    const double h_diff = (prev_H_ - H).cwiseAbs().maxCoeff();
    if (h_diff > matrix_compare_abs_tol_ + matrix_compare_rel_tol_ * h_scale) {
      return true;
    }

    if (n_constraints <= 0) {
      return false;
    }

    if (prev_A_.rows() != A.rows() || prev_A_.cols() != A.cols()) {
      return true;
    }

    const double a_scale = std::max(1.0, A.cwiseAbs().maxCoeff());
    const double a_diff = (prev_A_ - A).cwiseAbs().maxCoeff();
    return a_diff > matrix_compare_abs_tol_ + matrix_compare_rel_tol_ * a_scale;
  }

  void cacheMatrices(
    const RowMajorMatrix & H,
    const RowMajorMatrix & A,
    int n_constraints)
  {
    prev_H_ = H;
    if (n_constraints > 0) {
      prev_A_ = A;
    } else {
      prev_A_.resize(0, 0);
    }
  }

  std::unique_ptr<qpOASES::QProblem> qp_;
  std::unique_ptr<qpOASES::QProblemB> qp_bound_only_;
  bool qp_initialized_{false};
  int prev_n_vars_{0};
  int prev_n_constraints_{0};
  int max_iter_{200};
  double cpu_time_limit_{0.005};
  RowMajorMatrix prev_H_;
  RowMajorMatrix prev_A_;
  double matrix_compare_abs_tol_{1e-12};
  double matrix_compare_rel_tol_{1e-9};
};

}  // namespace mpc
}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__MPC__QP_SOLVER_HPP_
