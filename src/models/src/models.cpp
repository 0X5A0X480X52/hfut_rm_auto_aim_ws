#include "models/models.h"

#include <cmath>
#include <iostream>

Eigen::MatrixXd kroneckerProduct(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
  Eigen::MatrixXd result(A.rows() * B.rows(), A.cols() * B.cols());
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      result.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
    }
  }
  return result;
}

Models::Models()
: Dim(0)
, Q(Eigen::MatrixXd::Zero(0, 0))
, w_means(Eigen::VectorXd::Zero(0))
, F(Eigen::MatrixXd::Zero(0, 0))
, H(Eigen::MatrixXd::Zero(0, 0))
, R(Eigen::MatrixXd::Zero(0, 0))
, v_means(Eigen::VectorXd::Zero(0))
, X_after(Eigen::VectorXd::Zero(0))
, P_after(Eigen::MatrixXd::Zero(0, 0)) {}

/**
 * @brief 计算 Lambda 值。
 * @param Z 当前测量值。
 * @return Lambda 值。
 */
double Models::getLambda(const Eigen::VectorXd &Z) {
  Eigen::VectorXd r = Z - H.transpose() * X_after;

  std::cout << "H shape: " << H.rows() << " x " << H.cols() << std::endl;
  std::cout << "P_after shape: " << P_after.rows() << " x " << P_after.cols() << std::endl;
  std::cout << "R shape: " << R.rows() << " x " << R.cols() << std::endl;

  Eigen::MatrixXd S = H.transpose() * P_after * H + R;
  double detS = std::abs(S.determinant());
  double Lambda =
    (1 / std::sqrt(2 * M_PI * detS)) * std::exp(-0.5 * r.transpose() * S.inverse() * r);
  return Lambda;
}

/**
 * @brief 预测下一时刻的状态。
 * @return 预测的状态矩阵。
 */
Eigen::MatrixXd Models::predict() const {
    return H.transpose() * F * X_after;
}

/**
 * @brief 预测未来 N 时刻的状态。
 * @param N 未来时刻数。
 * @return 预测的状态矩阵。
 */
Eigen::MatrixXd Models::predict(int N) const {

    if (N == 0)
        return H.transpose() * X_after;

    Eigen::VectorXd X = X_after;
    // std::cout << "Models::predict(int N)" << std::endl;
    for (int i = 0; i < N; ++i) {
        // std::cout << "F.size(): " << F.rows() << "," <<  F.cols() << std::endl;
        // std::cout << "X.size(): " << X.rows() << "," <<  X.cols() << std::endl;
        X = F * X;
    }
    // std::cout << "Models::predict(int N)" << std::endl;
    return H.transpose() * X;
}