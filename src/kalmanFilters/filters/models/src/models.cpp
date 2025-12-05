#include "models/models.h"

#include <cmath>
#include <iostream>
#include <cstdlib>
#include <sstream>

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
, P_after(Eigen::MatrixXd::Zero(0, 0))
{
  // 初始化 debug 标志
  const char* env = std::getenv("MODELS_DEBUG");
  if (env && std::string(env) != "0") {
    debug_mode = true;
  } else {
    debug_mode = false;
  }
  debug_printed_performUpdate = false;
  debug_printed_getLambda = false;
  debug_printed_predict = false;
}

/**
 * @brief 计算 Lambda 值。
 * @param Z 当前测量值。
 * @return Lambda 值。
 */
double Models::getLambda(const Eigen::VectorXd &Z) {
  // Defensive shape checks: ensure H, P_after, X_after, R have compatible shapes.
  // If shapes are transposed compared to expectation, try to auto-correct by using transpose,
  // and print helpful diagnostic information.
  Eigen::MatrixXd Hm = H;
  Eigen::MatrixXd Pm = P_after;
  Eigen::MatrixXd Rm = R;
  Eigen::VectorXd Xm = X_after;

  auto shapes_str = [&]() {
    std::ostringstream oss;
    oss << " shapes: H(" << Hm.rows() << "," << Hm.cols() << ")"
        << " P_after(" << Pm.rows() << "," << Pm.cols() << ")"
        << " X_after(" << Xm.rows() << ")"
        << " R(" << Rm.rows() << "," << Rm.cols() << ")\n";
    return oss.str();
  };

  auto cond_print = [&](const std::string &s, bool &flag) {
    if (debug_mode || !flag) {
      std::cerr << s;
      if (!debug_mode) flag = true;
    }
  };

  // Ensure X_after is a column vector
  if (Xm.cols() != 1 && Xm.rows() == 1) {
    Xm = Xm.transpose();
  }

  // H * X_after: require H.cols() == X_after.rows()
  if (Hm.cols() != Xm.rows()) {
    // try transpose of H
    if (Hm.rows() == Xm.rows() && Hm.cols() != Xm.rows()) {
      cond_print(std::string("Warning: H appears transposed relative to X_after; using H.transpose() instead.\n"), debug_printed_getLambda);
      Hm = Hm.transpose();
    } else {
      // last attempt: if X_after looks transposed, try transposing X_after
      if (Xm.cols() == 1 && Xm.rows() != Hm.cols() && Xm.rows() == Hm.rows()) {
        cond_print(std::string("Warning: X_after appears to be row-oriented; transposing X_after.\n"), debug_printed_getLambda);
        Xm = Xm.transpose();
      }
    }
  }

  // After potential fixes, if still incompatible, print debug and throw
  if (Hm.cols() != Xm.rows()) {
    cond_print(std::string("Incompatible for H * X_after") + shapes_str(), debug_printed_getLambda);
    throw std::runtime_error("Incompatible shapes for H * X_after in getLambda()");
  }

  Eigen::VectorXd r = Z - Hm * Xm;

  // Now check P_after compatibility: H * P_after * H.transpose()
  if (Hm.cols() != Pm.rows()) {
    // try transpose of Pm
    if (Pm.cols() == Hm.cols() && Pm.rows() == Hm.cols()) {
      // unlikely, but try
      Pm = Pm.transpose();
      cond_print(std::string("Warning: P_after appears transposed; using P_after.transpose() in getLambda().\n"), debug_printed_getLambda);
    }
  }

  if (Hm.cols() != Pm.rows()) {
    cond_print(std::string("Incompatible for H * P_after") + shapes_str(), debug_printed_getLambda);
    throw std::runtime_error("Incompatible shapes for H * P_after in getLambda()");
  }

  if (Hm.rows() != Rm.rows() || Hm.rows() != Rm.cols()) {
    cond_print((std::string("Warning: R dimension does not match H * P * H^T; R(") + std::to_string(Rm.rows()) + "," + std::to_string(Rm.cols()) + ") expected (" + std::to_string(Hm.rows()) + "," + std::to_string(Hm.rows()) + "). Will try to resize or adapt.\n"), debug_printed_getLambda);
    // If R is a scalar (1x1), expand it
    if (Rm.rows() == 1 && Rm.cols() == 1) {
      Eigen::MatrixXd Rnew = Eigen::MatrixXd::Identity(Hm.rows(), Hm.rows()) * Rm(0, 0);
      Rm = Rnew;
      cond_print(std::string("Info: expanded scalar R to match measurement dimension.\n"), debug_printed_getLambda);
    }
  }

  Eigen::MatrixXd S = Hm * Pm * Hm.transpose() + Rm;
  double detS = std::abs(S.determinant());
  if (detS <= 0) {
    detS = 1e-12; // avoid division by zero / negative det
  }
  double exponent = -0.5 * (r.transpose() * S.inverse() * r)(0, 0);
  double coeff = 1.0 / std::sqrt(std::pow(2 * M_PI, S.rows()) * detS);
  double Lambda = coeff * std::exp(exponent);
  return Lambda;
}

void Models::performPredict() {
  // Sanity checks to avoid Eigen runtime assertion on invalid matrix products
  if (F.cols() != X_after.rows()) {
    std::cerr << "Error in performPredict: incompatible shapes F(" << F.rows() << "," << F.cols() << ")"
          << " X_after(" << X_after.rows() << "," << X_after.cols() << ")" << std::endl;
    throw std::runtime_error("performPredict: incompatible F and X_after shapes");
  }
  if (F.cols() != P_after.rows() || F.rows() != P_after.cols()) {
    // P_after should be square, and F.cols()==P_after.rows()
    std::cerr << "Warning in performPredict: P_after shape(" << P_after.rows() << "," << P_after.cols() << ")"
          << " may be incompatible with F(" << F.rows() << "," << F.cols() << ")" << std::endl;
  }

  X_prior = F * X_after;
  P_prior = F * P_after * F.transpose() + Q;
}

void Models::performUpdate(const Eigen::VectorXd &Z) {
  // Sanity checks before computing Kalman gain and update
  if (P_prior.rows() != P_prior.cols()) {
    std::cerr << "Error in performUpdate: P_prior is not square: (" << P_prior.rows() << "," << P_prior.cols() << ")" << std::endl;
    throw std::runtime_error("performUpdate: P_prior must be square");
  }
  if (H.cols() != P_prior.cols()) {
    std::cerr << "Error in performUpdate: incompatible shapes H(" << H.rows() << "," << H.cols() << ")"
          << " with P_prior(" << P_prior.rows() << "," << P_prior.cols() << ")" << std::endl;
    throw std::runtime_error("performUpdate: incompatible H and P_prior shapes");
  }
  if (H.cols() != X_prior.rows()) {
    std::cerr << "Error in performUpdate: incompatible shapes H.cols() (" << H.cols() << ")"
          << " and X_prior.rows() (" << X_prior.rows() << ")" << std::endl;
    throw std::runtime_error("performUpdate: incompatible H and X_prior shapes");
  }
  if (Z.rows() != H.rows()) {
    std::cerr << "Error in performUpdate: measurement Z size (" << Z.rows() << ") does not match H.rows() (" << H.rows() << ")" << std::endl;
    throw std::runtime_error("performUpdate: measurement dimension mismatch");
  }
  auto cond_print = [&](const std::string &s, bool &flag) {
    if (debug_mode || !flag) {
      std::cerr << s;
      if (!debug_mode) flag = true;
    }
  };

  std::ostringstream oss_shapes;
  oss_shapes << "performUpdate shapes: P_prior(" << P_prior.rows() << "," << P_prior.cols() << ")"
             << " H(" << H.rows() << "," << H.cols() << ")"
             << " R(" << R.rows() << "," << R.cols() << ")"
             << " X_prior(" << X_prior.rows() << "," << X_prior.cols() << ")"
             << " Z(" << Z.rows() << "," << Z.cols() << ")\n";
  cond_print(oss_shapes.str(), debug_printed_performUpdate);

  Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  if (S.rows() != S.cols()) {
    cond_print(std::string("Warning in performUpdate: innovation covariance S is not square: (") + std::to_string(S.rows()) + "," + std::to_string(S.cols()) + ")\n", debug_printed_performUpdate);
  }

  Eigen::MatrixXd Kal_Gain = P_prior * H.transpose() * S.inverse();
  cond_print((std::string("Kalman Gain shape: (") + std::to_string(Kal_Gain.rows()) + "," + std::to_string(Kal_Gain.cols()) + ")\n"), debug_printed_performUpdate);

  Eigen::VectorXd y = Z - H * X_prior;
  cond_print((std::string("innovation y shape: (") + std::to_string(y.rows()) + "," + std::to_string(y.cols()) + ")\n"), debug_printed_performUpdate);

  X_after = X_prior + Kal_Gain * y;

  // Ensure I is identity of proper size before computing P_after
  if (I.rows() != Kal_Gain.rows() || I.cols() != Kal_Gain.rows()) {
    I = Eigen::MatrixXd::Identity(Kal_Gain.rows(), Kal_Gain.rows());
    cond_print((std::string("Info: initialized identity matrix I to size (") + std::to_string(I.rows()) + "," + std::to_string(I.cols()) + ")\n"), debug_printed_performUpdate);
  }

  if ((I - Kal_Gain * H).rows() != P_prior.rows() || (I - Kal_Gain * H).cols() != P_prior.cols()) {
    cond_print((std::string("Warning: (I - K*H) shape (") + std::to_string((I - Kal_Gain * H).rows()) + "," + std::to_string((I - Kal_Gain * H).cols()) + ") differs from P_prior (" + std::to_string(P_prior.rows()) + "," + std::to_string(P_prior.cols()) + ")\n"), debug_printed_performUpdate);
  }

  P_after = (I - Kal_Gain * H) * P_prior;
}

/**
 * @brief 预测下一时刻的状态。
 * @return 预测的状态矩阵。
 */
Eigen::MatrixXd Models::predict() const {
  if (F.cols() != X_after.rows()) {
    std::cerr << "Error in predict(): incompatible shapes F(" << F.rows() << "," << F.cols() << ")"
          << " X_after(" << X_after.rows() << "," << X_after.cols() << ")" << std::endl;
    throw std::runtime_error("predict(): incompatible F and X_after shapes");
  }
  if (H.cols() != F.rows()) {
    std::cerr << "Warning in predict(): H.cols() (" << H.cols() << ") != F.rows() (" << F.rows() << ")" << std::endl;
  }
  return H * F * X_after;
}

/**
 * @brief 预测未来 N 时刻的状态。
 * @param N 未来时刻数。
 * @return 预测的状态矩阵。
 */
Eigen::MatrixXd Models::predict(int N) const {

    if (N == 0)
        return H * X_after;

    Eigen::VectorXd X = X_after;
    Eigen::MatrixXd P = P_after;
    
    // 进行N步预测，同时更新状态和协方差矩阵
    // 考虑不确定性的累积增长
    for (int i = 0; i < N; ++i) {
        // 状态预测
        X = F * X;
        // 协方差预测（考虑过程噪声导致的不确定性增长）
        P = F * P * F.transpose() + Q;
    }
    
    if (H.cols() != X.rows()) {
      std::cerr << "Error in predict(N): incompatible shapes H(" << H.rows() << "," << H.cols() << ")"
            << " X(" << X.rows() << "," << X.cols() << ")" << std::endl;
      throw std::runtime_error("predict(N): incompatible H and X shapes");
    }
    return H * X;
}