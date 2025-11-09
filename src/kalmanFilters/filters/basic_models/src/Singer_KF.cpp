#include "basic_models/Singer_KF.h"
#include <cmath>

Singer_KF::Singer_KF(double T, double a, double sigma, int Dim, const Eigen::MatrixXd& R)
    : Models(), a(a), sigma(sigma) {
    this->T = T;
    this->Dim = Dim;
    this->R = R;
    Eigen::MatrixXd I_dim = Eigen::MatrixXd::Identity(Dim, Dim);

    // 定义 Singer 模型的建模噪声
    double q11 = (1 - std::exp(-2 * a * T) + 2 * a * T + 2 * std::pow(a, 3) * std::pow(T, 3) / 3 - 2 * std::pow(a, 2) * std::pow(T, 2) - 4 * a * T * std::exp(-a * T)) / (2 * std::pow(a, 5));
    double q12 = (std::exp(-2 * a * T) + 1 - 2 * std::exp(-a * T) + 2 * a * T * std::exp(-a * T) - 2 * a * T + std::pow(a, 2) * std::pow(T, 2)) / (2 * std::pow(a, 4));
    double q13 = (1 - std::exp(-2 * a * T) - 2 * a * T * std::exp(-a * T)) / (2 * std::pow(a, 3));
    double q21 = q12;
    double q22 = (4 * std::exp(-a * T) - 3 - std::exp(-2 * a * T) + 2 * a * T) / (2 * std::pow(a, 3));
    double q23 = (std::exp(-2 * a * T) + 1 - 2 * std::exp(-a * T)) / (2 * std::pow(a, 2));
    double q31 = q13;
    double q32 = q23;
    double q33 = (1 - std::exp(-2 * a * T)) / (2 * a);

    Q_D1 = (Eigen::MatrixXd(3, 3) << q11, q12, q13, q21, q22, q23, q31, q32, q33).finished();
    Q = 2 * a * sigma * sigma * kroneckerProduct(I_dim, Q_D1).eval();
    w_means = Eigen::VectorXd::Zero(3 * Dim);

    // 定义 Singer 模型的状态转移矩阵
    Eigen::MatrixXd F_D1 = (Eigen::MatrixXd(3, 3) << 1, T, (a * T - 1 + std::exp(-a * T)) / (a * a), 0, 1, (1 - std::exp(-a * T)) / a, 0, 0, std::exp(-a * T)).finished();
    F = kroneckerProduct(I_dim, F_D1).eval();

    // 定义观测模型（仅含观测位置）
    Eigen::VectorXd H_D1 = (Eigen::VectorXd(3) << 1, 0, 0).finished();
    H = kroneckerProduct(I_dim, H_D1).eval();

    // 设置默认的初始状态
    Eigen::VectorXd X_0 = Eigen::VectorXd::Zero(3 * Dim);
    KalmanFilterInit(X_0);
}

void Singer_KF::KalmanFilterInit(const Eigen::VectorXd& X_0) {
    this->X_prior = X_0;
    this->X_after = X_prior;
    this->P_after = Eigen::MatrixXd::Zero(F.rows(), F.cols());
    this->I = Eigen::MatrixXd::Identity(3 * Dim, 3 * Dim);
}

Eigen::MatrixXd Singer_KF::KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) {
    std::vector<Eigen::VectorXd> predict(measurements.size());

    for (std::size_t k = 0; k < measurements.size(); ++k) {
        // 预测
        X_prior = F * X_after;
        P_prior = F * P_after * F.transpose() + Q;

        // 校正
        Eigen::MatrixXd Kal_Gain = P_prior * H * (H.transpose() * P_prior * H + R).inverse();
        Eigen::VectorXd Z = measurements[k];

        X_after = X_prior + Kal_Gain * (Z - H.transpose() * X_prior);
        P_after = (I - Kal_Gain * H.transpose()) * P_prior;

        predict[k] = H.transpose() * X_after;
    }

    Eigen::MatrixXd result(predict.size(), predict[0].size());
    for (std::size_t i = 0; i < predict.size(); ++i)
        result.row(i) = predict[i].transpose();
    return result;
}

Eigen::VectorXd Singer_KF::KalmanFilterIterator(const Eigen::VectorXd& Z) {
    // 预测
    X_prior = F * X_after;
    P_prior = F * P_after * F.transpose() + Q;

    // 校正
    Eigen::MatrixXd Kal_Gain = P_prior * H * (H.transpose() * P_prior * H + R).inverse();
    
    X_after = X_prior + Kal_Gain * (Z - H.transpose() * X_prior);
    P_after = (I - Kal_Gain * H.transpose()) * P_prior;

    return H.transpose() * X_after;
}
