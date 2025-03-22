#include "basic_models/CV_KF.h"
#include <cmath>

CV_KF::CV_KF(double T, int Dim, const Eigen::MatrixXd& R)
    : Models(){
    this->T = T;
    this->Dim = Dim;
    this->R = R;
    Eigen::MatrixXd I_dim = Eigen::MatrixXd::Identity(Dim, Dim);

    // 定义 CV 模型的建模噪声
    double q11 = std::pow(T, 3) / 3;
    double q12 = std::pow(T, 2) / 2;
    double q21 = q12;
    double q22 = T;

    Q_D1 = (Eigen::MatrixXd(2, 2) << q11, q12, q21, q22).finished();
    Q = kroneckerProduct(I_dim, Q_D1).eval();
    w_means = Eigen::VectorXd::Zero(2 * Dim);

    // 定义 CV 模型的状态转移矩阵
    Eigen::MatrixXd F_D1 = (Eigen::MatrixXd(2, 2) << 1, T, 0, 1).finished();
    F = kroneckerProduct(I_dim, F_D1).eval();

    // 定义观测模型（仅含观测位置）
    Eigen::VectorXd H_D1 = (Eigen::VectorXd(2) << 1, 0).finished();
    H = kroneckerProduct(I_dim, H_D1).eval();

    // 设置默认的初始状态
    Eigen::VectorXd X_0 = Eigen::VectorXd::Zero(2 * Dim);
    KalmanFilterInit(X_0);
}

void CV_KF::KalmanFilterInit(const Eigen::VectorXd& X_0) {
    this->X_prior = X_0;
    this->X_after = X_prior;
    this->P_after = Eigen::MatrixXd::Zero(F.rows(), F.cols());
    this->I = Eigen::MatrixXd::Identity(2 * Dim, 2 * Dim);
}

Eigen::MatrixXd CV_KF::KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) {
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

Eigen::VectorXd CV_KF::KalmanFilterIterator(const Eigen::VectorXd& Z) {
    // 预测
    X_prior = F * X_after;
    P_prior = F * P_after * F.transpose() + Q;

    // 校正
    Eigen::MatrixXd Kal_Gain = P_prior * H * (H.transpose() * P_prior * H + R).inverse();
    
    X_after = X_prior + Kal_Gain * (Z - H.transpose() * X_prior);
    P_after = (I - Kal_Gain * H.transpose()) * P_prior;

    return H.transpose() * X_after;
}
