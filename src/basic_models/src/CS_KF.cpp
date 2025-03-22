#include "basic_models/CS_KF.h"
#include <cmath>
#include <iostream>

CS_KF::CS_KF(double T, double a, double A_max, int Dim, const Eigen::MatrixXd& R) 
    : Models() {
    this->T = T;
    this->a = a;
    this->A_max = A_max;
    this->Dim = Dim;
    this->R = R;
    this->pi = 3.14;
    this->A_min = -A_max;
    this->A_k = std::vector<double>(Dim, 0.0);
    this->v_means = Eigen::VectorXd::Zero(Dim);

    Eigen::MatrixXd I_dim = Eigen::MatrixXd::Identity(Dim, Dim);

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

    Eigen::MatrixXd F_D1 = (Eigen::MatrixXd(3, 3) << 1, T, (a * T - 1 + std::exp(-a * T)) / (a * a), 0, 1, (1 - std::exp(-a * T)) / a, 0, 0, std::exp(-a * T)).finished();
    F = kroneckerProduct(I_dim, F_D1).eval();

    // 定义当前统计模型新增的输入控制矩阵
    double g1 = 1 / a * (-T + (a * T * T) / 2 + (1 - std::exp(-a * T)) / a);
    double g2 = T - (1 - std::exp(-a * T)) / a;
    double g3 = 1 - std::exp(-a * T);
    Eigen::VectorXd G_D1 = (Eigen::VectorXd(3) << g1, g2, g3).finished();
    G = kroneckerProduct(I_dim, G_D1).eval();

    // 定义观测模型（仅含观测位置）
    Eigen::VectorXd H_D1 = (Eigen::VectorXd(3) << 1, 0, 0).finished();
    H = kroneckerProduct(I_dim, H_D1).eval();

    Eigen::VectorXd X_0 = Eigen::VectorXd::Zero(3 * Dim);
    KalmanFilterInit(X_0);
}

void CS_KF::KalmanFilterInit(const Eigen::VectorXd& X_0) {
    this->X_prior = X_0;
    this->X_after = X_prior;
    this->P_after = Eigen::MatrixXd::Zero(F.rows(), F.cols());
    this->I = Eigen::MatrixXd::Identity(3 * Dim, 3 * Dim);
}

void CS_KF::updateQ() {
    std::vector<double> A_k(Dim, 0.0);
    std::vector<double> sigma(Dim, 0.0);
    std::vector<Eigen::MatrixXd> Q_D1_blocks(Dim);

    for (int i = 0; i < Dim; ++i) {
        A_k[i] = X_after[3 * i + 2];
        if (A_k[i] > 0) {
            sigma[i] = (4 - pi) / pi * std::pow(A_max - A_k[i], 2);
        } else {
            sigma[i] = (4 - pi) / pi * std::pow(A_min - A_k[i], 2);
        }

        Q_D1_blocks[i] = 2 * a * sigma[i] * Q_D1;
    }

    Q = Eigen::MatrixXd::Zero(3 * Dim, 3 * Dim);
    for (int i = 0; i < Dim; ++i) {
        Q.block(3 * i, 3 * i, 3, 3) = Q_D1_blocks[i];
    }
    this->A_k = A_k;
}

Eigen::MatrixXd CS_KF::KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) {
    std::vector<Eigen::VectorXd> predict(measurements.size());

    for (std::size_t k = 0; k < measurements.size(); ++k) {
        updateQ();

        X_prior = F * X_after;
        P_prior = F * P_after * F.transpose() + Q;

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

Eigen::VectorXd CS_KF::KalmanFilterIterator(const Eigen::VectorXd& Z) {
    updateQ();

    X_prior = F * X_after;
    P_prior = F * P_after * F.transpose() + Q;

    Eigen::MatrixXd Kal_Gain = P_prior * H * (H.transpose() * P_prior * H + R).inverse();
    
    X_after = X_prior + Kal_Gain * (Z - H.transpose() * X_prior);
    P_after = (I - Kal_Gain * H.transpose()) * P_prior;

    return H.transpose() * X_after;
}