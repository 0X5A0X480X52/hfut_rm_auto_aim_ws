#include "basic_models/CTRV_EKF.h"
#include <cmath>

CTRV_EKF::CTRV_EKF(double T, const Eigen::MatrixXd& R)
    : Models() {
    this->T = T;
    this->Dim = 5; // 状态维度 [x, y, v, theta, omega]
    this->R = R;
    this->Q = Eigen::MatrixXd::Identity(Dim, Dim) * 0.1; // 过程噪声协方差矩阵
    this->P_after = Eigen::MatrixXd::Identity(Dim, Dim); // 后验误差协方差矩阵
    this->X_after = Eigen::VectorXd::Zero(Dim); // 后验状态估计

    // 定义观测模型（仅含观测位置）: H为(2, 5)矩阵
    this->H = Eigen::MatrixXd::Zero(2, Dim);
    this->H(0, 0) = 1;  // 测量x对应状态x
    this->H(1, 1) = 1;  // 测量y对应状态y
    // 初始化状态转移矩阵为单位矩阵，避免外部调用 performPredict 时 F 为空导致乘法维度错误
    this->F = Eigen::MatrixXd::Identity(Dim, Dim);
}

void CTRV_EKF::KalmanFilterInit(const Eigen::VectorXd& X_0) {
    this->X_after = X_0; // 初始状态
    this->P_after = Eigen::MatrixXd::Identity(Dim, Dim); // 初始误差协方差矩阵
}

Eigen::MatrixXd CTRV_EKF::KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) {
    std::vector<Eigen::VectorXd> predict(measurements.size());

    for (std::size_t k = 0; k < measurements.size(); ++k) {
        predict[k] = KalmanFilterIterator(measurements[k]);
    }

    Eigen::MatrixXd result(predict.size(), predict[0].size());
    for (std::size_t i = 0; i < predict.size(); ++i)
        result.row(i) = predict[i].transpose();
    return result;
}

Eigen::VectorXd CTRV_EKF::KalmanFilterIterator(const Eigen::VectorXd& Z) {
    // 提取上一步的状态和协方差
    Eigen::VectorXd X_prior = X_after;
    Eigen::MatrixXd P_prior = P_after;
    double theta = X_prior(3);

    // 动态计算过程噪声协方差矩阵 Q
    double sigma_a = 0.1; // 加速度标准差，根据需要设置
    double sigma_omega_dot = 0.01; // 角速度变化率标准差，根据需要设置
    double delta_t = T;

    double q11 = std::pow(0.5 * delta_t * sigma_a * std::cos(theta), 2);
    double q12 = 0.25 * std::pow(delta_t, 4) * std::pow(sigma_a, 2) * std::sin(theta) * std::cos(theta);
    double q13 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_a, 2) * std::cos(theta);
    double q22 = std::pow(0.5 * delta_t * sigma_a * std::sin(theta), 2);
    double q23 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_a, 2) * std::sin(theta);
    double q33 = std::pow(delta_t * sigma_a, 2);
    double q44 = std::pow(0.5 * delta_t * sigma_omega_dot, 2);
    double q45 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_omega_dot, 2);
    double q55 = std::pow(delta_t * sigma_omega_dot, 2);

    Q = (Eigen::MatrixXd(Dim, Dim) << q11, q12, q13, 0, 0,
                                      q12, q22, q23, 0, 0,
                                      q13, q23, q33, 0, 0,
                                      0, 0, 0, q44, q45,
                                      0, 0, 0, q45, q55).finished();

    // 状态转移函数
    double omega = X_prior(4);
    Eigen::VectorXd X_predict(Dim);

    if (omega != 0) {
        X_predict(0) = X_prior(0) + (X_prior(2) / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        X_predict(1) = X_prior(1) + (X_prior(2) / omega) * (-std::cos(theta + omega * T) + std::cos(theta));
    } else {
        X_predict(0) = X_prior(0) + X_prior(2) * T * std::cos(theta);
        X_predict(1) = X_prior(1) + X_prior(2) * T * std::sin(theta);
    }

    X_predict(2) = X_prior(2);
    X_predict(3) = theta + omega * T;
    X_predict(4) = omega;

    // 计算状态转移矩阵的雅可比矩阵 F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(Dim, Dim);

    if (omega != 0) {
        F(0, 2) = (1 / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        F(0, 3) = (X_prior(2) / omega) * (std::cos(theta + omega * T) - std::cos(theta));
        F(0, 4) = (X_prior(2) / std::pow(omega, 2)) * (std::sin(theta) - std::sin(theta + omega * T)) + (X_prior(2) * T / omega) * std::cos(theta + omega * T);
        F(1, 2) = (1 / omega) * (-std::cos(theta + omega * T) + std::cos(theta));
        F(1, 3) = (X_prior(2) / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        F(1, 4) = (X_prior(2) / std::pow(omega, 2)) * (std::cos(theta + omega * T) - std::cos(theta)) + (X_prior(2) * T / omega) * std::sin(theta + omega * T);
    } else {
        F(0, 2) = T * std::cos(theta);
        F(0, 3) = -X_prior(2) * T * std::sin(theta);
        F(1, 2) = T * std::sin(theta);
        F(1, 3) = X_prior(2) * T * std::cos(theta);
    }

    F(3, 4) = T;

    // 更新成员变量 F，以便外部调用（如 performPredict）可使用最新的线性化矩阵
    this->F = F;

    // 预测协方差
    P_prior = F * P_prior * F.transpose() + Q;

    // 计算卡尔曼增益 (H为2x5矩阵)
    Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
    Eigen::MatrixXd Kal_Gain = P_prior * H.transpose() * S.inverse();

    // 更新状态估计
    Eigen::VectorXd Y = Z - H * X_predict;
    X_after = X_predict + Kal_Gain * Y;

    // 更新协方差矩阵
    P_after = (Eigen::MatrixXd::Identity(Dim, Dim) - Kal_Gain * H) * P_prior;

    // 返回完整状态估计 [x, y, v, theta, omega]
    return X_after;
}

void CTRV_EKF::performPredict() {
    // 使用与 KalmanFilterIterator 中相同的非线性预测和雅可比线性化
    Eigen::VectorXd X_prior_local = X_after;
    Eigen::MatrixXd P_prior_local = P_after;
    double theta = X_after(3);

    // 动态计算过程噪声协方差矩阵 Q（保持与迭代一致）
    double sigma_a = 0.1;
    double sigma_omega_dot = 0.01;
    double delta_t = T;

    double q11 = std::pow(0.5 * delta_t * sigma_a * std::cos(theta), 2);
    double q12 = 0.25 * std::pow(delta_t, 4) * std::pow(sigma_a, 2) * std::sin(theta) * std::cos(theta);
    double q13 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_a, 2) * std::cos(theta);
    double q22 = std::pow(0.5 * delta_t * sigma_a * std::sin(theta), 2);
    double q23 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_a, 2) * std::sin(theta);
    double q33 = std::pow(delta_t * sigma_a, 2);
    double q44 = std::pow(0.5 * delta_t * sigma_omega_dot, 2);
    double q45 = 0.5 * std::pow(delta_t, 3) * std::pow(sigma_omega_dot, 2);
    double q55 = std::pow(delta_t * sigma_omega_dot, 2);

    Q = (Eigen::MatrixXd(Dim, Dim) << q11, q12, q13, 0, 0,
                                      q12, q22, q23, 0, 0,
                                      q13, q23, q33, 0, 0,
                                      0, 0, 0, q44, q45,
                                      0, 0, 0, q45, q55).finished();

    double omega = X_after(4);
    Eigen::VectorXd X_predict(Dim);

    if (omega != 0) {
        X_predict(0) = X_after(0) + (X_after(2) / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        X_predict(1) = X_after(1) + (X_after(2) / omega) * (-std::cos(theta + omega * T) + std::cos(theta));
    } else {
        X_predict(0) = X_after(0) + X_after(2) * T * std::cos(theta);
        X_predict(1) = X_after(1) + X_after(2) * T * std::sin(theta);
    }

    X_predict(2) = X_after(2);
    X_predict(3) = theta + omega * T;
    X_predict(4) = omega;

    // 计算雅可比矩阵
    Eigen::MatrixXd F_local = Eigen::MatrixXd::Identity(Dim, Dim);
    if (omega != 0) {
        F_local(0, 2) = (1 / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        F_local(0, 3) = (X_after(2) / omega) * (std::cos(theta + omega * T) - std::cos(theta));
        F_local(0, 4) = (X_after(2) / std::pow(omega, 2)) * (std::sin(theta) - std::sin(theta + omega * T)) + (X_after(2) * T / omega) * std::cos(theta + omega * T);
        F_local(1, 2) = (1 / omega) * (-std::cos(theta + omega * T) + std::cos(theta));
        F_local(1, 3) = (X_after(2) / omega) * (std::sin(theta + omega * T) - std::sin(theta));
        F_local(1, 4) = (X_after(2) / std::pow(omega, 2)) * (std::cos(theta + omega * T) - std::cos(theta)) + (X_after(2) * T / omega) * std::sin(theta + omega * T);
    } else {
        F_local(0, 2) = T * std::cos(theta);
        F_local(0, 3) = -X_after(2) * T * std::sin(theta);
        F_local(1, 2) = T * std::sin(theta);
        F_local(1, 3) = X_after(2) * T * std::cos(theta);
    }
    F_local(3, 4) = T;

    // 更新成员状态
    this->F = F_local;
    this->X_prior = X_predict;
    this->P_prior = F_local * P_after * F_local.transpose() + Q;
}

/**
 * @brief 重写预测函数，使用非线性状态转移
 * @param N 预测步数
 * @return 预测的状态矩阵
 */
Eigen::MatrixXd CTRV_EKF::predict(int N) const {
    if (N == 0)
        return H * X_after;
    
    Eigen::VectorXd X = X_after;
    
    // 进行N步非线性状态预测
    for (int i = 0; i < N; ++i) {
        double theta = X(3);
        double omega = X(4);
        double v = X(2);
        
        // 非线性状态转移
        if (std::abs(omega) > 1e-6) {
            // 有角速度的情况
            X(0) += (v / omega) * (std::sin(theta + omega * T) - std::sin(theta));
            X(1) += (v / omega) * (-std::cos(theta + omega * T) + std::cos(theta));
        } else {
            // 角速度接近0，使用直线运动模型
            X(0) += v * T * std::cos(theta);
            X(1) += v * T * std::sin(theta);
        }
        
        // 速度保持不变（CTRV模型假设）
        // X(2) = X(2);
        
        // 角度更新
        X(3) += omega * T;
        
        // 角速度保持不变（CTRV模型假设）
        // X(4) = X(4);
    }
    
    return H * X;
}
