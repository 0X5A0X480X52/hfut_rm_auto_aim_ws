#ifndef CTRV_EKF_H
#define CTRV_EKF_H

#include "models/models.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @class CTRV_EKF
 * @brief 基于 CTRV 模型的扩展卡尔曼滤波器。
 */
class CTRV_EKF : public Models {
public:
    /**
     * @brief 构造函数。
     * @param T 采样时间。
     * @param R 观测噪声协方差矩阵。
     */
    CTRV_EKF(double T, const Eigen::MatrixXd& R);

    /**
     * @brief 初始化卡尔曼滤波器。
     * @param X_0 初始状态向量。
     */
    void KalmanFilterInit(const Eigen::VectorXd& X_0) override;

    /**
     * @brief 对于非线性 CTRV 模型，重写预测步骤以使用雅可比矩阵线性化。
     */
    void performPredict() override;

    /**
     * @brief 执行卡尔曼滤波器的整个过程。
     * @param measurements 测量值的向量。
     * @return 滤波后的状态矩阵。
     */
    Eigen::MatrixXd KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) override;

    /**
     * @brief 执行卡尔曼滤波器的单次迭代。
     * @param Z 当前测量值。
     * @return 滤波后的状态向量。
     */
    Eigen::VectorXd KalmanFilterIterator(const Eigen::VectorXd& Z) override;

    /**
     * @brief 重写预测函数，使用非线性状态转移
     * @param N 预测步数
     * @return 预测的状态矩阵
     */
    Eigen::MatrixXd predict(int N) const override;
};

#endif // CTRV_EKF_H
