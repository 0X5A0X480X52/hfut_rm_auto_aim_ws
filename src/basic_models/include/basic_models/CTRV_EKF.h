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
};

#endif // CTRV_EKF_H
