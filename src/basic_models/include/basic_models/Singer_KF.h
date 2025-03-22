#ifndef SINGER_KF_H
#define SINGER_KF_H

#include "models/models.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @class Singer_KF
 * @brief 基于 Singer 模型的卡尔曼滤波器。
 */
class Singer_KF : public Models {
public:
    /**
     * @brief 构造函数。
     * @param T 采样时间。
     * @param a 衰减系数。
     * @param sigma 噪声标准差。
     * @param Dim 状态维度。
     * @param R 观测噪声协方差矩阵。
     */
    Singer_KF(double T, double a, double sigma, int Dim, const Eigen::MatrixXd& R);

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

private:
    double a; ///< 衰减系数。
    double sigma; ///< 噪声标准差。
    Eigen::MatrixXd Q_D1; ///< 过程噪声协方差矩阵的基本块。
};

#endif // SINGER_KF_H
