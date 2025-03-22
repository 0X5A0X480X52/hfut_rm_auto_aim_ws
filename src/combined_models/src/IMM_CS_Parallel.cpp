#include "combined_models/IMM_CS_Parallel.h"
#include <iostream>

IMM_CS_Parallel::IMM_CS_Parallel()
{

    int Dim = 3;

    // 间隔时间
    double T = 1.0;

    // 设置观测噪声协方差矩阵
    Eigen::MatrixXd R(3, 3);
    R << 0.5, 0.001, 0.001,
        0.001, 0.5, 0.001,
        0.001, 0.001, 0.5;
    Eigen::VectorXd X_0(9);

    int X_len = 9;
    assert(X_0.size() == X_len && "X_0 must be of size 9");
    assert(R.rows() == 3 && R.cols() == 3 && "R must be of size 3x3");

    // 设置观测矩阵
    Eigen::MatrixXd H(3, 9);
    H.setZero();
    H(0, 0) = 1;
    H(1, 3) = 1;
    H(2, 6) = 1;

    // 设置模型转换概率矩阵
    Eigen::MatrixXd transformRateMat(3, 3);
    transformRateMat << 0.60, 0.20, 0.20,
                        0.20, 0.60, 0.20,
                        0.20, 0.20, 0.60;

    // X_0 = [x, vx, ax, y, vy, ay, z, vz, az]
    assert(X_len == 9 && "X_len must be 9");
    assert(X_0.size() == X_len && "X_0 must be of size 9");
    assert(R.rows() == 3 && R.cols() == 3 && "R must be of size 3x3");
    assert(H.rows() == 3 && H.cols() == 9 && "H must be of size 3x9");
    assert(transformRateMat.rows() == 3 && transformRateMat.cols() == 3 && "transformRateMat must be of size 3x3");

    // CS model 01
    double a_1 = 0.5;
    double A_max_1 = 5;
    CS_KF *cs_1 = new CS_KF(T, a_1, A_max_1, Dim, R);
    cs_1->KalmanFilterInit(X_0);
    modelList.push_back(cs_1);

    // CS model 02
    double a_2 = 5;
    double A_max_2 = 10;
    CS_KF *cs_2 = new CS_KF(T, a_2, A_max_2, Dim, R);
    cs_2->KalmanFilterInit(X_0);
    modelList.push_back(cs_2);

    // CS model 03
    double a_3 = 0.1;
    double A_max_3 = 1;
    CS_KF *cs_3 = new CS_KF(T, a_3, A_max_3, Dim, R);
    cs_3->KalmanFilterInit(X_0);
    modelList.push_back(cs_3);

    // IMM model init
    initIMM(modelList, X_len, H, transformRateMat);
}
