#include "combined_models/IMM_CV_CA_CS_3Dim.h"
#include <iostream>

IMM_CV_CA_CS_3Dim::IMM_CV_CA_CS_3Dim() {
    double T = 1.0;
    Eigen::MatrixXd R(3, 3);
    R << 0.5, 0.001, 0.001,
        0.001, 0.5, 0.001,
        0.001, 0.001, 0.5;
    Eigen::VectorXd X_0(9);
    X_0.setZero();
    
    IMM_CV_CA_CS_3Dim(T, X_0, R);
    std::cout << "IMM_CV_CA_CS_3Dim init 01" << std::endl;
}

IMM_CV_CA_CS_3Dim::IMM_CV_CA_CS_3Dim( const Eigen::VectorXd& X_0 ) {
    double T = 0.01;
    Eigen::MatrixXd R(3, 3);
    R << 0.005, 0.001, 0.001, 0.001, 0.005, 0.001, 0.001, 0.001, 0.005;
    
    IMM_CV_CA_CS_3Dim(T, X_0, R);
    std::cout << "IMM_CV_CA_CS_3Dim init 01" << std::endl;
}

IMM_CV_CA_CS_3Dim::IMM_CV_CA_CS_3Dim( double T, const Eigen::VectorXd& X_0, const Eigen::MatrixXd& R) {

    int X_len = 9;
    assert(X_0.size() == X_len && "X_0 must be of size 9");
    assert(R.rows() == 3 && R.cols() == 3 && "R must be of size 3x3");

    // 设置观测矩阵
    Eigen::MatrixXd H(3,9);
    H.setZero();
    H(0,0) = 1;
    H(1,3) = 1;
    H(2,6) = 1;

    // 设置模型转换概率矩阵
    Eigen::MatrixXd transformRateMat(3,3);
    transformRateMat << 0.60, 0.20, 0.20,
                        0.20, 0.60, 0.20,
                        0.20, 0.20, 0.60;

    IMM_CV_CA_CS_3Dim(T, X_len, H, transformRateMat, X_0, R);
    std::cout << "IMM_CV_CA_CS_3Dim init 02" << std::endl;
}

IMM_CV_CA_CS_3Dim::IMM_CV_CA_CS_3Dim(double T, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat, const Eigen::VectorXd& X_0, const Eigen::MatrixXd& R) {
    int Dim = 3;

    // X_0 = [x, vx, ax, y, vy, ay, z, vz, az]
    assert(X_len == 9 && "X_len must be 9");
    assert(X_0.size() == X_len && "X_0 must be of size 9");
    assert(R.rows() == 3 && R.cols() == 3 && "R must be of size 3x3");
    assert(H.rows() == 3 && H.cols() == 9 && "H must be of size 3x9");
    assert(transformRateMat.rows() == 3 && transformRateMat.cols() == 3 && "transformRateMat must be of size 3x3");

    // CV model
    // X_0_cv = [x, vx, y, vy, z, vz]
    Eigen::VectorXd X_0_cv(6);
    X_0_cv << X_0(0), X_0(1), X_0(3), X_0(4), X_0(6), X_0(7);
    CV_KF* cv = new CV_KF(T, Dim, R);
    cv->KalmanFilterInit(X_0_cv);
    modelList.push_back(cv);

    // CA model
    // X_0_ca = [x, vx, ax, y, vy, ay, z, vz, az]
    Eigen::VectorXd X_0_ca(9);
    X_0_ca = X_0.head<9>();
    CA_KF* ca = new CA_KF(T, Dim, R);
    ca->KalmanFilterInit(X_0_ca);
    modelList.push_back(ca);

    // CS model
    // X_0_cs = [x, vx, ax, y, vy, ay, z, vz, az]
    // double a = 5;
    // double A_max = 3;
    double a = 5;
    double A_max = 5;
    Eigen::VectorXd X_0_cs(9);
    X_0_cs = X_0.head<9>();
    CS_KF* cs = new CS_KF(T, a, A_max, Dim, R);
    cs->KalmanFilterInit(X_0_cs);
    modelList.push_back(cs);

    // IMM model init
    initIMM(modelList, X_len, H, transformRateMat);

    // CV model adapters
    cv->setSetXAfterFunction([cv](const Eigen::VectorXd& X) {
        Eigen::VectorXd temp = cv->defaultGetXAfter();
        temp.segment<2>(0) = X.segment<2>(0);
        temp.segment<2>(2) = X.segment<2>(3);
        temp.segment<2>(4) = X.segment<2>(6);
        cv->defaultSetXAfter(temp);
    });

    cv->setSetPAfterFunction([cv](const Eigen::MatrixXd& P) {
        Eigen::MatrixXd temp = cv->defaultGetPAfter();
        temp.block<2,2>(0,0) = P.block<2,2>(0,0);
        temp.block<2,2>(2,2) = P.block<2,2>(3,3);
        temp.block<2,2>(4,4) = P.block<2,2>(6,6);
        cv->defaultSetPAfter(temp);
    });

    cv->setGetXAfterFunction([cv]() {
        Eigen::VectorXd modelX = cv->defaultGetXAfter();
        Eigen::VectorXd result(9);
        result << modelX[0], modelX[1], 0.0, modelX[2], modelX[3], 0.0, modelX[4], modelX[5], 0.0;
        return result;
    });

    cv->setGetPAfterFunction([cv]() {
        Eigen::MatrixXd modelP = cv->defaultGetPAfter();
        Eigen::MatrixXd result = Eigen::MatrixXd::Zero(9,9);
        result.block<2,2>(0,0) = modelP.block<2,2>(0,0);
        result.block<2,2>(3,3) = modelP.block<2,2>(2,2);
        result.block<2,2>(6,6) = modelP.block<2,2>(4,4);
        return result;
    });

    // CA model 与 CS model 的状态矩阵形状与 IMM 的形状相同，不需要适配器
    std::cout << "IMM_CV_CA_CS_3Dim init 03" << std::endl;
}
