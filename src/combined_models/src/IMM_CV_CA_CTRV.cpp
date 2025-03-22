#include "combined_models/IMM_CV_CA_CTRV.h"
#include <iostream>

IMM_CV_CA_CTRV::IMM_CV_CA_CTRV(double T, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat, const Eigen::VectorXd& X_0, const Eigen::MatrixXd& R) {
    int Dim = 2;

    Eigen::VectorXd X_0_cv(4);
    X_0_cv << X_0(0), X_0(1), X_0(3), X_0(4);

    Eigen::VectorXd X_0_ca(6);
    X_0_ca << X_0(0), X_0(1), X_0(2), X_0(3), X_0(4), X_0(5);

    Eigen::VectorXd X_0_ctrv(5);
    X_0_ctrv << X_0(0), X_0(3), X_0(6), X_0(7), X_0(8);

    CV_KF* cv = new CV_KF(T, Dim, R);
    cv->KalmanFilterInit(X_0_cv);

    CA_KF* ca = new CA_KF(T, Dim, R);
    ca->KalmanFilterInit(X_0_ca);

    CTRV_EKF* ctrv = new CTRV_EKF(T, R);
    ctrv->KalmanFilterInit(X_0_ctrv);

    modelList.push_back(cv);
    modelList.push_back(ca);
    modelList.push_back(ctrv);

    initIMM(modelList, X_len, H, transformRateMat);

    // CV model adapters
    cv->setSetXAfterFunction([cv](const Eigen::VectorXd& X) {
        Eigen::VectorXd temp = cv->defaultGetXAfter();
        temp.segment<2>(0) = X.segment<2>(0);
        temp.segment<2>(2) = X.segment<2>(3);
        cv->defaultSetXAfter(temp);
    });

    cv->setSetPAfterFunction([cv](const Eigen::MatrixXd& P) {
        Eigen::MatrixXd temp = cv->defaultGetPAfter();
        temp.block<2,2>(0,0) = P.block<2,2>(0,0);
        temp.block<2,2>(2,2) = P.block<2,2>(3,3);
        cv->defaultSetPAfter(temp);
    });

    cv->setGetXAfterFunction([cv]() {
        Eigen::VectorXd modelX = cv->defaultGetXAfter();
        Eigen::VectorXd result(9);
        result << modelX[0], modelX[1], 0.0, modelX[2], modelX[3], 0.0, 0.0, 0.0, 0.0;
        return result;
    });

    cv->setGetPAfterFunction([cv]() {
        Eigen::MatrixXd modelP = cv->defaultGetPAfter();
        Eigen::MatrixXd result = Eigen::MatrixXd::Zero(9,9);
        result.block<2,2>(0,0) = modelP.block<2,2>(0,0);
        result.block<2,2>(3,3) = modelP.block<2,2>(2,2);
        return result;
    });

    // CA model adapters
    ca->setSetXAfterFunction([ca](const Eigen::VectorXd& X) {
        Eigen::VectorXd temp = ca->defaultGetXAfter();
        temp.segment<6>(0) = X.segment<6>(0);
        ca->defaultSetXAfter(temp);
    });

    ca->setSetPAfterFunction([ca](const Eigen::MatrixXd& P) {
        Eigen::MatrixXd temp = ca->defaultGetPAfter();
        temp.block<6,6>(0,0) = P.block<6,6>(0,0);
        ca->defaultSetPAfter(temp);
    });

    ca->setGetXAfterFunction([ca]() {
        Eigen::VectorXd modelX = ca->defaultGetXAfter();
        Eigen::VectorXd result(9);
        result << modelX.head<6>(), 0.0, 0.0, 0.0; 
        return result;
    });

    ca->setGetPAfterFunction([ca]() {
        Eigen::MatrixXd modelP = ca->defaultGetPAfter();
        Eigen::MatrixXd result = Eigen::MatrixXd::Zero(9,9);
        result.block<6,6>(0,0) = modelP.block<6,6>(0,0);
        return result;
    });

    // CTRV model adapters
    ctrv->setSetXAfterFunction([ctrv](const Eigen::VectorXd& X) {
        Eigen::VectorXd temp = ctrv->defaultGetXAfter();
        temp[0] = X[0];
        temp[1] = X[3];
        temp.segment<3>(2) = X.segment<3>(6);
        ctrv->defaultSetXAfter(temp);
    });

    ctrv->setSetPAfterFunction([ctrv](const Eigen::MatrixXd& P) {
        Eigen::MatrixXd temp = ctrv->defaultGetPAfter();
        temp(0,0) = P(0,0);
        temp(1,1) = P(3,3);
        temp.block<3,3>(2,2) = P.block<3,3>(6,6);
        ctrv->defaultSetPAfter(temp);
    });

    ctrv->setGetXAfterFunction([ctrv]() {
        Eigen::VectorXd modelX = ctrv->defaultGetXAfter();
        Eigen::VectorXd result(9);
        result << modelX[0], 0.0, 0.0, modelX[1], 0.0, 0.0, modelX.segment<3>(2);
        return result;
    });

    ctrv->setGetPAfterFunction([ctrv]() {
        Eigen::MatrixXd modelP = ctrv->defaultGetPAfter();
        Eigen::MatrixXd result = Eigen::MatrixXd::Zero(9,9);
        result(0,0) = modelP(0,0);
        result.block<2,2>(1,1) = Eigen::MatrixXd::Identity(2,2);
        result(3,3) = modelP(1,1);
        result.block<2,2>(4,4) = Eigen::MatrixXd::Identity(2,2);
        result.block<3,3>(6,6) = modelP.block<3,3>(2,2);
        return result;
    });
}
