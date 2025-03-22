#ifndef IMM_CV_CA_CTRV_H
#define IMM_CV_CA_CTRV_H

#include "combined_models/IMM.h"
#include "basic_models/CV_KF.h"
#include "basic_models/CA_KF.h"
#include "basic_models/CTRV_EKF.h"

class IMM_CV_CA_CTRV : public IMM {
public:
    IMM_CV_CA_CTRV(double T, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat, const Eigen::VectorXd& X_0, const Eigen::MatrixXd& R);
    ~IMM_CV_CA_CTRV() = default;
};

#endif // IMM_CV_CA_CTRV_H
