#ifndef IMM_CV_CA_CS_Singer_3Dim_H
#define IMM_CV_CA_CS_Singer_3Dim_H

#include "combined_models/IMM.h"
#include "basic_models/CV_KF.h"
#include "basic_models/CA_KF.h"
#include "basic_models/CS_KF.h"
#include "basic_models/Singer_KF.h"

class IMM_CV_CA_CS_Singer_3Dim : public IMM {
public:
    IMM_CV_CA_CS_Singer_3Dim(double T, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat, const Eigen::VectorXd& X_0, const Eigen::MatrixXd& R);
    ~IMM_CV_CA_CS_Singer_3Dim() = default;
};

#endif // IMM_CV_CA_CS_Singer_3Dim
