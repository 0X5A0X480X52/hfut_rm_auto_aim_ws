#ifndef IMM_H
#define IMM_H

#include "models/models.h"
#include <vector>

class IMM : public Models {
public:
    IMM(const std::vector<Models*>& modelList, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat);
    IMM() = default;
    ~IMM() = default;

    void initIMM(const std::vector<Models*>& modelList, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat);

    Eigen::VectorXd KalmanFilterIterator(const Eigen::VectorXd& Z) override;
    Eigen::MatrixXd KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) override;
    void KalmanFilterInit(const Eigen::VectorXd& X_0) override;

    /**
     * @brief 预测下一时刻的状态。
     * @return 预测的状态矩阵。
     */
    Eigen::MatrixXd predict() const;

    /**
         * @brief 预测未来 N 时刻的状态。
         * @param N 未来时刻数。
         * @return 预测的状态矩阵。
         */
    Eigen::MatrixXd predict(int N) const;

protected:
    std::vector<Models*> modelList;
    int model_cnt;
    Eigen::MatrixXd transformRateMat;
    Eigen::MatrixXd mergeRateMat;
    Eigen::MatrixXd X_after;
    Eigen::MatrixXd X_fusion;
    Eigen::MatrixXd P_after;
    Eigen::MatrixXd P_fusion;
    Eigen::VectorXd Lambda;
    Eigen::VectorXd confidence;
    Eigen::VectorXd confidence_prior;
    Eigen::VectorXd X_output;

    double getLambda(Models& model, const Eigen::VectorXd& Z);
};

#endif // IMM_H
