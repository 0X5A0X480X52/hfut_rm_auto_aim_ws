#include "combined_models/IMM.h"
#include <cmath>
#include <iostream> // 添加此行以使用 std::cout

IMM::IMM(const std::vector<Models*>& modelList, int X_len, const Eigen::MatrixXd& H, const Eigen::MatrixXd& transformRateMat) {
    initIMM(modelList, X_len, H, transformRateMat);
}

void IMM::KalmanFilterInit(const Eigen::VectorXd& X_0) {
    for (Models* model : modelList) {
        model->KalmanFilterInit(X_0);
    }
    std::cout << "IMM::KalmanFilterInit init" << std::endl;
}

Eigen::VectorXd IMM::KalmanFilterIterator(const Eigen::VectorXd& Z) {

    mergeRateMat = confidence.asDiagonal() * transformRateMat;
    mergeRateMat = mergeRateMat.array().colwise() / mergeRateMat.rowwise().sum().array();

    int X_len = X_after.cols();

    // 1) 计算 X_fusion
    for (int i = 0; i < model_cnt; ++i) {
        Eigen::VectorXd fused = Eigen::VectorXd::Zero(X_after.cols());
        for (int k = 0; k < model_cnt; ++k) {
            fused += mergeRateMat(i, k) * X_after.row(k).transpose();
        }
        X_fusion.row(i) = fused.transpose();
    }

    // 2) 计算 error
    Eigen::MatrixXd error = X_after - X_fusion;

    // 3) 计算 P_fusion
    for (int i = 0; i < model_cnt; ++i) {
        Eigen::MatrixXd Pfuse = Eigen::MatrixXd::Zero(X_after.cols(), X_after.cols());
        for (int k = 0; k < model_cnt; ++k) {
            Eigen::MatrixXd P_block_k = P_after.block(k * X_len, 0, X_len, X_len);
            Eigen::VectorXd e_k = error.row(k).transpose();
            Eigen::MatrixXd eDot_k = e_k * e_k.transpose();
            Pfuse += (P_block_k - eDot_k) * mergeRateMat(i, k);
        }
        P_fusion.block(i * X_len, 0, X_len, X_len) = Pfuse;
    }

    for (int i = 0; i < model_cnt; ++i) {
        Models* model = modelList[i];
        model->set_X_after(X_fusion.row(i));
        model->set_P_after(P_fusion.block(i * X_len, 0, X_len, X_len)); // 修改此行
        model->KalmanFilterIterator(Z);
        X_after.row(i) = model->get_X_after();
        P_after.block(i * X_len, 0, X_len, X_len) = model->get_P_after(); // 修改此行
        Lambda(i) = getLambda(*model, Z);
        // Lambda(i) = model->getLambda(Z);
    }

    confidence_prior = transformRateMat * confidence;
    std::cout << "confidence_prior.transpose(): " << confidence_prior.transpose() << std::endl;
    std::cout << "Lambda.transpose(): " << Lambda.transpose() << std::endl;

    // Handle NaN and inf values in Lambda
    for (int i = 0; i < Lambda.size(); ++i) {
        if (std::isnan(Lambda(i))) {
            Lambda(i) = 1e-9; // Replace NaN with a very small number
        } else if (std::isinf(Lambda(i))) {
            Lambda(i) = 1; // Replace inf with a very large number
        }
    }

    confidence = confidence_prior.cwiseProduct(Lambda) / (confidence_prior.cwiseProduct(Lambda)).sum();

    std::cout << "confidence.transpose(): " << confidence.transpose() << std::endl;
    if (confidence.hasNaN()) {
        int nan_count = 0;
        int zero_count = 0;
        for (int i = 0; i < confidence.size(); ++i) {
            if (std::isnan(confidence(i))) {
            nan_count++;
            } else if (confidence(i) == 0) {
            zero_count++;
            }
        }
        if (nan_count == 1 && zero_count == 2) {
            for (int i = 0; i < confidence.size(); ++i) {
            if (std::isnan(confidence(i))) {
                confidence(i) = 1.0 - 1e-5; // 将 NaN 改为趋近 1 的数
            } else if (confidence(i) == 0) {
                confidence(i) = 1e-5; // 将 0 改为极小的小数
            }
            }
        } else {
            throw std::runtime_error("Invalid combination of NaN and zero values in confidence vector.");
        }
    }

    X_output = confidence.transpose() * X_after;

    std::cout << "X_output.transpose(): " << X_output.transpose() << std::endl;
    if (X_output.hasNaN()) {
        throw std::runtime_error("X_output contains NaN values.");
    }

    return H * X_output;
}

Eigen::MatrixXd IMM::KalmanFilterWholeProcess(const std::vector<Eigen::VectorXd>& measurements) {
    Eigen::MatrixXd results(measurements.size(), H.rows());
    for (std::size_t k = 0; k < measurements.size(); ++k) {
        results.row(k) = KalmanFilterIterator(measurements[k]);
    }
    return results;
}

double IMM::getLambda(Models& model, const Eigen::VectorXd& Z) {
    double pi = 3.14;

    Eigen::VectorXd r = Z - model.get_H().transpose() * model.defaultGetXAfter();
    Eigen::MatrixXd S = model.get_H().transpose() * model.defaultGetPAfter() * model.get_H() + model.get_R();
    double det_S = S.determinant();
    if (det_S <= 0 || std::isnan(det_S) || std::isinf(det_S)) {
        det_S = 1e-9; // Replace invalid determinant with a very small positive number
    }

    // 计算 S 的逆
    Eigen::MatrixXd S_inv = S.inverse();

    // 检查 r.transpose() * S.inverse() * r 的值
    double exponent = -0.5 * r.transpose() * S_inv * r;
    if (std::isnan(exponent) || std::isinf(exponent)) {
        std::cerr << "Warning: exponent is NaN or inf." << std::endl;
        return 1e-9; // 返回一个非常小的数以避免 NaN 或 inf
    }

    // 计算 Lambda
    double Lambda = (1 / std::sqrt(std::abs(2 * pi * det_S))) * std::exp(exponent);
    if (std::isnan(Lambda) || std::isinf(Lambda)) {
        std::cerr << "Warning: Lambda is NaN or inf." << std::endl;
        return 1e-9; // 返回一个非常小的数以避免 NaN 或 inf
    }

    return Lambda;
}

void IMM::initIMM(const std::vector<Models*>& modelList, 
                  int X_len, 
                  const Eigen::MatrixXd& H, 
                  const Eigen::MatrixXd& transformRateMat)
{
    this->modelList       = modelList;
    this->H               = H;
    this->transformRateMat= transformRateMat;
    model_cnt             = modelList.size();
    mergeRateMat          = Eigen::MatrixXd::Zero(model_cnt, model_cnt);
    X_after               = Eigen::MatrixXd::Zero(model_cnt, X_len);
    X_fusion              = Eigen::MatrixXd::Zero(model_cnt, X_len);
    P_after               = Eigen::MatrixXd::Zero(model_cnt * X_len, X_len);
    P_fusion              = Eigen::MatrixXd::Zero(model_cnt * X_len, X_len);
    Lambda                = Eigen::VectorXd::Zero(model_cnt);
    confidence            = Eigen::VectorXd::Ones(model_cnt);
    confidence_prior      = Eigen::VectorXd::Ones(model_cnt);
    X_output              = Eigen::VectorXd::Zero(X_len);
}

Eigen::MatrixXd IMM::predict() const {
    Eigen::MatrixXd X_predict = Eigen::MatrixXd::Zero(model_cnt,(H * X_output).cols());
    for (int i = 0; i < model_cnt; ++i) {
        Models* model = modelList[i];
        X_predict.row(i) = model->predict();
        std::cout << "IMM::predict()"  << std::endl;
    }

    std::cout << "F.size(): " << confidence.transpose().rows() << "," <<  confidence.transpose().cols() << std::endl;
    std::cout << "X.size(): " << X_predict.rows() << "," <<  X_predict.cols() << std::endl;

    return confidence.transpose() * X_predict;
}

Eigen::MatrixXd IMM::predict(int N) const {
    Eigen::MatrixXd X_predict = Eigen::MatrixXd::Zero(model_cnt, H.rows());
    Eigen::VectorXd X_predict_output = Eigen::VectorXd::Zero(H.rows());
    for (int i = 0; i < model_cnt; ++i) {
        Models* model = modelList[i];
        // auto a = model->predict(N).transpose();
        // std::cout << "IMM::predict()"  << std::endl;
        // std::cout << "model->predict(N).size(): " << a.rows() << "," <<  a.cols() << std::endl;
        // std::cout << "X_predict.row(i).size(): " << X_predict.row(i).rows() << "," <<  X_predict.row(i).cols() << std::endl;
        X_predict.row(i) = model->predict(N).transpose();
        // std::cout << "IMM::predict() 2"  << std::endl;
    }

    // std::cout << "confidence.transpose().size(): " << confidence.transpose().rows() << "," <<  confidence.transpose().cols() << std::endl;
    // std::cout << "X_predict.size(): " << X_predict.rows() << "," <<  X_predict.cols() << std::endl;

    X_predict_output = confidence.transpose() * X_predict;

    return X_predict_output;
}