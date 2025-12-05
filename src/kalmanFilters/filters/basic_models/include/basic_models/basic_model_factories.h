#ifndef BASIC_MODEL_FACTORIES_H
#define BASIC_MODEL_FACTORIES_H

#include "models/model_factory.h"
#include "basic_models/CA_KF.h"
#include "basic_models/CV_KF.h"
#include "basic_models/CS_KF.h"
#include "basic_models/CTRV_EKF.h"
#include "basic_models/Singer_KF.h"

/**
 * @class CA_KF_Factory
 * @brief CA 模型工厂
 */
class CA_KF_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        auto model = std::make_unique<CA_KF>(config.T, config.Dim, config.R);
        // CA_KF state dimension is 3*Dim: [x, vx, ax, y, vy, ay, ...]
        int expected_dim = 3 * config.Dim;
        if (config.X_0.size() == expected_dim) {
            model->KalmanFilterInit(config.X_0);
        } else {
            // Use zero vector with correct dimension
            model->KalmanFilterInit(Eigen::VectorXd::Zero(expected_dim));
        }
        return model;
    }
    
    std::string getFactoryName() const override {
        return "CA_KF";
    }
};

/**
 * @class CV_KF_Factory
 * @brief CV 模型工厂
 */
class CV_KF_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        auto model = std::make_unique<CV_KF>(config.T, config.Dim, config.R);
        // CV_KF state dimension is 2*Dim: [x, vx, y, vy, ...]
        int expected_dim = 2 * config.Dim;
        if (config.X_0.size() == expected_dim) {
            model->KalmanFilterInit(config.X_0);
        } else {
            // Use zero vector with correct dimension
            model->KalmanFilterInit(Eigen::VectorXd::Zero(expected_dim));
        }
        return model;
    }
    
    std::string getFactoryName() const override {
        return "CV_KF";
    }
};

/**
 * @class CS_KF_Factory
 * @brief CS 模型工厂
 */
class CS_KF_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // CS_KF requires additional parameters: a (decay) and A_max (max acceleration)
        double a = 5.0;
        double A_max = 5.0;
        auto it_a = config.extra_params.find("a");
        if (it_a != config.extra_params.end()) a = it_a->second;
        auto it_A = config.extra_params.find("A_max");
        if (it_A != config.extra_params.end()) A_max = it_A->second;

        auto model = std::make_unique<CS_KF>(config.T, a, A_max, config.Dim, config.R);
        // CS_KF state dimension is 3*Dim: [x, vx, ax, y, vy, ay, ...]
        int expected_dim = 3 * config.Dim;
        if (config.X_0.size() == expected_dim) {
            model->KalmanFilterInit(config.X_0);
        } else {
            model->KalmanFilterInit(Eigen::VectorXd::Zero(expected_dim));
        }
        return model;
    }
    
    std::string getFactoryName() const override {
        return "CS_KF";
    }
};

/**
 * @class CTRV_EKF_Factory
 * @brief CTRV EKF 模型工厂
 */
class CTRV_EKF_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // CTRV_EKF constructor: CTRV_EKF(double T, const Eigen::MatrixXd& R)
        auto model = std::make_unique<CTRV_EKF>(config.T, config.R);
        // CTRV_EKF state dimension is fixed at 5: [x, y, v, theta, omega]
        int expected_dim = 5;
        if (config.X_0.size() == expected_dim) {
            model->KalmanFilterInit(config.X_0);
        } else {
            model->KalmanFilterInit(Eigen::VectorXd::Zero(expected_dim));
        }
        return model;
    }
    
    std::string getFactoryName() const override {
        return "CTRV_EKF";
    }
};

/**
 * @class Singer_KF_Factory
 * @brief Singer 模型工厂
 */
class Singer_KF_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // Singer_KF requires parameters: a (decay) and sigma (noise std)
        double a = 0.5;   // 默认衰减系数
        double sigma = 1.0; // 默认噪声标准差
        auto it_a = config.extra_params.find("a");
        if (it_a != config.extra_params.end()) a = it_a->second;
        auto it_sigma = config.extra_params.find("sigma");
        if (it_sigma != config.extra_params.end()) sigma = it_sigma->second;

        auto model = std::make_unique<Singer_KF>(config.T, a, sigma, config.Dim, config.R);
        // Singer_KF state dimension is 3*Dim: [x, vx, ax, y, vy, ay, ...]
        int expected_dim = 3 * config.Dim;
        if (config.X_0.size() == expected_dim) {
            model->KalmanFilterInit(config.X_0);
        } else {
            model->KalmanFilterInit(Eigen::VectorXd::Zero(expected_dim));
        }
        return model;
    }
    
    std::string getFactoryName() const override {
        return "Singer_KF";
    }
};

#endif // BASIC_MODEL_FACTORIES_H
