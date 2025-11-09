#ifndef COMBINED_MODEL_FACTORIES_H
#define COMBINED_MODEL_FACTORIES_H

#include "models/model_factory.h"
#include "combined_models/IMM.h"
#include "combined_models/IMM_CS_Parallel.h"
#include "combined_models/IMM_CV_CA_CS_3Dim.h"
#include "combined_models/IMM_CV_CA_CS_Singer_3Dim.h"
#include "combined_models/IMM_CV_CA_CTRV.h"

/**
 * @class IMM_CV_CA_CS_3Dim_Factory
 * @brief IMM_CV_CA_CS_3Dim 模型工厂
 */
class IMM_CV_CA_CS_3Dim_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // 使用默认构造函数或带参数的构造函数
        // IMM_CV_CA_CS_3Dim 提供了 (double T, const VectorXd& X_0, const MatrixXd& R) 构造
        auto model = std::make_unique<IMM_CV_CA_CS_3Dim>(config.T, config.X_0, config.R);
        model->KalmanFilterInit(config.X_0);
        return model;
    }
    
    std::string getFactoryName() const override {
        return "IMM_CV_CA_CS_3Dim";
    }
};

/**
 * @class IMM_CV_CA_CS_Singer_3Dim_Factory
 * @brief IMM_CV_CA_CS_Singer_3Dim 模型工厂
 */
class IMM_CV_CA_CS_Singer_3Dim_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // 该类只提供带完整参数的构造函数，确保必须提供 H 和 transform_rate_mat
        if (config.H.size() == 0) {
            throw std::runtime_error("IMM_CV_CA_CS_Singer_3Dim factory requires 'H' in ModelConfig");
        }
        if (config.transform_rate_mat.size() == 0) {
            throw std::runtime_error("IMM_CV_CA_CS_Singer_3Dim factory requires 'transform_rate_mat' in ModelConfig");
        }
        int X_len = static_cast<int>(config.X_0.size());
        auto model = std::make_unique<IMM_CV_CA_CS_Singer_3Dim>(
            config.T,
            X_len,
            config.H,
            config.transform_rate_mat,
            config.X_0,
            config.R
        );
        model->KalmanFilterInit(config.X_0);
        return model;
    }
    
    std::string getFactoryName() const override {
        return "IMM_CV_CA_CS_Singer_3Dim";
    }
};

/**
 * @class IMM_CV_CA_CTRV_Factory
 * @brief IMM_CV_CA_CTRV 模型工厂
 */
class IMM_CV_CA_CTRV_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        if (config.H.size() == 0) {
            throw std::runtime_error("IMM_CV_CA_CTRV factory requires 'H' in ModelConfig");
        }
        if (config.transform_rate_mat.size() == 0) {
            throw std::runtime_error("IMM_CV_CA_CTRV factory requires 'transform_rate_mat' in ModelConfig");
        }
        int X_len = static_cast<int>(config.X_0.size());
        auto model = std::make_unique<IMM_CV_CA_CTRV>(
            config.T,
            X_len,
            config.H,
            config.transform_rate_mat,
            config.X_0,
            config.R
        );
        model->KalmanFilterInit(config.X_0);
        return model;
    }
    
    std::string getFactoryName() const override {
        return "IMM_CV_CA_CTRV";
    }
};

/**
 * @class IMM_CS_Parallel_Factory
 * @brief IMM_CS_Parallel 模型工厂
 */
class IMM_CS_Parallel_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // IMM_CS_Parallel 提供默认构造函数
        auto model = std::make_unique<IMM_CS_Parallel>();
        model->KalmanFilterInit(config.X_0);
        return model;
    }
    
    std::string getFactoryName() const override {
        return "IMM_CS_Parallel";
    }
};

/**
 * @class IMM_Factory
 * @brief 通用 IMM 模型工厂，支持自定义子模型
 * 
 * 使用方法：
 * - 在 config.sub_models 中指定子模型名称列表
 * - 在 config.H 中指定观测矩阵
 * - 在 config.transform_rate_mat 中指定转移概率矩阵
 */
class IMM_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        // 创建子模型列表
        std::vector<Models*> sub_model_list;
        
        for (const auto& model_name : config.sub_models) {
            // 为每个子模型创建配置
            ModelConfig sub_config = config;
            
            // 使用工厂注册器创建子模型
            auto sub_model = ModelFactoryRegistry::getInstance().createModel(
                model_name, sub_config
            );
            
            // 注意：这里使用 new 是因为 IMM 接受原始指针
            // 在实际使用中需要确保 IMM 正确管理这些指针的生命周期
            sub_model_list.push_back(sub_model.release());
        }
        
        // 如果没有提供观测矩阵 H，生成默认 H：假设 X_0 是 [x,v,a,x,v,a,...]
        Eigen::MatrixXd H = config.H;
        int X_len = static_cast<int>(config.X_0.size());
        if (H.size() == 0) {
            int state_blocks = X_len / 3;
            H = Eigen::MatrixXd::Zero(3, X_len);
            // 将观测设置为每个维度的第一个分量（位置）
            H(0, 0) = 1;
            if (state_blocks > 1) H(1, 3) = 1;
            if (state_blocks > 2) H(2, 6) = 1;
        }

        // 如果没有提供 transform_rate_mat，则生成对角为 0.6，其余均匀分布的矩阵
        Eigen::MatrixXd transform = config.transform_rate_mat;
        if (transform.size() == 0) {
            int m = static_cast<int>(sub_model_list.size());
            transform = Eigen::MatrixXd::Constant(m, m, (1.0 - 0.6) / std::max(1, m-1));
            for (int i = 0; i < m; ++i) transform(i,i) = 0.6;
        }

        // 创建 IMM 模型
        auto model = std::make_unique<IMM>(
            sub_model_list,
            X_len,
            H,
            transform
        );
        
        model->KalmanFilterInit(config.X_0);
        
        return model;
    }
    
    std::string getFactoryName() const override {
        return "IMM";
    }
};

#endif // COMBINED_MODEL_FACTORIES_H
