#include "models/model_factory.h"
#include "models/model_config_loader.h"
#include "basic_models/basic_model_factories.h"
#include "combined_models/combined_model_factories.h"
#include <iostream>
#include <memory>

/**
 * @brief 演示如何使用模型工厂
 */
void demonstrateFactoryUsage() {
    std::cout << "=== Kalman Filter Model Factory Demo ===" << std::endl;
    
    // 1. 列出所有已注册的工厂
    std::cout << "\n1. Available model factories:" << std::endl;
    auto factories = ModelFactoryRegistry::getInstance().getRegisteredFactories();
    for (const auto& name : factories) {
        std::cout << "  - " << name << std::endl;
    }
    
    // 2. 手动创建 CA_KF 模型
    std::cout << "\n2. Creating CA_KF model manually:" << std::endl;
    {
        ModelConfig config;
        config.T = 0.01;
        config.Dim = 3;
        config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
        config.X_0 = Eigen::VectorXd::Zero(9);
        
        auto model = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
        std::cout << "  CA_KF model created successfully!" << std::endl;
        std::cout << "  State dimension: " << model->get_Dim() << std::endl;
    }
    
    // 3. 从配置文件创建 CV_KF 模型
    std::cout << "\n3. Creating CV_KF model from YAML config:" << std::endl;
    try {
        std::string config_path = "config/cv_kf_config.yaml";  // 相对于工作目录
        auto model = ModelConfigLoader::createModelFromYaml(config_path);
        std::cout << "  CV_KF model created from config file!" << std::endl;
        std::cout << "  State dimension: " << model->get_Dim() << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  Note: Config file not found (this is expected in demo)" << std::endl;
        std::cout << "  Error: " << e.what() << std::endl;
    }
    
    // 4. 创建 IMM 组合模型
    std::cout << "\n4. Creating IMM_CV_CA_CS_3Dim model:" << std::endl;
    {
        ModelConfig config;
        config.T = 0.01;
        config.Dim = 3;
        config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
        config.X_0 = Eigen::VectorXd::Zero(9);
        
        auto model = ModelFactoryRegistry::getInstance().createModel(
            "IMM_CV_CA_CS_3Dim", config
        );
        std::cout << "  IMM_CV_CA_CS_3Dim model created successfully!" << std::endl;
    }
    
    std::cout << "\n=== Demo completed ===" << std::endl;
}

int main() {
    try {
        demonstrateFactoryUsage();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
