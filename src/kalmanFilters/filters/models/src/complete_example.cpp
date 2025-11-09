#include "models/model_factory.h"
#include "models/model_config_loader.h"
#include "basic_models/basic_model_factories.h"
#include "combined_models/combined_model_factories.h"
#include <iostream>
#include <vector>

/**
 * @brief 使用工厂模式的完整示例
 */
int main() {
    try {
        std::cout << "=== Kalman Filter Factory Pattern Example ===" << std::endl;
        
        // ========== 示例 1: 创建基本模型并进行滤波 ==========
        std::cout << "\n[Example 1] Creating and using CA_KF model" << std::endl;
        {
            // 配置参数
            ModelConfig config;
            config.T = 0.01;                                    // 10ms 采样时间
            config.Dim = 3;                                     // 3D 空间
            config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005; // 观测噪声
            config.X_0 = Eigen::VectorXd::Zero(9);              // 初始状态
            
            // 使用工厂创建模型
            auto ca_model = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
            std::cout << "  CA_KF model created" << std::endl;
            
            // 模拟一些测量数据
            std::vector<Eigen::VectorXd> measurements;
            for (int i = 0; i < 5; ++i) {
                Eigen::VectorXd z(3);
                z << i * 0.1, i * 0.1, i * 0.1;  // 简单的匀速运动
                measurements.push_back(z);
            }
            
            // 执行滤波
            std::cout << "  Running filter on " << measurements.size() << " measurements" << std::endl;
            for (const auto& z : measurements) {
                Eigen::VectorXd filtered = ca_model->KalmanFilterIterator(z);
                std::cout << "    Filtered position: [" 
                          << filtered(0) << ", " 
                          << filtered(1) << ", " 
                          << filtered(2) << "]" << std::endl;
            }
        }
        
        // ========== 示例 2: 创建不同类型的模型 ==========
        std::cout << "\n[Example 2] Creating different model types" << std::endl;
        {
            ModelConfig config;
            config.T = 0.01;
            config.Dim = 3;
            config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
            config.X_0 = Eigen::VectorXd::Zero(6);  // CV 模型需要 6 维状态
            
            auto cv_model = ModelFactoryRegistry::getInstance().createModel("CV_KF", config);
            std::cout << "  CV_KF model created (state dim: " << config.X_0.size() << ")" << std::endl;
            
            // Singer 模型需要额外参数
            config.X_0 = Eigen::VectorXd::Zero(9);
            config.extra_params["alpha"] = 0.5;
            
            auto singer_model = ModelFactoryRegistry::getInstance().createModel("Singer_KF", config);
            std::cout << "  Singer_KF model created with alpha=" 
                      << config.extra_params["alpha"] << std::endl;
        }
        
        // ========== 示例 3: 创建 IMM 组合模型 ==========
        std::cout << "\n[Example 3] Creating IMM combined model" << std::endl;
        {
            ModelConfig config;
            config.T = 0.01;
            config.Dim = 3;
            config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
            config.X_0 = Eigen::VectorXd::Zero(9);
            
            auto imm_model = ModelFactoryRegistry::getInstance().createModel(
                "IMM_CV_CA_CS_3Dim", config
            );
            std::cout << "  IMM_CV_CA_CS_3Dim model created" << std::endl;
            std::cout << "  This model combines CV, CA, and CS filters using IMM algorithm" << std::endl;
            
            // 使用 IMM 模型进行滤波
            Eigen::VectorXd z(3);
            z << 1.0, 2.0, 3.0;
            Eigen::VectorXd result = imm_model->KalmanFilterIterator(z);
            std::cout << "  IMM filtered result: [" 
                      << result(0) << ", " 
                      << result(1) << ", " 
                      << result(2) << "]" << std::endl;
        }
        
        // ========== 示例 4: 使用通用 IMM 工厂创建自定义组合 ==========
        std::cout << "\n[Example 4] Creating custom IMM with generic factory" << std::endl;
        {
            ModelConfig config;
            config.T = 0.01;
            config.Dim = 3;
            config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
            config.X_0 = Eigen::VectorXd::Zero(9);
            
            // 指定子模型
            config.sub_models = {"CV_KF", "CA_KF"};
            
            // 设置观测矩阵
            config.H = Eigen::MatrixXd::Zero(3, 9);
            config.H(0, 0) = 1;
            config.H(1, 3) = 1;
            config.H(2, 6) = 1;
            
            // 设置转移概率矩阵
            config.transform_rate_mat = Eigen::MatrixXd::Identity(2, 2) * 0.7;
            config.transform_rate_mat(0, 1) = 0.3;
            config.transform_rate_mat(1, 0) = 0.3;
            
            auto custom_imm = ModelFactoryRegistry::getInstance().createModel("IMM", config);
            std::cout << "  Custom IMM model created with CV + CA sub-models" << std::endl;
        }
        
        // ========== 示例 5: 列出所有可用的工厂 ==========
        std::cout << "\n[Example 5] Listing all available factories" << std::endl;
        {
            auto factories = ModelFactoryRegistry::getInstance().getRegisteredFactories();
            std::cout << "  Total factories registered: " << factories.size() << std::endl;
            std::cout << "  Available models:" << std::endl;
            
            for (const auto& name : factories) {
                std::cout << "    - " << name << std::endl;
            }
        }
        
        // ========== 示例 6: 预测未来状态 ==========
        std::cout << "\n[Example 6] Predicting future states" << std::endl;
        {
            ModelConfig config;
            config.T = 0.01;
            config.Dim = 3;
            config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
            config.X_0 = Eigen::VectorXd::Zero(9);
            config.X_0.segment(1, 1) << 1.0;  // x 方向初始速度
            config.X_0.segment(4, 1) << 1.0;  // y 方向初始速度
            
            auto model = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
            model->KalmanFilterInit(config.X_0);
            
            // 进行一次观测更新
            Eigen::VectorXd z(3);
            z << 0.1, 0.1, 0.0;
            model->KalmanFilterIterator(z);
            
            // 预测未来 N 步
            int N = 10;
            Eigen::MatrixXd predictions = model->predict(N);
            std::cout << "  Predicted states for next " << N << " time steps:" << std::endl;
            std::cout << "  Shape: " << predictions.rows() << "x" << predictions.cols() << std::endl;
            
            // 显示前 3 个预测
            for (int i = 0; i < std::min(3, static_cast<int>(predictions.rows())); ++i) {
                std::cout << "    Step " << i+1 << ": [" 
                          << predictions(i, 0) << ", " 
                          << predictions(i, 1) << ", " 
                          << predictions(i, 2) << "]" << std::endl;
            }
        }
        
        std::cout << "\n=== All examples completed successfully ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
