#include <rclcpp/rclcpp.hpp>
#include "models/model_factory.h"
#include "models/model_config_loader.h"
#include "basic_models/basic_model_factories.h"
#include "combined_models/combined_model_factories.h"

/**
 * @class KalmanFilterNode
 * @brief 使用工厂模式的 ROS2 卡尔曼滤波节点示例
 */
class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        // 声明参数
        this->declare_parameter<std::string>("model_type", "CA_KF");
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<double>("T", 0.01);
        this->declare_parameter<int>("Dim", 3);
        
        // 初始化模型
        initializeModel();
        
        RCLCPP_INFO(this->get_logger(), "Kalman Filter Node initialized with %s model", 
                    model_type_.c_str());
    }
    
    /**
     * @brief 处理测量数据
     */
    Eigen::VectorXd processMeasurement(const Eigen::VectorXd& measurement) {
        if (!model_) {
            RCLCPP_ERROR(this->get_logger(), "Model not initialized!");
            return Eigen::VectorXd();
        }
        
        return model_->KalmanFilterIterator(measurement);
    }
    
    /**
     * @brief 预测未来状态
     */
    Eigen::MatrixXd predictFuture(int steps) {
        if (!model_) {
            RCLCPP_ERROR(this->get_logger(), "Model not initialized!");
            return Eigen::MatrixXd();
        }
        
        return model_->predict(steps);
    }
    
    /**
     * @brief 运行时切换模型
     */
    bool switchModel(const std::string& new_model_type) {
        try {
            std::string old_model = model_type_;
            model_type_ = new_model_type;
            initializeModel();
            
            RCLCPP_INFO(this->get_logger(), "Switched model from %s to %s", 
                        old_model.c_str(), new_model_type.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch model: %s", e.what());
            return false;
        }
    }

private:
    void initializeModel() {
        // 获取参数
        this->get_parameter("model_type", model_type_);
        std::string config_file;
        this->get_parameter("config_file", config_file);
        
        // 如果提供了配置文件，从文件加载
        if (!config_file.empty()) {
            try {
                model_ = ModelConfigLoader::createModelFromYaml(config_file);
                RCLCPP_INFO(this->get_logger(), "Model loaded from config file: %s", 
                            config_file.c_str());
                return;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), 
                           "Failed to load from config file: %s. Using parameters instead.", 
                           e.what());
            }
        }
        
        // 从参数创建模型
        ModelConfig config;
        this->get_parameter("T", config.T);
        this->get_parameter("Dim", config.Dim);
        
        // 设置默认的 R 矩阵
        config.R = Eigen::MatrixXd::Identity(config.Dim, config.Dim) * 0.005;
        
        // 根据模型类型设置初始状态
        if (model_type_ == "CV_KF") {
            config.X_0 = Eigen::VectorXd::Zero(2 * config.Dim);
        } else if (model_type_ == "CA_KF" || model_type_ == "CS_KF" || 
                   model_type_ == "Singer_KF") {
            config.X_0 = Eigen::VectorXd::Zero(3 * config.Dim);
        } else if (model_type_.find("IMM") != std::string::npos) {
            config.X_0 = Eigen::VectorXd::Zero(3 * config.Dim);
        } else {
            config.X_0 = Eigen::VectorXd::Zero(3 * config.Dim);
        }
        
        // 使用工厂创建模型
        model_ = ModelFactoryRegistry::getInstance().createModel(model_type_, config);
        
        RCLCPP_INFO(this->get_logger(), "Model created from parameters");
    }
    
    std::unique_ptr<Models> model_;
    std::string model_type_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<KalmanFilterNode>();
    
    // 示例：处理一些测量数据
    std::vector<Eigen::VectorXd> measurements;
    for (int i = 0; i < 10; ++i) {
        Eigen::VectorXd z(3);
        z << i * 0.1, i * 0.1, i * 0.1;
        measurements.push_back(z);
    }
    
    RCLCPP_INFO(node->get_logger(), "Processing %zu measurements...", measurements.size());
    for (size_t i = 0; i < measurements.size(); ++i) {
        auto result = node->processMeasurement(measurements[i]);
        RCLCPP_INFO(node->get_logger(), 
                   "Measurement %zu: [%.3f, %.3f, %.3f] -> Filtered: [%.3f, %.3f, %.3f]",
                   i, 
                   measurements[i](0), measurements[i](1), measurements[i](2),
                   result(0), result(1), result(2));
    }
    
    // 预测未来状态
    int prediction_steps = 5;
    auto predictions = node->predictFuture(prediction_steps);
    RCLCPP_INFO(node->get_logger(), "Predicted %d future states", prediction_steps);
    
    // 测试模型切换
    RCLCPP_INFO(node->get_logger(), "Testing model switching...");
    node->switchModel("IMM_CV_CA_CS_3Dim");
    
    // 处理更多数据
    auto result = node->processMeasurement(measurements[0]);
    RCLCPP_INFO(node->get_logger(), "New model result: [%.3f, %.3f, %.3f]",
               result(0), result(1), result(2));
    
    rclcpp::shutdown();
    return 0;
}
