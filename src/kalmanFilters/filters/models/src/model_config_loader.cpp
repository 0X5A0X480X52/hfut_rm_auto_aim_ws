#include "models/model_config_loader.h"
#include <fstream>
#include <stdexcept>

ModelConfig ModelConfigLoader::loadFromYaml(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    return loadFromYamlNode(config);
}

ModelConfig ModelConfigLoader::loadFromYamlNode(const YAML::Node& node) {
    ModelConfig config;
    
    // 加载基本参数
    if (node["T"]) {
        config.T = node["T"].as<double>();
    }
    
    if (node["Dim"]) {
        config.Dim = node["Dim"].as<int>();
    }
    
    // 加载观测噪声协方差矩阵 R
    if (node["R"]) {
        if (node["R"]["rows"] && node["R"]["cols"]) {
            int rows = node["R"]["rows"].as<int>();
            int cols = node["R"]["cols"].as<int>();
            config.R = loadMatrix(node["R"]["data"], rows, cols);
        }
    }
    
    // 加载初始状态向量 X_0
    if (node["X_0"]) {
        if (node["X_0"]["size"]) {
            int size = node["X_0"]["size"].as<int>();
            config.X_0 = loadVector(node["X_0"]["data"], size);
        }
    }
    
    // 加载观测矩阵 H (用于组合模型)
    if (node["H"]) {
        if (node["H"]["rows"] && node["H"]["cols"]) {
            int rows = node["H"]["rows"].as<int>();
            int cols = node["H"]["cols"].as<int>();
            config.H = loadMatrix(node["H"]["data"], rows, cols);
        }
    }
    
    // 加载转移概率矩阵 (用于 IMM 模型)
    if (node["transform_rate_mat"]) {
        if (node["transform_rate_mat"]["rows"] && node["transform_rate_mat"]["cols"]) {
            int rows = node["transform_rate_mat"]["rows"].as<int>();
            int cols = node["transform_rate_mat"]["cols"].as<int>();
            config.transform_rate_mat = loadMatrix(node["transform_rate_mat"]["data"], rows, cols);
        }
    }
    
    // 加载子模型列表 (用于 IMM 模型)
    if (node["sub_models"]) {
        for (const auto& model : node["sub_models"]) {
            config.sub_models.push_back(model.as<std::string>());
        }
    }
    
    // 加载额外参数
    if (node["extra_params"]) {
        for (const auto& param : node["extra_params"]) {
            config.extra_params[param.first.as<std::string>()] = param.second.as<double>();
        }
    }
    
    return config;
}

std::unique_ptr<Models> ModelConfigLoader::createModelFromYaml(const std::string& config_file) {
    YAML::Node config_node = YAML::LoadFile(config_file);
    
    // 获取模型类型
    if (!config_node["model_type"]) {
        throw std::runtime_error("Configuration file must specify 'model_type'");
    }
    
    std::string model_type = config_node["model_type"].as<std::string>();
    
    // 加载模型配置
    ModelConfig config = loadFromYamlNode(config_node);
    
    // 使用工厂创建模型
    return ModelFactoryRegistry::getInstance().createModel(model_type, config);
}

Eigen::MatrixXd ModelConfigLoader::loadMatrix(const YAML::Node& node, int rows, int cols) {
    Eigen::MatrixXd matrix(rows, cols);
    
    if (node.IsSequence() && node.size() == static_cast<size_t>(rows * cols)) {
        // 数据以一维数组形式存储（行优先）
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = node[i * cols + j].as<double>();
            }
        }
    } else if (node.IsSequence() && node.size() == static_cast<size_t>(rows)) {
        // 数据以二维数组形式存储
        for (int i = 0; i < rows; ++i) {
            if (node[i].size() != static_cast<size_t>(cols)) {
                throw std::runtime_error("Matrix row size mismatch");
            }
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = node[i][j].as<double>();
            }
        }
    } else {
        throw std::runtime_error("Invalid matrix data format");
    }
    
    return matrix;
}

Eigen::VectorXd ModelConfigLoader::loadVector(const YAML::Node& node, int size) {
    Eigen::VectorXd vector(size);
    
    if (!node.IsSequence() || node.size() != static_cast<size_t>(size)) {
        throw std::runtime_error("Vector size mismatch");
    }
    
    for (int i = 0; i < size; ++i) {
        vector(i) = node[i].as<double>();
    }
    
    return vector;
}
