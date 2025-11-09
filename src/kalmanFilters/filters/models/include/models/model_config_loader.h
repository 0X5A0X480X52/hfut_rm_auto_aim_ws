#ifndef MODEL_CONFIG_LOADER_H
#define MODEL_CONFIG_LOADER_H

#include "models/model_factory.h"
#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @class ModelConfigLoader
 * @brief 从配置文件加载模型配置的工具类
 */
class ModelConfigLoader {
public:
    /**
     * @brief 从 YAML 文件加载模型配置
     * @param config_file 配置文件路径
     * @return 模型配置
     */
    static ModelConfig loadFromYaml(const std::string& config_file);
    
    /**
     * @brief 从 YAML 节点加载模型配置
     * @param node YAML 节点
     * @return 模型配置
     */
    static ModelConfig loadFromYamlNode(const YAML::Node& node);
    
    /**
     * @brief 从配置文件创建模型
     * @param config_file 配置文件路径
     * @return 模型指针
     */
    static std::unique_ptr<Models> createModelFromYaml(const std::string& config_file);
    
private:
    /**
     * @brief 从 YAML 节点加载矩阵
     * @param node YAML 节点
     * @param rows 行数
     * @param cols 列数
     * @return Eigen 矩阵
     */
    static Eigen::MatrixXd loadMatrix(const YAML::Node& node, int rows, int cols);
    
    /**
     * @brief 从 YAML 节点加载向量
     * @param node YAML 节点
     * @param size 向量大小
     * @return Eigen 向量
     */
    static Eigen::VectorXd loadVector(const YAML::Node& node, int size);
};

#endif // MODEL_CONFIG_LOADER_H
