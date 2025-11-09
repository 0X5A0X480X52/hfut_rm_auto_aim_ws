#ifndef MODEL_FACTORY_H
#define MODEL_FACTORY_H

#include "models/models.h"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <functional>

/**
 * @struct ModelConfig
 * @brief 模型配置参数结构体
 */
struct ModelConfig {
    double T;                           ///< 采样时间
    int Dim;                            ///< 状态维度
    Eigen::MatrixXd R;                  ///< 观测噪声协方差矩阵
    Eigen::VectorXd X_0;                ///< 初始状态向量
    
    // 用于组合模型的额外参数
    std::vector<std::string> sub_models;      ///< 子模型列表
    Eigen::MatrixXd H;                        ///< 观测矩阵
    Eigen::MatrixXd transform_rate_mat;       ///< 转移概率矩阵
    
    // 用于特定模型的额外参数
    std::map<std::string, double> extra_params;  ///< 额外参数映射
    
    ModelConfig() 
        : T(0.01), Dim(3), 
          R(Eigen::MatrixXd::Identity(3, 3)),
          X_0(Eigen::VectorXd::Zero(9)) {}
};

/**
 * @class ModelFactoryBase
 * @brief 模型工厂基类，使用工厂方法模式
 */
class ModelFactoryBase {
public:
    virtual ~ModelFactoryBase() = default;
    
    /**
     * @brief 创建模型实例
     * @param config 模型配置参数
     * @return 模型指针
     */
    virtual std::unique_ptr<Models> createModel(const ModelConfig& config) const = 0;
    
    /**
     * @brief 获取工厂名称
     * @return 工厂名称
     */
    virtual std::string getFactoryName() const = 0;
};

/**
 * @class ModelFactoryRegistry
 * @brief 模型工厂注册器，管理所有模型工厂
 */
class ModelFactoryRegistry {
public:
    /**
     * @brief 获取单例实例
     * @return 工厂注册器单例
     */
    static ModelFactoryRegistry& getInstance();
    
    /**
     * @brief 注册模型工厂
     * @param name 模型名称
     * @param factory 工厂指针
     */
    void registerFactory(const std::string& name, std::shared_ptr<ModelFactoryBase> factory);
    
    /**
     * @brief 创建模型实例
     * @param name 模型名称
     * @param config 模型配置参数
     * @return 模型指针
     */
    std::unique_ptr<Models> createModel(const std::string& name, const ModelConfig& config);
    
    /**
     * @brief 检查模型是否已注册
     * @param name 模型名称
     * @return 是否已注册
     */
    bool hasFactory(const std::string& name) const;
    
    /**
     * @brief 获取所有已注册的模型名称
     * @return 模型名称列表
     */
    std::vector<std::string> getRegisteredFactories() const;
    
private:
    ModelFactoryRegistry() = default;
    ~ModelFactoryRegistry() = default;
    ModelFactoryRegistry(const ModelFactoryRegistry&) = delete;
    ModelFactoryRegistry& operator=(const ModelFactoryRegistry&) = delete;
    
    std::map<std::string, std::shared_ptr<ModelFactoryBase>> factories_;
};

/**
 * @class AutoRegisterFactory
 * @brief 自动注册工厂辅助类
 */
class AutoRegisterFactory {
public:
    AutoRegisterFactory(const std::string& name, std::shared_ptr<ModelFactoryBase> factory) {
        ModelFactoryRegistry::getInstance().registerFactory(name, factory);
    }
};

// 宏定义用于简化工厂注册
#define REGISTER_MODEL_FACTORY(FactoryClass, ModelName) \
    static AutoRegisterFactory auto_register_##FactoryClass( \
        ModelName, std::make_shared<FactoryClass>())

#endif // MODEL_FACTORY_H
