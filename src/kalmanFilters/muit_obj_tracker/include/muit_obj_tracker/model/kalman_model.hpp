#ifndef MUIT_OBJ_TRACKER_KALMAN_MODEL_HPP
#define MUIT_OBJ_TRACKER_KALMAN_MODEL_HPP

#include "muit_obj_tracker/model/imodel.hpp"
#include "models/models.h"
#include "models/model_factory.h"
#include <memory>
#include <functional>
#include <string>

namespace muit_obj_tracker {

/**
 * @class KalmanModel
 * @brief 基于卡尔曼滤波器的模型实现。
 * 
 * 封装了底层的 Models 类，实现了 IModel 接口。
 */
class KalmanModel : public IModel {
public:
    /**
     * @brief 测量值提取器函数类型。
     * 用于从 Detection 结构体中提取卡尔曼滤波器所需的测量向量。
     */
    using MeasurementExtractor = std::function<Eigen::VectorXd(const Detection&)>;

    /**
     * @brief 构造函数。
     * @param kf_impl 底层卡尔曼滤波器实现的共享指针。
     * @param extractor 测量值提取器函数。
     */
    KalmanModel(std::shared_ptr<Models> kf_impl, MeasurementExtractor extractor);

    /**
     * @brief 基于工厂创建模型的构造函数。
     * @param model_name 模型名称（如 "CV_KF"）。
     * @param config 模型配置参数。
     * @param extractor 测量值提取器函数。
     */
    KalmanModel(const std::string& model_name, const ModelConfig& config, MeasurementExtractor extractor);
    
    /**
     * @brief 执行状态预测。
     */
    void predict() override;

    /**
     * @brief 使用检测结果更新状态。
     * @param det 检测结果。
     */
    void update(const Detection& det) override;

    /**
     * @brief 获取当前状态向量。
     * @return 状态向量。
     */
    Eigen::VectorXd getState() const override;

private:
    std::shared_ptr<Models> kf;     ///< 底层卡尔曼滤波器实例
    MeasurementExtractor extractor; ///< 测量值提取器
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_KALMAN_MODEL_HPP
