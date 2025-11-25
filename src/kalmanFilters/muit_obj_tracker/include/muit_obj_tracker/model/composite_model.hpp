#ifndef MUIT_OBJ_TRACKER_COMPOSITE_MODEL_HPP
#define MUIT_OBJ_TRACKER_COMPOSITE_MODEL_HPP

#include "muit_obj_tracker/model/imodel.hpp"
#include <vector>
#include <memory>

namespace muit_obj_tracker {

/**
 * @class CompositeModel
 * @brief 复合目标模型。
 * 
 * 由多个子模型（如四个角点滤波器）组成，对外表现为单一模型。
 */
class CompositeModel : public IModel {
public:
    /**
     * @brief 添加子模型。
     * @param model 子模型指针。
     */
    void addSubModel(std::shared_ptr<IModel> model);
    
    /**
     * @brief 执行所有子模型的预测。
     */
    void predict() override;

    /**
     * @brief 更新所有子模型。
     * @param det 检测结果。
     */
    void update(const Detection& det) override;

    /**
     * @brief 获取复合模型的整体状态（如几何中心）。
     * @return 状态向量。
     */
    Eigen::VectorXd getState() const override; // Returns geometric center
    
    /**
     * @brief 获取预测的感兴趣区域（ROI）。
     * @return 预测的2D边界框，用于匹配。
     */
    cv::Rect getPredictedROI() const;          // Provides 2D box for matching

private:
    std::vector<std::shared_ptr<IModel>> sub_models; ///< 子模型列表
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_COMPOSITE_MODEL_HPP
