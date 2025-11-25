#ifndef MUIT_OBJ_TRACKER_IMODEL_HPP
#define MUIT_OBJ_TRACKER_IMODEL_HPP

#include <Eigen/Dense>
#include "muit_obj_tracker/utils/types.hpp"

namespace muit_obj_tracker {

/**
 * @class IModel
 * @brief 跟踪模型的抽象接口类。
 * 
 * 定义了所有跟踪模型必须实现的通用接口，包括预测、更新和获取状态。
 */
class IModel {
public:
    virtual ~IModel() = default;

    /**
     * @brief 执行状态预测。
     */
    virtual void predict() = 0;

    /**
     * @brief 使用检测结果更新状态。
     * @param det 检测结果。
     */
    virtual void update(const Detection& det) = 0;

    /**
     * @brief 获取当前状态向量。
     * @return 状态向量。
     */
    virtual Eigen::VectorXd getState() const = 0;
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_IMODEL_HPP
