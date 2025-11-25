#ifndef MUIT_OBJ_TRACKER_ITRACKER_HPP
#define MUIT_OBJ_TRACKER_ITRACKER_HPP

#include "muit_obj_tracker/utils/types.hpp"
#include <vector>

namespace muit_obj_tracker {

/**
 * @class ITracker
 * @brief 多目标跟踪器的抽象接口类。
 * 
 * 定义了所有跟踪器必须实现的通用接口，包括预测、更新、获取结果和重置。
 */
class ITracker {
public:
    virtual ~ITracker() = default;

    /**
     * @brief 执行跟踪器的预测步骤。
     * 对所有活跃的轨迹进行状态预测。
     */
    virtual void predict() = 0;

    /**
     * @brief 使用当前帧的检测结果更新跟踪器。
     * 包括数据关联、状态更新、轨迹管理（创建、删除）。
     * @param detections 当前帧的检测结果列表。
     */
    virtual void update(const std::vector<Detection>& detections) = 0;

    /**
     * @brief 获取当前的跟踪结果。
     * @return 跟踪结果列表。
     */
    virtual std::vector<TrackResult> getTracks() const = 0;

    /**
     * @brief 重置跟踪器状态。
     * 清除所有轨迹和计数器。
     */
    virtual void reset() = 0;
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_ITRACKER_HPP
