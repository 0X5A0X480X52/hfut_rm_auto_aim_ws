#ifndef MUIT_OBJ_TRACKER_SORT_TRACKER_HPP
#define MUIT_OBJ_TRACKER_SORT_TRACKER_HPP

#include "muit_obj_tracker/tracker/itracker.hpp"
#include "muit_obj_tracker/model/imodel.hpp"
#include "models/model_factory.h"
#include <list>
#include <memory>
#include <string>

namespace muit_obj_tracker {

/**
 * @class SortTracker
 * @brief SORT (Simple Online and Realtime Tracking) 算法实现。
 * 
 * 使用卡尔曼滤波器进行状态估计，使用 IOU 进行数据关联。
 */
class SortTracker : public ITracker {
public:
    /**
     * @brief 构造函数。
     * @param max_age 轨迹最大未更新帧数，超过此值将被删除。
     * @param min_hits 轨迹被确认所需的最小匹配次数。
     * @param iou_threshold IOU 匹配阈值，低于此值的匹配将被拒绝。
     * @param model_name 使用的模型名称（默认为 "CV_KF"）。
     * @param model_config 模型配置参数。
     */
    SortTracker(int max_age = 30, int min_hits = 3, double iou_threshold = 0.3, 
                const std::string& model_name = "CV_KF", const ModelConfig& model_config = ModelConfig());
    
    /**
     * @brief 执行预测步骤。
     */
    void predict() override;

    /**
     * @brief 执行更新步骤。
     * @param detections 检测结果列表。
     */
    void update(const std::vector<Detection>& detections) override;

    /**
     * @brief 获取跟踪结果。
     * @return 跟踪结果列表。
     */
    std::vector<TrackResult> getTracks() const override;

    /**
     * @brief 重置跟踪器。
     */
    void reset() override;

private:
    /**
     * @brief 内部轨迹结构体。
     */
    struct Track {
        int id;                         ///< 轨迹ID
        std::shared_ptr<IModel> model;  ///< 跟踪模型
        int time_since_update;          ///< 自上次更新以来的帧数
        int hits;                       ///< 总匹配次数
        int hit_streak;                 ///< 连续匹配次数
        int age;                        ///< 轨迹存活总帧数
        cv::Rect last_bbox;             ///< 上一次的边界框
    };

    std::list<Track> tracks;    ///< 活跃轨迹列表
    int max_age;                ///< 最大存活时间参数
    int min_hits;               ///< 最小命中次数参数
    double iou_threshold;       ///< IOU阈值参数
    int track_id_count;         ///< 轨迹ID计数器
    std::string model_name;     ///< 使用的模型名称
    ModelConfig model_config;   ///< 模型配置

    /**
     * @brief 计算两个矩形的交并比 (IOU)。
     * @param box1 矩形1。
     * @param box2 矩形2。
     * @return IOU 值。
     */
    double calculateIOU(const cv::Rect& box1, const cv::Rect& box2);

    /**
     * @brief 为新的检测创建跟踪模型。
     * @param det 检测结果。
     * @return 新创建的模型指针。
     */
    std::shared_ptr<IModel> createModel(const Detection& det);
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_SORT_TRACKER_HPP
