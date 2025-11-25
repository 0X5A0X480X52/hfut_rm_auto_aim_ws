#ifndef MUIT_OBJ_TRACKER_POINT_TRACKER_HPP
#define MUIT_OBJ_TRACKER_POINT_TRACKER_HPP

#include "muit_obj_tracker/tracker/itracker.hpp"
#include "muit_obj_tracker/model/imodel.hpp"
#include "models/model_factory.h"
#include <list>
#include <memory>
#include <string>

namespace muit_obj_tracker {

/**
 * @class PointTracker
 * @brief 基于距离匹配的简单跟踪器实现。
 * 
 * 适用于目标仅表现为点（2D或3D坐标）的情况，使用欧氏距离进行数据关联。
 * 相比于 SortTracker，它不计算 IOU，而是直接计算预测位置与检测位置的距离。
 */
class PointTracker : public ITracker {
public:
    /**
     * @brief 构造函数。
     * @param max_age 轨迹最大未更新帧数。
     * @param min_hits 轨迹被确认所需的最小匹配次数。
     * @param distance_threshold 距离匹配阈值，超过此值的匹配将被拒绝。
     * @param model_name 使用的模型名称（默认为 "CV_KF"）。
     * @param model_config 模型配置参数。
     */
    PointTracker(int max_age = 30, int min_hits = 3, double distance_threshold = 100.0,
                 const std::string& model_name = "CV_KF", const ModelConfig& model_config = ModelConfig());
    
    void predict() override;
    void update(const std::vector<Detection>& detections) override;
    std::vector<TrackResult> getTracks() const override;
    void reset() override;

private:
    struct Track {
        int id;
        std::shared_ptr<IModel> model;
        int time_since_update;
        int hits;
        int hit_streak;
        int age;
        cv::Rect last_bbox; // 仍然保留bbox用于输出，但匹配不依赖它
    };

    std::list<Track> tracks;
    int max_age;
    int min_hits;
    double distance_threshold;
    int track_id_count;
    std::string model_name;
    ModelConfig model_config;

    /**
     * @brief 计算两个点之间的欧氏距离。
     */
    double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);

    /**
     * @brief 从检测结果中提取中心点。
     */
    cv::Point2f getCenter(const Detection& det);
    
    /**
     * @brief 从模型状态中提取中心点。
     */
    cv::Point2f getCenter(const Eigen::VectorXd& state);

    std::shared_ptr<IModel> createModel(const Detection& det);
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_POINT_TRACKER_HPP
