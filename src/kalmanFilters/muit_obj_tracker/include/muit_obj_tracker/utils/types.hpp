#ifndef MUIT_OBJ_TRACKER_TYPES_HPP
#define MUIT_OBJ_TRACKER_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

namespace muit_obj_tracker {

/**
 * @brief 检测结果结构体。
 */
struct Detection {
    int id;                             ///< 检测ID
    cv::Rect bbox;                      ///< 边界框
    float confidence;                   ///< 置信度
    cv::Mat feature;                    ///< 特征向量（可选，用于ReID）
    std::vector<cv::Point2f> corners;   ///< 角点（用于四点模式）
};

/**
 * @brief 跟踪结果结构体。
 */
struct TrackResult {
    int track_id;           ///< 跟踪ID
    cv::Rect bbox;          ///< 边界框
    Eigen::VectorXd state;  ///< 状态向量
    bool is_active;         ///< 是否处于活跃状态
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_TYPES_HPP
