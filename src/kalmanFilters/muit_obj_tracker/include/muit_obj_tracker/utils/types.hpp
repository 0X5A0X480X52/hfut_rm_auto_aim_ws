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
    cv::Rect bbox;                      ///< 边界框（用于2D图像跟踪）
    Eigen::Vector3d position;           ///< 3D位置（米），用于3D点跟踪
    double yaw;                         ///< 偏航角（弧度）
    float confidence;                   ///< 置信度
    cv::Mat feature;                    ///< 特征向量（可选，用于ReID）
    std::vector<cv::Point2f> corners;   ///< 角点（用于四点模式）
    
    Detection() : id(0), position(Eigen::Vector3d::Zero()), yaw(0.0), confidence(0.0f) {}
};

/**
 * @brief 跟踪结果结构体。
 */
struct TrackResult {
    int track_id;           ///< 跟踪ID
    cv::Rect bbox;          ///< 边界框
    Eigen::VectorXd state;  ///< 状态向量
    bool is_active;         ///< 是否处于活跃状态
    int time_since_update;  ///< 距离上次更新的帧数（0表示刚被更新）
    int hits;               ///< 累计命中次数
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_TYPES_HPP
