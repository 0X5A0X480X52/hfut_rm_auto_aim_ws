#include "muit_obj_tracker/tracker/point_tracker.hpp"
#include "muit_obj_tracker/model/kalman_model.hpp"
#include "muit_obj_tracker/utils/hungarian.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace muit_obj_tracker {

PointTracker::PointTracker(int max_age, int min_hits, double distance_threshold,
                           const std::string& model_name, const ModelConfig& model_config)
    : max_age(max_age), min_hits(min_hits), distance_threshold(distance_threshold), 
      track_id_count(0), model_name(model_name), model_config(model_config) {}

void PointTracker::predict() {
    for (auto& track : tracks) {
        track.model->predict();
        track.age++;
        track.time_since_update++;
    }
}

// 定义调试宏，可以通过编译选项控制
#ifndef POINT_TRACKER_DEBUG
#define POINT_TRACKER_DEBUG 0
#endif

void PointTracker::update(const std::vector<Detection>& detections) {
#if POINT_TRACKER_DEBUG
    std::cerr << "[PointTracker::update] n_tracks=" << tracks.size() 
              << ", n_dets=" << detections.size() 
              << ", distance_threshold=" << distance_threshold << std::endl;
#endif
    
    // 0. First, run predict step for all existing tracks to ensure Kalman state is prepared
    //    This ensures P_prior/X_prior are valid before calling performUpdate on each model.
    predict();

    // 1. Get predicted positions
    std::vector<cv::Point2f> predicted_points;
    std::vector<std::list<Track>::iterator> track_iterators;
    
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        Eigen::VectorXd state = it->model->getState();
        cv::Point2f center = getCenter(state);
        predicted_points.push_back(center);
        track_iterators.push_back(it);
    }

    // 2. Compute Cost Matrix (Distance)
    int n_tracks = tracks.size();
    int n_dets = detections.size();
    
    // Handle empty cases
    if (n_tracks == 0) {
        for (const auto& det : detections) {
            Track new_track;
            new_track.id = ++track_id_count;
            new_track.model = createModel(det);
            new_track.time_since_update = 0;
            new_track.hits = 1;
            new_track.hit_streak = 1;
            new_track.age = 1;
            new_track.last_bbox = det.bbox;
            tracks.push_back(new_track);
        }
        return;
    }

    if (n_dets == 0) {
        for (auto it = tracks.begin(); it != tracks.end(); ) {
            if (it->time_since_update > max_age) {
                it = tracks.erase(it);
            } else {
                ++it;
            }
        }
        return;
    }

    std::vector<std::vector<double>> cost_matrix(n_tracks, std::vector<double>(n_dets));

    for (int i = 0; i < n_tracks; ++i) {
        for (int j = 0; j < n_dets; ++j) {
            double dist = calculateDistance(predicted_points[i], getCenter(detections[j]));
            cost_matrix[i][j] = dist;
        }
    }

    // 3. Solve Assignment
    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    hungarian.Solve(cost_matrix, assignment);

    // 4. Update Tracks
    std::vector<bool> det_matched(n_dets, false);
    
    for (int i = 0; i < n_tracks; ++i) {
        int det_idx = assignment[i];
        if (det_idx != -1) {
            // Check threshold
            if (cost_matrix[i][det_idx] > distance_threshold) {
                // Rejected (Too far)
                assignment[i] = -1;
            } else {
                // Matched
                det_matched[det_idx] = true;
                
                auto track_it = track_iterators[i];
                track_it->model->update(detections[det_idx]);
                track_it->hits++;
                track_it->hit_streak++;
                track_it->time_since_update = 0;
                track_it->last_bbox = detections[det_idx].bbox;
            }
        }
    }

    // 5. Create new tracks
    for (int i = 0; i < n_dets; ++i) {
        if (!det_matched[i]) {
            Track new_track;
            new_track.id = ++track_id_count;
            new_track.model = createModel(detections[i]);
            new_track.time_since_update = 0;
            new_track.hits = 1;
            new_track.hit_streak = 1;
            new_track.age = 1;
            new_track.last_bbox = detections[i].bbox;
            tracks.push_back(new_track);
        }
    }

    // 6. Remove dead tracks
    for (auto it = tracks.begin(); it != tracks.end(); ) {
        if (it->time_since_update > max_age) {
            it = tracks.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<TrackResult> PointTracker::getTracks() const {
    std::vector<TrackResult> results;
    for (const auto& track : tracks) {
        // 返回条件：
        // 1. time_since_update <= max_age (跟踪还未超时)
        // 2. hits >= min_hits (已达到稳定跟踪) 或 age <= min_hits (新跟踪处于初始化阶段)
        if (track.time_since_update <= max_age && 
            (track.hits >= min_hits || track.age <= min_hits)) {
             TrackResult res;
             res.track_id = track.id;
             res.bbox = track.last_bbox;
             res.state = track.model->getState();
             res.is_active = (track.hits >= min_hits);  // 仅达到min_hits时标记为active
             res.time_since_update = track.time_since_update;  // 用于判断是否刚被更新
             res.hits = track.hits;
             results.push_back(res);
        }
    }
    return results;
}

void PointTracker::reset() {
    tracks.clear();
    track_id_count = 0;
}

double PointTracker::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

cv::Point2f PointTracker::getCenter(const Detection& det) {
    // 对于3D跟踪，直接使用 Detection.position 字段（单位：米）
    // 返回 (x, y) 作为2D中心用于匹配
    return cv::Point2f(static_cast<float>(det.position.x()), static_cast<float>(det.position.y()));
}

cv::Point2f PointTracker::getCenter(const Eigen::VectorXd& state) {
    float x = 0, y = 0;
    
    // 根据状态向量大小推断滤波器类型并正确提取位置
    // 注意：这里的 y 坐标指的是状态向量中的第二个空间维度，不是状态索引
    // 
    // 3D CV_KF (6 状态): [x, vx, y, vy, z, vz] - x 在索引 0, y 在索引 2
    // 3D CA_KF/CS_KF/Singer_KF (9 状态): [x, vx, ax, y, vy, ay, z, vz, az] - x 在索引 0, y 在索引 3
    // 2D CV_KF (4 状态): [x, vx, y, vy] - x 在索引 0, y 在索引 2
    // 2D CA_KF/CS_KF/Singer_KF (6 状态): [x, vx, ax, y, vy, ay] - x 在索引 0, y 在索引 3
    // CTRV_EKF (5 状态): [x, y, v, theta, omega] - x 在索引 0, y 在索引 1
    
    if (state.size() == 9) {
        // 3D CA_KF/CS_KF/Singer_KF: [x, vx, ax, y, vy, ay, z, vz, az]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(3));
    } else if (state.size() == 6) {
        // 3D CV_KF: [x, vx, y, vy, z, vz] 或 2D CA_KF: [x, vx, ax, y, vy, ay]
        // 需要根据模型名称区分，这里默认按3D CV_KF处理
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(2));  // 3D CV_KF 情况
    } else if (state.size() == 5) {
        // CTRV_EKF: [x, y, v, theta, omega]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(1));
    } else if (state.size() == 4) {
        // 2D CV_KF: [x, vx, y, vy]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(2));
    } else if (state.size() >= 2) {
        // 默认情况：假设 [x, y, ...]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(1));
    }
    
    return cv::Point2f(x, y);
}

std::shared_ptr<IModel> PointTracker::createModel(const Detection& det) {
    // 使用配置创建模型
    ModelConfig config = model_config;
    
    // 初始化状态 - 直接使用米作为单位
    double x_pos = det.position.x();
    double y_pos = det.position.y();
    double z_pos = det.position.z();
    
    // 根据模型类型设置正确的初始状态位置
    // CV_KF (6 状态 for 3D): [x, vx, y, vy, z, vz] - positions at 0,2,4
    // CA_KF/CS_KF/Singer_KF (9 状态 for 3D): [x, vx, ax, y, vy, ay, z, vz, az] - positions at 0,3,6
    // CTRV_EKF (5 状态): [x, y, v, theta, omega] - positions at 0,1
    
    if (model_name == "CV_KF" && config.X_0.size() >= 6) {
        config.X_0(0) = x_pos;  // x
        config.X_0(2) = y_pos;  // y
        config.X_0(4) = z_pos;  // z
    } else if ((model_name == "CA_KF" || model_name == "CS_KF" || model_name == "Singer_KF") 
               && config.X_0.size() >= 9) {
        config.X_0(0) = x_pos;  // x
        config.X_0(3) = y_pos;  // y
        config.X_0(6) = z_pos;  // z
    } else if (model_name == "CTRV_EKF" && config.X_0.size() >= 2) {
        config.X_0(0) = x_pos;  // x
        config.X_0(1) = y_pos;  // y
    } else if (config.X_0.size() >= 3) {
        // 默认3D情况
        config.X_0(0) = x_pos;
        config.X_0(1) = y_pos;
        config.X_0(2) = z_pos;
    } else if (config.X_0.size() >= 2) {
        // 默认2D情况
        config.X_0(0) = x_pos;
        config.X_0(1) = y_pos;
    }
    
    // 创建 KalmanModel 包装器
    // extractor 用于从 Detection 中提取观测值（单位：米）
    // 根据模型类型确定观测维度
    auto extractor = [this](const Detection& d) -> Eigen::VectorXd {
        if (this->model_name == "CTRV_EKF") {
            // CTRV_EKF 只观测 x,y (2D)
            Eigen::VectorXd Z(2);
            Z << d.position.x(), d.position.y();
            return Z;
        } else {
            // 其他3D模型观测 x,y,z (3D)
            Eigen::VectorXd Z(3);
            Z << d.position.x(), d.position.y(), d.position.z();
            return Z;
        }
    };
    
    return std::make_shared<KalmanModel>(model_name, config, extractor);
}

} // namespace muit_obj_tracker
