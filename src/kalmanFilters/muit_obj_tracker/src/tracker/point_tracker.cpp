#include "muit_obj_tracker/tracker/point_tracker.hpp"
#include "muit_obj_tracker/model/kalman_model.hpp"
#include "muit_obj_tracker/utils/hungarian.hpp"
#include <algorithm>
#include <cmath>

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

void PointTracker::update(const std::vector<Detection>& detections) {
    // 1. Get predicted positions
    std::vector<cv::Point2f> predicted_points;
    std::vector<std::list<Track>::iterator> track_iterators;
    
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        Eigen::VectorXd state = it->model->getState();
        predicted_points.push_back(getCenter(state));
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
        if ((track.time_since_update < 1) && (track.hits >= min_hits || track.age <= min_hits)) {
             TrackResult res;
             res.track_id = track.id;
             res.bbox = track.last_bbox;
             res.state = track.model->getState();
             res.is_active = true;
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
    return cv::Point2f(det.bbox.x + det.bbox.width / 2.0f, det.bbox.y + det.bbox.height / 2.0f);
}

cv::Point2f PointTracker::getCenter(const Eigen::VectorXd& state) {
    float x = 0, y = 0;
    
    // 根据状态向量大小推断滤波器类型并正确提取位置
    // CV_KF (4 状态): [x, vx, y, vy] - y 在索引 2
    // CA_KF/CS_KF/Singer_KF (6 状态): [x, vx, ax, y, vy, ay] - y 在索引 3
    // CTRV_EKF (5 状态): [x, y, v, theta, omega] - y 在索引 1
    
    if (state.size() == 4) {
        // CV_KF: [x, vx, y, vy]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(2));
    } else if (state.size() == 6) {
        // CA_KF/CS_KF/Singer_KF: [x, vx, ax, y, vy, ay]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(3));
    } else if (state.size() == 5) {
        // CTRV_EKF: [x, y, v, theta, omega]
        x = static_cast<float>(state(0));
        y = static_cast<float>(state(1));
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
    
    // 初始化状态
    cv::Point2f center = getCenter(det);
    
    // 根据模型类型设置正确的初始状态位置
    // CV_KF (4 状态): [x, vx, y, vy] - y 在索引 2
    // CA_KF/CS_KF/Singer_KF (6 状态): [x, vx, ax, y, vy, ay] - y 在索引 3
    // CTRV_EKF (5 状态): [x, y, v, theta, omega] - y 在索引 1
    
    if (model_name == "CV_KF" && config.X_0.size() >= 4) {
        config.X_0(0) = center.x;  // x
        config.X_0(2) = center.y;  // y
    } else if ((model_name == "CA_KF" || model_name == "CS_KF" || model_name == "Singer_KF") 
               && config.X_0.size() >= 6) {
        config.X_0(0) = center.x;  // x
        config.X_0(3) = center.y;  // y
    } else if (model_name == "CTRV_EKF" && config.X_0.size() >= 2) {
        config.X_0(0) = center.x;  // x
        config.X_0(1) = center.y;  // y
    } else if (config.X_0.size() >= 2) {
        // 默认情况
        config.X_0(0) = center.x;
        config.X_0(1) = center.y;
    }
    
    // 创建 KalmanModel 包装器
    auto extractor = [](const Detection& d) {
        Eigen::VectorXd Z(2);
        Z << d.bbox.x + d.bbox.width / 2.0, d.bbox.y + d.bbox.height / 2.0;
        return Z;
    };
    
    return std::make_shared<KalmanModel>(model_name, config, extractor);
}

} // namespace muit_obj_tracker
