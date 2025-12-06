#include "muit_obj_tracker/tracker/sort_tracker.hpp"
#include "muit_obj_tracker/model/kalman_model.hpp"
#include "muit_obj_tracker/utils/hungarian.hpp"
#include <algorithm>
#include <iostream>

namespace muit_obj_tracker {

SortTracker::SortTracker(int max_age, int min_hits, double iou_threshold, 
                         const std::string& model_name, const ModelConfig& model_config)
    : max_age(max_age), min_hits(min_hits), iou_threshold(iou_threshold), 
      track_id_count(0), model_name(model_name), model_config(model_config) {}

void SortTracker::predict() {
    for (auto& track : tracks) {
        track.model->predict();
        track.age++;
        track.time_since_update++;
    }
}

void SortTracker::update(const std::vector<Detection>& detections) {
    // 1. Predict step for all existing tracks to prepare Kalman priors
    //    Ensures P_prior/X_prior are valid before calling update on each model.
    predict();

    // 2. Get predicted tracks
    std::vector<cv::Rect> predicted_boxes;
    std::vector<std::list<Track>::iterator> track_iterators;
    
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        Eigen::VectorXd state = it->model->getState();
        float x = 0, y = 0;
        if (state.size() >= 3) { // CV_KF Dim=2: x, vx, y, vy
             x = static_cast<float>(state(0));
             y = static_cast<float>(state(2));
        } else if (state.size() >= 2) {
             x = static_cast<float>(state(0));
             y = static_cast<float>(state(1));
        }
        
        float w = it->last_bbox.width;
        float h = it->last_bbox.height;
        
        cv::Rect pred_box(x - w/2, y - h/2, w, h);
        predicted_boxes.push_back(pred_box);
        track_iterators.push_back(it);
    }

    // 2. Compute Cost Matrix (IOU)
    int n_tracks = tracks.size();
    int n_dets = detections.size();
    
    if (n_tracks == 0 && n_dets == 0) return;

    std::vector<std::vector<double>> cost_matrix(n_tracks, std::vector<double>(n_dets));

    for (int i = 0; i < n_tracks; ++i) {
        for (int j = 0; j < n_dets; ++j) {
            double iou = calculateIOU(predicted_boxes[i], detections[j].bbox);
            cost_matrix[i][j] = 1.0 - iou;
        }
    }

    // 3. Solve Assignment
    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    hungarian.Solve(cost_matrix, assignment);

    // 4. Update Tracks
    std::vector<bool> det_matched(n_dets, false);
    std::vector<bool> track_matched(n_tracks, false);

    for (int i = 0; i < n_tracks; ++i) {
        int det_idx = assignment[i];
        if (det_idx != -1) {
            // Check threshold
            if (cost_matrix[i][det_idx] > (1.0 - iou_threshold)) {
                // Rejected
                assignment[i] = -1;
            } else {
                // Matched
                track_matched[i] = true;
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

std::vector<TrackResult> SortTracker::getTracks() const {
    std::vector<TrackResult> results;
    for (const auto& track : tracks) {
        if ((track.time_since_update < 1) && (track.hits >= min_hits || track.age <= min_hits)) {
             TrackResult res;
             res.track_id = track.id;
             
             Eigen::VectorXd state = track.model->getState();
             float x = 0, y = 0;
             if (state.size() >= 3) {
                 x = static_cast<float>(state(0));
                 y = static_cast<float>(state(2));
             } else if (state.size() >= 2) {
                 x = static_cast<float>(state(0));
                 y = static_cast<float>(state(1));
             }
             
             res.bbox = cv::Rect(x - track.last_bbox.width/2, y - track.last_bbox.height/2, track.last_bbox.width, track.last_bbox.height);
             res.state = state;
             res.is_active = true;
             results.push_back(res);
        }
    }
    return results;
}

void SortTracker::reset() {
    tracks.clear();
    track_id_count = 0;
}

double SortTracker::calculateIOU(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    if (x1 >= x2 || y1 >= y2) return 0.0;

    double intersection = (x2 - x1) * (y2 - y1);
    double union_area = box1.area() + box2.area() - intersection;
    
    return intersection / union_area;
}

std::shared_ptr<IModel> SortTracker::createModel(const Detection& det) {
    // 使用配置创建模型
    ModelConfig config = model_config;
    
    // 初始化状态
    // 假设状态向量的前两个元素是 x, y (对于 Dim >= 2)
    // 注意：这里需要根据具体的模型类型来设置初始状态，这可能需要更通用的初始化逻辑
    // 目前简单假设前两个状态是位置
    if (config.X_0.size() >= 2) {
        config.X_0(0) = det.bbox.x + det.bbox.width / 2.0;
        // 假设 y 在索引 1 或 2 (取决于模型定义，例如 CV 模型可能是 x, vx, y, vy)
        // 这里做一个简单的启发式判断，或者依赖配置
        // 为了通用性，我们假设用户在配置文件中正确设置了 X_0 的大小，
        // 但我们需要在这里注入当前的测量值。
        
        // 这是一个简化的假设，实际情况可能需要更复杂的映射
        // 对于 CV_KF (x, vx, y, vy)，y 在索引 2
        if (model_name == "CV_KF" && config.X_0.size() >= 3) {
             config.X_0(2) = det.bbox.y + det.bbox.height / 2.0;
        } else if (config.X_0.size() >= 2) {
             config.X_0(1) = det.bbox.y + det.bbox.height / 2.0;
        }
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
