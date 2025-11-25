#include "muit_obj_tracker/model/composite_model.hpp"
#include <limits>
#include <algorithm>

namespace muit_obj_tracker {

void CompositeModel::addSubModel(std::shared_ptr<IModel> model) {
    sub_models.push_back(model);
}

void CompositeModel::predict() {
    for (auto& model : sub_models) {
        model->predict();
    }
}

void CompositeModel::update(const Detection& det) {
    for (auto& model : sub_models) {
        model->update(det);
    }
}

Eigen::VectorXd CompositeModel::getState() const {
    if (sub_models.empty()) {
        return Eigen::VectorXd();
    }

    // Calculate average state
    Eigen::VectorXd sum_state = sub_models[0]->getState();
    for (size_t i = 1; i < sub_models.size(); ++i) {
        sum_state += sub_models[i]->getState();
    }
    return sum_state / static_cast<double>(sub_models.size());
}

cv::Rect CompositeModel::getPredictedROI() const {
    if (sub_models.empty()) {
        return cv::Rect();
    }

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& model : sub_models) {
        Eigen::VectorXd state = model->getState();
        
        // Assuming CV_KF structure: [pos, vel, pos, vel, ...]
        // x is at index 0, y is at index 2 (if Dim >= 2)
        
        float x = 0.0f;
        float y = 0.0f;

        if (state.size() >= 3) {
             x = static_cast<float>(state(0));
             y = static_cast<float>(state(2));
        } else if (state.size() >= 2) {
             // Fallback for simple [x, y] state
             x = static_cast<float>(state(0));
             y = static_cast<float>(state(1));
        } else if (state.size() == 1) {
             x = static_cast<float>(state(0));
        }

        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    }
    
    return cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
}

} // namespace muit_obj_tracker
