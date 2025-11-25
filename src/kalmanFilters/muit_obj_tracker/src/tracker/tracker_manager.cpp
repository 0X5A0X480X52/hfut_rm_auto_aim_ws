#include "muit_obj_tracker/tracker/tracker_manager.hpp"
#include "muit_obj_tracker/tracker/sort_tracker.hpp"
#include "muit_obj_tracker/tracker/point_tracker.hpp"
#include "models/model_config_loader.h"
#include <stdexcept>

namespace muit_obj_tracker {

TrackerManager::TrackerManager(TrackerType t, const std::string& config_file, const std::string& model_name) 
    : config_file(config_file), model_name(model_name) {
    setTrackerType(t);
}

void TrackerManager::loadConfig(const std::string& config_file, const std::string& model_name) {
    this->config_file = config_file;
    this->model_name = model_name;
    // Re-initialize tracker with new config
    setTrackerType(current_type);
}

void TrackerManager::setTrackerType(TrackerType t) {
    current_type = t;
    
    ModelConfig config;
    if (!config_file.empty()) {
        try {
            config = ModelConfigLoader::loadFromYaml(config_file);
        } catch (...) {
            // Fallback to default config if loading fails
        }
    }

    switch (t) {
        case TrackerType::SORT:
            tracker = std::make_unique<SortTracker>(30, 3, 0.3, model_name, config);
            break;
        case TrackerType::POINT:
            tracker = std::make_unique<PointTracker>(30, 3, 100.0, model_name, config);
            break;
        case TrackerType::DEEPSORT:
            // TODO: Implement DeepSortTracker
            tracker = std::make_unique<SortTracker>(30, 3, 0.3, model_name, config); // Fallback
            break;
        case TrackerType::COMPOSITE:
            // TODO: Implement CompositeTracker
            tracker = std::make_unique<SortTracker>(30, 3, 0.3, model_name, config); // Fallback
            break;
        case TrackerType::KALMANNET:
            // TODO: Implement KalmanNetTracker
            tracker = std::make_unique<SortTracker>(30, 3, 0.3, model_name, config); // Fallback
            break;
        default:
            throw std::invalid_argument("Unknown TrackerType");
    }
}

void TrackerManager::predict() {
    if (tracker) {
        tracker->predict();
    }
}

void TrackerManager::update(const std::vector<Detection>& detections) {
    if (tracker) {
        tracker->update(detections);
    }
}

std::vector<TrackResult> TrackerManager::getTracks() const {
    if (tracker) {
        return tracker->getTracks();
    }
    return {};
}

} // namespace muit_obj_tracker
