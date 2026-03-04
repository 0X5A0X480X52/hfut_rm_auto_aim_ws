// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_BASE_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_BASE_TRACKER_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim {

enum class TrackerState { INITIALIZING = 0, TRACKING = 1, TEMP_LOST = 2, LOST = 3 };

inline std::string tracker_state_to_string(TrackerState s) {
  switch (s) {
    case TrackerState::INITIALIZING: return "INITIALIZING";
    case TrackerState::TRACKING:     return "TRACKING";
    case TrackerState::TEMP_LOST:    return "TEMP_LOST";
    case TrackerState::LOST:         return "LOST";
  }
  return "UNKNOWN";
}

/// Abstract tracker base (state machine + time sync).
class BaseTracker {
 public:
  explicit BaseTracker(double dt) : dt_(dt) {}
  virtual ~BaseTracker() = default;

  /* ---------- pure virtual ---------- */
  virtual void initialize(const std::vector<ObservationData> &obs,
                          double r1 = 0.15, double r2 = 0.20,
                          double dza = 0.0) = 0;

  virtual void predict(std::optional<double> target_time = std::nullopt) = 0;

  virtual bool update(const std::vector<ObservationData> &obs) = 0;

  virtual Eigen::Vector3d get_center_position() const = 0;
  virtual double get_yaw() const = 0;
  virtual std::pair<double, double> get_radii() const = 0;

  /* ---------- state machine ---------- */
  TrackerState state() const { return state_; }
  bool is_initialized() const { return state_ != TrackerState::INITIALIZING; }
  bool is_tracking() const { return state_ == TrackerState::TRACKING; }
  bool is_temp_lost() const { return state_ == TrackerState::TEMP_LOST; }
  bool is_lost() const { return state_ == TrackerState::LOST; }
  int frame_count() const { return frame_count_; }

  /// Unified process interface: auto-predict then update.
  bool process(const std::vector<ObservationData> &obs,
               std::optional<double> current_time = std::nullopt) {
    double target_time_val = 0.0;
    bool has_target = false;
    if (!obs.empty() && obs[0].timestamp.has_value()) {
      target_time_val = obs[0].timestamp.value();
      has_target = true;
    } else if (current_time.has_value()) {
      target_time_val = current_time.value();
      has_target = true;
    }

    if (!obs.empty()) {
      return update(obs);
    } else {
      if (has_target) predict(target_time_val);
      else predict(std::nullopt);
      handle_observation_loss(2, 8);
      return false;
    }
  }

 protected:
  void transition_to(TrackerState s) {
    if (state_ != s) state_ = s;
  }
  void increment_frame() { ++frame_count_; }

  void handle_observation_loss(int /*tracking_thres*/, int lost_thres) {
    ++lost_count_;
    if (lost_count_ >= lost_thres) transition_to(TrackerState::LOST);
    else if (state_ == TrackerState::TRACKING) transition_to(TrackerState::TEMP_LOST);
  }

  void handle_observation_received(int tracking_thres) {
    lost_count_ = 0;
    if (state_ == TrackerState::TEMP_LOST) transition_to(TrackerState::TRACKING);
    else if (state_ == TrackerState::INITIALIZING && frame_count_ >= tracking_thres)
      transition_to(TrackerState::TRACKING);
  }

  double compute_dt(std::optional<double> target_time) const {
    if (!target_time.has_value() || !current_time_.has_value()) return dt_;
    double d = target_time.value() - current_time_.value();
    if (d < 0) return 0.0;
    return std::clamp(d, min_dt_, max_dt_);
  }

  void update_time(double t) {
    current_time_ = t;
    last_update_time_ = t;
  }

  double dt_;
  TrackerState state_ = TrackerState::INITIALIZING;
  int frame_count_ = 0;
  int lost_count_ = 0;

  std::optional<double> last_update_time_;
  std::optional<double> current_time_;
  double max_dt_ = 0.5;
  double min_dt_ = 0.001;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_BASE_TRACKER_HPP_
