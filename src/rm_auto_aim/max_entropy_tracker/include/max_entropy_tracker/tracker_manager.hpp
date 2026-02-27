// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"

namespace fyt::auto_aim {

/// Manages per-robot AdaptiveArmorTracker instances.
class TrackerManager {
 public:
  struct TrackerEntry {
    std::unique_ptr<AdaptiveArmorTracker> tracker;
    double last_update_time = 0.0;
    int observation_count = 0;
  };

  explicit TrackerManager(const UnifiedConfig &config, double dt = 0.01,
                          double default_r1 = 0.15, double default_r2 = 0.20,
                          double default_dza = 0.0,
                          double timeout_seconds = 3.0,
                          bool enable_oscillation = false)
      : config_(config),
        dt_(dt),
        default_r1_(default_r1),
        default_r2_(default_r2),
        default_dza_(default_dza),
        timeout_(timeout_seconds),
        enable_osc_(enable_oscillation) {}

  /// Get existing tracker or create+initialize a new one.
  AdaptiveArmorTracker *get_or_create(
      const std::string &robot_id,
      const std::vector<ObservationData> &initial_obs = {}) {
    auto it = trackers_.find(robot_id);
    if (it != trackers_.end()) return it->second.tracker.get();
    if (initial_obs.empty()) return nullptr;

    auto t = std::make_unique<AdaptiveArmorTracker>(config_, dt_, enable_osc_);
    t->initialize(initial_obs, default_r1_, default_r2_, default_dza_);

    auto *ptr = t.get();
    TrackerEntry entry;
    entry.tracker = std::move(t);
    entry.last_update_time = now();
    entry.observation_count = static_cast<int>(initial_obs.size());
    trackers_[robot_id] = std::move(entry);
    return ptr;
  }

  /// Update a robot's tracker; auto-creates if needed.
  bool update(const std::string &robot_id,
              const std::vector<ObservationData> &obs,
              double current_time = -1.0) {
    if (obs.empty()) return false;
    double t = (current_time < 0) ? now() : current_time;

    auto it = trackers_.find(robot_id);
    if (it == trackers_.end()) {
      return get_or_create(robot_id, obs) != nullptr;
    }

    bool ok = it->second.tracker->update(obs);
    if (ok) {
      it->second.last_update_time = t;
      it->second.observation_count += static_cast<int>(obs.size());
    }
    return ok;
  }

  /// Predict all active trackers to target_time.
  void predict_all(std::optional<double> target_time = std::nullopt) {
    for (auto &[id, entry] : trackers_) {
      if (entry.tracker->is_initialized())
        entry.tracker->predict(target_time);
    }
  }

  /// Remove trackers not updated within timeout.
  std::vector<std::string> remove_stale(double current_time = -1.0) {
    double t = (current_time < 0) ? now() : current_time;
    std::vector<std::string> removed;
    for (auto it = trackers_.begin(); it != trackers_.end();) {
      if (t - it->second.last_update_time > timeout_) {
        removed.push_back(it->first);
        it = trackers_.erase(it);
      } else {
        ++it;
      }
    }
    return removed;
  }

  AdaptiveArmorTracker *get(const std::string &id) {
    auto it = trackers_.find(id);
    return (it != trackers_.end()) ? it->second.tracker.get() : nullptr;
  }

  const std::unordered_map<std::string, TrackerEntry> &trackers() const {
    return trackers_;
  }

  std::vector<std::string> tracking_robot_ids() const {
    std::vector<std::string> ids;
    for (const auto &[id, e] : trackers_)
      if (e.tracker->is_tracking()) ids.push_back(id);
    return ids;
  }

  int num_trackers() const { return static_cast<int>(trackers_.size()); }
  void clear() { trackers_.clear(); }

 private:
  static double now() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
               steady_clock::now().time_since_epoch())
        .count();
  }

  UnifiedConfig config_;
  double dt_, default_r1_, default_r2_, default_dza_, timeout_;
  bool enable_osc_;
  std::unordered_map<std::string, TrackerEntry> trackers_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_
