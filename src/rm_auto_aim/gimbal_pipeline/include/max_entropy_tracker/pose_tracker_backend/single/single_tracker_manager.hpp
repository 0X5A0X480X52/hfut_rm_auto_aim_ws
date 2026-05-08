// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_MANAGER_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_MANAGER_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/ids.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.hpp"
#include "max_entropy_tracker/pose_tracker_backend/single/single_tracker_types.hpp"
#include "max_entropy_tracker/tracking2d/armor_2d_types.hpp"

namespace fyt::auto_aim {

/// Configuration for SingleArmorProxyManager.
struct SingleProxyManagerConfig {
  int max_lost_frames = 15;
  int kin_summary_window_size = 8;
  double yaw_rate_jump_gate = 2.0;
};

/// Manages one AmbiguousSingleArmorFilterAdapter per Track2DId.
///
/// Consumes 3D observations + 2D track evidence, routes observations to
/// the correct proxy filter by track2d_id, and exports per-proxy
/// SingleArmorTrackEvidence including kinematic summaries.
class SingleArmorProxyManager {
 public:
  explicit SingleArmorProxyManager(const SingleProxyManagerConfig &cfg,
                                   const UnifiedConfig &unified);

  /// Predict all alive proxies by dt.
  void predict_all(double dt);

  /// Route observations to matching proxies via track2d_id.
  /// Creates new proxies for previously unseen track2d_ids.
  void update(const std::vector<ObservationData> &obs,
              const std::vector<Armor2DTrackEvidence> &track_evidences);

  /// Remove proxies whose missed count exceeds max_lost_frames.
  std::vector<SingleArmorTrackEvidence> prune_lost();

  /// Export evidence for all currently alive proxies.
  std::vector<SingleArmorTrackEvidence> collect_evidence() const;

  /// Number of alive proxies.
  int size() const { return static_cast<int>(proxies_.size()); }

  /// Reset all state.
  void reset();

 private:
  struct ProxyEntry {
    Track2DId track2d_id = -1;
    Track3DId proxy_id = -1;
    std::unique_ptr<AmbiguousSingleArmorFilterAdapter> filter;
    int age = 0;
    int hits = 0;
    int missed = 0;
    int panel_id = -1;

    // Rolling windows for kinematic summary.
    std::deque<Eigen::Vector3d> vel_history;
    std::deque<double> yaw_rate_history;
    Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();
  };

  void refresh_kinematic_summary(ProxyEntry &entry);
  static double compute_variance(const std::deque<Eigen::Vector3d> &vals);
  static double compute_variance(const std::deque<double> &vals);

  SingleProxyManagerConfig cfg_;
  UnifiedConfig unified_cfg_;

  std::unordered_map<Track2DId, ProxyEntry> proxies_;
  Track3DId next_proxy_id_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_MANAGER_HPP_
