// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKING2D_IOU_2D_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKING2D_IOU_2D_TRACKER_HPP_

#include <cstddef>
#include <vector>

#include "max_entropy_tracker/tracking2d/armor_2d_tracker.hpp"
#include "max_entropy_tracker/tracking2d/armor_2d_types.hpp"

namespace fyt::auto_aim {

/// Configuration for IoU-based 2D tracker.
struct IoU2DTrackerConfig {
  double iou_threshold = 0.30;
  int max_center_dist_px = 120;
  int min_hits = 2;
  int max_missed = 15;
  double velocity_smoothing = 0.30;
};

/// Lightweight IOU + center-distance 2D tracker.
///
/// Migrated from armor_detector_nn::InternalIoUTrackerStrategy.
/// Outputs Armor2DTrackEvidence with association_quality,
/// track lifecycle (age/hits/missed/confirmed), and 2D velocity.
class IoUArmor2DTracker : public IArmor2DTracker {
 public:
  explicit IoUArmor2DTracker(const IoU2DTrackerConfig& config);

  std::vector<Armor2DTrackEvidence> update(
      const std::vector<Armor2DDetection>& detections,
      double timestamp) override;

  void reset() override;

 private:
  struct InternalTrack {
    Track2DId id = -1;
    double bbox_x = 0.0;
    double bbox_y = 0.0;
    double bbox_w = 0.0;
    double bbox_h = 0.0;
    double center_x = 0.0;
    double center_y = 0.0;
    int age = 0;
    int hits = 0;
    int missed = 0;
    double last_timestamp = 0.0;
    bool confirmed = false;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
  };

  IoU2DTrackerConfig config_;
  std::vector<InternalTrack> tracks_;
  Track2DId next_id_ = 0;

  static double computeIoU(double ax, double ay, double aw, double ah,
                           double bx, double by, double bw, double bh);

  void predictBbox(InternalTrack& track, double timestamp);

  static std::vector<std::pair<int, int>> hungarianMatch(
      const std::vector<std::vector<double>>& cost_matrix);
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKING2D_IOU_2D_TRACKER_HPP_
