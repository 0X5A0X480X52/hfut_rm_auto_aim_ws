// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TRACKER_HPP_

#include <vector>

#include "max_entropy_tracker/tracking2d/armor_2d_types.hpp"

namespace fyt::auto_aim {

/// Pure 2D image-domain tracker interface.
///
/// Consumes bbox/corners/confidence, produces Track2DId evidence.
/// Does NOT read 3D pose (x/y/z/yaw) and does NOT update single-armor
/// or structured-robot backends.
class IArmor2DTracker {
 public:
  virtual ~IArmor2DTracker() = default;

  virtual std::vector<Armor2DTrackEvidence> update(
      const std::vector<Armor2DDetection>& detections,
      double timestamp) = 0;

  virtual void reset() = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TRACKER_HPP_
