// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_AMBIGUOUS_SINGLE_ARMOR_FILTER_ADAPTER_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_AMBIGUOUS_SINGLE_ARMOR_FILTER_ADAPTER_HPP_

#include <Eigen/Dense>
#include <memory>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim {

// Forward declaration for legacy KF (always available).
class OutpostAmbiguousKF;

// Forward declaration for IMM core (kalmanFilters layer, optional).
namespace kalman {
class SingleArmorIMMTracker;
struct SingleArmorIMMConfig;
}  // namespace kalman

/// Adapter that provides the same interface as OutpostAmbiguousKF but can
/// internally route to either the legacy KF or the new SingleArmorIMMTracker.
///
/// The routing is controlled by OutpostParameters::ambiguous_backend_use_imm_adapter.
class AmbiguousSingleArmorFilterAdapter {
 public:
  explicit AmbiguousSingleArmorFilterAdapter(const UnifiedConfig &config,
                                              double dt = 0.05);
  ~AmbiguousSingleArmorFilterAdapter();

  void initialize(const ObservationData &obs);
  void predict(double dt);
  void update(const ObservationData &obs,
              double position_confidence = 1.0,
              double yaw_confidence = 1.0);

  bool initialized() const;

  Eigen::Vector3d armor_position() const;
  Eigen::Vector3d armor_velocity() const;
  double armor_yaw() const;
  double armor_yaw_rate() const;

 private:
  void build_imm_config();

  UnifiedConfig config_;
  double dt_ = 0.05;
  bool use_imm_ = false;

  // Legacy KF (always constructed as fallback / reference implementation).
  std::unique_ptr<OutpostAmbiguousKF> legacy_kf_;

  // IMM core (only constructed when use_imm_ is true).
  std::unique_ptr<kalman::SingleArmorIMMTracker> imm_tracker_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_AMBIGUOUS_SINGLE_ARMOR_FILTER_ADAPTER_HPP_
