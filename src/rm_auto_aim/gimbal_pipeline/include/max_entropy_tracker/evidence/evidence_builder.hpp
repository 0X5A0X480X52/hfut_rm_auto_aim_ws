// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_BUILDER_HPP_
#define MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_BUILDER_HPP_

#include <memory>
#include <optional>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/evidence/evidence_frame.hpp"
#include "max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.hpp"
#include "max_entropy_tracker/tracking2d/iou_2d_tracker.hpp"

namespace fyt::auto_aim::evidence {

/// Configuration for the evidence builder pipeline.
struct EvidenceBuilderConfig {
  bool enable_2d_tracker = false;
  bool enable_proxy_manager = false;
  IoU2DTrackerConfig iou_2d;
  SingleProxyManagerConfig proxy;
};

/// Orchestrates the full per-frame evidence pipeline:
///
///   ObservationData[] → Armor2DDetection[] → IArmor2DTracker.update()
///     → SingleArmorProxyManager.update() → ArmorEvidenceFrame
///
/// All stages are individually toggleable via config. When a stage is
/// disabled, the corresponding fields in ArmorEvidenceFrame are empty.
class EvidenceBuilder {
 public:
  explicit EvidenceBuilder(const EvidenceBuilderConfig &cfg,
                           const UnifiedConfig &unified);

  /// Run the full pipeline for one frame.
  ArmorEvidenceFrame build(const std::vector<ObservationData> &obs,
                           double timestamp);

  /// Reset all internal tracker / proxy state.
  void reset();

  // Read-only access for external wiring (e.g. debug, manual query).
  const IoUArmor2DTracker *tracker_2d() const { return tracker_2d_.get(); }
  const SingleArmorProxyManager *proxy_manager() const {
    return proxy_manager_.get();
  }

 private:
  static std::vector<Armor2DDetection>
  observations_to_detections(const std::vector<ObservationData> &obs,
                             double timestamp);

  static RelationEvidence
  build_relation(const std::vector<ObservationData> &obs,
                 const ArmorEvidenceFrame &frame);

  EvidenceBuilderConfig cfg_;
  std::unique_ptr<IoUArmor2DTracker> tracker_2d_;
  std::unique_ptr<SingleArmorProxyManager> proxy_manager_;
  std::optional<double> last_timestamp_;
};

}  // namespace fyt::auto_aim::evidence

#endif  // MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_BUILDER_HPP_
