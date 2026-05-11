#pragma once

#include <vector>
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/association/association_gate.hpp"

namespace armor_pnp_refiner {

// Lightweight short-term associator for PnP refinement windows.
// Generates refine_track_id (not system-level target id).
// Uses external_track_id when available, otherwise runs internal IoU+center matching.
class ShortTermAssociator {
public:
  explicit ShortTermAssociator(const PnpRefinerConfig& config);

  // Associate current observation to a refine track.
  // Returns refine_track_id (>0 matched, -1 new track needed).
  int associate(const PnpRefineInput& input);

  // Reset all internal tracks.
  void reset();

  // Prune expired tracks.
  void pruneExpired(double now_sec);

private:
  struct InternalTrack {
    int refine_track_id{-1};
    cv::Rect2f last_bbox{};
    cv::Point2f last_center{};
    Eigen::Vector3d last_t{Eigen::Vector3d::Zero()};
    double last_yaw{0.0};
    double last_stamp_sec{0.0};
    ArmorSizeType armor_type{ArmorSizeType::SMALL};
    std::string armor_number;
    int hits{0};
    int misses{0};
  };

  int next_id_{1};
  PnpRefinerConfig config_;
  AssociationGate gate_;
  std::vector<InternalTrack> tracks_;

  double computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) const;
  double computeAssociationScore(const InternalTrack& track,
                                  const PnpRefineInput& obs) const;
};

}  // namespace armor_pnp_refiner
