#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

// Gate checks for short-term association candidate pairs.
class AssociationGate {
public:
  explicit AssociationGate(const PnpRefinerConfig& config);

  // Check if two observations can be associated.
  bool passIoU(const cv::Rect2f& bbox_a, const cv::Rect2f& bbox_b) const;
  bool passCenterDistance(const cv::Point2f& c_a, const cv::Point2f& c_b) const;
  bool passArmorNumber(const std::string& num_a, const std::string& num_b) const;
  bool passArmorType(ArmorSizeType type_a, ArmorSizeType type_b) const;
  bool passTimeGap(double stamp_a, double stamp_b) const;
  bool passPoseDelta(const Eigen::Vector3d& t_a, const Eigen::Vector3d& t_b,
                     double yaw_a, double yaw_b) const;

  // Combined gate.
  bool passAll(const PnpRefineInput& obs_a, const PnpRefineInput& obs_b) const;

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
