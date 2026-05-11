#include "armor_pnp_refiner/association/association_gate.hpp"

#include <cmath>

namespace armor_pnp_refiner {

AssociationGate::AssociationGate(const PnpRefinerConfig& config)
  : config_(config) {}

bool AssociationGate::passIoU(const cv::Rect2f& a, const cv::Rect2f& b) const
{
  float inter_x = std::max(0.0f, std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x));
  float inter_y = std::max(0.0f, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
  float inter = inter_x * inter_y;
  float area_a = a.width * a.height;
  float area_b = b.width * b.height;
  float iou = inter / (area_a + area_b - inter + 1e-6f);
  return iou > config_.iou_match_threshold;
}

bool AssociationGate::passCenterDistance(const cv::Point2f& c_a, const cv::Point2f& c_b) const
{
  float dx = c_a.x - c_b.x;
  float dy = c_a.y - c_b.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  return dist < config_.center_distance_threshold_px;
}

bool AssociationGate::passArmorNumber(const std::string& num_a, const std::string& num_b) const
{
  if (!config_.require_same_armor_number) return true;
  return num_a == num_b;
}

bool AssociationGate::passArmorType(ArmorSizeType type_a, ArmorSizeType type_b) const
{
  if (!config_.require_same_armor_type) return true;
  return type_a == type_b;
}

bool AssociationGate::passTimeGap(double stamp_a, double stamp_b) const
{
  double gap_ms = std::abs(stamp_a - stamp_b) * 1000.0;
  return gap_ms < config_.max_time_gap_ms;
}

bool AssociationGate::passPoseDelta(const Eigen::Vector3d& t_a, const Eigen::Vector3d& t_b,
                                     double yaw_a, double yaw_b) const
{
  double pos_delta = (t_a - t_b).norm();
  double yaw_delta = std::abs(std::atan2(std::sin(yaw_a - yaw_b),
                                          std::cos(yaw_a - yaw_b)));
  return pos_delta < config_.max_pose_delta_m &&
         yaw_delta < config_.max_yaw_delta_rad;
}

bool AssociationGate::passAll(const PnpRefineInput& obs_a, const PnpRefineInput& obs_b) const
{
  return passIoU(obs_a.bbox, obs_b.bbox) ||
         passCenterDistance(obs_a.center, obs_b.center);
}

}  // namespace armor_pnp_refiner
