#include "armor_fusion/fusion_utils.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/quaternion.hpp>

namespace fyt::auto_aim {

namespace {
constexpr double kEpsilon = 1e-9;

std::string modeString(const std::vector<std::string> & values)
{
  if (values.empty()) {
    return "";
  }

  std::unordered_map<std::string, size_t> counts;
  counts.reserve(values.size());
  for (const auto & value : values) {
    counts[value]++;
  }

  size_t max_count = 0;
  std::string mode_value = values.front();
  for (const auto & item : counts) {
    if (item.second > max_count) {
      max_count = item.second;
      mode_value = item.first;
    }
  }
  return mode_value;
}

geometry_msgs::msg::Quaternion averageQuaternion(const std::vector<Eigen::Quaterniond> & quaternions)
{
  geometry_msgs::msg::Quaternion result;
  result.w = 1.0;

  if (quaternions.empty()) {
    return result;
  }

  Eigen::Vector4d sum = Eigen::Vector4d::Zero();
  Eigen::Vector4d reference(
    quaternions.front().x(),
    quaternions.front().y(),
    quaternions.front().z(),
    quaternions.front().w());

  for (const auto & quaternion : quaternions) {
    Eigen::Vector4d coeffs(
      quaternion.x(),
      quaternion.y(),
      quaternion.z(),
      quaternion.w());
    if (reference.dot(coeffs) < 0.0) {
      coeffs = -coeffs;
    }
    sum += coeffs;
  }

  if (sum.norm() < kEpsilon) {
    return result;
  }

  sum.normalize();
  result.x = sum.x();
  result.y = sum.y();
  result.z = sum.z();
  result.w = sum.w();
  return result;
}
}  // namespace

rm_interfaces::msg::Armor fuseCluster(const std::vector<ArmorMeasurement> & cluster)
{
  rm_interfaces::msg::Armor fused_armor;

  if (cluster.empty()) {
    fused_armor.number = "";
    fused_armor.type = "";
    fused_armor.distance_to_image_center = 1.0F;
    fused_armor.pose.orientation.w = 1.0;
    return fused_armor;
  }

  std::vector<std::string> numbers;
  std::vector<std::string> types;
  std::vector<Eigen::Quaterniond> quaternions;
  numbers.reserve(cluster.size());
  types.reserve(cluster.size());
  quaternions.reserve(cluster.size());

  // Weighted average uses a distance-based uncertainty model to keep behavior
  // close to the current Python implementation.
  Eigen::Vector3d fused_position = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;
  double best_distance_to_center = 1.0;

  for (const auto & measurement : cluster) {
    numbers.push_back(measurement.number);
    types.push_back(measurement.armor_type);
    quaternions.push_back(measurement.orientation);

    const double sigma = 0.01 + 0.001 * measurement.position.norm();
    const double weight = 1.0 / std::max(kEpsilon, 3.0 * sigma * sigma);
    fused_position += weight * measurement.position;
    weight_sum += weight;

    best_distance_to_center = std::min(
      best_distance_to_center,
      std::clamp(measurement.distance_to_image_center, 0.0, 1.0));
  }

  if (weight_sum > kEpsilon) {
    fused_position /= weight_sum;
  } else {
    // Degenerate case: keep a safe fallback instead of zero vector.
    fused_position = cluster.front().position;
  }

  fused_armor.number = modeString(numbers);
  fused_armor.type = modeString(types);
  fused_armor.distance_to_image_center = static_cast<float>(best_distance_to_center);

  fused_armor.pose.position.x = fused_position.x();
  fused_armor.pose.position.y = fused_position.y();
  fused_armor.pose.position.z = fused_position.z();
  fused_armor.pose.orientation = averageQuaternion(quaternions);

  // 保留来源相机信息（取 cluster 中出现最多的 source_frame）
  std::vector<std::string> source_frames;
  source_frames.reserve(cluster.size());
  for (const auto & measurement : cluster) {
    source_frames.push_back(measurement.source_frame);
  }
  fused_armor.source_frame = modeString(source_frames);

  return fused_armor;
}

}  // namespace fyt::auto_aim
