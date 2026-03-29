#include "armor_fusion/clustering_utils.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>

namespace fyt::auto_aim {

Eigen::Vector3d computeClusterCenter(const std::vector<ArmorMeasurement> & cluster)
{
  if (cluster.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto & measurement : cluster) {
    center += measurement.position;
  }
  return center / static_cast<double>(cluster.size());
}

std::vector<std::vector<ArmorMeasurement>> clusterMeasurements(
  const std::vector<ArmorMeasurement> & measurements,
  double eps,
  int min_samples,
  const ClusterMergeConstraint & merge_constraint)
{
  std::vector<std::vector<ArmorMeasurement>> clusters;
  clusters.reserve(measurements.size());

  for (const auto & measurement : measurements) {
    bool assigned = false;
    for (auto & cluster : clusters) {
      const Eigen::Vector3d center = computeClusterCenter(cluster);
      if ((measurement.position - center).norm() <= eps) {
        const Eigen::Vector3d candidate_center =
          (center * static_cast<double>(cluster.size()) + measurement.position) /
          static_cast<double>(cluster.size() + 1U);
        if (merge_constraint && !merge_constraint(measurement, cluster, candidate_center)) {
          continue;
        }
        cluster.push_back(measurement);
        assigned = true;
        break;
      }
    }

    if (!assigned) {
      clusters.push_back({measurement});
    }
  }

  std::vector<std::vector<ArmorMeasurement>> filtered;
  filtered.reserve(clusters.size());
  for (auto & cluster : clusters) {
    if (static_cast<int>(cluster.size()) >= std::max(1, min_samples)) {
      filtered.push_back(cluster);
    }
  }

  return filtered;
}

std::vector<std::vector<ArmorMeasurement>> mergeCloseClusters(
  const std::vector<std::vector<ArmorMeasurement>> & clusters,
  double max_cluster_noise,
  const ClusterPairMergeConstraint & pair_merge_constraint)
{
  if (clusters.size() <= 1U) {
    return clusters;
  }

  std::vector<Eigen::Vector3d> centers;
  centers.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    centers.push_back(computeClusterCenter(cluster));
  }

  std::vector<size_t> parent(clusters.size());
  std::iota(parent.begin(), parent.end(), 0U);

  const auto find_root = [&parent](size_t x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };

  for (size_t i = 0; i < clusters.size(); ++i) {
    for (size_t j = i + 1; j < clusters.size(); ++j) {
      if ((centers[i] - centers[j]).norm() >= max_cluster_noise) {
        continue;
      }
      if (pair_merge_constraint) {
        const double size_i = static_cast<double>(clusters[i].size());
        const double size_j = static_cast<double>(clusters[j].size());
        const double denom = std::max(1.0, size_i + size_j);
        const Eigen::Vector3d merged_center =
          (centers[i] * size_i + centers[j] * size_j) / denom;
        if (!pair_merge_constraint(clusters[i], clusters[j], merged_center)) {
          continue;
        }
      }
      const size_t ri = find_root(i);
      const size_t rj = find_root(j);
      if (ri != rj) {
        parent[rj] = ri;
      }
    }
  }

  std::unordered_map<size_t, std::vector<ArmorMeasurement>> merged_map;
  for (size_t i = 0; i < clusters.size(); ++i) {
    const size_t root = find_root(i);
    auto & merged_cluster = merged_map[root];
    merged_cluster.insert(merged_cluster.end(), clusters[i].begin(), clusters[i].end());
  }

  std::vector<std::vector<ArmorMeasurement>> merged_clusters;
  merged_clusters.reserve(merged_map.size());
  for (auto & kv : merged_map) {
    merged_clusters.push_back(std::move(kv.second));
  }

  return merged_clusters;
}

}  // namespace fyt::auto_aim
