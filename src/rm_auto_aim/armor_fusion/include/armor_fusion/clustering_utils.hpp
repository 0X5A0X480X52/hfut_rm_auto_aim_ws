#ifndef ARMOR_FUSION__CLUSTERING_UTILS_HPP_
#define ARMOR_FUSION__CLUSTERING_UTILS_HPP_

#include <functional>
#include <vector>

#include <Eigen/Core>

#include "armor_fusion/measurement_types.hpp"

namespace fyt::auto_aim {

// Compute arithmetic center for a cluster.
Eigen::Vector3d computeClusterCenter(const std::vector<ArmorMeasurement> & cluster);

// Optional callback to veto cluster assignment under additional constraints.
using ClusterMergeConstraint = std::function<bool(
  const ArmorMeasurement & candidate,
  const std::vector<ArmorMeasurement> & cluster,
  const Eigen::Vector3d & candidate_center)>;

// Optional callback to veto merging two clusters under additional constraints.
using ClusterPairMergeConstraint = std::function<bool(
  const std::vector<ArmorMeasurement> & cluster_a,
  const std::vector<ArmorMeasurement> & cluster_b,
  const Eigen::Vector3d & merged_center)>;

// Greedy radius clustering used as a lightweight fallback in C++ node.
std::vector<std::vector<ArmorMeasurement>> clusterMeasurements(
  const std::vector<ArmorMeasurement> & measurements,
  double eps,
  int min_samples,
  const ClusterMergeConstraint & merge_constraint = ClusterMergeConstraint());

// Merge cluster pairs whose centers are close enough.
std::vector<std::vector<ArmorMeasurement>> mergeCloseClusters(
  const std::vector<std::vector<ArmorMeasurement>> & clusters,
  double max_cluster_noise,
  const ClusterPairMergeConstraint & pair_merge_constraint = ClusterPairMergeConstraint());

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__CLUSTERING_UTILS_HPP_
