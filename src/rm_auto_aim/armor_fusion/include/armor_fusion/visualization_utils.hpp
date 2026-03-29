#ifndef ARMOR_FUSION__VISUALIZATION_UTILS_HPP_
#define ARMOR_FUSION__VISUALIZATION_UTILS_HPP_

#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_fusion/measurement_types.hpp"
#include "rm_interfaces/msg/armor.hpp"

namespace fyt::auto_aim {

// Build RViz markers for raw cluster points and fused targets.
visualization_msgs::msg::MarkerArray buildVisualizationMarkers(
  const std::vector<std::vector<ArmorMeasurement>> & clusters,
  const std::vector<rm_interfaces::msg::Armor> & fused_armors,
  const std::string & target_frame,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__VISUALIZATION_UTILS_HPP_
