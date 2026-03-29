#ifndef ARMOR_FUSION__FUSION_UTILS_HPP_
#define ARMOR_FUSION__FUSION_UTILS_HPP_

#include <vector>

#include "armor_fusion/measurement_types.hpp"
#include "rm_interfaces/msg/armor.hpp"

namespace fyt::auto_aim {

// Fuse one cluster into one armor output with weighted position averaging.
rm_interfaces::msg::Armor fuseCluster(const std::vector<ArmorMeasurement> & cluster);

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__FUSION_UTILS_HPP_
