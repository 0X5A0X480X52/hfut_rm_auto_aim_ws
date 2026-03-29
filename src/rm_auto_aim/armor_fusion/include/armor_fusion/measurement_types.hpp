#ifndef ARMOR_FUSION__MEASUREMENT_TYPES_HPP_
#define ARMOR_FUSION__MEASUREMENT_TYPES_HPP_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>

namespace fyt::auto_aim {

// Unified in-memory measurement after TF transform.
struct ArmorMeasurement {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  std::string number;
  std::string armor_type;
  std::string source_frame;
  builtin_interfaces::msg::Time stamp;
  double distance_to_image_center{1.0};
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__MEASUREMENT_TYPES_HPP_
