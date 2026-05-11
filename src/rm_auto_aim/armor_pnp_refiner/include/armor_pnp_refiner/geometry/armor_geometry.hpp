#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <vector>

namespace armor_pnp_refiner {
namespace geometry {

constexpr double kPi = 3.14159265358979323846;

// Standard armor plate dimensions in meters.
constexpr double SMALL_ARMOR_WIDTH = 0.133;
constexpr double SMALL_ARMOR_HEIGHT = 0.050;
constexpr double LARGE_ARMOR_WIDTH = 0.225;
constexpr double LARGE_ARMOR_HEIGHT = 0.050;

// Generate 4-point object model for NN detector (left_bottom, left_top, right_top, right_bottom).
inline std::vector<Eigen::Vector3d> objectPointsNn4(double half_w, double half_h) {
  return {
    {-half_w, -half_h, 0.0},
    {-half_w,  half_h, 0.0},
    { half_w,  half_h, 0.0},
    { half_w, -half_h, 0.0}
  };
}

// Generate 6-point object model for traditional detector.
inline std::vector<Eigen::Vector3d> objectPointsTrad6(double half_w, double half_h) {
  return {
    {-half_w, -half_h, 0.0},
    {-half_w,  0.0,    0.0},
    {-half_w,  half_h, 0.0},
    { half_w,  half_h, 0.0},
    { half_w,  0.0,    0.0},
    { half_w, -half_h, 0.0}
  };
}

// Normalize angle to [-pi, pi].
inline double normalizeAngle(double rad) {
  while (rad > kPi) rad -= 2.0 * kPi;
  while (rad < -kPi) rad += 2.0 * kPi;
  return rad;
}

}  // namespace geometry
}  // namespace armor_pnp_refiner
