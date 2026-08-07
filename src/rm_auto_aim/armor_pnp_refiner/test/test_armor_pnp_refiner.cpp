#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <string>

#include "armor_pnp_refiner/core/armor_pnp_refiner.hpp"
#include "armor_pnp_refiner/geometry/camera_model.hpp"

namespace armor_pnp_refiner {
namespace {

PnpRefineInput makeInput() {
  PnpRefineInput input;
  input.t_camera_armor = Eigen::Vector3d(0.0, 0.0, 3.0);
  input.q_camera_armor = Eigen::Quaterniond::Identity();
  input.camera_matrix =
    (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0, 0.0, 1000.0, 512.0, 0.0, 0.0, 1.0);
  input.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
  input.object_points = {
    {-0.0665F, -0.025F, 0.0F},
    {-0.0665F, 0.025F, 0.0F},
    {0.0665F, 0.025F, 0.0F},
    {0.0665F, -0.025F, 0.0F},
  };

  for (const auto &point : input.object_points) {
    const auto pixel =
      geometry::projectPoint(Eigen::Vector3d(point.x, point.y, point.z) + input.t_camera_armor,
                             input.camera_matrix,
                             input.dist_coeffs);
    input.image_points.emplace_back(static_cast<float>(pixel.x()), static_cast<float>(pixel.y()));
  }
  return input;
}

TEST(ArmorPnpRefiner, DisabledModePassesThroughRawPnp) {
  PnpRefinerConfig config;
  config.mode = "none";
  ArmorPnpRefiner refiner(config);
  const auto input = makeInput();

  const auto output = refiner.refine(input);

  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.refined);
  EXPECT_EQ(output.mode, RefineMode::PNP_FALLBACK);
  EXPECT_TRUE(output.t_camera_armor.isApprox(input.t_camera_armor));
}

TEST(ArmorPnpRefiner, InvalidCorrespondencesFallBackToRawPnp) {
  PnpRefinerConfig config;
  config.mode = "single_xyz_yaw";
  ArmorPnpRefiner refiner(config);
  auto input = makeInput();
  input.image_points.pop_back();

  const auto output = refiner.refine(input);

  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.refined);
  EXPECT_NE(output.reason.find("insufficient image points"), std::string::npos);
}

TEST(ArmorPnpRefiner, RefinesConsistentSingleFrameObservation) {
  PnpRefinerConfig config;
  config.mode = "single_xyz_yaw";
  ArmorPnpRefiner refiner(config);

  const auto output = refiner.refine(makeInput());

  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.refined);
  EXPECT_EQ(output.mode, RefineMode::G2O_SINGLE_XYZ_YAW);
  EXPECT_NEAR(output.reproj_error_refined_px, 0.0, 1e-3);
  EXPECT_NEAR(output.t_camera_armor.z(), 3.0, 1e-4);
}

}  // namespace
}  // namespace armor_pnp_refiner
