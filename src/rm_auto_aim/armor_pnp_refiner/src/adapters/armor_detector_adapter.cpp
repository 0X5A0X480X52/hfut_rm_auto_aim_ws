#include "armor_pnp_refiner/adapters/armor_detector_adapter.hpp"

#include "armor_pnp_refiner/core/armor_pnp_refiner.hpp"

namespace armor_pnp_refiner {

ArmorDetectorAdapter::ArmorDetectorAdapter(std::shared_ptr<ArmorPnpRefiner> refiner)
  : refiner_(std::move(refiner)) {}

ArmorDetectorAdapter::~ArmorDetectorAdapter() = default;

PnpRefineInput ArmorDetectorAdapter::buildInput(
    const Eigen::Vector3d& t_camera_armor,
    const Eigen::Quaterniond& q_camera_armor,
    double yaw_rad, double pitch_rad, double roll_rad,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& K, const cv::Mat& D,
    const std::string& armor_number,
    ArmorSizeType armor_type,
    double stamp_sec,
    const Eigen::Matrix3d& R_imu_camera)
{
  PnpRefineInput input;
  input.t_camera_armor = t_camera_armor;
  input.q_camera_armor = q_camera_armor;
  input.yaw_rad = yaw_rad;
  input.pitch_rad = pitch_rad;
  input.roll_rad = roll_rad;
  input.rvec = rvec;
  input.tvec = tvec;
  input.image_points = image_points;
  input.object_points = object_points;
  input.camera_matrix = K;
  input.dist_coeffs = D;
  input.armor_number = armor_number;
  input.armor_type = armor_type;
  input.stamp_sec = stamp_sec;
  input.R_imu_camera = R_imu_camera;
  input.use_fixed_pitch_roll = true;
  input.fixed_pitch_rad = pitch_rad;
  input.fixed_roll_rad = roll_rad;
  return input;
}

PnpRefineOutput ArmorDetectorAdapter::refine(const PnpRefineInput& input)
{
  return refiner_->refine(input);
}

}  // namespace armor_pnp_refiner
