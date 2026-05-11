#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace armor_pnp_refiner {
namespace geometry {

// Project a 3D point in camera frame to pixel coordinates.
// Supports distortion via OpenCV projectPoints.
inline Eigen::Vector2d projectPoint(
    const Eigen::Vector3d& p_cam,
    const cv::Mat& K,
    const cv::Mat& D)
{
  std::vector<cv::Point3f> obj{ cv::Point3f(p_cam.x(), p_cam.y(), p_cam.z()) };
  std::vector<cv::Point2f> img;
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::projectPoints(obj, rvec, tvec, K, D, img);
  return { img[0].x, img[0].y };
}

// Build camera-armor rotation matrix from yaw, pitch, roll.
// R_camera_armor = R_z(yaw) * R_y(pitch) * R_x(roll)
inline Eigen::Matrix3d buildCameraArmorRotation(
    double yaw, double pitch, double roll)
{
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
  return R;
}

// Build camera-armor rotation with IMU-camera convention compatibility.
inline Eigen::Matrix3d buildCameraArmorRotationWithImu(
    double yaw, double pitch, double roll,
    const Eigen::Matrix3d& R_imu_camera)
{
  Eigen::Matrix3d R_yaw   = Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Matrix3d R_roll  = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()).toRotationMatrix();
  // R_camera_armor = R_camera_imu * R_z(yaw) * R_y(pitch) * R_x(roll)
  return R_imu_camera.transpose() * R_yaw * R_pitch * R_roll;
}

}  // namespace geometry
}  // namespace armor_pnp_refiner
