#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace armor_pnp_refiner {
namespace geometry {

// Extract roll, pitch, yaw from rotation matrix (ZYX convention).
inline Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& R) {
  double pitch = std::asin(-R(2, 0));
  double roll = std::atan2(R(2, 1), R(2, 2));
  double yaw  = std::atan2(R(1, 0), R(0, 0));
  return {roll, pitch, yaw};
}

// Convert rvec/tvec from OpenCV to Eigen.
inline void cvToEigen(const cv::Mat& rvec, const cv::Mat& tvec,
                       Eigen::Matrix3d& R, Eigen::Vector3d& t) {
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  for (int i = 0; i < 3; ++i) {
    t(i) = tvec.at<double>(i);
    for (int j = 0; j < 3; ++j) {
      R(i, j) = R_cv.at<double>(i, j);
    }
  }
}

// Convert Eigen rotation/translation to OpenCV rvec/tvec.
inline void eigenToCv(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                       cv::Mat& rvec, cv::Mat& tvec) {
  cv::Mat R_cv(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_cv.at<double>(i, j) = R(i, j);
    }
  }
  cv::Rodrigues(R_cv, rvec);
  tvec = (cv::Mat_<double>(3, 1) << t(0), t(1), t(2));
}

// Convert Eigen Quaternion to rvec/tvec.
inline void eigenToCv(const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
                       cv::Mat& rvec, cv::Mat& tvec) {
  Eigen::Matrix3d R = q.toRotationMatrix();
  eigenToCv(R, t, rvec, tvec);
}

}  // namespace geometry
}  // namespace armor_pnp_refiner
