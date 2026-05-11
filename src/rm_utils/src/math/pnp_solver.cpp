// Created by Chengfu Zou on 2024.1.19
// Copyright(C) FYT Vision Group. All rights resevred.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_utils/math/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Dense>

namespace fyt {
PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix,
                     const std::vector<double> &distortion_coefficients,
                     cv::SolvePnPMethod method)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone())
, distortion_coefficients_(
    cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion_coefficients.data())).clone())
, method_(method) {}

void PnPSolver::setObjectPoints(const std::string &coord_frame_name,
                                const std::vector<cv::Point3f> &object_points) noexcept {
  object_points_map_[coord_frame_name] = object_points;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept {
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::calculateReprojectionError(const std::vector<cv::Point2f> &image_points,
                                             const cv::Mat &rvec,
                                             const cv::Mat &tvec,
                                             const std::string &coord_frame_name) const noexcept {
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto &object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
    double error = 0;
    for (std::size_t i = 0; i < image_points.size(); ++i) {
      error += cv::norm(image_points[i] - reprojected_points[i]);
    }
    return error;
  } else {
    return 0;
  }
}

bool PnPSolver::calculatePnPCovariance(
    const std::vector<cv::Point2f> &image_points,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    const std::string &coord_frame_name,
    double pixel_noise_sigma,
    Eigen::Matrix3d &pos_cov_out,
    double &yaw_var_out) const {

  auto it = object_points_map_.find(coord_frame_name);
  if (it == object_points_map_.end()) return false;

  const auto &obj_pts = it->second;
  const int N = static_cast<int>(image_points.size());
  if (N < 4) return false;  // 至少需要 4 个点做有意义的协方差估计

  const double eps = 1e-5;
  Eigen::MatrixXd J(2 * N, 6);  // 2N rows (u_i, v_i), 6 cols (rx,ry,rz, tx,ty,tz)

  // 获取基线投影
  std::vector<cv::Point2f> proj_base;
  cv::projectPoints(obj_pts, rvec, tvec, camera_matrix_, distortion_coefficients_, proj_base);

  // 对 6 个参数分别扰动
  for (int p = 0; p < 6; ++p) {
    cv::Mat rvec_p = rvec.clone();
    cv::Mat tvec_p = tvec.clone();

    if (p < 3) {
      rvec_p.at<double>(p) += eps;
    } else {
      tvec_p.at<double>(p - 3) += eps;
    }

    std::vector<cv::Point2f> proj_pert;
    cv::projectPoints(obj_pts, rvec_p, tvec_p, camera_matrix_, distortion_coefficients_, proj_pert);

    for (int i = 0; i < N; ++i) {
      J(2 * i, p)     = (proj_pert[i].x - proj_base[i].x) / eps;
      J(2 * i + 1, p) = (proj_pert[i].y - proj_base[i].y) / eps;
    }
  }

  // Σ_param = σ² * (JᵀJ)⁻¹
  double sigma2 = pixel_noise_sigma * pixel_noise_sigma;
  Eigen::Matrix<double, 6, 6> JtJ = J.transpose() * J;
  Eigen::Matrix<double, 6, 6> JtJ_reg =
      JtJ + Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
  Eigen::Matrix<double, 6, 6> Sigma_param = sigma2 * JtJ_reg.inverse();

  // Extract tvec covariance (params 3-5: tx, ty, tz)
  // d(camera_point)/d(tvec) = I₃, so camera-frame position covariance = Sigma_tvec
  Eigen::Matrix3d Sigma_xyz_cam = Sigma_param.block<3, 3>(3, 3);

  // Compute yaw variance from rotation parameter covariance (params 0-2)
  Eigen::Matrix3d Sigma_rvec = Sigma_param.block<3, 3>(0, 0);
  // Conservative approximation: treat all rotation components as equally
  // contributing to yaw uncertainty, then inflate by 3x
  yaw_var_out = Sigma_rvec.trace();

  // Caller applies TF rotation to transform to the target frame
  pos_cov_out = Sigma_xyz_cam;

  return true;
}

}  // namespace fyt
