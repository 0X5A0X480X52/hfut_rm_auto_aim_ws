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

  // ── Step 1: 数值计算投影 Jacobian (2N × 6) ──
  // 参数: [rvec(3), tvec(3)] → 图像点 (2N)
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

  // ── Step 2: 从像素噪声计算参数协方差 (6×6) ──
  // Σ_param = σ² * (JᵀJ)⁻¹
  double sigma2 = pixel_noise_sigma * pixel_noise_sigma;
  Eigen::MatrixXd JtJ = J.transpose() * J;
  Eigen::MatrixXd JtJ_reg = JtJ + Eigen::MatrixXd::Identity(6, 6) * 1e-6;  // 正则化
  Eigen::Matrix<double, 6, 6> Sigma_param = sigma2 * JtJ_reg.inverse();

  // ── Step 3: 提取 tvec 的协方差 (3×3，参数中 tvec 对应索引 3-5) ──
  Eigen::Matrix3d Sigma_tvec = Sigma_param.block<3, 3>(3, 3);

  // ── Step 4: 将 tvec 协方差传播到世界坐标系的 armor center (x,y,z) ──
  // 在 PnP 中: camera_point = R_obj2cam * obj_point + tvec
  // 其中 R_obj2cam 由 rvec 通过 Rodrigues 得到
  // 这里取 obj_pts 的中心在 camera 系下的位置作为线性化点
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_obj2cam;
  cv::cv2eigen(rmat, R_obj2cam);

  // obj_pts 中心
  cv::Point3f obj_center(0, 0, 0);
  for (const auto &pt : obj_pts) {
    obj_center.x += pt.x / N;
    obj_center.y += pt.y / N;
    obj_center.z += pt.z / N;
  }
  Eigen::Vector3d obj_c(obj_center.x, obj_center.y, obj_center.z);

  // camera_point = R_obj2cam * obj_c + tvec
  // d(camera_point)/d(tvec) = I₃ (恒等矩阵)
  // 所以位置协方差在 camera 系下 = Sigma_tvec
  // 注意：这是简化假设，忽略了 rvec 不确定性对旋转的耦合
  Eigen::Matrix3d Sigma_xyz_cam = Sigma_tvec;

  // ── Step 5: 计算 yaw 方差 ──
  // 使用 Rodrigues Jacobian 的 yaw 分量近似
  // yaw 通过 RPY 分解得到，δyaw ≈ atan2(R(1,0), R(0,0))
  // 保守估计：σ²_yaw = trace(Sigma_rvec) 并除以 3 作为平均
  Eigen::Matrix3d Sigma_rvec = Sigma_param.block<3, 3>(0, 0);
  double mean_rvec_var = Sigma_rvec.trace() / 3.0;

  // 一阶近似：σ²_yaw ≈ σ²_rx + σ²_ry + σ²_rz（每个旋转分量等权重贡献）
  // 乘以保守因子
  yaw_var_out = mean_rvec_var * 3.0;

  // ── Step 6: 将 camera 系协方差传递到输出，调用方负责 TF 变换 ──
  pos_cov_out = Sigma_xyz_cam;

  return true;
}

}  // namespace fyt
