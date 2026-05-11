// Copyright (C) FYT Vision Group. All rights reserved.
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

#include "armor_detector/armor_pose_estimator.hpp"

#include <algorithm>
#include <array>

#include "armor_detector/types.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
ArmorPoseEstimator::ArmorPoseEstimator(
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
  // Setup pnp solver
  pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  pnp_solver_->setObjectPoints(
      "small", Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH,
                                                     SMALL_ARMOR_HEIGHT));
  pnp_solver_->setObjectPoints(
      "large", Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH,
                                                     LARGE_ARMOR_HEIGHT));
  // BA solver
  ba_solver_ = std::make_unique<BaSolver>(camera_info->k, camera_info->d);
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      camera_matrix_.at<double>(r, c) = camera_info->k[r * 3 + c];
    }
  }
  dist_coeffs_ =
      cv::Mat(static_cast<int>(camera_info->d.size()), 1, CV_64F);
  for (size_t i = 0; i < camera_info->d.size(); ++i) {
    dist_coeffs_.at<double>(static_cast<int>(i), 0) = camera_info->d[i];
  }

  R_gimbal_camera_ = Eigen::Matrix3d::Identity();
  R_gimbal_camera_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
}

void ArmorPoseEstimator::configurePnpRefiner(bool enable, const std::string &mode) {
  use_pnp_refiner_ = enable;
  pnp_refiner_mode_ = mode;
  if (!use_pnp_refiner_ || pnp_refiner_mode_ == "none") {
    pnp_refiner_.reset();
    return;
  }

  armor_pnp_refiner::PnpRefinerConfig cfg;
  cfg.mode = pnp_refiner_mode_;
  // Keep Phase0 behavior as close to legacy detector BA as possible.
  cfg.prefer_legacy_single_yaw = true;
  pnp_refiner_ = std::make_unique<armor_pnp_refiner::ArmorPnpRefiner>(cfg);
}

std::vector<rm_interfaces::msg::Armor>
ArmorPoseEstimator::extractArmorPoses(const std::vector<Armor> &armors,
                                   Eigen::Matrix3d R_imu_camera) {
  std::vector<rm_interfaces::msg::Armor> armors_msg;

  for (const auto &armor : armors) {
    std::vector<cv::Mat> rvecs, tvecs;

    // Use PnP to get the initial pose information
    if (pnp_solver_->solvePnPGeneric(
            armor.landmarks(), rvecs, tvecs,
            (armor.type == ArmorType::SMALL ? "small" : "large"))) {
      sortPnPResult(armor, rvecs, tvecs);
      cv::Mat rmat;
      cv::Rodrigues(rvecs[0], rmat);

      Eigen::Matrix3d R = utils::cvToEigen(rmat);
      Eigen::Vector3d t = utils::cvToEigen(tvecs[0]);

      double armor_roll =
          rotationMatrixToRPY(R_gimbal_camera_ * R)[0] * 180 / M_PI;

      const bool use_legacy_single_yaw =
          use_pnp_refiner_ && pnp_refiner_mode_ == "single_yaw";
      if ((use_ba_ || use_legacy_single_yaw) && armor_roll < 15) {
        // Use BA alogorithm to optimize the pose from PnP
        // solveBa() will modify the rotation_matrix
        R = ba_solver_->solveBa(armor, t, R, R_imu_camera);
      }

      std::optional<armor_pnp_refiner::PnpRefineOutput> refined_opt;
      if (use_pnp_refiner_ && pnp_refiner_ != nullptr && pnp_refiner_mode_ != "single_yaw") {
        armor_pnp_refiner::PnpRefineInput input;
        input.t_camera_armor = t;
        input.q_camera_armor = Eigen::Quaterniond(R);
        auto rpy = rotationMatrixToRPY(R);
        input.roll_rad = rpy[0];
        input.pitch_rad = rpy[1];
        input.yaw_rad = rpy[2];
        input.rvec = rvecs[0];
        input.tvec = tvecs[0];
        input.image_points = armor.landmarks();
        input.object_points =
            (armor.type == ArmorType::SMALL)
                ? Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH,
                                                        SMALL_ARMOR_HEIGHT)
                : Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH,
                                                        LARGE_ARMOR_HEIGHT);
        input.camera_matrix = camera_matrix_;
        input.dist_coeffs = dist_coeffs_;
        input.stamp_sec = 0.0;
        input.armor_number = armor.number;
        input.armor_type = (armor.number == "outpost")
                               ? armor_pnp_refiner::ArmorSizeType::OUTPOST
                               : (armor.type == ArmorType::SMALL
                                      ? armor_pnp_refiner::ArmorSizeType::SMALL
                                      : armor_pnp_refiner::ArmorSizeType::LARGE);
        input.R_imu_camera = R_imu_camera;
        input.use_fixed_pitch_roll = false;

        const auto refined = pnp_refiner_->refine(input);
        refined_opt = refined;
        if (refined.valid) {
          R = refined.q_camera_armor.toRotationMatrix();
          t = refined.t_camera_armor;
        }
      }
      Eigen::Quaterniond q(R);

      // Fill the armor message
      rm_interfaces::msg::Armor armor_msg;

      // Fill basic info
      armor_msg.type = armorTypeToString(armor.type);
      armor_msg.number = armor.number;

      // Fill pose
      armor_msg.pose.position.x = t(0);
      armor_msg.pose.position.y = t(1);
      armor_msg.pose.position.z = t(2);
      armor_msg.pose.orientation.x = q.x();
      armor_msg.pose.orientation.y = q.y();
      armor_msg.pose.orientation.z = q.z();
      armor_msg.pose.orientation.w = q.w();

      // 计算 PnP 协方差并填入消息
      {
        Eigen::Matrix3d pos_cov;
        double yaw_var;
        if (pnp_solver_->calculatePnPCovariance(
                armor.landmarks(), rvecs[0], tvecs[0],
                (armor.type == ArmorType::SMALL ? "small" : "large"),
                1.0, pos_cov, yaw_var)) {
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
              armor_msg.pos_covariance[r * 3 + c] = pos_cov(r, c);
          armor_msg.yaw_variance = yaw_var;
        } else {
          armor_msg.pos_covariance.fill(0.0);
          armor_msg.pos_covariance[0] = armor_msg.pos_covariance[4] = armor_msg.pos_covariance[8] = 0.01;
          armor_msg.yaw_variance = 0.01;
        }
      }

      // Fill the distance to image center
      armor_msg.distance_to_image_center =
          pnp_solver_->calculateDistanceToCenter(armor.center);

      // Fill optional 2D image geometry for downstream 2D evidence pipeline.
      armor_msg.detection_confidence = armor.confidence;
      armor_msg.has_image_geometry = true;
      const auto corners = std::array<cv::Point2f, 4>{
          armor.left_light.bottom, armor.left_light.top,
          armor.right_light.top, armor.right_light.bottom};
      float min_x = corners[0].x;
      float min_y = corners[0].y;
      float max_x = corners[0].x;
      float max_y = corners[0].y;
      for (int i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point32 p;
        p.x = corners[i].x;
        p.y = corners[i].y;
        p.z = 0.0f;
        armor_msg.image_corners[i] = p;
        min_x = std::min(min_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_x = std::max(max_x, corners[i].x);
        max_y = std::max(max_y, corners[i].y);
      }
      armor_msg.bbox_xywh[0] = min_x;
      armor_msg.bbox_xywh[1] = min_y;
      armor_msg.bbox_xywh[2] = std::max(0.0f, max_x - min_x);
      armor_msg.bbox_xywh[3] = std::max(0.0f, max_y - min_y);
      // 0: LB,LT,RT,RB (legacy detector convention).
      armor_msg.corners_ordering = 0;

      // Fill refiner quality metadata (Phase 1: covariance always invalid)
      if (refined_opt.has_value()) {
        const auto &ref = *refined_opt;
        armor_msg.pose_estimate_mode = static_cast<uint8_t>(ref.mode);
        armor_msg.pose_quality_score = static_cast<float>(ref.confidence);
        armor_msg.reproj_error_raw = static_cast<float>(ref.reproj_error_raw_px);
        armor_msg.reproj_error_refined = static_cast<float>(ref.reproj_error_refined_px);
        armor_msg.pose_condition_number = static_cast<float>(ref.condition_number);
        armor_msg.pose_num_points = static_cast<uint16_t>(ref.num_points);
        armor_msg.pose_num_inliers = static_cast<uint16_t>(ref.num_inliers);
        armor_msg.pose_covariance_valid = ref.covariance_valid;
        for (int r = 0; r < 4; ++r) {
          for (int c = 0; c < 4; ++c) {
            armor_msg.pose_covariance_xyz_yaw[r * 4 + c] =
                ref.covariance_xyz_yaw(r, c);
          }
        }
      }

      armors_msg.push_back(std::move(armor_msg));
    } else {
      FYT_WARN("armor_detector", "PnP Failed!");
    }
  }

  return armors_msg;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d &R) {
  // Transform to camera frame
  Eigen::Quaterniond q(R);
  // Get armor yaw
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  Eigen::Vector3d rpy;
  tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

void ArmorPoseEstimator::sortPnPResult(const Armor &armor,
                                    std::vector<cv::Mat> &rvecs,
                                    std::vector<cv::Mat> &tvecs) const {
  constexpr float PROJECT_ERR_THRES = 3.0;

  // 获取这两个解
  cv::Mat &rvec1 = rvecs.at(0);
  cv::Mat &tvec1 = tvecs.at(0);
  cv::Mat &rvec2 = rvecs.at(1);
  cv::Mat &tvec2 = tvecs.at(1);

  // 将旋转向量转换为旋转矩阵
  cv::Mat R1_cv, R2_cv;
  cv::Rodrigues(rvec1, R1_cv);
  cv::Rodrigues(rvec2, R2_cv);

  // 转换为Eigen矩阵
  Eigen::Matrix3d R1 = utils::cvToEigen(R1_cv);
  Eigen::Matrix3d R2 = utils::cvToEigen(R2_cv);

  // 计算云台系下装甲板的RPY角
  auto rpy1 = rotationMatrixToRPY(R_gimbal_camera_ * R1);
  auto rpy2 = rotationMatrixToRPY(R_gimbal_camera_ * R2);

  std::string coord_frame_name =
      (armor.type == ArmorType::SMALL ? "small" : "large");
  double error1 = pnp_solver_->calculateReprojectionError(
      armor.landmarks(), rvec1, tvec1, coord_frame_name);
  double error2 = pnp_solver_->calculateReprojectionError(
      armor.landmarks(), rvec2, tvec2, coord_frame_name);

  // 两个解的重投影误差差距较大或者roll角度较大时，不做选择
  if ((error2 / error1 > PROJECT_ERR_THRES) || (rpy1[0] > 10 * 180 / M_PI) ||
      (rpy2[0] > 10 * 180 / M_PI)) {
    return;
  }

  // 计算灯条在图像中的倾斜角度
  double l_angle =
      std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180 / M_PI;
  double r_angle =
      std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180 /
      M_PI;
  double angle = (l_angle + r_angle) / 2;
  angle += 90.0;

  if (armor.number == "outpost") angle = -angle;

  // 根据倾斜角度选择解
  // 如果装甲板左倾（angle > 0），选择Yaw为负的解
  // 如果装甲板右倾（angle < 0），选择Yaw为正的解
  if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) ||
      (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
    std::swap(rvec1, rvec2);
    std::swap(tvec1, tvec2);
    FYT_DEBUG("armor_detector", "PnP Solution 2 Selected");
  }
}

} // namespace fyt::auto_aim
