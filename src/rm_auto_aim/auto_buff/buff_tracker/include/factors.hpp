#pragma once

#include "types.hpp"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <opencv2/core.hpp>

#include <boost/optional.hpp>

namespace auto_buff {

// Compatibility: OptionalMatrixType = boost::optional<Matrix&>
using OptionalMatrixH = boost::optional<gtsam::Matrix&>;

// Zero velocity position consistency factor
class ConstPositionFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3>;

public:
  ConstPositionFactor(const gtsam::SharedNoiseModel &model, gtsam::Key x_pre,
                      gtsam::Key x_cur);

  gtsam::Vector evaluateError(const gtsam::Point3 &x_pre,
                              const gtsam::Point3 &x_cur,
                              OptionalMatrixH H1,
                              OptionalMatrixH H2) const override;
};

// Constant angular velocity constraint for small buff
class RollFactor
    : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>;

public:
  RollFactor(const gtsam::SharedNoiseModel &model, gtsam::Key r_pre,
             gtsam::Key w_pre, gtsam::Key r_cur, double dt);
  gtsam::Vector evaluateError(const gtsam::Rot2 &r_pre, const double &w_pre,
                              const gtsam::Rot2 &r_cur,
                              OptionalMatrixH H1,
                              OptionalMatrixH H2,
                              OptionalMatrixH H3) const override;

private:
  double dt_;
};

// Constant vroll constraint
class ConstVRollFactor : public gtsam::NoiseModelFactorN<double, double> {
  using Base = gtsam::NoiseModelFactorN<double, double>;

public:
  ConstVRollFactor(const gtsam::SharedNoiseModel &model, gtsam::Key w_pre,
                   gtsam::Key w_cur);

  gtsam::Vector evaluateError(const double &w_pre, const double &w_cur,
                              OptionalMatrixH H1,
                              OptionalMatrixH H2) const override;
};

// Reprojection factor for a single blade's keypoints
class BuffBladeReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

public:
  BuffBladeReprojFactor(const gtsam::SharedNoiseModel &model,
                        gtsam::Key buff_blade_pose_key,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &distortion_coefficients,
                        BuffPointPosition point_position,
                        Eigen::Vector2d px_point);
  gtsam::Vector evaluateError(const gtsam::Pose3 &armor_pose_camera,
                              OptionalMatrixH H) const override;

private:
  gtsam::Point2 px_point_;
  gtsam::Point3 buff_point_;
  gtsam::Cal3DS2 calib_;
};

// Constraint connecting a blade to the buff center (roll + xyz)
class BuffBladeFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base =
      gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot2, gtsam::Point3>;

public:
  BuffBladeFactor(const gtsam::SharedNoiseModel &model,
                  gtsam::Key buff_blade_pose_key, gtsam::Key roll_key,
                  gtsam::Key center_point_key,
                  const Eigen::Isometry3d &T_camera_to_odom,
                  BuffBladeIndex blade_index);

  gtsam::Vector evaluateError(const gtsam::Pose3 &buff_blade_pose_camera,
                              const gtsam::Rot2 &center_roll,
                              const gtsam::Point3 &center_point,
                              OptionalMatrixH H1,
                              OptionalMatrixH H2,
                              OptionalMatrixH H3) const override;

private:
  Eigen::Isometry3d T_camera_to_odom_;
  BuffBladeIndex blade_index_;
};

} // namespace auto_buff
