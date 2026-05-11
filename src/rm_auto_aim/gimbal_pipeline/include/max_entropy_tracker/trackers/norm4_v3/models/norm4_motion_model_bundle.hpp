// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MOTION_MODEL_BUNDLE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MOTION_MODEL_BUNDLE_HPP_

#include <Eigen/Dense>
#include <memory>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"

namespace fyt::auto_aim::norm4_v3 {

class IMotionModelBundle {
 public:
  virtual ~IMotionModelBundle() = default;

  virtual int state_dim() const = 0;
  virtual const DynamicStateIndex &state_idx() const = 0;

  virtual Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt) = 0;
  virtual Eigen::MatrixXd build_Q(double dt) const = 0;

  virtual Eigen::VectorXd initial_state(const ObservationData &obs,
                                         int panel_id, double r1, double r2,
                                         double dza) const = 0;
  virtual Eigen::MatrixXd initial_covariance() const = 0;
};

/// Motion model bundle that wraps the existing CompositeProcessModel
/// (V1-compatible: Singer/CA/CV translation + CV rotation + structural).
class NativeProcessModelBundle : public IMotionModelBundle {
 public:
  explicit NativeProcessModelBundle(
      std::shared_ptr<CompositeProcessModel> model);

  int state_dim() const override { return model_->state_dim(); }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt) override {
    return model_->predict(x, dt);
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    return model_->build_Q(dt);
  }

  Eigen::VectorXd initial_state(const ObservationData &obs, int panel_id,
                                 double r1, double r2, double dza) const override;

  Eigen::MatrixXd initial_covariance() const override {
    return model_->get_initial_covariance();
  }

  const CompositeProcessModel &model() const { return *model_; }

 private:
  std::shared_ptr<CompositeProcessModel> model_;
  DynamicStateIndex state_idx_;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MOTION_MODEL_BUNDLE_HPP_
