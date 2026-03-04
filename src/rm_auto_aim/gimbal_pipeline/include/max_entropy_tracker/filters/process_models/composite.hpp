// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_COMPOSITE_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_COMPOSITE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/process_models/base.hpp"
#include "max_entropy_tracker/filters/process_models/rotation.hpp"
#include "max_entropy_tracker/filters/process_models/structural.hpp"
#include "max_entropy_tracker/filters/process_models/translation.hpp"

namespace fyt::auto_aim {

/// Combines translation + rotation + structural into one process model.
class CompositeProcessModel {
 public:
  explicit CompositeProcessModel(
      std::vector<std::shared_ptr<ProcessModelComponent>> components)
      : components_(std::move(components)) {
    build_layout();
  }

  /* ---------- accessors ---------- */
  const StateLayout &layout() const { return layout_; }
  int state_dim() const { return layout_.dim(); }
  const std::vector<std::shared_ptr<ProcessModelComponent>> &components() const {
    return components_;
  }

  /* ---------- predict / Q ---------- */
  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt) const {
    Eigen::VectorXd x_next = x;
    for (const auto &comp : components_) {
      Eigen::VectorXd xc = comp->extract(x);
      Eigen::VectorXd xc_next = comp->predict(xc, dt, &x);
      x_next = comp->inject(x_next, xc_next);
    }
    return x_next;
  }

  Eigen::MatrixXd build_Q(double dt) const {
    int n = state_dim();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    for (const auto &comp : components_) {
      int off = comp->state_offset();
      int dim = comp->state_dim();
      Q.block(off, off, dim, dim) = comp->build_Q(dt);
    }
    return Q;
  }

  Eigen::VectorXd get_initial_state() const {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(state_dim());
    for (const auto &comp : components_) {
      int off = comp->state_offset();
      int dim = comp->state_dim();
      x.segment(off, dim) = comp->get_initial_state();
    }
    return x;
  }

  Eigen::MatrixXd get_initial_covariance() const {
    int n = state_dim();
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
    for (const auto &comp : components_) {
      int off = comp->state_offset();
      int dim = comp->state_dim();
      P.block(off, off, dim, dim) = comp->get_initial_covariance();
    }
    return P;
  }

 private:
  void build_layout() {
    int offset = 0;
    for (auto &comp : components_) {
      comp->set_state_offset(offset);
      auto spec = comp->get_state_spec();
      for (const auto &name : spec.names) {
        layout_.register_state(name, offset);
        ++offset;
      }
    }
    layout_.freeze();
  }

  std::vector<std::shared_ptr<ProcessModelComponent>> components_;
  StateLayout layout_;
};

/// Dynamic state-index accessor (wraps StateLayout with operator[]).
class DynamicStateIndex {
 public:
  explicit DynamicStateIndex(const StateLayout &layout) : layout_(&layout) {}

  int operator[](const std::string &name) const { return layout_->get(name); }
  int get(const std::string &name) const { return layout_->get(name); }
  bool has(const std::string &name) const { return layout_->has(name); }

  // Convenience accessors for common states
  int X() const { return layout_->get("X"); }
  int VX() const { return layout_->get("VX"); }
  int Y() const { return layout_->get("Y"); }
  int VY() const { return layout_->get("VY"); }
  int Z() const { return layout_->get("Z"); }
  int VZ() const { return layout_->get("VZ"); }
  int DELTA() const { return layout_->get("DELTA"); }
  int DELTA_RATE() const { return layout_->get("DELTA_RATE"); }
  int R1() const { return layout_->get("R1"); }
  int R2() const { return layout_->get("R2"); }
  int DZA() const { return layout_->get("DZA"); }

  // CA/Singer extra states (may not exist)
  int AX() const { return layout_->get("AX"); }
  int AY() const { return layout_->get("AY"); }
  int AZ() const { return layout_->get("AZ"); }

 private:
  const StateLayout *layout_;
};

/* =========== Factory =========== */

/// Create a default composite model from config enums.
inline std::shared_ptr<CompositeProcessModel> create_default_process_model(
    TranslationModel translation_type = TranslationModel::CV,
    RotationModel rotation_type = RotationModel::CV,
    const TranslationConfig &trans_cfg = {},
    const RotationConfig &rot_cfg = {},
    const StructuralConfig &struct_cfg = {},
    int n_dims = 3) {
  std::vector<std::shared_ptr<ProcessModelComponent>> components;
  components.push_back(create_translation_model(translation_type, trans_cfg, n_dims));
  components.push_back(create_rotation_model(rotation_type, rot_cfg));
  components.push_back(std::make_shared<StructuralModel>(struct_cfg));
  return std::make_shared<CompositeProcessModel>(std::move(components));
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_COMPOSITE_HPP_
