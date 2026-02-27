// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_BASE_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_BASE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fyt::auto_aim {

// ======================== StateLayout ========================

/// Dynamic state-vector index manager (equivalent to Python StateLayout)
class StateLayout {
 public:
  void register_state(const std::string &name, int index) {
    if (frozen_)
      throw std::runtime_error("StateLayout is frozen");
    if (indices_.count(name))
      throw std::runtime_error("State '" + name + "' already registered");
    indices_[name] = index;
    names_[index] = name;
    dim_ = std::max(dim_, index + 1);
  }

  int register_block(const std::vector<std::string> &names, int start) {
    for (size_t i = 0; i < names.size(); ++i)
      register_state(names[i], start + static_cast<int>(i));
    return start + static_cast<int>(names.size());
  }

  void freeze() { frozen_ = true; }

  int get(const std::string &name) const {
    auto it = indices_.find(name);
    if (it == indices_.end())
      throw std::runtime_error("Unknown state: '" + name + "'");
    return it->second;
  }

  bool has(const std::string &name) const { return indices_.count(name) > 0; }
  int dim() const { return dim_; }
  const std::unordered_map<std::string, int> &indices() const {
    return indices_;
  }

 private:
  std::unordered_map<std::string, int> indices_;
  std::unordered_map<int, std::string> names_;
  int dim_ = 0;
  bool frozen_ = false;
};

// ======================== ComponentStateSpec ========================

struct ComponentStateSpec {
  std::vector<std::string> names;
  int dim;

  static ComponentStateSpec from_names(const std::vector<std::string> &n) {
    return {n, static_cast<int>(n.size())};
  }
};

// ======================== ProcessModelComponent (abstract) ========================

class ProcessModelComponent {
 public:
  virtual ~ProcessModelComponent() = default;

  virtual ComponentStateSpec get_state_spec() const = 0;
  virtual Eigen::VectorXd predict(const Eigen::VectorXd &x_component,
                                  double dt,
                                  const Eigen::VectorXd *full_state = nullptr) const = 0;
  virtual Eigen::MatrixXd build_Q(double dt) const = 0;

  virtual Eigen::VectorXd get_initial_state() const {
    return Eigen::VectorXd::Zero(state_dim());
  }
  virtual Eigen::MatrixXd get_initial_covariance() const {
    return Eigen::MatrixXd::Identity(state_dim(), state_dim());
  }

  int state_dim() const { return get_state_spec().dim; }
  const std::vector<std::string> &state_names() const {
    // cache on first call
    if (cached_names_.empty()) cached_names_ = get_state_spec().names;
    return cached_names_;
  }

  int state_offset() const { return offset_; }
  void set_state_offset(int o) { offset_ = o; }

  Eigen::VectorXd extract(const Eigen::VectorXd &full) const {
    return full.segment(offset_, state_dim());
  }
  Eigen::VectorXd inject(const Eigen::VectorXd &full,
                         const Eigen::VectorXd &comp) const {
    Eigen::VectorXd out = full;
    out.segment(offset_, state_dim()) = comp;
    return out;
  }

 private:
  int offset_ = 0;
  mutable std::vector<std::string> cached_names_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_BASE_HPP_
