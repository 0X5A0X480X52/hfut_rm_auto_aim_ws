// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

namespace {

constexpr double kLog3 = 1.0986122886681098;

double clamp01(double x) {
  return std::clamp(x, 0.0, 1.0);
}

double angle_abs_diff(double a, double b) {
  return std::abs(normalize_angle(a - b));
}

double median_of_vector(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  if (n % 2 == 1) return v[n / 2];
  return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

OutpostArmorTracker::OutpostArmorTracker(const UnifiedConfig &config, double dt,
                                         bool enable_oscillation)
    : BaseTracker(dt),
      config_(config),
      radius_(std::max(0.05, config.outpost.radius)),
      z_offsets_{config.outpost.z_offset_0, config.outpost.z_offset_1,
                 config.outpost.z_offset_2},
      outpost_ukf_(config, dt),
      maneuver_detector_(config.maneuver) {
  (void)enable_oscillation;
  const double step = (config.outpost.panel_angle_step > 1e-6)
                          ? config.outpost.panel_angle_step
                          : (2.0 * M_PI / 3.0);
  panel_angles_ = {0.0, step, 2.0 * step};
}

void OutpostArmorTracker::initialize(const std::vector<ObservationData> &obs,
                                     double /*r1*/, double /*r2*/,
                                     double /*dza*/) {
  if (obs.empty()) {
    throw std::invalid_argument("OutpostArmorTracker requires one observation");
  }

  const ObservationData *selected = select_observation(obs);
  if (selected == nullptr) {
    throw std::invalid_argument("OutpostArmorTracker cannot select observation");
  }

  int init_panel = 0;
  double min_abs_cz = std::abs(selected->z - z_offsets_[0]);
  for (int i = 1; i < 3; ++i) {
    const double cz_i = selected->z - z_offsets_[i];
    const double abs_cz_i = std::abs(cz_i);
    if (abs_cz_i < min_abs_cz) {
      min_abs_cz = abs_cz_i;
      init_panel = i;
    }
  }

  selected_panel_id_ = init_panel;
  last_best_panel_id_ = init_panel;

  mode_ = TrackMode::AMBIGUOUS_SINGLE_ARMOR;
  stable_counter_ = 0;
  entropy_norm_ = 1.0;
  max_prob_ = 1.0 / 3.0;

  center_z_history_.clear();
  push_center_z_history(selected->z - z_offsets_[init_panel]);

  outpost_ukf_.initialize({*selected}, radius_, radius_, 0.0, init_panel);
  sync_internal_state_from_filter();

  if (selected->timestamp.has_value()) {
    current_time_ = selected->timestamp.value();
    last_update_time_ = selected->timestamp.value();
    last_internal_update_time_ = selected->timestamp.value();
  } else {
    last_internal_update_time_.reset();
  }

  mark_initialized();
  transition_to(TrackerState::INITIALIZING);
  increment_frame();

  update_publish_state();
}

void OutpostArmorTracker::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;

  const double dt = compute_dt(target_time);

  outpost_ukf_.predict(dt);
  apply_motion_constraints_from_config();
  sync_internal_state_from_filter();

  if (target_time.has_value()) {
    current_time_ = target_time.value();
  } else if (current_time_.has_value()) {
    current_time_ = current_time_.value() + dt;
  }

  update_publish_state();
}

bool OutpostArmorTracker::update(const std::vector<ObservationData> &obs) {
  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
    return false;
  }

  const ObservationData *selected = select_observation(obs);
  if (selected == nullptr) {
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
    return false;
  }

  // Predict to observation timestamp when possible.
  if (selected->timestamp.has_value() && current_time_.has_value()) {
    const double d = selected->timestamp.value() - current_time_.value();
    if (d > min_dt_) predict(selected->timestamp.value());
  }

  handle_observation_received(config_.tracker.tracking_thres);

  double dt_for_update = dt_;
  if (selected->timestamp.has_value() && last_internal_update_time_.has_value()) {
    dt_for_update = std::clamp(selected->timestamp.value() -
                                   last_internal_update_time_.value(),
                               min_dt_, max_dt_);
  }

  if (selected->timestamp.has_value()) {
    last_internal_update_time_ = selected->timestamp.value();
  }

  const double hist_center_z = center_z_history_.empty()
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : history_center_z_median();

  auto hyps = evaluate_hypotheses(*selected, center_position_est_.z(),
                                  hist_center_z);
  compute_probabilities(hyps);

  int best_idx = 0;
  for (int i = 1; i < 3; ++i) {
    if (hyps[i].probability > hyps[best_idx].probability) best_idx = i;
  }

  selected_panel_id_ = hyps[best_idx].panel_id;
  max_prob_ = hyps[best_idx].probability;

  double entropy = 0.0;
  for (const auto &h : hyps) {
    const double p = std::max(h.probability, 1e-12);
    entropy -= p * std::log(p);
  }
  entropy_norm_ = std::clamp(entropy / kLog3, 0.0, 1.0);

  update_mode_from_entropy(selected_panel_id_, entropy_norm_, max_prob_);
  if (!update_internal_state(*selected, hyps[best_idx], dt_for_update)) {
    return false;
  }
  push_center_z_history(hyps[best_idx].center_z);

  if (selected->timestamp.has_value()) {
    update_time(selected->timestamp.value());
  }

  update_publish_state();

  increment_frame();
  return true;
}

Eigen::Vector3d OutpostArmorTracker::get_center_position() const {
  return publish_position_;
}

double OutpostArmorTracker::get_yaw() const {
  if (is_ambiguous_single_mode()) {
    return normalize_angle(center_yaw_est_ + panel_angles_[selected_panel_id_]);
  }
  return center_yaw_est_;
}

std::pair<double, double> OutpostArmorTracker::get_radii() const {
  return {radius_, radius_};
}

ManeuverResult OutpostArmorTracker::assess_maneuver() const {
  const double innov_norm = outpost_ukf_.last_innov_xyz().size() >= 3
                                ? outpost_ukf_.last_innov_xyz().norm()
                                : 0.0;
  return maneuver_detector_.detect(
      outpost_ukf_.last_nis(), innov_norm, outpost_ukf_.last_update_type());
}

bool OutpostArmorTracker::is_ambiguous_single_mode() const {
  return mode_ == TrackMode::AMBIGUOUS_SINGLE_ARMOR;
}

int OutpostArmorTracker::effective_num_armors() const {
  return is_ambiguous_single_mode() ? 1 : 3;
}

int OutpostArmorTracker::selected_panel_id() const { return selected_panel_id_; }

double OutpostArmorTracker::normalized_entropy() const { return entropy_norm_; }

double OutpostArmorTracker::max_panel_probability() const { return max_prob_; }

double OutpostArmorTracker::confidence_scale() const {
  if (!is_ambiguous_single_mode()) return 1.0;
  return clamp01(config_.outpost.single_mode_confidence_scale);
}

std::vector<geometry_msgs::msg::Pose>
OutpostArmorTracker::build_armors_offset_for_message() const {
  std::vector<geometry_msgs::msg::Pose> offsets;

  if (is_ambiguous_single_mode()) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    offsets.push_back(pose);
    return offsets;
  }

  offsets.reserve(3);
  for (int i = 0; i < 3; ++i) {
    const double angle = panel_angles_[i];
    geometry_msgs::msg::Pose pose;
    pose.position.x = -radius_ * std::cos(angle);
    pose.position.y = -radius_ * std::sin(angle);
    pose.position.z = z_offsets_[i];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle + M_PI);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
  }

  return offsets;
}

const ObservationData *OutpostArmorTracker::select_observation(
    const std::vector<ObservationData> &obs) const {
  if (obs.empty()) return nullptr;

  const ObservationData *best = &obs.front();
  for (const auto &o : obs) {
    if (o.timestamp.has_value()) {
      if (!best->timestamp.has_value() ||
          o.timestamp.value() > best->timestamp.value()) {
        best = &o;
        continue;
      }
    }

    if ((!o.timestamp.has_value() && !best->timestamp.has_value()) &&
        (o.confidence > best->confidence)) {
      best = &o;
    }
  }
  return best;
}

std::array<OutpostArmorTracker::PanelHypothesis, 3>
OutpostArmorTracker::evaluate_hypotheses(const ObservationData &obs,
                                         double predicted_center_z,
                                         double history_center_z) const {
  std::array<PanelHypothesis, 3> hyps;

  const bool has_history = std::isfinite(history_center_z);
  const double w_yaw = std::max(0.0, config_.outpost.weight_yaw);
  const double w_z_state = std::max(0.0, config_.outpost.weight_z_state);
  const double w_z_hist = std::max(0.0, config_.outpost.weight_z_history);

  for (int i = 0; i < 3; ++i) {
    PanelHypothesis h;
    h.panel_id = i;
    h.center_yaw = normalize_angle(obs.yaw - panel_angles_[i]);
    h.center_z = obs.z - z_offsets_[i];

    const double yaw_err = angle_abs_diff(h.center_yaw, center_yaw_est_);
    const double z_state_err = std::abs(h.center_z - predicted_center_z);
    const double z_hist_err = has_history ? std::abs(h.center_z - history_center_z)
                                          : 0.0;

    h.cost = w_yaw * yaw_err + w_z_state * z_state_err + w_z_hist * z_hist_err;
    hyps[i] = h;
  }

  return hyps;
}

void OutpostArmorTracker::compute_probabilities(
    std::array<PanelHypothesis, 3> &hyps) const {
  const double temp = std::max(1e-3, config_.outpost.softmax_temperature);
  double min_cost = hyps[0].cost;
  for (int i = 1; i < 3; ++i) {
    min_cost = std::min(min_cost, hyps[i].cost);
  }

  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double scaled = -(hyps[i].cost - min_cost) / temp;
    hyps[i].probability = std::exp(scaled);
    sum += hyps[i].probability;
  }

  sum = std::max(sum, 1e-12);
  for (int i = 0; i < 3; ++i) {
    hyps[i].probability /= sum;
  }
}

void OutpostArmorTracker::update_mode_from_entropy(int best_panel,
                                                   double entropy_norm,
                                                   double max_prob) {
  const double h_enter = std::clamp(config_.outpost.entropy_enter, 0.0, 1.0);
  const double h_exit = std::clamp(config_.outpost.entropy_exit, 0.0, 1.0);
  const double p_enter = std::clamp(config_.outpost.max_prob_enter, 0.0, 1.0);
  const double p_exit = std::clamp(config_.outpost.max_prob_exit, 0.0, 1.0);
  const int stable_required = std::max(1, config_.outpost.stable_frames);

  if (mode_ == TrackMode::STRUCTURED_3_ARMORS) {
    const bool to_ambiguous = (entropy_norm > h_enter) || (max_prob < p_enter);
    if (to_ambiguous) {
      mode_ = TrackMode::AMBIGUOUS_SINGLE_ARMOR;
      stable_counter_ = 0;
    }
    return;
  }

  if (best_panel == last_best_panel_id_) {
    ++stable_counter_;
  } else {
    last_best_panel_id_ = best_panel;
    stable_counter_ = 1;
  }

  const bool to_structured = (entropy_norm < h_exit) && (max_prob > p_exit) &&
                             (stable_counter_ >= stable_required);
  if (to_structured) {
    mode_ = TrackMode::STRUCTURED_3_ARMORS;
  }
}

double OutpostArmorTracker::history_center_z_median() const {
  std::vector<double> values(center_z_history_.begin(), center_z_history_.end());
  return median_of_vector(values);
}

void OutpostArmorTracker::push_center_z_history(double center_z) {
  center_z_history_.push_back(center_z);
  const int window = std::max(1, config_.outpost.z_history_window);
  while (static_cast<int>(center_z_history_.size()) > window) {
    center_z_history_.pop_front();
  }
}

bool OutpostArmorTracker::update_internal_state(const ObservationData &obs,
                                                const PanelHypothesis &best,
                                                double /*dt*/) {
  ObservationData obs_with_panel = obs;
  obs_with_panel.panel_id = best.panel_id;

  // Use max panel probability to scale measurement trust in ambiguous periods.
  const double position_confidence = std::clamp(max_prob_, 0.10, 1.0);
  if (!outpost_ukf_.update_with_panel(obs_with_panel, best.panel_id,
                                      position_confidence)) {
    return false;
  }

  apply_motion_constraints_from_config();
  sync_internal_state_from_filter();
  return true;
}

void OutpostArmorTracker::apply_motion_constraints_from_config() {
  auto &x = outpost_ukf_.x();
  const auto idx = outpost_ukf_.state_idx();

  if (config_.outpost.assume_static_center) {
    const double lin_damping =
        std::clamp(config_.outpost.linear_velocity_damping, 0.0, 1.0);
    x(idx.VX()) *= lin_damping;
    x(idx.VY()) *= lin_damping;
    x(idx.VZ()) *= lin_damping;
  }

  const double yaw_damping =
      std::clamp(config_.outpost.yaw_rate_damping, 0.0, 1.0);
  x(idx.DELTA_RATE()) *= yaw_damping;

  Eigen::Vector3d vel(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  const double max_center_speed =
      std::max(0.01, config_.outpost.max_center_speed);
  const double speed_norm = vel.norm();
  if (speed_norm > max_center_speed) {
    const double ratio = max_center_speed / speed_norm;
    x(idx.VX()) *= ratio;
    x(idx.VY()) *= ratio;
    x(idx.VZ()) *= ratio;
  }

  const double max_yaw_rate = std::max(0.01, config_.outpost.max_yaw_rate);
  x(idx.DELTA_RATE()) =
      std::clamp(x(idx.DELTA_RATE()), -max_yaw_rate, max_yaw_rate);
}

void OutpostArmorTracker::sync_internal_state_from_filter() {
  const auto &x = outpost_ukf_.x();
  const auto idx = outpost_ukf_.state_idx();
  center_position_est_ = outpost_ukf_.get_center_position();
  center_velocity_est_ = Eigen::Vector3d(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  center_yaw_est_ = outpost_ukf_.get_yaw();
  yaw_rate_est_ = x(idx.DELTA_RATE());
}

void OutpostArmorTracker::update_publish_state() {
  if (is_ambiguous_single_mode()) {
    const double theta = center_yaw_est_ + panel_angles_[selected_panel_id_];
    publish_position_ = center_position_est_ +
                        Eigen::Vector3d(radius_ * std::cos(theta),
                                        radius_ * std::sin(theta),
                                        z_offsets_[selected_panel_id_]);

    const Eigen::Vector3d tangential(
        -yaw_rate_est_ * radius_ * std::sin(theta),
        yaw_rate_est_ * radius_ * std::cos(theta), 0.0);
    publish_velocity_ = center_velocity_est_ + tangential;
    return;
  }

  publish_position_ = center_position_est_;
  publish_velocity_ = center_velocity_est_;
}

}  // namespace fyt::auto_aim
