#include "targets.hpp"
#include "buff_fitter.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "types.hpp"

#include "opencv2/calib3d.hpp"
#include "opencv2/core/types.hpp"
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using namespace gtsam::symbol_shorthand;

namespace {

// Inlined from tools/math/angle_tools.hpp
inline double angle2Radian(double angle) {
  return angle * M_PI / 180.0;
}

inline double limitRadian(double radian,
                          std::pair<double, double> range = {
                              -M_PI, M_PI}) {
  const double low = range.first;
  const double high = range.second;
  const double width = high - low;
  radian = std::fmod(radian - low, width);
  if (radian < 0.0)
    radian += width;
  return radian + low;
}

inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R) {
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2)));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return {roll, pitch, yaw};
}

const char *trackStateToString(auto_buff::TrackState::State s) {
  switch (s) {
  case auto_buff::TrackState::State::LOST: return "LOST";
  case auto_buff::TrackState::State::TEMPLOST: return "TEMPLOST";
  case auto_buff::TrackState::State::TRACKING: return "TRACKING";
  }
  return "UNKNOWN";
}

} // namespace

auto_buff::BuffTarget::BuffTarget(rclcpp::Logger logger,
                                  const BuffBladeMatchConfig &match_conf,
                                  const BuffBladeNoiseConfig &blade_config,
                                  const cv::Mat &camera_matrix,
                                  const cv::Mat &distortion_coefficients)
    : logger_(std::move(logger)), match_config_(match_conf),
      blade_config_(blade_config), camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients) {}

double auto_buff::BuffTarget::get(const std::string &str) const { return 0; }

std::optional<auto_buff::BladePositionRPYPoints>
auto_buff::BuffTarget::solvePNP(const BuffBlade &blade) const {
  std::array image_points{
      blade.points.r_center, blade.points.bottom_right, blade.points.top_right,
      blade.points.top_left, blade.points.bottom_left,
  };
  cv::Mat rvec, tvec;
  if (auto success = cv::solvePnP(BUFF_BLADE_OBJ_POINTS, image_points,
                                  camera_matrix_, distortion_coefficients_,
                                  rvec, tvec, false, cv::SOLVEPNP_IPPE);
      !success)
    return std::nullopt;
  BladePositionRPYPoints result;
  result.position.x() = tvec.at<double>(0);
  result.position.y() = tvec.at<double>(1);
  result.position.z() = tvec.at<double>(2);
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  cv::cv2eigen(R_cv, R);
  Eigen::Vector3d rpy = rotationMatrixToRPY(R);
  result.type = blade.type;
  result.roll = gtsam::Rot2::fromAngle(rpy(0));
  result.pitch = rpy(1);
  result.yaw = rpy(2);
  result.points = image_points;
  return result;
}

auto_buff::BuffState auto_buff::BuffTarget::getTargetStateFromBlade(
    const BladePositionRPYPoints &blade) {
  BuffState state;
  state.center_position = blade.position;
  state.center_roll = blade.roll.theta();
  return state;
}

std::optional<auto_buff::BuffMatchResult> auto_buff::BuffTarget::matchBlade(
    const BuffState &state, const BladePositionRPYPoints &blade_odom) const {
  std::vector<BuffMatchResult> results;
  auto i{0};
  for (const auto &blade : state.blades()) {
    auto distance = (blade.position - blade_odom.position).norm();
    auto roll_diff = std::abs(blade_odom.roll.localCoordinates(blade.roll).x());
    if (distance <= match_config_.max_match_distance_m &&
        roll_diff <=
            angle2Radian(match_config_.max_match_roll_diff_degree))
      results.emplace_back(static_cast<BuffBladeIndex>(i), distance, roll_diff);
    i++;
  }
  std::sort(results.begin(), results.end(),
            [](const BuffMatchResult &a, const BuffMatchResult &b) {
              return a.roll_diff < b.roll_diff;
            });
  return results.empty() ? std::nullopt : std::optional{results.front()};
}

std::vector<
    std::pair<auto_buff::BladePositionRPYPoints, auto_buff::BuffBladeIndex>>
auto_buff::BuffTarget::matchBlades(
    const BuffState &state,
    const std::vector<BladePositionRPYPoints> &blades_camera,
    const Eigen::Isometry3d &T_camera_to_odom) const {
  std::array<bool, 5> used_index{false, false, false, false, false};
  std::vector<
      std::pair<auto_buff::BladePositionRPYPoints, auto_buff::BuffBladeIndex>>
      match_result;
  for (const auto &blade : blades_camera)
    if (auto match_result_opt =
            matchBlade(state, blade.transform(T_camera_to_odom));
        match_result_opt.has_value()) {
      if (auto index = match_result_opt->index;
          !used_index.at(static_cast<int>(index))) {
        match_result.emplace_back(blade, index);
        used_index.at(static_cast<int>(index)) = true;
      }
    }
  return match_result;
}

void auto_buff::BuffTarget::addBladeReprojValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    gtsam::Key blade_pose_key, const BladePositionRPYPoints &blade_camera,
    std::uint64_t k) const {
  values.insert(blade_pose_key,
                gtsam::Pose3{gtsam::Rot3{blade_camera.getRotation()},
                             blade_camera.position});
  for (auto position : std::array{
           BuffPointPosition::Center,
           BuffPointPosition::BottomRight,
           BuffPointPosition::TopRight,
           BuffPointPosition::TopLeft,
           BuffPointPosition::BottomLeft,

       }) {
    auto point = blade_camera.points.at(static_cast<int>(position));
    graph.add(BuffBladeReprojFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{
            blade_config_.pixel_error.x,
            blade_config_.pixel_error.y,
        }),
        blade_pose_key,
        camera_matrix_,
        distortion_coefficients_,
        position,
        {point.x, point.y},
    });
  }
}

inline gtsam::Key getBladePoseKeyFromIndex(auto_buff::BuffBladeIndex index,
                                           std::uint64_t k) {
  if (index == auto_buff::BuffBladeIndex::_0)
    return G(k);
  if (index == auto_buff::BuffBladeIndex::_1)
    return H(k);
  if (index == auto_buff::BuffBladeIndex::_2)
    return J(k);
  if (index == auto_buff::BuffBladeIndex::_3)
    return K(k);
  if (index == auto_buff::BuffBladeIndex::_4)
    return L(k);
  return {};
}

void auto_buff::BuffTarget::addBladeValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const std::vector<std::pair<BladePositionRPYPoints, BuffBladeIndex>>
        &blades_camera_indexs,
    const Eigen::Isometry3d &T_camera_to_odom, std::uint64_t k) const {
  for (const auto &[blade, index] : blades_camera_indexs) {
    auto blade_pose_key = getBladePoseKeyFromIndex(index, k);
    addBladeReprojValuesFactors(values, graph, blade_pose_key, blade, k);
    graph.add(BuffBladeFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
            blade_config_.position_noise_m.x,
            blade_config_.position_noise_m.y,
            blade_config_.position_noise_m.z,
            angle2Radian(blade_config_.roll_noise_degree),
        }),
        blade_pose_key,
        R(k),
        X(k),
        T_camera_to_odom,
        index,
    });
  }
  RCLCPP_DEBUG(logger_, "[Buff Target]: add %zu buff factors. k = %lu.",
               blades_camera_indexs.size(), k);
}

auto_buff::SmallBuffTarget::SmallBuffTarget(
    rclcpp::Logger logger, const SmallBuffConfig &config,
    const cv::Mat &camera_matrix, const cv::Mat &distortion_coefficients)
    : BuffTarget(std::move(logger), config.match_conf, config.blade_conf,
                 camera_matrix, distortion_coefficients),
      config_(config) {
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = std::chrono::system_clock::from_time_t(0);
  track_state_.stamp_last_tracking = std::chrono::system_clock::from_time_t(0);
  track_state_.k = 0;
}

double auto_buff::SmallBuffTarget::get(const std::string &str) const {
  std::scoped_lock lk{state_mtx_};
  if (str == "vroll")
    return target_state_.center_vroll;
  return 0;
}

auto_buff::SmallBuffState
auto_buff::SmallBuffTarget::predictBuffState(const BuffState &state,
                                             double vroll, double dt) {
  SmallBuffState state_pre{state, vroll};
  state_pre.center_roll = limitRadian(state.center_roll + vroll * dt);
  return state_pre;
}

auto_buff::SmallBuffState
auto_buff::SmallBuffTarget::predictBuffState(double dt) const {
  return predictBuffState(target_state_, target_state_.center_vroll, dt);
}

std::pair<auto_buff::BuffState, auto_buff::TrackState>
auto_buff::SmallBuffTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  return {
      target_state_.getStateWithPredictFunc(
          [vroll = target_state_.center_vroll](const BuffState &state,
                                               double dt) {
            return predictBuffState(state, vroll, dt);
          }),
      track_state_,
  };
}

void auto_buff::SmallBuffTarget::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const SmallBuffState &state, std::uint64_t k, double dt) const {
  values.insert(X(k), state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(state.center_roll));
  values.insert(W(k), state.center_vroll);
  if (k == 0) {
    graph.addPrior(X(0), state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.center_conf.position_prior_noise_m.x,
                       config_.center_conf.position_prior_noise_m.y,
                       config_.center_conf.position_prior_noise_m.z,
                   }));
    graph.addPrior(R(0), gtsam::Rot2::fromAngle(state.center_roll),
                   gtsam::noiseModel::Isotropic::Sigma(
                       1, angle2Radian(
                              config_.center_conf.roll_prior_noise_degree)));
    graph.addPrior(W(0), state.center_vroll,
                   gtsam::noiseModel::Isotropic::Sigma(
                       1, config_.center_conf.vroll_prior_noise_rad));
  } else {
    graph.add(ConstPositionFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.center_conf.position_consistency_noise_m.x,
            config_.center_conf.position_consistency_noise_m.y,
            config_.center_conf.position_consistency_noise_m.z,
        }),
        X(k - 1),
        X(k),
    });
    graph.add(RollFactor{
        gtsam::noiseModel::Isotropic::Sigma(
            1, angle2Radian(config_.center_conf.roll_noise_degree)),
        R(k - 1),
        W(k - 1),
        R(k),
        dt,
    });
    graph.add(ConstVRollFactor{
        gtsam::noiseModel::Isotropic::Sigma(
            1, config_.center_conf.vroll_noise_rad),
        W(k - 1),
        W(k),
    });
  }
  RCLCPP_DEBUG(logger_,
               "[SmallBuff]: add motion values factors. k = %lu, dt = %.3f.", k,
               dt);
}

auto_buff::TrackState::State auto_buff::SmallBuffTarget::track(
    const std::vector<BuffBlade> &blades,
    const std::chrono::system_clock::time_point &stamp,
    const Eigen::Isometry3d &T_camera_to_odom) {
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - track_state_.stamp_last_update)
                  .count();
  std::vector<BladePositionRPYPoints> blades_camera;
  for (const auto &blade : blades)
    if (auto result = this->solvePNP(blade); result.has_value())
      blades_camera.emplace_back(result.value());
  auto [estimated_target_state, updated_track_state] =
      update(blades_camera, dt, T_camera_to_odom);
  if (updated_track_state == TrackState::State::TRACKING) {
    if (track_state_.state != TrackState::State::TRACKING) {
      RCLCPP_INFO(logger_, "[SmallBuff]: %s -> TRACKING. dt=%.3f, k=%lu.",
                  trackStateToString(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    if (track_state_.state != TrackState::State::TEMPLOST) {
      RCLCPP_INFO(logger_, "[SmallBuff]: TRACKING -> TEMPLOST. dt=%.3f, k=%lu.",
                  dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::LOST) {
    if (track_state_.state != TrackState::State::LOST) {
      RCLCPP_INFO(logger_, "[SmallBuff]: %s -> LOST. dt=%.3f, k=%lu.",
                  trackStateToString(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = SmallBuffState{};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    isam2_ = gtsam::ISAM2{};
  }
  return track_state_.state;
}

std::pair<auto_buff::SmallBuffState, auto_buff::TrackState::State>
auto_buff::SmallBuffTarget::update(
    const std::vector<BladePositionRPYPoints> &blades_camera, double dt,
    const Eigen::Isometry3d &T_camera_to_odom) const {
  auto dt_tracking_to_update =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          track_state_.stamp_last_update - track_state_.stamp_last_tracking)
          .count();
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    RCLCPP_INFO(logger_, "[SmallBuff]: Time out! dt=%.3f.",
                dt + dt_tracking_to_update);
    return {{}, TrackState::State::LOST};
  }
  auto target_state = predictBuffState(dt);
  if (track_state_.state == TrackState::State::LOST) {
    if (blades_camera.empty())
      return {{}, TrackState::State::LOST};
    else
      target_state =
          SmallBuffState{getTargetStateFromBlade(
                             blades_camera.front().transform(T_camera_to_odom)),
                         0};
  }
  auto matched_blades =
      matchBlades(target_state, blades_camera, T_camera_to_odom);
  if (matched_blades.size() < blades_camera.size()) {
    RCLCPP_DEBUG(logger_, "[SmallBuff]: Miss match %zu armors! k = %lu.",
                 blades_camera.size() - matched_blades.size(), track_state_.k);
  }
  target_state.inactivated_flag.fill(false);
  for (const auto &[blade, index] : matched_blades)
    if (blade.type == BuffBladeType::Inactivated)
      target_state.inactivated_flag.at(static_cast<int>(index)) = true;

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addBladeValuesFactors(values, graph, matched_blades, T_camera_to_odom,
                        track_state_.k);

  try {
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_roll =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vroll =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    return {target_state, matched_blades.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[SmallBuff]: %s\ncurrent k: %lu.", e.what(),
                 track_state_.k);
    return {{}, TrackState::State::LOST};
  }
}

// ---- BigBuffTarget ----

auto_buff::BigBuffTarget::BigBuffTarget(rclcpp::Logger logger,
                                        const BigBuffConfig &config,
                                        const cv::Mat &camera_matrix,
                                        const cv::Mat &distortion_coefficients)
    : BuffTarget(logger, config.match_conf, config.blade_conf, camera_matrix,
                 distortion_coefficients),
      config_(config), fitter_(logger, config_.fitter_conf) {
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = std::chrono::system_clock::from_time_t(0);
  track_state_.stamp_last_tracking = std::chrono::system_clock::from_time_t(0);
  track_state_.k = 0;
}

auto_buff::BigBuffState auto_buff::BigBuffTarget::predictBuffState(
    const BuffState &state, double dt, double dt_from_start, double a,
    double omega, double c, double d, double vroll) {
  BigBuffState state_pre{state, dt + dt_from_start, a, omega, c, d, vroll};
  state_pre.center_roll = getBuffCurvePoint(state_pre.dt_from_start, a, omega,
                                            c, d, vroll > 0.0 ? 1.0 : -1.0);
  return state_pre;
}

auto_buff::BigBuffState
auto_buff::BigBuffTarget::predictBuffState(double dt) const {
  return predictBuffState(target_state_, dt, target_state_.dt_from_start,
                          target_state_.a, target_state_.omega, target_state_.c,
                          target_state_.d, target_state_.center_vroll);
}

std::pair<auto_buff::BuffState, auto_buff::TrackState>
auto_buff::BigBuffTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  return {
      target_state_.getStateWithPredictFunc(
          [dt_from_start = target_state_.dt_from_start, a = target_state_.a,
           omega = target_state_.omega, c = target_state_.c,
           d = target_state_.d, vroll = target_state_.center_vroll](
              const BuffState &state, double dt) {
            auto make_predict_fn = [a, omega, c, d, vroll](auto &&self,
                                                           double dt_base)
                -> std::function<BuffState(const BuffState &, double)> {
              return [a, omega, c, d, vroll, dt_base,
                      self](const BuffState &state_inner,
                            double dt_inner) -> BuffState {
                auto predicted_state = predictBuffState(
                    state_inner, dt_inner, dt_base, a, omega, c, d, vroll);
                return predicted_state.getStateWithPredictFunc(
                    self(self, dt_base + dt_inner));
              };
            };
            return make_predict_fn(make_predict_fn, dt_from_start)(state, dt);
          }),
      track_state_,
  };
}

double auto_buff::BigBuffTarget::get(const std::string &str) const {
  std::scoped_lock lk{state_mtx_};
  if (str == "dt_from_start")
    return target_state_.dt_from_start;
  if (str == "a")
    return target_state_.a;
  if (str == "omega")
    return target_state_.omega;
  if (str == "c")
    return target_state_.c;
  if (str == "d")
    return target_state_.d;
  if (str == "vroll")
    return static_cast<double>(target_state_.center_vroll);
  return 0;
}

void auto_buff::BigBuffTarget::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const BigBuffState &state, std::uint64_t k, double dt) const {
  values.insert(X(k), state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(state.center_roll));
  values.insert(W(k), state.center_vroll);
  if (k == 0) {
    graph.addPrior(X(0), state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.center_conf.position_prior_noise_m.x,
                       config_.center_conf.position_prior_noise_m.y,
                       config_.center_conf.position_prior_noise_m.z,
                   }));
    graph.addPrior(R(0), gtsam::Rot2::fromAngle(state.center_roll),
                   gtsam::noiseModel::Isotropic::Sigma(
                       1, angle2Radian(
                              config_.center_conf.roll_prior_noise_degree)));
    graph.addPrior(W(0), 0.0,
                   gtsam::noiseModel::Isotropic::Sigma(
                       1, config_.center_conf.vroll_prior_noise_rad));
  } else {
    graph.add(ConstPositionFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.center_conf.position_consistency_noise_m.x,
            config_.center_conf.position_consistency_noise_m.y,
            config_.center_conf.position_consistency_noise_m.z,
        }),
        X(k - 1),
        X(k),
    });
    graph.add(RollFactor{
        gtsam::noiseModel::Isotropic::Sigma(
            1, angle2Radian(config_.center_conf.roll_noise_degree)),
        R(k - 1),
        W(k - 1),
        R(k),
        dt,
    });
    graph.add(ConstVRollFactor{
        gtsam::noiseModel::Isotropic::Sigma(
            1, config_.center_conf.vroll_noise_rad),
        W(k - 1),
        W(k),
    });
  }
  RCLCPP_DEBUG(logger_,
               "[BigBuff]: add motion values factors. k = %lu, dt = %.3f.", k,
               dt);
}

std::pair<auto_buff::BigBuffState, auto_buff::TrackState::State>
auto_buff::BigBuffTarget::update(
    const std::vector<BladePositionRPYPoints> &blades_camera, double dt,
    const Eigen::Isometry3d &T_camera_to_odom) const {
  auto dt_tracking_to_update =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          track_state_.stamp_last_update - track_state_.stamp_last_tracking)
          .count();
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    RCLCPP_INFO(logger_, "[BigBuff]: Time out! dt=%.3f.",
                dt + dt_tracking_to_update);
    return {{}, TrackState::State::LOST};
  }
  auto target_state = target_state_;
  target_state.center_roll =
      target_state.center_roll + dt * target_state.center_vroll;
  if (track_state_.state == TrackState::State::LOST) {
    if (blades_camera.empty())
      return {{}, TrackState::State::LOST};
    else
      target_state = BigBuffState{
          getTargetStateFromBlade(
              blades_camera.front().transform(T_camera_to_odom)),
          0.0,
          0.9125,
          1.942,
          0,
          0,
          0,
      };
  }
  auto matched_blades =
      matchBlades(target_state, blades_camera, T_camera_to_odom);
  if (matched_blades.size() < blades_camera.size()) {
    RCLCPP_DEBUG(logger_, "[BigBuff]: Miss match %zu armors! k = %lu.",
                 blades_camera.size() - matched_blades.size(), track_state_.k);
  }
  target_state.inactivated_flag.fill(false);
  for (const auto &[blade, index] : matched_blades)
    if (blade.type == BuffBladeType::Inactivated)
      target_state.inactivated_flag.at(static_cast<int>(index)) = true;

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addBladeValuesFactors(values, graph, matched_blades, T_camera_to_odom,
                        track_state_.k);

  try {
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_roll =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vroll =
        isam2_.calculateEstimate<double>(W(track_state_.k));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[BigBuff]: %s\ncurrent k: %lu.", e.what(),
                 track_state_.k);
    return {{}, TrackState::State::LOST};
  }

  if (auto fit_result_opt = this->fitter_.update(track_state_.k == 0 ? 0 : dt,
                                                 target_state.center_roll,
                                                 target_state.center_vroll);
      fit_result_opt.has_value()) {
    auto [param, direction, dt_start_to_update, dt_start_to_fit] =
        fit_result_opt.value();
    target_state = predictBuffState(
        target_state, dt_start_to_update - dt_start_to_fit, dt_start_to_fit,
        param[0], param[1], param[2], param[3], target_state.center_vroll);
  } else {
    target_state.dt_from_start += dt;
    RCLCPP_DEBUG(logger_, "[BigBuff]: Fitter not converged.");
  }
  return {target_state, matched_blades.empty() ? TrackState::State::TEMPLOST
                                               : TrackState::State::TRACKING};
}

auto_buff::TrackState::State auto_buff::BigBuffTarget::track(
    const std::vector<BuffBlade> &blades,
    const std::chrono::system_clock::time_point &stamp,
    const Eigen::Isometry3d &T_camera_to_odom) {
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - track_state_.stamp_last_update)
                  .count();
  std::vector<BladePositionRPYPoints> blades_camera;
  for (const auto &blade : blades)
    if (auto result = this->solvePNP(blade); result.has_value())
      blades_camera.emplace_back(result.value());
  auto [estimated_target_state, updated_track_state] =
      update(blades_camera, dt, T_camera_to_odom);
  if (updated_track_state == TrackState::State::TRACKING) {
    if (track_state_.state != TrackState::State::TRACKING) {
      RCLCPP_INFO(logger_, "[BigBuff]: %s -> TRACKING. dt=%.3f, k=%lu.",
                  trackStateToString(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    if (track_state_.state != TrackState::State::TEMPLOST) {
      RCLCPP_INFO(logger_, "[BigBuff]: TRACKING -> TEMPLOST. dt=%.3f, k=%lu.",
                  dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::LOST) {
    if (track_state_.state != TrackState::State::LOST) {
      RCLCPP_INFO(logger_, "[BigBuff]: %s -> LOST. dt=%.3f, k=%lu.",
                  trackStateToString(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = BigBuffState{};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    isam2_ = gtsam::ISAM2{};
    fitter_.reset();
  }
  return track_state_.state;
}
