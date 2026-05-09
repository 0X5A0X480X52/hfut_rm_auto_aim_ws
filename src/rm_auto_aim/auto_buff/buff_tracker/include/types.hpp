#pragma once

#include "rm_interfaces/msg/rune_target.hpp"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <gtsam/geometry/Rot2.h>
#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>

namespace auto_buff {

// Inlined from types/EnemyColor.hpp
enum class EnemyColor : uint8_t { Red = 0, Blue = 1, Extinguished = 2 };

// Inlined from types/BuffBladeType.hpp
enum class BuffBladeType : uint8_t { Inactivated = 0, Activated = 1, Unknown = 255 };

// Inlined from types/TaskMode.hpp
enum class TaskMode { Idle, Armor, SmallBuff, BigBuff };

// NOTE:
//                ^ z
//                |
//                0
//                |
//        1       |        4
//                |
// <--------------x
// y
//           2          3
enum class BuffBladeIndex {
  _0 = 0,
  _1,
  _2,
  _3,
  _4,
};

// Rune object points
// r_tag, bottom_right, top_right, top_left, bottom_left
inline const std::vector<cv::Point3f> BUFF_BLADE_OBJ_POINTS{
    cv::Point3f(0, 0, 0) / 1000,        cv::Point3f(0, -186, 541.5) / 1000,
    cv::Point3f(0, -160, 858.5) / 1000, cv::Point3f(0, 160, 858.5) / 1000,
    cv::Point3f(0, 186, 541.5) / 1000,
};
constexpr auto BUFF_RADIUS{0.7};
inline const cv::Point3f BUFF_BLADE_HIT_OBJ_POINT(0, 0, BUFF_RADIUS);

enum class BuffPointPosition {
  Center,
  BottomRight,
  TopRight,
  TopLeft,
  BottomLeft,
};

struct BuffBladePoints {
  cv::Point2f r_center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;
};

struct BuffBlade {
  BuffBlade() = default;
  explicit BuffBlade(const rm_interfaces::msg::RuneTarget &msg);
  std::string frame_id;
  std::chrono::system_clock::time_point stamp;
  bool heart_beat;
  EnemyColor color;
  BuffBladeType type;
  float confidence;
  BuffBladePoints points;
};

struct BladePositionRoll {
  BuffBladeType type;
  Eigen::Vector3d position;
  gtsam::Rot2 roll;
  Eigen::Vector3d getHitPosition() const;
};

struct BladePositionRPYPoints : BladePositionRoll {
  double pitch;
  double yaw;
  std::array<cv::Point2f, 5> points;
  Eigen::Quaterniond getRotation() const;
  BladePositionRPYPoints transform(const Eigen::Isometry3d &T) const;
};

struct BuffState {
  BuffState() = default;
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  double center_roll{0};
  std::array<bool, 5> inactivated_flag{false, false, false, false, false};
  BuffState getStateWithPredictFunc(
      std::function<BuffState(const BuffState &, double)> &&func) const;
  std::array<BladePositionRoll, 5> blades() const;

private:
  friend class TrackerNode;
  BuffState predict(double dt) const;
  std::function<BuffState(const BuffState &self, double dt)> predict_fn_;
};

struct SmallBuffState : public BuffState {
  SmallBuffState() = default;
  SmallBuffState(const BuffState &state, double vroll);
  double center_vroll{0};
};

struct IntervalTimeBuffRoll {
  double dt_from_start;
  double buff_roll;
};

struct BuffParamVRollDeltaTime {
  std::array<double, 4> param;
  double vroll;
  double dt_start_to_update;
  double dt_start_to_fit;
};

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
inline auto getBuffCurvePoint(T1 t, T2 a, T3 omega, T4 c, T5 d, T6 sign) {
  return (-a / omega * ceres::cos(omega * (t + d)) + (2.09 - a) * (t + d) + c) *
         sign;
}

struct BigBuffState : public BuffState {
  BigBuffState() = default;
  BigBuffState(const BuffState &state, double dt_from_start, double a,
               double omega, double c, double d, double vroll);
  double dt_from_start{0};
  double a{0};
  double omega{0};
  double c{0};
  double d{0};
  double center_vroll{0};
};

struct TrackState {
  enum class State {
    LOST,
    TEMPLOST,
    TRACKING,
  } state{State::LOST};
  std::uint64_t k{0};
  std::chrono::system_clock::time_point stamp_last_update;
  std::chrono::system_clock::time_point stamp_last_tracking;
};

struct BuffMatchResult {
  BuffBladeIndex index;
  double distance;
  double roll_diff;
};

struct YawPitchFlyTimeIndex {
  double yaw;
  double pitch;
  double fly_time;
  BuffBladeIndex index;
};

struct BuffIndexPredictTime {
  BuffBladeIndex index;
  double predict_time;
};

} // namespace auto_buff
