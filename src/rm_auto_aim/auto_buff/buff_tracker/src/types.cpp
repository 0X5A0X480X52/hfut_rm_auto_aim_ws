#include "types.hpp"

#include <gtsam/geometry/Rot2.h>

#include <array>
#include <cmath>
#include <functional>

namespace {

// Inlined from tools/math/angle_tools.hpp
Eigen::Quaterniond rpyToQuaterniond(const Eigen::Vector3d &rpy_angle) {
  Eigen::AngleAxisd roll(rpy_angle.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(rpy_angle.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(rpy_angle.z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q{yaw * pitch * roll};
  q.normalize();
  return q;
}

Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R) {
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2)));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return {roll, pitch, yaw};
}

} // namespace

auto_buff::BuffBlade::BuffBlade(const rm_interfaces::msg::RuneTarget &msg)
    : frame_id(msg.header.frame_id),
      stamp(std::chrono::nanoseconds{
          static_cast<int64_t>(msg.header.stamp.sec) * 1'000'000'000LL +
          static_cast<int64_t>(msg.header.stamp.nanosec)}),
      heart_beat(!msg.is_lost),
      color(EnemyColor::Blue),
      type(msg.blade_type == rm_interfaces::msg::RuneTarget::BLADE_INACTIVATED
               ? BuffBladeType::Inactivated
               : msg.blade_type == rm_interfaces::msg::RuneTarget::BLADE_ACTIVATED
                     ? BuffBladeType::Activated
                     : BuffBladeType::Unknown),
      confidence(msg.confidence),
      points({
          .r_center = {static_cast<float>(msg.pts[0].x),
                       static_cast<float>(msg.pts[0].y)},
          .bottom_right = {static_cast<float>(msg.pts[1].x),
                           static_cast<float>(msg.pts[1].y)},
          .top_right = {static_cast<float>(msg.pts[2].x),
                        static_cast<float>(msg.pts[2].y)},
          .top_left = {static_cast<float>(msg.pts[3].x),
                       static_cast<float>(msg.pts[3].y)},
          .bottom_left = {static_cast<float>(msg.pts[4].x),
                          static_cast<float>(msg.pts[4].y)},
      }) {}

Eigen::Vector3d auto_buff::BladePositionRoll::getHitPosition() const {
  auto z_hit = std::cos(this->roll.theta()) * BUFF_RADIUS + this->position.z();
  auto horizontal_bias = -std::sin(this->roll.theta()) * BUFF_RADIUS;
  auto center_aim_yaw = std::atan2(this->position.y(), this->position.x());
  auto x_hit = this->position.x() - horizontal_bias * std::sin(center_aim_yaw);
  auto y_hit = this->position.y() + horizontal_bias * std::cos(center_aim_yaw);
  return {x_hit, y_hit, z_hit};
}

auto_buff::BuffState auto_buff::BuffState::getStateWithPredictFunc(
    std::function<BuffState(const BuffState &, double)> &&func) const {
  auto state = *this;
  state.predict_fn_ = func;
  return state;
}

auto_buff::BuffState auto_buff::BuffState::predict(double dt) const {
  return predict_fn_(*this, dt);
}

std::array<auto_buff::BladePositionRoll, 5>
auto_buff::BuffState::blades() const {
  std::array<BladePositionRoll, 5> blades;
  for (int i = 0; i < 5; i++) {
    blades.at(i) = BladePositionRoll{
        .type = inactivated_flag.at(i) ? BuffBladeType::Inactivated
                                       : BuffBladeType::Activated,
        .position = center_position,
        .roll = center_roll + i * 2 * M_PI / 5,
    };
  }
  return blades;
}

Eigen::Quaterniond auto_buff::BladePositionRPYPoints::getRotation() const {
  return rpyToQuaterniond({roll.theta(), pitch, yaw});
}

auto_buff::BladePositionRPYPoints
auto_buff::BladePositionRPYPoints::transform(const Eigen::Isometry3d &T) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(this->position);
  pose.rotate(this->getRotation());
  auto result = T * pose;
  auto rpy = rotationMatrixToRPY(result.rotation());
  BladePositionRPYPoints blade{*this};
  blade.position = result.translation();
  blade.roll = gtsam::Rot2::fromAngle(rpy(0));
  blade.pitch = rpy(1);
  blade.yaw = rpy(2);
  return blade;
}

auto_buff::SmallBuffState::SmallBuffState(const BuffState &state, double vroll)
    : BuffState(state), center_vroll(vroll) {}

auto_buff::BigBuffState::BigBuffState(const BuffState &state,
                                      double dt_from_start, double a,
                                      double omega, double c, double d,
                                      double vroll)
    : BuffState(state), dt_from_start(dt_from_start), a(a), omega(omega), c(c),
      d(d), center_vroll(vroll) {}
