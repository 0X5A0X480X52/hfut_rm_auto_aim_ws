#include "gimbal_pipeline/adapters/buff_target_adapter.hpp"

#include <algorithm>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace fyt::auto_aim::adapters
{

BuffTargetAdapter::BuffTargetAdapter(rclcpp::Node & node, const Config & config)
: node_(node), config_(config), enabled_(config.enable)
{
  if (!config_.enable) {
    return;
  }

  sub_ = node_.create_subscription<rm_interfaces::msg::TrackedRobot>(
    config_.topic,
    rclcpp::SensorDataQoS(),
    std::bind(&BuffTargetAdapter::onTrackedRobot, this, std::placeholders::_1));
}

void BuffTargetAdapter::setEnabled(bool enable)
{
  std::scoped_lock lk(mutex_);
  enabled_ = enable;
}

void BuffTargetAdapter::onTrackedRobot(const rm_interfaces::msg::TrackedRobot::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  auto normalized = normalizeBuffRobot(*msg);
  std::scoped_lock lk(mutex_);
  latest_stamp_ = rclcpp::Time(normalized.header.stamp);
  latest_robot_ = std::move(normalized);
}

std::optional<rm_interfaces::msg::TrackedRobot> BuffTargetAdapter::latestValid(
  const rclcpp::Time & now) const
{
  std::scoped_lock lk(mutex_);
  if (!enabled_ || !latest_robot_.has_value()) {
    return std::nullopt;
  }

  const double age_s = std::max(0.0, (now - latest_stamp_).seconds());
  if (age_s > config_.timeout_s) {
    return std::nullopt;
  }

  return latest_robot_;
}

rm_interfaces::msg::TrackedRobot BuffTargetAdapter::normalizeBuffRobot(
  const rm_interfaces::msg::TrackedRobot & msg) const
{
  auto robot = robot_description::TrackedRobotUsage::normalizeState(msg);
  robot.header.frame_id = config_.target_frame;

  robot.track_state = rm_interfaces::msg::TrackedRobot::TRACKING;
  robot.is_visible = true;
  robot.visible_armor_count = std::max(robot.visible_armor_count, 1);

  if (robot.robot_id.empty()) {
    robot.robot_id = "big_buff";
  }

  if (robot.armors_offset.empty()) {
    geometry_msgs::msg::Pose zero_offset;
    zero_offset.position.x = 0.0;
    zero_offset.position.y = 0.0;
    zero_offset.position.z = 0.0;
    zero_offset.orientation.w = 1.0;
    zero_offset.orientation.x = 0.0;
    zero_offset.orientation.y = 0.0;
    zero_offset.orientation.z = 0.0;
    robot.armors_offset.push_back(zero_offset);
    robot.representation_mode = rm_interfaces::msg::TrackedRobot::REP_AMBIGUOUS_SINGLE_ARMOR;
    robot.num_armors = 1;
  } else {
    robot.num_armors = std::max(robot.num_armors, static_cast<int32_t>(robot.armors_offset.size()));
    if (robot.num_armors >= 3) {
      robot.representation_mode = rm_interfaces::msg::TrackedRobot::REP_STRUCTURED_ROBOT;
    }
  }

  return robot;
}

}  // namespace fyt::auto_aim::adapters
