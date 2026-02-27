// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MAX_ENTROPY_TRACKER_NODE_HPP_
#define MAX_ENTROPY_TRACKER_MAX_ENTROPY_TRACKER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <rm_interfaces/msg/tracked_robots.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/tf_handler.hpp"
#include "max_entropy_tracker/tracker_manager.hpp"

namespace fyt::auto_aim {

class MaxEntropyTrackerNode : public rclcpp::Node {
 public:
  explicit MaxEntropyTrackerNode(const rclcpp::NodeOptions &options);

 private:
  void declare_parameters();
  void apply_parameters_to_config();
  void armors_callback(const rm_interfaces::msg::Armors::SharedPtr msg);
  void publish_results(const std_msgs::msg::Header &header);

  rm_interfaces::msg::Target build_target_message(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      AdaptiveArmorTracker &tracker);

  rm_interfaces::msg::TrackedRobot build_tracked_robot_message(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      AdaptiveArmorTracker &tracker);

  uint8_t infer_robot_type(const std::string &robot_id) const;
  int infer_num_armors(const std::string &robot_id, int robot_type) const;

  // Config
  UnifiedConfig config_;
  std::string target_frame_;
  std::string source_frame_;
  double predict_rate_;
  bool debug_mode_;
  std::string visualization_frame_;

  // Core modules
  std::unique_ptr<TFHandler> tf_handler_;
  std::unique_ptr<TrackerManager> tracker_manager_;

  // ROS pub/sub
  rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackedRobots>::SharedPtr tracked_robots_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Per-robot last visible count
  std::unordered_map<std::string, int> last_obs_counts_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_MAX_ENTROPY_TRACKER_NODE_HPP_
