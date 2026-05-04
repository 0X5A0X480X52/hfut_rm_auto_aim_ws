// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under
// Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

// std
#include <memory>
#include <string>
#include <vector>
// ros2
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <deque>
#include <mutex>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
// project
#include "armor_detector/armor_detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/blind.hpp"
#include "rm_interfaces/msg/blinds.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

// Armor Detector Node
// Subscribe to the image topic, run the armor detection alogorithm and publish
// the detected armors
class ArmorDetectorNode : public rclcpp::Node {
public:
  ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  // void targetCallback(const rm_interfaces::msg::Target::SharedPtr
  // target_msg);

  std::unique_ptr<Detector> initDetector();

  std::vector<Armor>
  detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  void createDebugPublishers() noexcept;
  void destroyDebugPublishers() noexcept;


  void setModeCallback(
      const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
      std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // Dynamic Parameter
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(std::vector<rclcpp::Parameter> parameters);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      on_set_parameters_callback_handle_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  // Detected armors publisher
  rm_interfaces::msg::Armor armor_msg_;
  rm_interfaces::msg::Armors armors_msg_;
  rm_interfaces::msg::Blinds blinds_msg_;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Blinds>::SharedPtr blinds_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  tf2::Quaternion q_mounting_;
  std::string gimbal_frame_;
  std::string odom_frame_;

  // Timestamped camera orientation history
  struct CameraPoseSample {
    rclcpp::Time stamp;
    double yaw;
    double pitch;
  };
  std::deque<CameraPoseSample> pose_history_;
  std::mutex pose_mutex_;
  static constexpr size_t kMaxPoseHistory = 200;

  // Image and FOV parameters for angle estimation
  int image_width_;
  int image_height_;
  float h_fov_;  // Horizontal field of view in degrees
  float v_fov_;  // Vertical field of view in degrees

  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Target subscription
  // rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
  // rm_interfaces::msg::Target::SharedPtr tracked_target_;
  std::deque<Armor> tracked_armors_;

  // Enable/Disable Armor Detector
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr
      lights_data_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr
      armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;
};

} // namespace fyt::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_NODE_HPP_
