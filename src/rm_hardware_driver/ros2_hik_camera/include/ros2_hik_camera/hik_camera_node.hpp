// Copyright (c) 2025
// Licensed under the MIT License.

#ifndef ROS2_HIK_CAMERA__HIK_CAMERA_NODE_HPP_
#define ROS2_HIK_CAMERA__HIK_CAMERA_NODE_HPP_

// HIKVision Camera SDK
#include <MvCameraControl.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace ros2_hik_camera
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options);
  ~HikCameraNode() override;

private:
  // Initialization
  void declareParameters();
  bool initCamera();
  
  // Camera control
  void startGrabbing();
  void stopGrabbing();
  
  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // HIKVision SDK handles and structures
  void * camera_handle_ = nullptr;
  MV_CC_DEVICE_INFO_LIST device_list_;
  MV_FRAME_OUT frame_out_;
  
  // Image buffers
  unsigned char * bgr_buffer_ = nullptr;
  unsigned int bgr_buffer_size_ = 0;
  unsigned char * frame_buffer_ = nullptr;

  // ROS2 publishers
  image_transport::CameraPublisher camera_pub_;

  // Camera info
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // Image message
  sensor_msgs::msg::Image image_msg_;

  // Capture thread
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  
  // Parameters
  bool flip_image_;
  int image_width_;
  int image_height_;
  int fail_count_ = 0;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace ros2_hik_camera

#endif  // ROS2_HIK_CAMERA__HIK_CAMERA_NODE_HPP_
