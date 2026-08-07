#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/rune_target_array.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "yolo.hpp"

namespace auto_buff {

class DetectorNode : public rclcpp::Node {
public:
  DetectorNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void onSetMode(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                 std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<rm_interfaces::msg::RuneTarget>::SharedPtr rune_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RuneTargetArray>::SharedPtr runes_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr result_img_compressed_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  std::string image_topic_;
  std::string rune_topic_;
  std::string runes_topic_;
  std::string result_img_topic_;
  std::string result_img_compressed_topic_;
  bool is_big_rune_{true};
  bool mode_managed_{true};
  bool inference_enabled_{false};
  bool debug_view_{false};
  float min_confidence_{0.35F};
  int debug_jpeg_quality_{70};

  std::unique_ptr<YOLO> yolo_;
};

}  // namespace auto_buff
