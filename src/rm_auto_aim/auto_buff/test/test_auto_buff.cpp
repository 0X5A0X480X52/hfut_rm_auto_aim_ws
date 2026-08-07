#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "auto_buff/transform_utils.hpp"
#include "yolo.hpp"

namespace auto_buff {

TEST(Yolo, MissingModelFailsFast) {
  YoloParams params;
  params.model_path = "/tmp/auto_buff_model_does_not_exist.onnx";
  EXPECT_ANY_THROW({ YOLO detector(params); });
}

TEST(Yolo, EmptyOutputProducesNoRunes) {
  YoloParams params;
  params.model_path = AUTO_BUFF_TEST_MODEL_PATH;
  YOLO detector(params);

  ov::Tensor output(ov::element::f32, {1, 4725, 15});
  std::fill(output.data<float>(), output.data<float>() + output.get_size(), 0.0F);
  EXPECT_TRUE(detector.postProcess(output).empty());
}

TEST(Transform, MissingFrameHonorsTimeout) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::PointStamped input;
  input.header.frame_id = "missing_camera_frame";
  input.header.stamp = clock->now();
  geometry_msgs::msg::PointStamped first_out;
  geometry_msgs::msg::PointStamped second_out;
  std::string error;

  EXPECT_FALSE(
    transformPointPair(buffer, input, input, "odom", 0.001, first_out, second_out, error));
  EXPECT_FALSE(error.empty());
}

}  // namespace auto_buff
