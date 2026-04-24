#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace blind_vision
{
class USBCameraNode : public rclcpp::Node
{
public:
  explicit USBCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("usb_camera_node", options)
  {
    // 声明并读取参数
    camera_name_ = this->declare_parameter<std::string>("camera_name", "blind_camera_1");
    width_       = this->declare_parameter<int>("width", 640);
    height_      = this->declare_parameter<int>("height", 480);
    fps_         = this->declare_parameter<double>("fps", 30.0);
    frame_id_    = this->declare_parameter<std::string>("frame_id", camera_name_ + "_optical_frame");

    // 初始化相机
    init_camera(cap_, camera_name_);

    // 创建独立的发布者
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    rclcpp::QoS image_qos = use_sensor_data_qos
      ? rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::BestEffort)
      : rclcpp::QoS(rclcpp::KeepLast(10));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", image_qos);

    // 加载相机内参
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter<std::string>(
      "camera_info_url", "package://usb_camera_driver/config/camera_info_1.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s, using default", camera_info_url.c_str());
      // 使用默认 camera_info
      camera_info_msg_.header.frame_id = frame_id_;
      camera_info_msg_.width = width_;
      camera_info_msg_.height = height_;
      // 设置默认内参矩阵 (需要根据实际相机标定)
      camera_info_msg_.k = {500.0, 0.0, width_ / 2.0, 0.0, 500.0, height_ / 2.0, 0.0, 0.0, 1.0};
      camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
      camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info_msg_.p = {500.0, 0.0, width_ / 2.0, 0.0, 0.0, 500.0, height_ / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    }
    camera_info_msg_.header.frame_id = frame_id_;

    // 定时器发布帧
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
      std::bind(&USBCameraNode::timer_callback, this));
  }

  ~USBCameraNode()
  {
    if (cap_.isOpened()) cap_.release();
  }

private:
  std::string camera_name_;
  int width_;
  int height_;
  double fps_;
  std::string frame_id_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  void init_camera(cv::VideoCapture& cap, const std::string& side)
  {
    std::string name = std::string("/dev/camera_") + side;
    cap.open(name.c_str(), cv::CAP_V4L2);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera device %s", side.c_str(), name.c_str());
      throw std::runtime_error(side + " camera open failed");
    }

    // 设置相机参数
    if (!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'))) {
      RCLCPP_WARN(get_logger(), "%s camera does not support MJPG, fallback to YUYV", side.c_str());
      cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
    }
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap.set(cv::CAP_PROP_FPS, fps_);

    RCLCPP_INFO(this->get_logger(),
      "%s camera initialized: %.0fx%.0f @ %.2ffps",
      side.c_str(),
      cap.get(cv::CAP_PROP_FRAME_WIDTH),
      cap.get(cv::CAP_PROP_FRAME_HEIGHT),
      cap.get(cv::CAP_PROP_FPS));
  }

  void process_and_publish()
  {
    cv::Mat bgr_frame, rgb_frame;
    if (!cap_.read(bgr_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read frame from %s camera", camera_name_.c_str());
      return;
    }

    try {
      cv::cvtColor(bgr_frame, rgb_frame, cv::COLOR_BGR2RGB);
      auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "rgb8",
        rgb_frame
      ).toImageMsg();

      auto stamp = this->now();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;

      camera_info_msg_.header.stamp = stamp;

      image_pub_->publish(*msg);
      camera_info_pub_->publish(camera_info_msg_);
    }
    catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "%s camera processing error: %s", camera_name_.c_str(), e.what());
    }
  }

  void timer_callback()
  {
    process_and_publish();
  }
};
}  // namespace blind_vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blind_vision::USBCameraNode)
