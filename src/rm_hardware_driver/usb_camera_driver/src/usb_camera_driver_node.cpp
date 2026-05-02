#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rm_utils/heartbeat.hpp>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

// V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE not defined in older kernel headers
#ifndef V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE
#define V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE 0x009a0903
#endif

#ifndef V4L2_EXPOSURE_MANUAL
#define V4L2_EXPOSURE_MANUAL 1
#endif
#ifndef V4L2_EXPOSURE_APERTURE_PRIORITY
#define V4L2_EXPOSURE_APERTURE_PRIORITY 3
#endif

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

    // 曝光参数
    auto_exposure_             = this->declare_parameter<bool>("auto_exposure", true);
    exposure_                  = this->declare_parameter<int>("exposure", 100);
    exposure_dynamic_framerate_ = this->declare_parameter<bool>("exposure_dynamic_framerate", false);
    white_balance_automatic_   = this->declare_parameter<bool>("white_balance_automatic", true);

    // 初始化相机
    init_camera(cap_, camera_name_);

    // 创建图像发布者
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    rclcpp::QoS image_qos = use_sensor_data_qos
      ? rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::BestEffort)
      : rclcpp::QoS(rclcpp::KeepLast(10));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos);

    // 定时器发布帧
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
      std::bind(&USBCameraNode::timer_callback, this));

    // Heartbeat
    heartbeat_ = fyt::HeartBeatPublisher::create(this);
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
  bool auto_exposure_;
  int exposure_;
  bool exposure_dynamic_framerate_;
  bool white_balance_automatic_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  fyt::HeartBeatPublisher::SharedPtr heartbeat_;

  // 在 OpenCV 打开设备之前通过 V4L2 ioctl 预设置 controls
  // UVC 驱动通常会在设备关闭后保留 control 值
  void preconfig_v4l2(const std::string& device)
  {
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "V4L2 preconfig: cannot open %s", device.c_str());
      return;
    }

    auto set_ctrl = [fd, this](uint32_t id, int32_t value, const char* name) {
      struct v4l2_control ctrl = {id, value};
      if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
        RCLCPP_INFO(get_logger(), "V4L2 preconfig: %s = %d", name, value);
      } else {
        RCLCPP_WARN(get_logger(), "V4L2 preconfig: %s set to %d failed", name, value);
      }
    };

    // 白平衡
    set_ctrl(V4L2_CID_AUTO_WHITE_BALANCE, white_balance_automatic_ ? 1 : 0,
             "auto_white_balance");

    // 曝光模式
    if (auto_exposure_) {
      set_ctrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY,
               "auto_exposure");
    } else {
      set_ctrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
               "auto_exposure");
      set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_,
               "exposure_absolute");
    }

    // 动态帧率（防止因长曝光而降帧）
    set_ctrl(V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE,
             exposure_dynamic_framerate_ ? 1 : 0,
             "exposure_dynamic_framerate");

    close(fd);
  }

  void init_camera(cv::VideoCapture& cap, const std::string& side)
  {
    std::string name = std::string("/dev/camera_") + side;

    // 先通过 V4L2 ioctl 预设置 OpenCV 不支持的控制
    preconfig_v4l2(name);

    // 再通过 OpenCV 打开设备
    cap.open(name.c_str(), cv::CAP_V4L2);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera device %s", side.c_str(), name.c_str());
      throw std::runtime_error(side + " camera open failed");
    }

    // 通过 OpenCV 设置相机参数（V4L2 preconfig 已保留，这里只设 OpenCV 管理的参数）
    if (!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'))) {
      RCLCPP_WARN(get_logger(), "%s camera does not support MJPG, fallback to YUYV", side.c_str());
      cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
    }
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

      msg->header.stamp = this->now();
      msg->header.frame_id = frame_id_;

      image_pub_->publish(*msg);
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
