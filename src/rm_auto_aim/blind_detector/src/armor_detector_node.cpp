// std
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>
// ros2
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// third party
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "armor_detector/armor_detector_node.hpp"
#include "armor_detector/types.hpp"
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/url_resolver.hpp"

namespace fyt::auto_aim {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
    : Node("blind_detector", options)
{
  FYT_REGISTER_LOGGER("blind_detector", "~/fyt2024-log", INFO);
  FYT_INFO("blind_detector", "Starting BlindArmorDetectorNode!");

  // 解析 odom→gimbal_link 变换，组合安装偏移得到相机朝向
  odom_frame_ = this->declare_parameter("target_frame", "odom");
  gimbal_frame_ = this->declare_parameter("gimbal_frame", "gimbal_link");

  // 预计算安装偏移四元数: optical_frame → gimbal_link
  // URDF: gimbal→link rpy=(0, 0.14, π) + link→optical rpy=(-π/2, 0, -π/2)
  double mount_yaw = this->declare_parameter("blind_mounting_yaw", 3.14159);
  double mount_pitch = this->declare_parameter("blind_mounting_pitch", 0.14);
  tf2::Quaternion q_link_in_gimbal, q_optical_in_link;
  q_link_in_gimbal.setRPY(0.0, mount_pitch, mount_yaw);
  q_optical_in_link.setRPY(-M_PI_2, 0.0, -M_PI_2);
  q_mounting_ = q_link_in_gimbal * q_optical_in_link;

  // FOV parameters for angle estimation (still from config)
  h_fov_ = this->declare_parameter("h_fov", 60.0);
  v_fov_ = this->declare_parameter("v_fov", 45.0);

  // Image dimensions (从配置文件中读取)
  image_width_ = this->declare_parameter("blind_image_width", 640);
  image_height_ = this->declare_parameter("blind_image_height", 480);

  // 初始化 Detector
  detector_ = initDetector();

  // 构建话题名称 (相对话题名，在命名空间下解析)
  const std::string img_topic = "image_raw";
  const std::string blinds_topic = "blinds";

  //Targets Publisher
  blinds_pub_ = this->create_publisher<rm_interfaces::msg::Blinds>(
    blinds_topic, rclcpp::SensorDataQoS());

  // Debug 参数
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    createDebugPublishers();
  }
  // 动态监控 debug 参数变化
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
      "debug", [this](const rclcpp::Parameter &p) {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
      });

  // Image subscription (direct, no MessageFilter)
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      img_topic, rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  // /tf subscription — 解析 odom→gimbal_link 变换，无需 lookupTransform
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::tfCallback, this, std::placeholders::_1));

  // Set Mode 服务 (节点私有服务，在命名空间下解析)
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
      "~/set_mode",
      std::bind(&ArmorDetectorNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::tfCallback(
    const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto &transform : msg->transforms) {
    if (transform.header.frame_id == odom_frame_ &&
        transform.child_frame_id == gimbal_frame_) {
      // 获取 gimbal_link 在 odom 坐标系中的朝向
      tf2::Quaternion q_gimbal;
      tf2::fromMsg(transform.transform.rotation, q_gimbal);
      // 组合: optical →(q_mounting_)→ gimbal →(q_gimbal)→ odom
      tf2::Quaternion q_camera = q_gimbal * q_mounting_;
      // 直接计算光轴 (+Z) 在 odom 中的方向，避免 getRPY 在大角度下的分解问题
      tf2::Vector3 optical_axis(0, 0, 1);
      tf2::Vector3 axis_odom = tf2::quatRotate(q_camera, optical_axis);
      double yaw = std::atan2(axis_odom.y(), axis_odom.x());
      double pitch = std::atan2(axis_odom.z(),
          std::sqrt(axis_odom.x() * axis_odom.x() + axis_odom.y() * axis_odom.y()));
      // 存储带时间戳的位姿样本
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_history_.push_back({transform.header.stamp,
                                 yaw * 180.0 / M_PI,
                                 pitch * 180.0 / M_PI});
        while (pose_history_.size() > kMaxPoseHistory) {
          pose_history_.pop_front();
        }
      }
      return;
    }
  }
}

void ArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {

  // 从缓存查找与图像时间戳最近似的相机位姿
  double camera_yaw_current = 0.0;
  double camera_pitch_current = 0.0;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (pose_history_.empty()) {
      FYT_WARN("blind_detector", "No pose data available yet, skipping frame");
      return;
    }

    rclcpp::Time img_stamp = img_msg->header.stamp;
    if (img_stamp.nanoseconds() == 0) {
      // 图像无时间戳，使用最新位姿
      camera_yaw_current = pose_history_.back().yaw;
      camera_pitch_current = pose_history_.back().pitch;
    } else {
      // 寻找时间戳最近似的位姿样本
      auto closest = pose_history_.begin();
      double min_diff = std::abs((closest->stamp - img_stamp).seconds());
      for (auto it = pose_history_.begin(); it != pose_history_.end(); ++it) {
        double diff = std::abs((it->stamp - img_stamp).seconds());
        if (diff < min_diff) {
          min_diff = diff;
          closest = it;
        }
      }
      if (min_diff > 0.5) {
        FYT_WARN("blind_detector",
                 "Closest pose is {:.1f}s away from image stamp", min_diff);
      }
      camera_yaw_current = closest->yaw;
      camera_pitch_current = closest->pitch;
    }
  }

  // Detect armors
  auto armors = detectArmors(img_msg);

  // 发布所有检测到的装甲板（无优先级比较，由下游 gimbal_pipeline 选目标）
  blinds_msg_.header = img_msg->header;
  blinds_msg_.blinds.clear();
  blinds_msg_.blinds.reserve(armors.size());

  for (auto &armor : armors) {
    // 1. 计算目标在机器人坐标系中的yaw角
    // 公式：yaw = 相机中心yaw + (归一化位置 - 0.5) * 水平视场角
    float normalized_x = static_cast<double>(armor.center.x) / static_cast<double>(image_width_);
    float yaw = camera_yaw_current - (normalized_x - 0.5) * h_fov_;

    // 2. 计算目标在机器人坐标系中的pitch角
    // 公式：pitch = 相机中心pitch + (归一化位置 - 0.5) * 垂直视场角
    // 注意：图像y轴向下，所以pitch向上为正
    float normalized_y = static_cast<double>(armor.center.y) / static_cast<double>(image_height_);
    float pitch = camera_pitch_current - (normalized_y - 0.5) * v_fov_;

    rm_interfaces::msg::Blind b;
    b.number = armor.classfication_result;
    b.yaw = yaw;
    b.pitch = pitch;
    b.confi = armor.confidence;
    blinds_msg_.blinds.push_back(b);
  }

  blinds_pub_->publish(blinds_msg_);
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  Detector::LightParams l_params = {
      .min_ratio = declare_parameter("light.min_ratio", 0.08),
      .max_ratio = declare_parameter("light.max_ratio", 0.4),
      .max_angle = declare_parameter("light.max_angle", 40.0),
      .color_diff_thresh =
          static_cast<int>(declare_parameter("light.color_diff_thresh", 25))};

  Detector::ArmorParams a_params = {
      .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
      .min_small_center_distance =
          declare_parameter("armor.min_small_center_distance", 0.8),
      .max_small_center_distance =
          declare_parameter("armor.max_small_center_distance", 3.2),
      .min_large_center_distance =
          declare_parameter("armor.min_large_center_distance", 3.2),
      .max_large_center_distance =
          declare_parameter("armor.max_large_center_distance", 5.0),
      .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, EnemyColor::RED,
                                             l_params, a_params);

  // Init classifier
  namespace fs = std::filesystem;
  fs::path model_path = utils::URLResolver::getResolvedPath(
      "package://blind_detector/model/lenet.onnx");
  fs::path label_path = utils::URLResolver::getResolvedPath(
      "package://blind_detector/model/label.txt");
  FYT_ASSERT_MSG(fs::exists(model_path) && fs::exists(label_path),
                 model_path.string() + " Not Found!");

  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes = this->declare_parameter(
      "ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);

  // Set dynamic parameter callback
  on_set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  auto armors = detector_->detect(img);

  // Publish debug info
  if (debug_) {
    // Sort lights and armors data by x coordinate
    std::sort(detector_->debug_lights.data.begin(),
              detector_->debug_lights.data.end(),
              [](const auto &l1, const auto &l2) {
                return l1.center_x < l2.center_x;
              });
    std::sort(detector_->debug_armors.data.begin(),
              detector_->debug_armors.data.end(),
              [](const auto &a1, const auto &a2) {
                return a1.center_x < a2.center_x;
              });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    detector_->drawResults(img);

    // Draw FPS (基于帧间隔而非处理延迟)
    static rclcpp::Time last_frame_time = this->now();
    auto now = this->now();
    double frame_interval = (now - last_frame_time).seconds();
    last_frame_time = now;
    double fps = frame_interval > 0.0 ? 1.0 / frame_interval : 0.0;
    std::stringstream fps_ss;
    fps_ss << "Frame rate: " << std::fixed << std::setprecision(1) << fps << " fps";
    auto fps_s = fps_ss.str();
    cv::putText(img, fps_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

rcl_interfaces::msg::SetParametersResult
ArmorDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "binary_thres") {
      detector_->binary_thres = param.as_int();
    } else if (param.get_name() == "classifier_threshold") {
      detector_->classifier->threshold = param.as_double();
    } else if (param.get_name() == "light.min_ratio") {
      detector_->light_params.min_ratio = param.as_double();
    } else if (param.get_name() == "light.max_ratio") {
      detector_->light_params.max_ratio = param.as_double();
    } else if (param.get_name() == "light.max_angle") {
      detector_->light_params.max_angle = param.as_double();
    } else if (param.get_name() == "light.color_diff_thresh") {
      detector_->light_params.color_diff_thresh = param.as_int();
    } else if (param.get_name() == "armor.min_light_ratio") {
      detector_->armor_params.min_light_ratio = param.as_double();
    } else if (param.get_name() == "armor.min_small_center_distance") {
      detector_->armor_params.min_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_small_center_distance") {
      detector_->armor_params.max_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.min_large_center_distance") {
      detector_->armor_params.min_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_large_center_distance") {
      detector_->armor_params.max_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_angle") {
      detector_->armor_params.max_angle = param.as_double();
    }
  }
  return result;
}

void ArmorDetectorNode::createDebugPublishers() noexcept {
  lights_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugLights>(
      "~/debug_lights", 10);
  armors_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugArmors>(
      "~/debug_armors", 10);

  this->declare_parameter("result_img.jpeg_quality", 50);

  result_img_pub_ =
      image_transport::create_publisher(this, "~/result_img");
}


void ArmorDetectorNode::destroyDebugPublishers() noexcept {
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  //binary_img_pub_.shutdown();
  //number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  response->message = "0";

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("blind_detector", "Invalid mode: {}", request->mode);
    return;
  }

  auto createImageSub = [this]() {
    if (img_sub_ == nullptr) {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image_raw", rclcpp::SensorDataQoS(),
          std::bind(&ArmorDetectorNode::imageCallback, this,
                    std::placeholders::_1));
    }
  };

  switch (mode) {
  case VisionMode::AUTO_AIM_RED: {
    detector_->detect_color = EnemyColor::RED;
    createImageSub();
    break;
  }
  case VisionMode::AUTO_AIM_BLUE: {
    detector_->detect_color = EnemyColor::BLUE;
    createImageSub();
    break;
  }
  default: {
    img_sub_.reset();
  }
  }

  FYT_WARN("blind_detector", "Set mode to {}", mode_name);
}

} // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNode)
