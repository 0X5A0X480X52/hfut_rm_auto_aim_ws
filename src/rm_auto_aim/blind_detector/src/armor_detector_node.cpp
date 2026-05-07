// std
#include <algorithm>
#include <cmath>
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

  // 直接从 TF 查询 camera_optical_frame → odom 的变换
  odom_frame_ = this->declare_parameter("target_frame", "odom");
  camera_frame_id_ = this->declare_parameter("camera_frame_id", "blind_camera_1_optical_frame");

  // FOV parameters for angle estimation (still from config)
  h_fov_ = this->declare_parameter("h_fov", 60.0);
  v_fov_ = this->declare_parameter("v_fov", 45.0);

  // Image dimensions (从配置文件中读取)
  image_width_ = this->declare_parameter("blind_image_width", 640);
  image_height_ = this->declare_parameter("blind_image_height", 480);

  // 水平/垂直焦距（像素），用于距离估算
  // 默认值从 FOV 推算：fx = (image_width/2) / tan(h_fov/2), fy = (image_height/2) / tan(v_fov/2)
  // 若有标定值则优先从 launch.py 或配置文件传入
  {
    float fx_default = (image_width_ / 2.0f) /
        std::tan(h_fov_ * static_cast<float>(M_PI) / 180.0f / 2.0f);
    camera_fx_ = this->declare_parameter("camera_fx", fx_default);
    float fy_default = (image_height_ / 2.0f) /
        std::tan(v_fov_ * static_cast<float>(M_PI) / 180.0f / 2.0f);
    camera_fy_ = this->declare_parameter("camera_fy", fy_default);
  }

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

  // tf2 buffer + listener
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Image subscription via tf2_ros::MessageFilter, synchronized with TF
  img_mf_sub_.subscribe(this, img_topic, rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::Image>>(
      img_mf_sub_, *tf2_buffer_, odom_frame_,
      /*queue_size=*/10,
      get_node_logging_interface(), get_node_clock_interface(),
      std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&ArmorDetectorNode::imageCallback, this);

  // Set Mode 服务 (节点私有服务，在命名空间下解析)
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
      "~/set_mode",
      std::bind(&ArmorDetectorNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {

  geometry_msgs::msg::TransformStamped odom_to_camera;
  if (tf2_buffer_->canTransform(
          odom_frame_, camera_frame_id_, img_msg->header.stamp,
          tf2::durationFromSec(0.02))) {
    odom_to_camera = tf2_buffer_->lookupTransform(
        odom_frame_, camera_frame_id_, img_msg->header.stamp);
  } else {
    FYT_WARN("blind_detector", "TF at image timestamp {}.{}s not available, "
             "fallback to latest transform",
             img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);
    try {
      odom_to_camera = tf2_buffer_->lookupTransform(
          odom_frame_, camera_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      FYT_WARN("blind_detector", "TF lookup failed: {}", ex.what());
      return;
    }
  }
  // odom_to_camera 已经包含 URDF 中的安装偏移 + 光学帧变换，
  // 其表示的旋转就是 camera_optical_frame 在 odom 中的朝向。
  tf2::Quaternion q;
  tf2::fromMsg(odom_to_camera.transform.rotation, q);
  // 将光学帧光轴 (+Z) 旋转到 odom 坐标系，计算 yaw/pitch
  tf2::Vector3 optical_axis(0, 0, 1);
  tf2::Vector3 axis_odom = tf2::quatRotate(q, optical_axis);
  double camera_yaw = std::atan2(axis_odom.y(), axis_odom.x()) * 180.0 / M_PI;
  double camera_pitch = std::atan2(axis_odom.z(),
      std::sqrt(axis_odom.x() * axis_odom.x() + axis_odom.y() * axis_odom.y())) * 180.0 / M_PI;

  // Detect armors
  auto armors = detectArmors(img_msg);

  // 发布结果，由下游 gimbal_pipeline 选择目标
  blinds_msg_.header = img_msg->header;
  blinds_msg_.blinds.clear();
  blinds_msg_.blinds.reserve(armors.size());

  for (auto &armor : armors) {
    float normalized_x = static_cast<double>(armor.center.x) / static_cast<double>(image_width_);
    float yaw = camera_yaw - (normalized_x - 0.5) * h_fov_;

    float normalized_y = static_cast<double>(armor.center.y) / static_cast<double>(image_height_);
    float pitch = camera_pitch - (normalized_y - 0.5) * v_fov_;

    // 距离估算：分别用装甲板宽度和高度通过针孔模型估算，最后取平均
    float distance = -1.0f;
    if (armor.type != ArmorType::INVALID) {
      // 透视补偿：目标偏离光轴时像素宽度/高度被压缩
      float h_fov_rad = h_fov_ * static_cast<float>(M_PI) / 180.0f;
      float v_fov_rad = v_fov_ * static_cast<float>(M_PI) / 180.0f;
      float cos_angle_x = std::cos((normalized_x - 0.5f) * h_fov_rad);
      float cos_angle_y = std::cos((normalized_y - 0.5f) * v_fov_rad);

      // 宽度估算（需知 small/large 类型）
      float pixel_width = armor.right_light.center.x - armor.left_light.center.x;
      if (pixel_width >= 1.0f) {
        float real_width = (armor.type == ArmorType::SMALL) ? SMALL_ARMOR_WIDTH : LARGE_ARMOR_WIDTH;
        distance = camera_fx_ * real_width / pixel_width * cos_angle_x;
      }
      // 高度估算（左右灯条长度取平均，small/large 高度相同）+ 取平均
      float pixel_height = (armor.left_light.length + armor.right_light.length) / 2.0f;
      if (pixel_height >= 1.0f) {
        float dist_from_height = camera_fy_ * SMALL_ARMOR_HEIGHT / pixel_height * cos_angle_y;
        if (distance >= 0.0f) {
          distance = (distance + dist_from_height) / 2.0f;
        } else {
          distance = dist_from_height;
        }
      }
    }

    rm_interfaces::msg::Blind b;
    b.number = armor.classfication_result;
    b.yaw = yaw;
    b.pitch = pitch;
    b.confi = armor.confidence;
    b.distance = distance;
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
    img_mf_sub_.subscribe(this, "image_raw", rmw_qos_profile_sensor_data);
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
    img_mf_sub_.unsubscribe();
  }
  }

  FYT_WARN("blind_detector", "Set mode to {}", mode_name);
}

} // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNode)
