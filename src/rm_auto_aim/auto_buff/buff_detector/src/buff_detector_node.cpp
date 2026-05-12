#include "buff_detector_node.hpp"

#include <algorithm>
#include <vector>
#include <limits>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace auto_buff
{

namespace
{
int quantizeBladeSlot(const RunePoints & pts)
{
  const float cx = pts.center.x;
  const float cy = pts.center.y;
  const float bx = 0.25F * (pts.bottom_right.x + pts.top_right.x + pts.top_left.x + pts.bottom_left.x);
  const float by = 0.25F * (pts.bottom_right.y + pts.top_right.y + pts.top_left.y + pts.bottom_left.y);
  const double angle = std::atan2(static_cast<double>(by - cy), static_cast<double>(bx - cx));
  constexpr double two_pi = 6.28318530717958647692;
  double normalized = std::fmod(angle + two_pi, two_pi);
  if (normalized < 0.0) {
    normalized += two_pi;
  }
  const int slot = static_cast<int>(std::floor((normalized / two_pi) * 5.0 + 0.5)) % 5;
  return std::clamp(slot, 0, 4);
}
}  // namespace

DetectorNode::DetectorNode()
: Node("buff_detector_node")
{
  image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
  rune_topic_ = declare_parameter<std::string>("rune_topic", "/rune_target");
  runes_topic_ = declare_parameter<std::string>("runes_topic", "/rune_targets");
  result_img_topic_ = declare_parameter<std::string>("result_img_topic", "/auto_buff/debug/result_img");
  result_img_compressed_topic_ = declare_parameter<std::string>(
    "result_img_compressed_topic", "/auto_buff/debug/result_img/compressed");
  is_big_rune_ = declare_parameter<bool>("is_big_rune", true);
  mode_managed_ = declare_parameter<bool>("mode_managed", true);
  debug_view_ = declare_parameter<bool>("debug_view", false);
  min_confidence_ = declare_parameter<double>("min_confidence", 0.35);
  debug_jpeg_quality_ = declare_parameter<int>("debug_jpeg_quality", 70);
  std::string default_model_path;
  try {
    default_model_path =
      ament_index_cpp::get_package_share_directory("auto_buff") + "/model/yolox_rune_3.6m.onnx";
  } catch (const std::exception &) {
    default_model_path = "";
  }
  const auto model_path = declare_parameter<std::string>("model_path", default_model_path);
  const auto device = declare_parameter<std::string>("device", "CPU");
  const auto use_latency_mode = declare_parameter<bool>("use_latency_performance_mode", true);
  const auto top_k = declare_parameter<int>("top_k", 30);
  const auto nms_threshold = declare_parameter<double>("nms_threshold", 0.45);
  const auto merge_conf_error = declare_parameter<double>("merge_conf_error", 0.2);
  const auto merge_min_iou = declare_parameter<double>("merge_min_iou", 0.85);

  if (model_path.empty()) {
    throw std::runtime_error("Parameter 'model_path' is required for OpenVINO detector");
  }
  YoloParams params;
  params.model_path = model_path;
  params.device = device;
  params.use_latency_performance_mode = use_latency_mode;
  params.threshold = static_cast<float>(min_confidence_);
  params.top_k = top_k;
  params.nms_threshold = static_cast<float>(nms_threshold);
  params.merge_conf_error = static_cast<float>(merge_conf_error);
  params.merge_min_iou = static_cast<float>(merge_min_iou);
  yolo_ = std::make_unique<YOLO>(params);

  rune_pub_ = create_publisher<rm_interfaces::msg::RuneTarget>(rune_topic_, rclcpp::SensorDataQoS());
  runes_pub_ =
    create_publisher<rm_interfaces::msg::RuneTargetArray>(runes_topic_, rclcpp::SensorDataQoS());
  const auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  result_img_pub_ =
    create_publisher<sensor_msgs::msg::Image>(result_img_topic_, debug_qos);
  result_img_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    result_img_compressed_topic_, debug_qos);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));
  set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
    "~/set_mode",
    std::bind(&DetectorNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2));
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!inference_enabled_) {
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cv_bridge failed: %s", e.what());
    return;
  }
  const cv::Mat & bgr = cv_ptr->image;
  if (bgr.empty()) {
    return;
  }

  rm_interfaces::msg::RuneTarget out;
  out.header = msg->header;
  out.is_big_rune = is_big_rune_;
  out.is_lost = true;
  out.blade_type = rm_interfaces::msg::RuneTarget::BLADE_UNKNOWN;
  out.blade_slot_hint = -1;
  out.confidence = 0.0F;
  out.track_id = 0u;
  rm_interfaces::msg::RuneTargetArray out_array;
  out_array.header = msg->header;

  auto input_tensor = yolo_->preProcess(bgr);
  auto request = yolo_->requestInfer(input_tensor);
  request.infer();
  auto runes = yolo_->postProcess(request.get_output_tensor());
  for (const auto & rune : runes) {
    if (rune.prob < min_confidence_) {
      continue;
    }
    rm_interfaces::msg::RuneTarget t;
    t.header = msg->header;
    t.is_big_rune = is_big_rune_;
    t.is_lost = false;
    t.blade_type =
      (rune.type == BuffBladeType::Inactivated)
      ? rm_interfaces::msg::RuneTarget::BLADE_INACTIVATED
      : rm_interfaces::msg::RuneTarget::BLADE_ACTIVATED;
    t.blade_slot_hint = quantizeBladeSlot(rune.points);
    t.confidence = rune.prob;
    t.track_id = static_cast<uint32_t>(out_array.targets.size() + 1u);
    t.pts[0].x = rune.points.center.x; t.pts[0].y = rune.points.center.y;
    t.pts[1].x = rune.points.bottom_right.x; t.pts[1].y = rune.points.bottom_right.y;
    t.pts[2].x = rune.points.top_right.x; t.pts[2].y = rune.points.top_right.y;
    t.pts[3].x = rune.points.top_left.x; t.pts[3].y = rune.points.top_left.y;
    t.pts[4].x = rune.points.bottom_left.x; t.pts[4].y = rune.points.bottom_left.y;
    out_array.targets.push_back(t);
  }
  runes_pub_->publish(out_array);

  if (!runes.empty()) {
    const auto best_it = std::max_element(
      runes.begin(), runes.end(),
      [](const RuneObject & a, const RuneObject & b) {return a.prob < b.prob;});
    if (best_it != runes.end() && best_it->prob >= min_confidence_) {
      const auto & pts = best_it->points;
      out.is_lost = false;
      out.blade_type =
        (best_it->type == BuffBladeType::Inactivated)
        ? rm_interfaces::msg::RuneTarget::BLADE_INACTIVATED
        : rm_interfaces::msg::RuneTarget::BLADE_ACTIVATED;
      out.blade_slot_hint = quantizeBladeSlot(best_it->points);
      out.confidence = best_it->prob;
      out.track_id = 1u;
      out.pts[0].x = pts.center.x; out.pts[0].y = pts.center.y;
      out.pts[1].x = pts.bottom_right.x; out.pts[1].y = pts.bottom_right.y;
      out.pts[2].x = pts.top_right.x; out.pts[2].y = pts.top_right.y;
      out.pts[3].x = pts.top_left.x; out.pts[3].y = pts.top_left.y;
      out.pts[4].x = pts.bottom_left.x; out.pts[4].y = pts.bottom_left.y;
    }
  }

  cv::Mat dbg;
  if (debug_view_) {
    dbg = bgr.clone();
  }
  if (debug_view_) {
    for (size_t i = 0; i < out_array.targets.size(); ++i) {
      const auto & t = out_array.targets[i];
      std::vector<cv::Point> poly = {
        cv::Point(static_cast<int>(t.pts[1].x), static_cast<int>(t.pts[1].y)),
        cv::Point(static_cast<int>(t.pts[2].x), static_cast<int>(t.pts[2].y)),
        cv::Point(static_cast<int>(t.pts[3].x), static_cast<int>(t.pts[3].y)),
        cv::Point(static_cast<int>(t.pts[4].x), static_cast<int>(t.pts[4].y))};
      const cv::Scalar c = (i == 0) ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 0);
      cv::polylines(dbg, poly, true, c, 2);
      cv::circle(
        dbg, cv::Point(static_cast<int>(t.pts[0].x), static_cast<int>(t.pts[0].y)), 4,
        c, -1);
    }
    if (!out.is_lost) {
      cv::circle(
        dbg, cv::Point(static_cast<int>(out.pts[0].x), static_cast<int>(out.pts[0].y)), 6,
        cv::Scalar(0, 0, 255), 2);
    }
  }

  if (debug_view_ && !dbg.empty()) {
    result_img_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg());
    std::vector<uchar> jpg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(debug_jpeg_quality_, 20, 100)};
    if (cv::imencode(".jpg", dbg, jpg_buf, params)) {
      sensor_msgs::msg::CompressedImage cmsg;
      cmsg.header = msg->header;
      cmsg.format = "jpeg";
      cmsg.data.assign(jpg_buf.begin(), jpg_buf.end());
      result_img_compressed_pub_->publish(std::move(cmsg));
    }
  }

  rune_pub_->publish(out);
}

void DetectorNode::onSetMode(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  response->success = true;
  if (!request) {
    response->success = false;
    response->message = "null request";
    return;
  }
  const int mode = static_cast<int>(request->mode);
  const bool rune_mode = (mode == 2 || mode == 3 || mode == 4 || mode == 5);
  inference_enabled_ = rune_mode;
  if (!mode_managed_) {
    response->message = "mode_managed=false, keep current is_big_rune";
    return;
  }

  if (mode == 2 || mode == 3)
  {
    is_big_rune_ = false;
    response->message = "switched to small rune";
    return;
  }
  if (mode == 4 || mode == 5)
  {
    is_big_rune_ = true;
    response->message = "switched to big rune";
    return;
  }
  response->message = "mode is not rune mode, keep current rune size";
}

}  // namespace auto_buff

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_buff::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
