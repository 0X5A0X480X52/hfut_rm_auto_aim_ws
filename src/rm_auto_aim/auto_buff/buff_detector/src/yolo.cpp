#include "yolo.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace auto_buff
{

YOLO::YOLO(const YoloParams & params)
: config_(params)
{
  generateGridsAndStride();

  auto model = core_.read_model(config_.model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();
  input.tensor()
    .set_element_type(ov::element::f32)
    .set_shape({1, yolo_input_size, yolo_input_size, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);
  input.model().set_layout("NCHW");
  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB);
  model = ppp.build();

  compiled_model_ = core_.compile_model(
    model, config_.device,
    ov::hint::performance_mode(
      config_.use_latency_performance_mode ?
      ov::hint::PerformanceMode::LATENCY : ov::hint::PerformanceMode::THROUGHPUT));
}

YOLO::~YOLO() = default;

ov::Tensor YOLO::preProcess(const cv::Mat & img)
{
  const int img_h = img.rows;
  const int img_w = img.cols;
  const float scale = std::min(yolo_input_size * 1.0F / img_h, yolo_input_size * 1.0F / img_w);
  const int resize_h = static_cast<int>(std::round(img_h * scale));
  const int resize_w = static_cast<int>(std::round(img_w * scale));

  const int pad_h = yolo_input_size - resize_h;
  const int pad_w = yolo_input_size - resize_w;
  const float half_h = pad_h * 0.5F;
  const float half_w = pad_w * 0.5F;

  const int top = static_cast<int>(std::round(half_h - 0.1F));
  const int bottom = static_cast<int>(std::round(half_h + 0.1F));
  const int left = static_cast<int>(std::round(half_w - 0.1F));
  const int right = static_cast<int>(std::round(half_w + 0.1F));

  ov::Tensor input_tensor{ov::element::f32, {1, yolo_input_size, yolo_input_size, 3}};
  cv::Mat input_mat(yolo_input_size, yolo_input_size, CV_32FC3, input_tensor.data<float>());

  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));
  cv::Mat float_img;
  resized_img.convertTo(float_img, CV_32FC3, 1.0);
  if (!is_recoded_image_parameters_) {
    is_recoded_image_parameters_ = true;
    getTransformMatrix(half_h, half_w, scale);
    input_image_size_ = img.size();
  }
  cv::copyMakeBorder(
    float_img, input_mat, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return input_tensor;
}

void YOLO::getTransformMatrix(float half_h, float half_w, float scale)
{
  transform_matrix_ = cv::Matx33f(
    1.0F / scale, 0.0F, -half_w / scale,
    0.0F, 1.0F / scale, -half_h / scale,
    0.0F, 0.0F, 1.0F);
}

void YOLO::generateGridsAndStride()
{
  std::vector<int> strides = {8, 16, 32};
  for (const auto stride : strides) {
    const int num_grid_w = yolo_input_size / stride;
    const int num_grid_h = yolo_input_size / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides_.push_back({g0, g1, stride});
      }
    }
  }
}

ov::InferRequest YOLO::requestInfer(const ov::Tensor & input_tensor)
{
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  return infer_request;
}

float YOLO::intersectionArea(const RuneObject & a, const RuneObject & b) const
{
  const cv::Rect_<float> inter = a.box & b.box;
  return inter.area();
}

std::vector<RuneObject> YOLO::postProcess(const ov::Tensor & output_tensor)
{
  const auto output_shape = output_tensor.get_shape();
  cv::Mat output_buffer(
    output_shape[1], output_shape[2], CV_32F, const_cast<float *>(output_tensor.data<const float>()));

  std::vector<RuneObject> objs_tmp;
  std::vector<RuneObject> objs_result;
  std::vector<int> indices;
  generateProposals(objs_tmp, output_buffer);
  std::sort(
    objs_tmp.begin(), objs_tmp.end(),
    [](const RuneObject & a, const RuneObject & b) {return a.prob > b.prob;});
  if (objs_tmp.size() > static_cast<size_t>(config_.top_k)) {
    objs_tmp.resize(config_.top_k);
  }
  nmsMergeSortedBboxes(objs_tmp, indices);
  for (size_t i = 0; i < indices.size(); i++) {
    objs_result.push_back(std::move(objs_tmp[indices[i]]));
    float weights = objs_result[i].prob;
    objs_result[i].points = objs_result[i].points * objs_result[i].prob;
    for (size_t j = 0; j < objs_result[i].points.children.size(); j++) {
      objs_result[i].points = objs_result[i].points + objs_result[i].points.children[j];
      weights += objs_result[i].points.probs[j];
    }
    objs_result[i].points = objs_result[i].points / weights;
    objs_result[i].prob = weights / static_cast<float>(objs_result[i].points.children.size() + 1);
  }
  return objs_result;
}

void YOLO::generateProposals(std::vector<RuneObject> & output_objs, const cv::Mat & output_buffer) const
{
  for (int anchor_idx = 0; anchor_idx < static_cast<int>(grid_strides_.size()); anchor_idx++) {
    const float confidence = output_buffer.at<float>(anchor_idx, yolo_point_number * 2);
    if (confidence < config_.threshold) {
      continue;
    }
    const int grid0 = grid_strides_[anchor_idx].grid0;
    const int grid1 = grid_strides_[anchor_idx].grid1;
    const int stride = grid_strides_[anchor_idx].stride;

    double color_score = 0.0;
    double class_score = 0.0;
    cv::Point color_id, class_id;
    cv::Mat color_scores = output_buffer.row(anchor_idx).colRange(
      yolo_point_number * 2 + 1, yolo_point_number * 2 + 1 + yolo_color_number);
    cv::Mat class_scores = output_buffer.row(anchor_idx).colRange(
      yolo_point_number * 2 + 1 + yolo_color_number,
      yolo_point_number * 2 + 1 + yolo_color_number + yolo_class_number);
    cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_id);
    cv::minMaxLoc(class_scores, nullptr, &class_score, nullptr, &class_id);

    const float x_1 = (output_buffer.at<float>(anchor_idx, 0) + grid0) * stride;
    const float y_1 = (output_buffer.at<float>(anchor_idx, 1) + grid1) * stride;
    const float x_2 = (output_buffer.at<float>(anchor_idx, 2) + grid0) * stride;
    const float y_2 = (output_buffer.at<float>(anchor_idx, 3) + grid1) * stride;
    const float x_3 = (output_buffer.at<float>(anchor_idx, 4) + grid0) * stride;
    const float y_3 = (output_buffer.at<float>(anchor_idx, 5) + grid1) * stride;
    const float x_4 = (output_buffer.at<float>(anchor_idx, 6) + grid0) * stride;
    const float y_4 = (output_buffer.at<float>(anchor_idx, 7) + grid1) * stride;
    const float x_5 = (output_buffer.at<float>(anchor_idx, 8) + grid0) * stride;
    const float y_5 = (output_buffer.at<float>(anchor_idx, 9) + grid1) * stride;

    const cv::Vec3f p1 = transform_matrix_ * cv::Vec3f(x_1, y_1, 1.0F);
    const cv::Vec3f p2 = transform_matrix_ * cv::Vec3f(x_2, y_2, 1.0F);
    const cv::Vec3f p3 = transform_matrix_ * cv::Vec3f(x_3, y_3, 1.0F);
    const cv::Vec3f p4 = transform_matrix_ * cv::Vec3f(x_4, y_4, 1.0F);
    const cv::Vec3f p5 = transform_matrix_ * cv::Vec3f(x_5, y_5, 1.0F);

    RuneObject obj;
    obj.points.center = cv::Point2f(p1[0], p1[1]);
    obj.points.bottom_left = cv::Point2f(p2[0], p2[1]);
    obj.points.top_left = cv::Point2f(p3[0], p3[1]);
    obj.points.top_right = cv::Point2f(p4[0], p4[1]);
    obj.points.bottom_right = cv::Point2f(p5[0], p5[1]);
    obj.box = cv::boundingRect(obj.points.toVector2f());
    obj.color = color_id.x ? EnemyColor::Blue : EnemyColor::Red;
    obj.type = class_id.x ? BuffBladeType::Activated : BuffBladeType::Inactivated;
    obj.prob = confidence;
    output_objs.push_back(std::move(obj));
  }
}

void YOLO::nmsMergeSortedBboxes(std::vector<RuneObject> & rune_objects, std::vector<int> & indices) const
{
  indices.clear();
  const int object_num = static_cast<int>(rune_objects.size());
  std::vector<float> areas(object_num);
  for (int i = 0; i < object_num; i++) {
    areas[i] = rune_objects[i].box.area();
  }
  for (int i = 0; i < object_num; i++) {
    RuneObject & waiting = rune_objects[i];
    if (areas[i] <= 0) {
      continue;
    }
    bool keep = true;
    for (const auto idx : indices) {
      RuneObject & merged = rune_objects[idx];
      const float inter_area = intersectionArea(waiting, merged);
      const float union_area = areas[i] + areas[idx] - inter_area;
      const float iou = inter_area / union_area;
      if (iou > config_.nms_threshold || std::isnan(iou)) {
        keep = false;
        if (
          waiting.type == merged.type &&
          waiting.color == merged.color &&
          iou > config_.merge_min_iou &&
          std::abs(waiting.prob - merged.prob) < config_.merge_conf_error)
        {
          merged.points.children.push_back(waiting.points);
          merged.points.probs.push_back(waiting.prob);
        }
      }
    }
    if (keep) {
      indices.push_back(i);
    }
  }
}

}  // namespace auto_buff
