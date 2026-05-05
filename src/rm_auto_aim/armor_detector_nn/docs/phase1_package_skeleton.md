# Phase 1: Package Skeleton & Interface Compatibility

## 1. Overview

**Goal:** Establish the `armor_detector_nn` package with ROS2 component registration, full external-interface compatibility with the legacy `armor_detector`, and structural scaffolding for all subsequent phases. The node must be a drop-in replacement — downstream tracking, fusion, and gimbal nodes must not require any changes to subscribe to `/armor_detector/armors` or call `/armor_detector/set_mode`.

**Scope:** package boilerplate, ROS2 lifecycle, parameter system, topic/service stubs, data-type headers, CMake/ament build integration. No inference, no image processing.

---

## 2. Package Scaffolding

### 2.1 Final Directory Layout (Phase 1 deliverables in **bold**)

```
armor_detector_nn/
  **CMakeLists.txt**
  **package.xml**

  **launch/**
    **armor_detector_nn.launch.py**

  **config/**
    **armor_detector_nn.yaml**
    **label_map.yaml**

  model/
    demo01/
      best.onnx
      best_int8_openvino_model/
        best.xml
        best.bin
        metadata.yaml

  **include/armor_detector_nn/**
    **armor_detector_nn_node.hpp**

    core/
      armor_detector_nn.hpp          # (forward-declare only)
      detector_config.hpp
      detection_types.hpp
      label_map.hpp
      preprocessor.hpp               # (forward-declare only)
      postprocessor.hpp              # (forward-declare only)
      armor_pose_estimator_adapter.hpp # (forward-declare only)

    backend/
      inference_backend.hpp
      inference_backend_factory.hpp
      onnxruntime_backend.hpp        # (forward-declare only)
      openvino_backend.hpp           # (forward-declare only)
      tensorrt_backend.hpp           # (forward-declare only)

    postprocess/
      decode_strategy.hpp
      ultralytics_pose_decode_strategy.hpp  # (forward-declare only)
      decode_strategy_factory.hpp

    geometry/
      armor_geometry_builder.hpp     # (forward-declare only)
      armor_type_resolver.hpp        # (forward-declare only)
      nms.hpp

    debug/
      debug_drawer.hpp               # (forward-declare only)
      profiler.hpp

  **src/**
    **armor_detector_nn_node.cpp**
    core/
    backend/
    postprocess/
    geometry/
    debug/

  docs/
    armor_detector_nn_design.md
    phase1_package_skeleton.md       # this file

  test/
```

Phase 1 only creates files marked with `**`. Header files for future modules ship as minimally-valid stubs (virtual destructors, forward declarations) so that later phases can fill in implementations without breaking the compile graph.

### 2.2 `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>armor_detector_nn</name>
  <version>0.1.0</version>
  <description>Neural-network-based armor detector as drop-in replacement for armor_detector.</description>
  <maintainer email="fyt@example.com">FYT Vision Group</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_eigen</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>rm_interfaces</depend>
  <depend>rm_utils</depend>
  <!-- Third-party deps should use rosdep keys in package.xml.
       Keep CMake's find_package(OpenCV/Eigen3) in CMakeLists.txt. -->
  <depend>opencv4</depend>
  <depend>eigen</depend>
  <depend>yaml-cpp</depend>

  <!-- Conditional: only needed when ONNX Runtime backend is built -->
  <!-- <depend>libonnxruntime</depend> -->

  <!-- Conditional: only needed when OpenVINO backend is built -->
  <!-- <depend>openvino</depend> -->

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Key decisions:
- Dependencies that are conditional on backend selection (onnxruntime, openvino) are listed as comments for now. They will be guarded by CMake options in later phases.
- `rm_interfaces` and `rm_utils` are hard deps — the former supplies `Armors`, `SetMode`; the latter supplies `HeartBeatPublisher`, `assert`, `logger`.

### 2.3 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(armor_detector_nn)

# --- ament boilerplate ---
find_package(ament_cmake REQUIRED)

# --- ROS2 deps ---
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)

# --- third-party libs ---
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- backend toggles (off by default until implemented) ---
option(BUILD_ONNX_BACKEND   "Build ONNX Runtime backend"   OFF)
option(BUILD_OPENVINO_BACKEND "Build OpenVINO backend"     OFF)
option(BUILD_TENSORRT_BACKEND  "Build TensorRT backend"    OFF)

# --- sources (expand per phase) ---
set(CORE_SOURCES)
set(BACKEND_SOURCES)
set(POSTPROCESS_SOURCES)
set(GEOMETRY_SOURCES)
set(DEBUG_SOURCES)

# Phase 1: node only
set(NODE_SOURCES
  src/armor_detector_nn_node.cpp
)

# --- library target ---
add_library(${PROJECT_NAME} SHARED
  ${NODE_SOURCES}
  ${CORE_SOURCES}
  ${BACKEND_SOURCES}
  ${POSTPROCESS_SOURCES}
  ${GEOMETRY_SOURCES}
  ${DEBUG_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_eigen
  geometry_msgs
  visualization_msgs
  rm_interfaces
  rm_utils
  OpenCV
  Eigen3
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::ArmorDetectorNNNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# --- install ---
install(DIRECTORY
  launch/
  config/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# --- tests (stub) ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
endif()

ament_package()
```

Design notes:
- `BUILD_ONNX_BACKEND`, `BUILD_OPENVINO_BACKEND`, `BUILD_TENSORRT_BACKEND` options are declared but OFF by default. Each phase toggles the relevant option ON.
- The library target is built as a single shared lib to match the `rclcpp_components` model used by the legacy `armor_detector`.
- `rclcpp_components_register_node` registers the component so a launch file or composable-container can load it by class name.

### 2.4 Launch File

`launch/armor_detector_nn.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("armor_detector_nn")
    config_path = os.path.join(pkg_share, "config", "armor_detector_nn.yaml")

    container = ComposableNodeContainer(
        name="armor_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="armor_detector_nn",
                plugin="fyt::auto_aim::ArmorDetectorNNNode",
                name="armor_detector",   # compatible with legacy node name
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
```

Key decisions:
- Node name is `armor_detector` (not `armor_detector_nn`), matching the legacy node name so downstream remaps are unnecessary.
- `use_intra_process_comms` eliminates the serialization hop when other nodes in the same container subscribe to published topics.

---

## 3. Data Types (Phase 1 Headers)

All type headers in `include/armor_detector_nn/core/` are created now so that later phases can reference them without changing the include graph.

### 3.1 `detection_types.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_DETECTION_TYPES_HPP_
#define ARMOR_DETECTOR_NN_DETECTION_TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace fyt::auto_aim {

// Raw model output — before label-map and filtering.
struct RawDetection {
  int class_id{-1};
  float class_score{0.0F};
  float object_score{1.0F};   // fused-objectness when head provides it
  float confidence{0.0F};     // = max(class_score, object_score) or combined
  cv::Rect2f bbox;            // xywh in original image coordinates
  std::array<cv::Point2f, 4> keypoints;  // kpt0..kpt3 in image coords
};

// Label-mapped detection — ready for downstream filtering and PnP.
struct ArmorDetection {
  std::string model_label;     // "B1", "R2", "BO", ...
  std::string publish_number;  // "1", "2", "outpost", "sentry"
  std::string publish_type;    // "small", "large"
  EnemyColor color;            // RED / BLUE
  float confidence{0.0F};
  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;
  cv::Point2f center;          // bbox center in image coords
};

// Per-frame detection bundle returned by the core pipeline.
struct FrameDetections {
  std_msgs::msg::Header header;              // original Image header
  std::vector<ArmorDetection> detections;
  // Reserved for future per-detection metadata (mask, embedding, etc.)
  // std::vector<cv::Mat> feature_maps;  // phase 7+
};

// Platform-agnostic tensor description.
struct TensorInfo {
  std::string name;
  std::vector<int64_t> shape;
  enum class DType { FLOAT32, FLOAT16, INT8, INT32 } dtype{DType::FLOAT32};
};

struct TensorInput {
  TensorInfo info;
  std::vector<float> host_data;
};

struct TensorOutput {
  TensorInfo info;
  std::vector<float> host_data;  // backend converts to float before returning
};

} // namespace fyt::auto_aim

#endif
```

### 3.2 `detector_config.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_
#define ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fyt::auto_aim {

enum class BackendType { ONNX_RUNTIME, OPENVINO, TENSORRT };
enum class Precision { FP32, FP16, INT8 };
enum class SchedulingMode { SYNC, ASYNC_LATEST, ASYNC_BATCH };
enum class ColorFilterSource { MODEL, IMAGE, DISABLED };
enum class DetectMode { RED, BLUE, DISABLED };
enum class CopyPolicy { ALWAYS_COPY, COPY_ON_WRITE_DEBUG, ZERO_COPY_STRICT };
enum class PlatformProfile { JETSON, NUC_CPUONLY, NUC_WITH_GPU, CUSTOM };

struct BackendConfig {
  BackendType type{BackendType::ONNX_RUNTIME};
  std::string device{"cpu"};
  Precision precision{Precision::FP32};
  std::string model_path;        // .onnx
  std::string engine_path;       // .engine (TensorRT)
  std::string openvino_xml_path; // .xml (OpenVINO)
  std::string openvino_bin_path; // .bin (OpenVINO)
  std::string calibration_cache; // calibration.cache (int8)
  std::string input_name{"images"};
  std::vector<std::string> output_names{"output0"};
  int warmup_iterations{10};
  int num_threads{2};
  bool preallocate_buffers{true};
  bool use_pinned_memory{true};   // effective for CUDA/TensorRT path
  int cuda_stream_count{1};
  bool gpu_preprocess{false};     // enable in Jetson TensorRT phase
  bool gpu_decode{false};         // enable when GPU decode is implemented
  bool allow_fallback{false};
  BackendType fallback_type{BackendType::ONNX_RUNTIME};
};

struct PreprocessConfig {
  int input_width{640};
  int input_height{640};
  std::string input_layout{"nchw"};  // "nchw" or "nhwc"
  std::string input_color{"rgb"};    // "rgb" or "bgr"
  std::string resize_mode{"letterbox"};
  bool normalize{true};
  std::vector<float> mean{0.0F, 0.0F, 0.0F};
  std::vector<float> std{255.0F, 255.0F, 255.0F};
  float pad_value{114.0F};
};

struct PostprocessConfig {
  std::string strategy{"ultralytics_pose"};
  std::string output_layout{"channels_first"};
  int num_classes{14};
  int num_keypoints{4};
  int keypoint_dims{2};
  int bbox_offset{0};
  int class_offset{4};
  int keypoint_offset{18};
  std::string box_format{"cxcywh"};
  float conf_threshold{0.35F};
  float nms_threshold{0.45F};
  int max_detections{32};
  bool class_agnostic_nms{false};
  std::vector<int> keypoint_remap{0, 1, 2, 3};   // map model kpt idx → canonical idx
  // Canonical order: [left_bottom, left_top, right_top, right_bottom]
};

struct LabelMapConfig {
  std::string path;
};

struct PoseConfig {
  bool use_ba{true};
  std::string pnp_method{"ippe"};
  double small_armor_width{0.133};
  double small_armor_height{0.050};
  double large_armor_width{0.225};
  double large_armor_height{0.050};
};

struct RuntimeConfig {
  PlatformProfile platform_profile{PlatformProfile::CUSTOM};
  ColorFilterSource color_filter_source{ColorFilterSource::MODEL};
  bool publish_empty{true};
  bool drop_frame_when_busy{true};
  CopyPolicy copy_policy{CopyPolicy::COPY_ON_WRITE_DEBUG};
  SchedulingMode scheduling_mode{SchedulingMode::SYNC};
  int frame_queue_size{2};
  int batch_min_size{1};
  int batch_max_size{1};         // capped by backend capability
  double batch_timeout_ms{2.0};
  double max_observation_age_ms{50.0};
  bool publish_out_of_order{false};
  bool profile{true};
};

struct DetectorConfig {
  bool debug{false};
  std::string target_frame{"odom"};

  BackendConfig backend;
  PreprocessConfig preprocess;
  PostprocessConfig postprocess;
  LabelMapConfig label_map;
  PoseConfig pose;
  RuntimeConfig runtime;
};

// Backend capability exposed by each IInferenceBackend implementation.
struct BackendInfo {
  std::string backend_name;
  std::string precision;
  int min_batch_size{1};
  int max_batch_size{1};
  bool dynamic_batch{false};
};

} // namespace fyt::auto_aim

#endif
```

### 3.3 `inference_backend.hpp` (abstract interface)

```cpp
#ifndef ARMOR_DETECTOR_NN_INFERENCE_BACKEND_HPP_
#define ARMOR_DETECTOR_NN_INFERENCE_BACKEND_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include <vector>

namespace fyt::auto_aim {

class IInferenceBackend {
public:
  virtual ~IInferenceBackend() = default;

  virtual void load(const BackendConfig& config) = 0;
  virtual std::vector<TensorOutput> infer(const TensorInput& input) = 0;
  virtual void warmup(int iterations) = 0;
  virtual BackendInfo info() const = 0;

protected:
  IInferenceBackend() = default;
  IInferenceBackend(const IInferenceBackend&) = delete;
  IInferenceBackend& operator=(const IInferenceBackend&) = delete;
  IInferenceBackend(IInferenceBackend&&) = default;
  IInferenceBackend& operator=(IInferenceBackend&&) = default;
};

} // namespace fyt::auto_aim

#endif
```

### 3.4 `decode_strategy.hpp` (abstract interface)

```cpp
#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include <vector>

namespace fyt::auto_aim {

// Metadata about the original image needed for coordinate restoration.
struct ImageMeta {
  int original_width{0};
  int original_height{0};
  float scale_x{1.0F};   // letterbox scale factor
  float scale_y{1.0F};
  float pad_left{0.0F};
  float pad_top{0.0F};
};

class IDecodeStrategy {
public:
  virtual ~IDecodeStrategy() = default;

  virtual std::vector<RawDetection> decode(
    const std::vector<TensorOutput>& outputs,
    const ImageMeta& image_meta,
    const PostprocessConfig& config) = 0;

protected:
  IDecodeStrategy() = default;
  IDecodeStrategy(const IDecodeStrategy&) = delete;
  IDecodeStrategy& operator=(const IDecodeStrategy&) = delete;
};

} // namespace fyt::auto_aim

#endif
```

### 3.5 `label_map.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_LABEL_MAP_HPP_
#define ARMOR_DETECTOR_NN_LABEL_MAP_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

struct LabelEntry {
  int class_id;
  std::string model_label;
  EnemyColor color;
  std::string publish_number;
  std::string publish_type;
};

class LabelMap {
public:
  LabelMap() = default;

  // Load from yaml path. Throws on validation failure.
  void load(const std::string& yaml_path);

  // Map a model class_id to its label entry.
  // Returns nullptr if class_id is not found or in ignore set.
  const LabelEntry* lookup(int class_id) const;

  // Filter detections by enemy color.
  std::vector<ArmorDetection>
  filterByColor(const std::vector<ArmorDetection>& detections,
                EnemyColor target_color) const;

  // Validate that all model class_ids [0..max_id] are covered.
  // Returns false + error message on gap or inconsistency.
  bool validate(std::string& error) const;

  // Accessors for reserved future use (image color classification, etc.)
  const std::unordered_map<int, LabelEntry>& entries() const { return entries_; }
  const std::vector<int>& ignored() const { return ignored_ids_; }

private:
  std::unordered_map<int, LabelEntry> entries_;
  std::vector<int> ignored_ids_;
  int max_class_id_{0};
};

} // namespace fyt::auto_aim

#endif
```

---

## 4. Node Implementation

### 4.1 `armor_detector_nn_node.hpp`

```cpp
#ifndef ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_NODE_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <rm_utils/heartbeat.hpp>
#include <memory>
#include <string>
#include <deque>

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

class ArmorDetectorNNNode : public rclcpp::Node {
public:
  explicit ArmorDetectorNNNode(const rclcpp::NodeOptions& options);

private:
  // --- lifecycle ---
  void initializeParameters();
  void validateParameters();

  // --- subscriptions ---
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& ci_msg);

  // --- service ---
  void setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // --- publishing ---
  void publishEmptyArmors(const std_msgs::msg::Header& header);
  void createDebugPublishers();
  void destroyDebugPublishers();

  // --- parameter callback ---
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(const std::vector<rclcpp::Parameter>& params);

  // --- config ---
  DetectorConfig config_;
  DetectMode current_mode_{DetectMode::DISABLED};

  // --- subscriptions ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  cv::Point2f cam_center_;

  // --- publishers ---
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // --- debug publishers ---
  bool debug_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;
  // Future: armor_detector/debug_nn, armor_detector/profile

  // --- service ---
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // --- heartbeat ---
  HeartBeatPublisher::SharedPtr heartbeat_;

  // --- parameter handle ---
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;

  // --- tracking aids (reserved for future phases) ---
  std::deque<FrameDetections> recent_detections_;
};

} // namespace fyt::auto_aim

#endif
```

### 4.2 `armor_detector_nn_node.cpp`

Key implementation notes for the Phase 1 node skeleton:

```cpp
#include "armor_detector_nn/armor_detector_nn_node.hpp"
#include <rm_utils/logger/log.hpp>

namespace fyt::auto_aim {

ArmorDetectorNNNode::ArmorDetectorNNNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("armor_detector", options)  // node name fixed for compatibility
{
  FYT_INFO("armor_detector_nn", "Starting armor_detector_nn node (Phase 1 skeleton)");

  initializeParameters();
  validateParameters();

  // --- subscriptions ---
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNNNode::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNNNode::cameraInfoCallback, this, std::placeholders::_1));

  // --- publishers ---
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
    "armor_detector/armors", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "armor_detector/marker", rclcpp::SensorDataQoS());

  // --- service ---
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode",
    std::bind(&ArmorDetectorNNNode::setModeCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // --- debug publishers ---
  if (debug_) {
    createDebugPublishers();
  }

  // --- heartbeat ---
  heartbeat_ = std::make_shared<HeartBeatPublisher>(this);

  // --- dynamic parameters ---
  on_set_parameters_callback_handle_ =
    this->add_on_set_parameters_callback(
      std::bind(&ArmorDetectorNNNode::onSetParameters, this, std::placeholders::_1));

  FYT_INFO("armor_detector_nn", "Node initialized. Mode: DISABLED (awaiting set_mode)");
}
```

**imageCallback (Phase 1 stub):**

```cpp
void ArmorDetectorNNNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  if (current_mode_ == DetectMode::DISABLED) {
    return;
  }
  // Phase 1: publish empty Armors to keep the topic alive.
  // This lets downstream nodes verify topic connectivity without a real model.
  publishEmptyArmors(img_msg->header);
}
```

**setModeCallback:**

```cpp
void ArmorDetectorNNNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  switch (request->mode) {
    case rm_interfaces::srv::SetMode::Request::AUTO_AIM_RED:
      current_mode_ = DetectMode::RED;
      response->success = true;
      FYT_INFO("armor_detector_nn", "Mode set to RED");
      break;
    case rm_interfaces::srv::SetMode::Request::AUTO_AIM_BLUE:
      current_mode_ = DetectMode::BLUE;
      response->success = true;
      FYT_INFO("armor_detector_nn", "Mode set to BLUE");
      break;
    default:
      current_mode_ = DetectMode::DISABLED;
      response->success = true;
      FYT_INFO("armor_detector_nn", "Mode set to DISABLED");
      break;
  }
}
```

**publishEmptyArmors:**

```cpp
void ArmorDetectorNNNode::publishEmptyArmors(const std_msgs::msg::Header& header)
{
  rm_interfaces::msg::Armors msg;
  msg.header = header;
  msg.number = {};
  // Remaining fields default to zero / empty.
  armors_pub_->publish(msg);
}
```

---

## 5. Configuration Files

### 5.1 `config/armor_detector_nn.yaml`

Full parameter set — only `debug` and `target_frame` are active in Phase 1; the rest are declared and validated but not consumed until later phases.

```yaml
armor_detector:
  ros__parameters:
    debug: true
    target_frame: "odom"

    backend:
      type: "onnxruntime"        # onnxruntime / openvino / tensorrt
      device: "cpu"
      precision: "fp32"
      model_path: ""
      engine_path: ""
      openvino_model_xml: ""
      openvino_model_bin: ""
      calibration_cache: ""
      input_name: "images"
      output_names: ["output0"]
      warmup_iterations: 10
      num_threads: 2
      preallocate_buffers: true
      use_pinned_memory: true
      cuda_stream_count: 1
      gpu_preprocess: false
      gpu_decode: false
      allow_fallback: false
      fallback_type: "onnxruntime"

    preprocess:
      input_width: 640
      input_height: 640
      input_layout: "nchw"
      input_color: "rgb"
      resize_mode: "letterbox"
      normalize: true
      mean: [0.0, 0.0, 0.0]
      std: [255.0, 255.0, 255.0]
      pad_value: 114

    postprocess:
      strategy: "ultralytics_pose"
      output_layout: "channels_first"
      num_classes: 14
      num_keypoints: 4
      keypoint_dims: 2
      bbox_offset: 0
      class_offset: 4
      keypoint_offset: 18
      box_format: "cxcywh"
      conf_threshold: 0.35
      nms_threshold: 0.45
      max_detections: 32
      class_agnostic_nms: false
      keypoint_remap: [0, 1, 2, 3]

    label_map:
      path: ""

    pose:
      use_ba: true
      pnp_method: "ippe"
      small_armor_width: 0.133
      small_armor_height: 0.050
      large_armor_width: 0.225
      large_armor_height: 0.050

    runtime:
      platform_profile: "custom"   # jetson / nuc_cpuonly / nuc_with_gpu / custom
      color_filter_source: "model"
      publish_empty: true
      drop_frame_when_busy: true
      copy_policy: "copy_on_write_debug"
      scheduling_mode: "sync"
      frame_queue_size: 2
      batch_min_size: 1
      batch_max_size: 2
      batch_timeout_ms: 2.0
      max_observation_age_ms: 50.0
      publish_out_of_order: false
      profile: true
```

### 5.2 `config/label_map.yaml`

Static data file (not a ROS parameter file) — identical to the one defined in the overall design doc. Refer to [armor_detector_nn_design.md section 9](armor_detector_nn_design.md) for the full content.

---

## 6. Parameter Declaration & Validation

### 6.1 `initializeParameters()`

All nested parameters must be explicitly declared so the ROS parameter system recognizes them. The declaration uses the `rclcpp::ParameterDescriptor` API to attach constraints (ranges, allowed-values) where applicable. Nested structs are flattened to dot-separated names.

Rule for Phase 1: declare everything, even if the value is not consumed yet. This prevents "undeclared parameter" warnings when the YAML is loaded.

### 6.2 `validateParameters()`

Phase 1 checks:

| Parameter | Check |
|-----------|-------|
| `backend.type` | must be one of `onnxruntime`, `openvino`, `tensorrt` |
| `backend.precision` | must be one of `fp32`, `fp16`, `int8` |
| `postprocess.conf_threshold` | must be in [0.0, 1.0] |
| `postprocess.nms_threshold` | must be in [0.0, 1.0] |
| `postprocess.num_classes` | must be >= 1 |
| `runtime.scheduling_mode` | must be one of `sync`, `async_latest`, `async_batch` |
| `runtime.platform_profile` | must be one of `jetson`, `nuc_cpuonly`, `nuc_with_gpu`, `custom` |
| `runtime.copy_policy` | must be one of `always_copy`, `copy_on_write_debug`, `zero_copy_strict` |
| `runtime.batch_min_size` | must be >= 1 |
| `runtime.batch_max_size` | must be >= `batch_min_size` |
| `runtime.frame_queue_size` | must be >= 1 |

On validation failure, log the error and fall back to compiled-in defaults. Do not crash — this is important for field debugging where the config file may be stale.

### 6.3 Platform Profile Load Order

For `runtime.platform_profile`, use deterministic merge order:

```text
1) Apply platform profile defaults
2) Apply YAML explicit overrides
3) Validate and clamp by backend capabilities (batch/device/backend availability)
4) Emit one startup summary log with effective config
```

Phase 1 only defines this contract and parsing; capability probing is implemented in backend phases.

---

## 7. Interfaces to Future Phases

### 7.1 `inference_backend_factory.hpp` (Phase 1 stub)

```cpp
#ifndef ARMOR_DETECTOR_NN_INFERENCE_BACKEND_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_INFERENCE_BACKEND_FACTORY_HPP_

#include "armor_detector_nn/backend/inference_backend.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include <memory>

namespace fyt::auto_aim {

class InferenceBackendFactory {
public:
  // Create a backend instance based on config.
  // Returns nullptr if the requested backend is not compiled in.
  static std::unique_ptr<IInferenceBackend> create(const BackendConfig& config);

  // Check whether a given backend type is available at runtime.
  static bool isAvailable(BackendType type);
};

} // namespace fyt::auto_aim

#endif
```

Phase 1 implementation returns `nullptr` for all types since no backends are compiled in. The factory is created now so that the node's pipeline-initialization code can call `create()` without conditional compilation.

### 7.2 `decode_strategy_factory.hpp` (Phase 1 stub)

```cpp
#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_FACTORY_HPP_

#include "armor_detector_nn/postprocess/decode_strategy.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include <memory>

namespace fyt::auto_aim {

class DecodeStrategyFactory {
public:
  static std::unique_ptr<IDecodeStrategy> create(const PostprocessConfig& config);
};

} // namespace fyt::auto_aim

#endif
```

### 7.3 `armor_detector_nn.hpp` (forward-declare, Phase 1 stub)

```cpp
#ifndef ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_

#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/detection_types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace fyt::auto_aim {

// Core pipeline — owns backend, preprocessor, postprocessor, pose adapter.
// Phase 1: minimal stub, no-op detect().
class ArmorDetectorNN {
public:
  explicit ArmorDetectorNN(const DetectorConfig& config);
  ~ArmorDetectorNN();

  // Batch-oriented interface from day one, even though Phase 1 is sync + batch=1.
  // Returns one entry per input image; inner vector may be empty (no detections).
  std::vector<FrameDetections> detectBatch(const std::vector<cv::Mat>& images,
                                           const std::vector<std_msgs::msg::Header>& headers);

  const DetectorConfig& config() const { return config_; }

private:
  DetectorConfig config_;
  // Backend, preprocessor, etc. are populated in later phases.
};

} // namespace fyt::auto_aim

#endif
```

Design rationale for `detectBatch(const std::vector<cv::Mat>&)`:
- Even though Phase 1 is batch=1, the interface takes a vector so that later async-batch phases do not require a signature change.
- Callers in Phase 1 pass a single-element vector and receive a single-element return.

---

## 8. Test Plan

### 8.1 `test_interface_compatibility.cpp`

| Test | Verification |
|------|-------------|
| Node starts and registers topics | `ros2 topic list` shows `/armor_detector/armors`, `/armor_detector/marker` |
| Service exists | `ros2 service list` shows `/armor_detector/set_mode` |
| Topic publishes empty Armors on `set_mode RED` | `ros2 topic echo /armor_detector/armors` shows empty message with correct header |
| Topic stops on `set_mode DISABLED` | No messages after mode switch |
| `Armors.header.stamp` matches `Image.header.stamp` | Compare timestamps |
| Dynamic parameter update works | Change `debug` via `ros2 param set` and verify |

### 8.2 `test_label_map.cpp`

Implement in Phase 1 because the label map has no downstream dependencies:

| Test | Verification |
|------|-------------|
| `B1 → blue / 1 / small` | `lookup(0)` returns correct entry |
| `R2 → red / 2 / large` | `lookup(8)` returns correct entry |
| `RO → red / outpost / large` | `lookup(12)` returns correct entry |
| Missing class_id → validation error | `validate()` returns false |
| Duplicate class_id → validation error | `validate()` returns false |
| `ignore_labels` entry exists in model_labels | invalid config is rejected |
| `publish_number` not in allowed set | invalid config is rejected |
| `publish_type` not in `{small,large,invalid}` | invalid config is rejected |
| `color` not in `{red,blue,unknown}` | invalid config is rejected |
| `filterByColor(RED)` drops all blue detections | Correct filtering |

---

## 9. Phase 1 Acceptance Criteria

1. Package builds with `colcon build --packages-select armor_detector_nn`.
2. Node runs in composable container via `ros2 launch armor_detector_nn armor_detector_nn.launch.py`.
3. `/armor_detector/armors` publishes empty messages when `set_mode` is RED or BLUE.
4. `/armor_detector/set_mode` service responds correctly.
5. All parameters declared in YAML are recognized by `ros2 param list`.
6. Dynamic parameter update triggers `onSetParameters` without crash.
7. `test_label_map` passes all validation cases.
8. Node shuts down cleanly on SIGINT (no resource leaks).

---

## 10. Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| `package://` URI resolution in `model_path` differs between launch and direct-run | Phase 1 default values are empty strings; resolution is deferred to Phase 2 |
| Legacy `armor_detector` publishes on same topic name | Launch file must ensure only one node uses the name `armor_detector` |
| Parameter schema evolves between phases | Declare all parameters upfront so ROS param system doesn't reject new keys |
| `rclcpp_components` template instantiation overhead | Already handled by standard CMake macro |
