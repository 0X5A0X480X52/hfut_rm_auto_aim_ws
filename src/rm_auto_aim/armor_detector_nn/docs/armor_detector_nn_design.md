# armor_detector_nn 设计方案

## 1. 背景与目标

当前 `armor_detector` 基于 OpenCV 灯条提取、灯条匹配、数字分类和 PnP 解算实现装甲板检测。在 Jetson 等边缘平台上，传统图像处理和 CPU 调度容易成为性能瓶颈，因此计划新增 `armor_detector_nn` 包，以神经网络检测替代原始检测流程。

`armor_detector_nn` 的设计目标是对前后节点透明：

- 对上游相机节点保持相同订阅接口：`image_raw`、`camera_info`。
- 对下游跟踪、融合、解算节点保持相同发布接口：`armor_detector/armors`。
- 保持 `armor_detector/set_mode` 服务语义。
- 保持 `rm_interfaces/msg/Armors` 中 `number`、`type`、`pose`、`distance_to_image_center` 的含义。
- 支持 ONNX Runtime、OpenVINO、TensorRT 多推理后端。
- 支持 fp32、fp16、int8 等精度配置。
- 使用策略模式扩展不同模型输出格式和推理后端。
- 使用 label-map 将模型输出类别 id 映射为原 `armor_detector` 兼容标签。

第一阶段以当前 Ultralytics YOLOv13 armor pose 模型为主：

```text
Input:
  images: [1, 3, 640, 640]

Output:
  output0: [1, 26, 8400]

Task:
  pose

Classes:
  0: B1
  1: B2
  2: B3
  3: B4
  4: B5
  5: BO
  6: BS
  7: R1
  8: R2
  9: R3
  10: R4
  11: R5
  12: RO
  13: RS

kpt_shape:
  [4, 2]
```

## 2. 外部接口兼容性

新节点建议仍注册为 ROS2 component，并在 launch 中使用节点名 `armor_detector`。包名可以是 `armor_detector_nn`，但节点名、话题名和服务名应默认保持原包兼容。

### 2.1 订阅

```text
image_raw                 sensor_msgs/msg/Image
camera_info               sensor_msgs/msg/CameraInfo
```

### 2.2 发布

```text
armor_detector/armors     rm_interfaces/msg/Armors
armor_detector/marker     visualization_msgs/msg/MarkerArray
```

debug 模式下建议发布：

```text
armor_detector/result_img     sensor_msgs/msg/Image
armor_detector/debug_nn       可选，自定义或诊断信息
armor_detector/profile        可选，诊断信息
```

原始 `debug_lights`、`binary_img`、`number_img` 与神经网络方案不完全对应，可以不作为第一阶段强兼容项。但如果已有可视化工具依赖这些 topic，可通过空消息或替代 debug topic 做过渡。

### 2.3 服务

```text
armor_detector/set_mode    rm_interfaces/srv/SetMode
```

语义保持：

```text
AUTO_AIM_RED   -> 启用检测，仅发布红方装甲板 R*
AUTO_AIM_BLUE  -> 启用检测，仅发布蓝方装甲板 B*
其他模式       -> 暂停检测或停止订阅 image_raw
```

注意：原 `armor_detector` 中 `AUTO_AIM_RED` 表示检测红色目标，`AUTO_AIM_BLUE` 表示检测蓝色目标。新模型类别中已经包含颜色前缀，因此 set_mode 应优先作为类别过滤条件。

## 3. 模型输出约定

当前模型为 Ultralytics pose 模型，输出张量为：

```text
output0: [1, 26, 8400]
```

实现上需要注意不同导出/后端可能出现等价布局：

```text
[1, 26, 8400] 或 [1, 8400, 26]
```

因此建议在 `DecodeConfig` 中显式配置布局，不在代码中隐式假设固定轴顺序。

对每个候选位置，26 维建议按如下方式解释：

```text
0..3      bbox: cx, cy, w, h
4..17     class scores: 14 classes
18..25    keypoints: 4 x 2, 即 x1,y1,x2,y2,x3,y3,x4,y4
```

该解释与 Ultralytics pose 导出的常见输出一致，但实现时必须通过配置显式声明，不能在通用 postprocess 中写死。

候选数量为：

```text
8400 = 80x80 + 40x40 + 20x20
```

模型输入尺寸固定为 `640x640`，第一阶段按静态 batch=1 设计。后续如需动态尺寸或动态 batch，通过 backend 和 preprocessor 配置扩展。

## 4. 包架构

推荐目录：

```text
armor_detector_nn/
  CMakeLists.txt
  package.xml

  launch/
    armor_detector_nn.launch.py

  config/
    armor_detector_nn.yaml
    label_map.yaml

  model/
    model_xxx/
      README.md
      best.onnx
      best_fp16.engine
      best_int8.engine
      calibration.cache
      openvino/
        best.xml
        best.bin
    ...

  include/armor_detector_nn/
    armor_detector_nn_node.hpp

    core/
      armor_detector_nn.hpp
      detector_config.hpp
      detection_types.hpp
      label_map.hpp
      preprocessor.hpp
      postprocessor.hpp
      armor_pose_estimator_adapter.hpp

    backend/
      inference_backend.hpp
      inference_backend_factory.hpp
      onnxruntime_backend.hpp
      openvino_backend.hpp
      tensorrt_backend.hpp

    postprocess/
      decode_strategy.hpp
      ultralytics_pose_decode_strategy.hpp
      decode_strategy_factory.hpp

    geometry/
      armor_geometry_builder.hpp
      armor_type_resolver.hpp
      nms.hpp

    debug/
      debug_drawer.hpp
      profiler.hpp

  src/
    armor_detector_nn_node.cpp
    core/
    backend/
    postprocess/
    geometry/
    debug/

  docs/
    armor_detector_nn_design.md

  test/
    test_label_map.cpp
    test_ultralytics_pose_decode.cpp
    test_interface_compatibility.cpp
```

## 5. 核心数据流

```text
sensor_msgs/Image
  -> cv_bridge
  -> Preprocessor
  -> IInferenceBackend::infer()
  -> IDecodeStrategy::decode()
  -> LabelMap
  -> color / confidence / NMS filter
  -> keypoints restore to original image coordinates
  -> ArmorPoseEstimatorAdapter
  -> rm_interfaces/msg/Armors
```

## 6. Jetson 优先的内存路径设计

在 Jetson 上，性能优化优先级应为：

```text
减少内存读写/搬运 > 提升算子吞吐 > 提高并发复杂度
```

推荐约束：

- 单帧尽量保持单次主路径：`camera buffer -> preprocess -> infer -> decode -> publish`。
- 避免重复的 `cv::Mat` 深拷贝。
- 避免每帧重新申请 host/device 大块内存。
- 避免 host-device 来回搬运中间结果。
- debug 与主路径解耦，主路径默认不做任何可视化绘制。

### 6.1 目标内存路径

Jetson + TensorRT 主路径建议：

```text
Image msg (CPU)
  -> 预分配 pinned host 输入缓冲
  -> CUDA stream 异步 H2D
  -> GPU preprocess (resize/letterbox/normalize/color convert)
  -> TensorRT enqueueV3
  -> GPU decode/NMS (可选，后续优化)
  -> 必要的最小结果拷回 CPU（候选框/关键点）
  -> PnP + 发布
```

第一阶段允许 CPU preprocess + GPU infer，但接口必须预留 GPU preprocess 路径，避免后续大改。

### 6.2 复制策略

建议统一策略：

```text
release 模式:
  检测主路径使用共享/只读图像视图，不做额外拷贝。

debug 模式:
  仅在要绘制调试图时执行 copy-on-write（toCvCopy）。
```

这样可以兼顾性能和安全：非 debug 时低拷贝，debug 时避免修改共享缓冲。

### 6.3 缓冲区与分配策略

建议后端实现具备：

- 固定大小输入输出缓冲区预分配。
- batch=1~2 的双缓冲或环形缓冲。
- pinned host memory（`cudaHostAlloc`）用于 H2D。
- 复用 CUDA stream，不在每帧创建/销毁 stream 或 event。

建议暴露统计：

```text
h2d_ms
d2h_ms
gpu_preprocess_ms
gpu_decode_ms
host_copy_bytes_per_frame
device_copy_bytes_per_frame
```

### 6.4 何时允许更高复杂度

当以下任一条件满足时，可以接受实现复杂度上升：

- `infer_ms` 明显低于 `h2d+d2h+preprocess`（说明搬运是主瓶颈）。
- GPU 利用率低但 CPU 占用高。
- batch=1 延迟已达标但吞吐不足。

对应优化顺序建议：

1. 预分配 + pinned memory
2. copy-on-write debug 路径
3. CUDA preprocess
4. GPU decode/NMS
5. 双缓冲 + 异步流水线

节点层只处理 ROS 生命周期、参数、话题、服务、TF 和发布。算法层不直接依赖 ROS topic。

## 7. 核心类型设计

### 6.1 模型原始检测结果

```cpp
struct RawDetection {
  int class_id{-1};
  float class_score{0.0F};
  float object_score{1.0F};
  float confidence{0.0F};
  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;
};
```

当前模型没有单独 objectness 时，`object_score` 可固定为 1.0，`confidence = max_class_score`。

### 6.2 label-map 后的检测结果

```cpp
struct ArmorDetection {
  std::string model_label;      // B1, R1, BO 等
  std::string publish_number;   // 1, 2, outpost, sentry 等
  std::string publish_type;     // small, large
  EnemyColor color;             // red / blue
  float confidence{0.0F};
  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;
  cv::Point2f center;
};
```

### 6.3 推理输入输出

```cpp
struct TensorInput {
  std::string name;
  std::vector<int64_t> shape;
  std::vector<float> host_data;
};

struct TensorOutput {
  std::string name;
  std::vector<int64_t> shape;
  std::vector<float> host_data;
};
```

第一阶段可先统一使用 float 输出。TensorRT fp16/int8 后端内部负责反量化或转 float，避免 postprocess 关心具体推理精度。

## 8. 推理后端策略

### 7.1 接口

```cpp
class IInferenceBackend {
public:
  virtual ~IInferenceBackend() = default;

  virtual void load(const BackendConfig& config) = 0;
  virtual std::vector<TensorOutput> infer(const TensorInput& input) = 0;
  virtual void warmup(int iterations) = 0;
  virtual BackendInfo info() const = 0;
};
```

### 7.2 后端实现

```text
OnnxRuntimeBackend
  - CPUExecutionProvider
  - CUDAExecutionProvider
  - 支持 fp32/fp16 onnx，int8 onnx 视 ORT 支持情况而定

OpenVINOBackend
  - CPU
  - 主要面向 Intel CPU / NUC
  - 使用 xml/bin 或直接读取 onnx 后 compile

TensorRTBackend
  - Jetson 优先路径
  - 加载预构建 engine
  - 支持 fp16 / int8
  - int8 engine 依赖 calibration.cache 或离线构建产物
```

不建议节点启动时自动从 ONNX 构建 TensorRT engine。Jetson 上构建耗时长且容易受 TensorRT 版本、GPU 架构、workspace 限制影响。推荐部署阶段离线生成 engine，节点只加载。

## 9. 后处理策略

### 8.1 通用接口

```cpp
class IDecodeStrategy {
public:
  virtual ~IDecodeStrategy() = default;

  virtual std::vector<RawDetection> decode(
      const std::vector<TensorOutput>& outputs,
      const ImageMeta& image_meta,
      const DecodeConfig& config) = 0;
};
```

### 8.2 当前模型策略：UltralyticsPoseDecodeStrategy

该策略专门处理：

```text
output shape: [1, 26, 8400]
or: [1, 8400, 26]
num_classes: 14
num_keypoints: 4
keypoint_dims: 2
box_format: cxcywh
```

解析流程：

1. 根据 `output_layout` 归一化为统一视图（逻辑上 `C x N`），其中 `C=26`，`N=8400`。
2. 对每个候选 `i`：
   - 读取 bbox：`cx, cy, w, h`。
   - 读取 14 个类别分数。
   - 取最大类别分数和对应 `class_id`。
   - 若低于 `conf_threshold`，丢弃。
   - 读取 4 个关键点。
   - 将 bbox 从 `cxcywh` 转为 `xyxy` 或 `cv::Rect2f`。
3. 根据 letterbox 信息将 bbox 和 keypoints 映射回原图坐标。
4. 执行 NMS。
5. 输出 `RawDetection`。

NMS 建议优先使用按类别 NMS：

```text
same class + IoU > nms_threshold -> 保留 confidence 高者
```

如果实战中同一装甲板颜色/数字抖动明显，可配置为 class-agnostic NMS，再用最高分结果决定类别。

### 8.3 后续扩展策略

```text
YoloDetectDecodeStrategy
  仅 bbox + class，无 keypoint。需要额外角点估计，不推荐作为主路径。

AnchorFreePoseDecodeStrategy
  center/scale/keypoint 风格输出。

TwoStageArmorDecodeStrategy
  一阶段 armor 检测，二阶段 number 分类。

CustomTensorDecodeStrategy
  适配自定义模型。
```

## 10. Label Map 设计

模型类别包含颜色信息，而原始 `armor_detector` 发布的 `number` 不包含颜色。颜色用于 set_mode 过滤，发布时只发布原兼容标签。

推荐 `config/label_map.yaml`：

```yaml
version: 1

model_labels:
  0:
    model_label: "B1"
    color: "blue"
    publish_number: "1"
    publish_type: "small"
  1:
    model_label: "B2"
    color: "blue"
    publish_number: "2"
    publish_type: "large"
  2:
    model_label: "B3"
    color: "blue"
    publish_number: "3"
    publish_type: "small"
  3:
    model_label: "B4"
    color: "blue"
    publish_number: "4"
    publish_type: "small"
  4:
    model_label: "B5"
    color: "blue"
    publish_number: "5"
    publish_type: "small"
  5:
    model_label: "BO"
    color: "blue"
    publish_number: "outpost"
    publish_type: "large"
  6:
    model_label: "BS"
    color: "blue"
    publish_number: "sentry"
    publish_type: "small"
  7:
    model_label: "R1"
    color: "red"
    publish_number: "1"
    publish_type: "small"
  8:
    model_label: "R2"
    color: "red"
    publish_number: "2"
    publish_type: "large"
  9:
    model_label: "R3"
    color: "red"
    publish_number: "3"
    publish_type: "small"
  10:
    model_label: "R4"
    color: "red"
    publish_number: "4"
    publish_type: "small"
  11:
    model_label: "R5"
    color: "red"
    publish_number: "5"
    publish_type: "small"
  12:
    model_label: "RO"
    color: "red"
    publish_number: "outpost"
    publish_type: "large"
  13:
    model_label: "RS"
    color: "red"
    publish_number: "sentry"
    publish_type: "small"

ignore_labels: []
```

是否将 `sentry` 设为 small 或 large 需要根据训练数据和实车规范确认。第一版可以通过 label-map 配置，不在代码中硬编码。

`LabelMap` 启动时必须校验：

- `model_labels` 覆盖所有模型类别 id。
- 每个 `publish_number` 属于兼容集合：`1,2,3,4,5,outpost,sentry,base,negative`。
- 每个 `publish_type` 属于：`small,large,invalid`。
- `color` 属于：`red,blue,unknown`。
- `ignore_labels` 中的标签存在。

如果模型未来输出不带颜色的类别，例如 `1,2,3,4,5,outpost,sentry`，也通过 label-map 将 `color` 设为 `unknown`，再由配置决定是否启用图像颜色过滤。

## 11. 关键点顺序与 PnP

当前模型输出 `4 x 2` 关键点。为了和 PnP 物体点稳定对应，必须定义关键点顺序。

推荐约定：

```text
kpt0: left_bottom
kpt1: left_top
kpt2: right_top
kpt3: right_bottom
```

该顺序与当前老 `Armor::landmarks()` 在 `N_LANDMARKS == 4` 时一致：

```text
left_light.bottom
left_light.top
right_light.top
right_light.bottom
```

如果训练集标注顺序不是上述顺序，应在 `DecodeConfig` 中提供重排配置：

```yaml
postprocess:
  keypoint_order: ["left_bottom", "left_top", "right_top", "right_bottom"]
  keypoint_remap: [0, 1, 2, 3]
```

或者：

```yaml
postprocess:
  keypoint_remap: [3, 0, 1, 2]
```

不要依赖 bbox 几何自动猜测顺序作为主路径。自动排序只能作为保护逻辑，用于检测明显异常的关键点。

## 12. Pose 解算方案

第一阶段建议保留原 `ArmorPoseEstimator` 的几何语义，但不要直接耦合旧 `Detector`。新增 `ArmorPoseEstimatorAdapter`：

```text
ArmorDetection
  -> 构造 PnP 输入 landmarks
  -> 根据 publish_type 选择 small / large object points
  -> solvePnPGeneric / IPPE
  -> 可选 BA
  -> rm_interfaces/msg/Armor
```

装甲板尺寸保持当前老包定义：

```text
small width:  0.133 m
small height: 0.050 m
large width:  0.225 m
large height: 0.050 m
```

`rm_interfaces/msg/Armor` 填充规则：

```text
number = LabelMap.publish_number
type = LabelMap.publish_type
pose = PnP/BA result
distance_to_image_center = distance(center, camera_center)
```

`distance_to_image_center` 应与老 `PnPSolver::calculateDistanceToCenter()` 行为保持一致，避免下游选择策略变化。

## 13. 颜色与模式过滤

内部维护当前检测模式：

```text
DetectMode::RED
DetectMode::BLUE
DetectMode::DISABLED
```

过滤规则：

```text
AUTO_AIM_RED:
  only keep LabelMap.color == red

AUTO_AIM_BLUE:
  only keep LabelMap.color == blue

其他:
  clear image subscription 或 infer disabled
```

如果后续模型不输出颜色：

```yaml
runtime:
  color_filter_source: "model"   # model / image / disabled
```

当前模型应使用：

```yaml
runtime:
  color_filter_source: "model"
```

## 14. 参数配置建议

`config/armor_detector_nn.yaml`：

```yaml
armor_detector:
  ros__parameters:
    debug: true
    target_frame: "odom"

    backend:
      type: "tensorrt"        # onnxruntime / openvino / tensorrt
      device: "gpu"           # cpu / gpu
      precision: "fp16"       # fp32 / fp16 / int8
      model_path: "package://armor_detector_nn/model/model_xxx/best.onnx"
      engine_path: "package://armor_detector_nn/model/model_xxx/best_fp16.engine"
      openvino_model_xml: "package://armor_detector_nn/model/model_xxx/openvino/best.xml"
      openvino_model_bin: "package://armor_detector_nn/model/model_xxx/openvino/best.bin"
      calibration_cache: "package://armor_detector_nn/model/model_xxx/calibration.cache"
      input_name: "images"
      output_names: ["output0"]
      warmup_iterations: 10
      num_threads: 2
      preallocate_buffers: true
      use_pinned_memory: true
      cuda_stream_count: 1
      gpu_preprocess: true
      gpu_decode: false

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
      output_layout: "channels_first"   # channels_first:[1,26,8400], channels_last:[1,8400,26]
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
      path: "package://armor_detector_nn/config/label_map.yaml"

    pose:
      use_ba: true
      pnp_method: "ippe"
      small_armor_width: 0.133
      small_armor_height: 0.050
      large_armor_width: 0.225
      large_armor_height: 0.050

    runtime:
      platform_profile: "jetson"   # jetson / nuc_cpuonly / nuc_with_gpu / custom
      color_filter_source: "model"
      publish_empty: true
      copy_policy: "copy_on_write_debug"  # always_copy / copy_on_write_debug / zero_copy_strict
      scheduling_mode: "async_latest"   # sync / async_latest / async_batch
      frame_queue_size: 2
      batch_min_size: 1
      batch_max_size: 2
      batch_timeout_ms: 2.0
      max_observation_age_ms: 50.0
      publish_out_of_order: false
      profile: true
```

### 14.1 设备 Profile 机制

建议新增 `runtime.platform_profile`，用于表达部署环境目标，而不是把设备优化散落在多个参数中手工拼装。

可选值：

```text
jetson
nuc_cpuonly
nuc_with_gpu
custom
```

加载顺序建议：

```text
1) 先应用 profile 默认值
2) 再应用 YAML 显式覆盖
3) 启动时做能力探测（backend/device/batch/dynamic）
4) 不满足能力时按 fallback 规则降级并告警
```

### 14.2 Profile 默认策略建议

`jetson`（主部署路径）：

```text
backend.type = tensorrt
backend.device = gpu
backend.precision = fp16 (优先) / int8
backend.preallocate_buffers = true
backend.use_pinned_memory = true
backend.cuda_stream_count = 1
backend.gpu_preprocess = true
runtime.scheduling_mode = async_latest
runtime.copy_policy = copy_on_write_debug
runtime.frame_queue_size = 2
runtime.batch_max_size = 2
```

`nuc_cpuonly`：

```text
backend.type = onnxruntime 或 openvino
backend.device = cpu
backend.precision = fp32 或 int8(openvino)
backend.num_threads = 按物理核配置
backend.gpu_preprocess = false
runtime.scheduling_mode = async_latest
runtime.copy_policy = copy_on_write_debug
runtime.batch_max_size = 1
```

`nuc_with_gpu`（Intel iGPU）：

```text
backend.type = openvino
backend.device = gpu
backend.precision = fp16/int8
backend.gpu_preprocess = true (若 native preprocess 已验证一致性)
runtime.scheduling_mode = async_latest 或 async_batch
runtime.batch_max_size = 2
```

`custom`：

```text
不注入平台默认值，仅使用 YAML 显式配置。
```

### 14.3 能力探测与降级规则

Profile 只是“意图”，最终配置需要结合运行时能力收敛：

```text
若 tensorrt 不可用:
  jetson profile -> fallback_type (openvino/onnxruntime) 或启动失败（按配置）

若 backend.max_batch_size == 1:
  runtime.batch_max_size 自动降为 1
  async_batch 自动退化为 async_latest 语义

若 gpu_preprocess 不可用:
  自动回退到 CPU preprocess，并记录告警
```

实现建议：把降级结果输出到一次性启动摘要日志，方便现场排障。

调度参数约束建议：

```text
1 <= batch_min_size <= batch_max_size <= 2   (第一阶段建议上限 2)
frame_queue_size >= batch_max_size
batch_timeout_ms 建议 <= 3ms
```

## 15. TensorRT / INT8 部署设计

当前 ONNX metadata 中：

```text
half: false
int8: true
dynamic: false
batch: 1
imgsz: [640, 640]
```

注意：ONNX metadata 的 `int8: true` 不一定意味着 ONNX 本身已经是可直接高效部署的 INT8 TensorRT engine。部署时建议区分：

```text
best.onnx
  通用交换格式。

best_fp16.engine
  Jetson TensorRT FP16 推理文件。

best_int8.engine
  Jetson TensorRT INT8 推理文件。

calibration.cache
  INT8 校准缓存，仅用于构建或校验。
```

TensorRT 后端启动时校验：

- engine 文件存在。
- engine binding shape 与配置一致。
- input name / output name 匹配或可通过 binding index 映射。
- precision 与配置一致。
- engine metadata 中的 TensorRT 版本、GPU 架构、输入尺寸匹配当前环境。

如果 engine 不匹配，应明确报错，不建议自动退回 ONNX Runtime，除非配置允许：

```yaml
backend:
  allow_fallback: true
  fallback_type: "onnxruntime"
```

同时建议记录 engine 元数据文件（例如 `engine.meta.json`），至少包含：

```text
model_name
model_sha256
tensorrt_version
cuda_version
device_cc
input_shape
max_batch_size
precision
build_time
```

加载时进行一致性校验，可显著减少“能启动但性能/结果异常”的灰色故障。

## 16. 异步线程与低延迟调度

Jetson 上避免 image callback 阻塞 ROS executor 过久。实时自瞄链路的目标不是处理所有输入帧，而是在可控延迟内提供足够高频、带准确时间戳的观测。对于 YOLO 类模型，更推荐采用“接收与推理解耦 + 小容量最新帧缓存 + 动态小 batch + 主动丢旧帧”的架构。

核心原则：

- 不追求处理所有帧。
- 不允许旧帧在队列中堆积。
- 推理线程优先处理最新帧。
- batch 只在极短时间窗内 opportunistic 合并。
- 发布消息使用原始图像时间戳。
- tracker 根据观测时间戳计算 `dt`，完成预测和更新。

### 15.1 简单同步模式

```text
imageCallback -> preprocess -> infer -> postprocess -> pose -> publish
```

优点是实现简单、调试方便、时间戳链路清晰。缺点是 callback 会被推理阻塞，当相机频率高于推理能力时，ROS executor 容易产生回调堆积或处理过期帧。该模式可作为 bring-up 和单元测试路径保留。

### 15.2 最新帧异步模式

该模式适合 batch=1 的低延迟推理：

```text
imageCallback:
  receive image
  push frame into latest-frame queue
  if queue full:
    drop oldest frame

worker:
  pop newest available frame
  preprocess
  infer batch=1
  postprocess
  pose
  publish with original image header
```

队列容量建议为 1 或 2：

```text
frame_queue_size = 1:
  最低延迟，只保留最新帧。

frame_queue_size = 2:
  允许 worker 与 callback 有轻微时序抖动，仍不会产生明显积压。
```

当输入频率超过处理能力时，应主动丢弃旧帧，而不是排队处理。这样系统表现为“降低有效观测帧率，但保持观测新鲜度”。

### 15.3 动态小 batch 异步模式

该模式适合 TensorRT GPU 推理，尤其当 batch=1 无法充分利用 GPU，而 batch=2 又不会显著增加端到端延迟时使用。

```text
imageCallback:
  push latest frame into bounded queue
  if queue full:
    drop oldest

batch worker:
  wait until at least batch_min_size frame is available
  collect up to batch_max_size latest frames
  do not wait longer than batch_timeout_ms
  preprocess frames into batched tensor
  infer batch=N
  split outputs by frame
  postprocess each frame independently
  publish each Armors message with its own original header
```

推荐第一版预留接口：

```yaml
runtime:
  scheduling_mode: "async_batch"
  frame_queue_size: 2
  batch_min_size: 1
  batch_max_size: 2
  batch_timeout_ms: 2.0
```

`batch_timeout_ms` 必须很小。它的含义不是为了凑满 batch 主动等待，而是在两帧几乎同时到达时合并推理。若等待时间超过该阈值，即使只有 1 帧也立即推理。

### 15.4 丢帧策略

推荐使用 latest-first 的有界缓存：

```text
on new frame:
  if queue.size >= frame_queue_size:
    drop oldest frame
  push new frame
```

batch worker 取帧时：

```text
if queue has many frames:
  prefer newest frames
  discard frames older than max_observation_age_ms
```

`max_observation_age_ms` 用于保护极端情况，例如 GPU 短时阻塞后队列中仍有旧帧。超过年龄阈值的帧不进入推理，直接丢弃。

### 15.5 时间戳与 tracker 协同

`armor_detector_nn` 发布的 `Armors.header` 必须来自输入 `Image.header`：

```text
Armors.header.stamp = Image.header.stamp
Armors.header.frame_id = Image.header.frame_id
```

不要使用推理完成时刻作为观测时间。推理完成时刻可以作为 profiler latency 记录，但不能改变观测时间戳。

下游 tracker 应基于观测时间戳计算：

```text
dt = observation_stamp - last_update_stamp
```

并按以下流程工作：

```text
predict state to observation_stamp
associate detections
update with detections
predict/render to current control time if needed
```

这样即使 detector 主动丢帧，只要观测频率足够，tracker 仍能通过 `dt` 维持稳定估计。系统优化目标应是：

```text
高频观测 + 低延迟 + 稳定估计
```

而不是：

```text
处理所有帧
```

为减少时钟误差传播，建议统一使用 ROS 时间源，并在文档中约定：

```text
观测时间戳：Image.header.stamp
处理开始时间：node->now() at dequeue
处理结束时间：node->now() before publish
```

其中后两者仅用于性能统计，不参与状态估计时序。

### 15.6 输出顺序

动态 batch 可能导致多个图像在同一次推理后同时发布。默认要求按图像时间戳递增发布：

```yaml
runtime:
  publish_out_of_order: false
```

如果后续为了极致低延迟允许乱序发布，tracker 必须显式支持乱序观测。第一阶段不建议启用乱序发布。

### 15.7 后端 batch 能力约束

不同后端对动态 batch 的支持不同：

```text
ONNX Runtime:
  取决于 ONNX 是否 dynamic batch。
  当前模型 batch=1, dynamic=false，因此默认只支持 batch=1。

TensorRT:
  如果 engine 按 explicit batch 并支持 optimization profile，可支持 batch=1..2。
  如果 engine 固定 batch=1，则 async_batch 应自动退化为 batch=1。

OpenVINO:
  可通过 reshape 或编译模型配置支持 batch，但第一版可仅要求 batch=1。
```

因此 `IInferenceBackend` 需要暴露能力查询：

```cpp
struct BackendInfo {
  std::string backend_name;
  std::string precision;
  int min_batch_size{1};
  int max_batch_size{1};
  bool dynamic_batch{false};
};
```

调度器根据 `BackendInfo` 决定实际 batch 上限：

```text
effective_batch_max = min(config.batch_max_size, backend.max_batch_size)
```

如果当前模型或 engine 只支持 batch=1，架构仍保持异步 latest-frame，不影响低延迟目标。

### 15.8 预留类设计

建议将调度从节点和 backend 中拆出：

```cpp
class FrameScheduler {
public:
  virtual ~FrameScheduler() = default;
  virtual void submit(FramePacket frame) = 0;
  virtual std::optional<FrameBatch> waitBatch() = 0;
  virtual SchedulerStats stats() const = 0;
};

class LatestFrameScheduler : public FrameScheduler {};
class DynamicBatchScheduler : public FrameScheduler {};
```

关键数据结构：

```cpp
struct FramePacket {
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::ConstSharedPtr image_msg;
  rclcpp::Time receive_time;
  uint64_t sequence_id{0};
};

struct FrameBatch {
  std::vector<FramePacket> frames;
};

struct SchedulerStats {
  uint64_t received_frames{0};
  uint64_t dropped_old_frames{0};
  uint64_t dropped_stale_frames{0};
  uint64_t inferred_frames{0};
  double queue_age_ms{0.0};
};
```

第一版可以只实现同步模式，但 `ArmorDetectorNNNode` 与 `ArmorDetectorNN` 的接口应按 batch 输入输出设计，避免后续重构：

```cpp
std::vector<FrameDetections> detectBatch(const std::vector<cv::Mat>& images);
```

即使实际 batch=1，也返回长度为 1 的结果数组。

## 17. Debug 与性能观测

建议提供 `Profiler`，记录：

```text
preprocess_ms
infer_ms
decode_ms
nms_ms
pose_ms
total_ms
num_raw_candidates
num_after_conf
num_after_nms
num_published
backend
precision
queue_size
dropped_old_frames
dropped_stale_frames
batch_size
batch_wait_ms
observation_age_ms
end_to_end_latency_ms
```

建议再补两个统计口径，便于和 tracker 联调：

```text
effective_observation_hz
tracker_input_delay_ms
```

debug 图像绘制：

- bbox。
- 4 个 keypoints。
- `model_label`、`publish_number`、confidence。
- 当前 backend 和 FPS。

不要在非 debug 模式下执行昂贵绘制。

## 18. 测试计划

### 17.1 单元测试

```text
test_label_map
  - B1 -> blue / 1 / small
  - R2 -> red / 2 / large
  - RO -> red / outpost / large
  - 缺失 class id 启动失败

test_ultralytics_pose_decode
  - 构造 [1,26,N] 假输出
  - 验证 class offset、keypoint offset、bbox 解码
  - 验证 conf_threshold
  - 验证 keypoint_remap

test_nms
  - class-aware NMS
  - class-agnostic NMS

test_pose_adapter
  - keypoints 顺序到 object points 的对应关系

test_frame_scheduler
  - queue full 时丢弃 oldest
  - batch_timeout_ms 到期时 batch=1 立即返回
  - batch_max_size=2 时最多取 2 帧
  - stale frame 超过 max_observation_age_ms 被丢弃
```

### 17.2 接口兼容测试

使用测试节点发布图像和 camera_info，检查：

```text
/armor_detector/armors 存在
/armor_detector/set_mode 存在
Armors.header == Image.header
Armor.number 属于老标签集合
Armor.type 属于 small/large
异步模式下 Armors.header.stamp 仍等于对应 Image.header.stamp
```

### 17.3 rosbag 回放测试

对同一 rosbag 对比：

```text
armor_detector:
  fps
  latency
  armors count
  tracker stability

armor_detector_nn:
  fps
  latency
  observation age
  drop rate
  effective batch size
  armors count
  tracker stability
```

## 19. 实施里程碑

### 阶段 1：包骨架和接口兼容

- 新增 `armor_detector_nn` 包。
- 注册 component 节点，节点名默认 `armor_detector`。
- 订阅 `image_raw`、`camera_info`。
- 发布空 `armor_detector/armors`。
- 实现 `armor_detector/set_mode`。
- 参数中预留 `scheduling_mode`、`frame_queue_size`、`batch_max_size`、`batch_timeout_ms`。

### 阶段 2：ONNX Runtime CPU 跑通

- 实现 `IInferenceBackend`。
- 实现 `OnnxRuntimeBackend`。
- 实现 `Preprocessor`。
- 能读取 `best.onnx` 并得到 `output0`。

### 阶段 3：Ultralytics pose 后处理

- 实现 `UltralyticsPoseDecodeStrategy`。
- 实现 label-map。
- 输出 2D 检测 debug 图。

### 阶段 4：PnP 输出 Armors

- 实现 `ArmorPoseEstimatorAdapter`。
- 输出完整 `rm_interfaces/msg/Armors`。
- 与下游 tracker/fusion 联调。

### 阶段 5：Jetson TensorRT

- 实现 `TensorRTBackend`。
- 支持 fp16 engine 加载。
- 加入 warmup 和 profiler。
- 根据 engine 能力暴露 `min_batch_size`、`max_batch_size`、`dynamic_batch`。

### 阶段 6：INT8 和 OpenVINO

- 支持 TensorRT INT8 engine。
- 支持 OpenVINO CPU。
- 完善 backend fallback 和部署校验。

### 阶段 7：异步低延迟调度

- 实现 `LatestFrameScheduler`。
- 支持小容量队列和主动丢旧帧。
- 保证发布 header 使用原始图像时间戳。
- 增加 drop rate、observation age、queue age 指标。

### 阶段 8：动态小 batch

- 实现 `DynamicBatchScheduler`。
- 支持 batch=1..2 的延迟受限合批。
- TensorRT engine 支持时启用 batch=2。
- engine 不支持动态 batch 时自动退化到 batch=1。

### 阶段 9：系统验收与回归基线

- 固定 rosbag + 固定配置，产出 detector/tracker 的基线报表。
- 验收指标至少包含：
  - `P95 end_to_end_latency_ms`
  - `effective_observation_hz`
  - `drop_rate`
  - `tracker stability`（丢失率、抖动）
- 建立变更回归门槛，避免后续优化造成隐性退化。

## 20. 主要风险

### 19.1 关键点顺序不一致

如果训练标注顺序与 PnP object points 不一致，会直接导致姿态错误。必须通过文档、配置和测试固定顺序。

### 19.2 类别 id 漂移

模型训练数据或导出流程变化会改变类别顺序。发布前必须经过 label-map，不允许直接把 class id 转字符串发布。

### 19.3 TensorRT engine 环境绑定

TensorRT engine 与 Jetson 型号、TensorRT 版本、CUDA 版本相关。部署时需要按平台构建，不应跨平台复用 engine。

### 19.4 与老 detector 行为差异

NN 直接输出完整装甲板，传统方法通过灯条几何约束过滤。新方案可能在遮挡、强反光、远距离小目标下出现不同错误模式。需要通过 rosbag 和实车数据调 `conf_threshold`、NMS、keypoint 质量过滤。

## 21. 第一版推荐决策

第一版实现建议固定以下选择：

```text
backend:
  优先 ONNX Runtime CPU/GPU 跑通功能
  Jetson 部署使用 TensorRT FP16

postprocess:
  UltralyticsPoseDecodeStrategy
  output [1,26,8400]
  class-aware NMS

label:
  B*/R* 映射为原 number
  set_mode 根据 color 过滤

pose:
  4 keypoints PnP
  保持 small/large 尺寸与老包一致

ROS:
  节点名 armor_detector
  默认发布 armor_detector/armors
```

调度:
  默认 async_latest
  frame_queue_size=2
  batch=1
  仅在 engine 明确支持时启用 batch=2

这样能以最小下游改动替换当前 `armor_detector`，同时把后端、后处理和 label-map 都留下清晰扩展点。
