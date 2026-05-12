cmake_minimum_required(VERSION 3.16)
project(armor_detector_nn)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-Wall)

# --- backend toggles ---
option(BUILD_ONNX_BACKEND     "Build ONNX Runtime backend"   OFF)
option(BUILD_OPENVINO_BACKEND "Build OpenVINO backend"       ON )
option(BUILD_TENSORRT_BACKEND "Build TensorRT backend"       OFF)
option(BUILD_WITH_GPU         "Build all GPU/backends (TensorRT, ONNX, OpenVINO GPU parts)" OFF)

if(BUILD_WITH_GPU)
  message(STATUS "armor_detector_nn: BUILD_WITH_GPU=ON -> GPU-enabled build path")
else()
  message(STATUS "armor_detector_nn: BUILD_WITH_GPU=OFF -> CPU-only build path")
  if(BUILD_ONNX_BACKEND)
    message(WARNING "BUILD_ONNX_BACKEND is ignored when BUILD_WITH_GPU=OFF; forcing OFF")
    set(BUILD_ONNX_BACKEND OFF CACHE BOOL "Build ONNX Runtime backend" FORCE)
  endif()
  if(BUILD_TENSORRT_BACKEND)
    message(WARNING "BUILD_TENSORRT_BACKEND is ignored when BUILD_WITH_GPU=OFF; forcing OFF")
    set(BUILD_TENSORRT_BACKEND OFF CACHE BOOL "Build TensorRT backend" FORCE)
  endif()
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

# --- core sources ---
set(CORE_SOURCES
  src/core/label_map.cpp
  src/core/preprocessor.cpp
  src/core/frame_scheduler.cpp
  src/core/armor_pose_estimator_adapter.cpp
  src/core/armor_detector_nn.cpp
  src/core/number_classifier_adapter.cpp
  src/core/pose_refine/single_yaw_refiner.cpp
  src/core/pose_refine/sliding_window_refiner.cpp
  src/core/tracker/internal_iou_tracker_strategy.cpp
  src/core/corner_refine/roi_pca_corner_refiner.cpp
)

# --- backend sources ---
set(BACKEND_SOURCES
  src/backend/inference_backend_factory.cpp
)

# --- ONNX 后端（仅 GPU 路径启用） ---
if(BUILD_WITH_GPU AND BUILD_ONNX_BACKEND)
  set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime root directory")
  if(NOT ONNXRUNTIME_ROOT)
    set(ONNXRUNTIME_ROOT $ENV{ONNXRUNTIME_ROOT})
  endif()
  if(NOT ONNXRUNTIME_ROOT)
    message(FATAL_ERROR "BUILD_ONNX_BACKEND=ON, but ONNXRUNTIME_ROOT is not set")
  endif()
  set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT}/include)
  set(ONNXRUNTIME_LIB ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so)
  if(NOT EXISTS ${ONNXRUNTIME_LIB})
    message(FATAL_ERROR "Cannot find libonnxruntime.so at ${ONNXRUNTIME_LIB}")
  endif()
  list(APPEND BACKEND_SOURCES src/backend/onnxruntime_backend.cpp)
endif()

# --- OpenVINO 后端 ---
if(BUILD_OPENVINO_BACKEND)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  list(APPEND BACKEND_SOURCES src/backend/openvino_backend.cpp)
endif()

# --- TensorRT 后端（仅 GPU 路径启用） ---
if(BUILD_WITH_GPU AND BUILD_TENSORRT_BACKEND)
  find_package(CUDAToolkit QUIET)
  if(NOT CUDAToolkit_FOUND)
    message(FATAL_ERROR "BUILD_TENSORRT_BACKEND=ON but CUDA not found. Set CUDAToolkit_ROOT or install CUDA.")
  endif()

  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS /usr/include /usr/local/include /usr/include/aarch64-linux-gnu)
  find_library(TENSORRT_NVINFER_LIB nvinfer
    HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)

  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_NVINFER_LIB)
    message(FATAL_ERROR
      "BUILD_TENSORRT_BACKEND=ON but TensorRT headers/libs not found "
      "(need NvInfer.h and libnvinfer)")
  endif()

  list(APPEND BACKEND_SOURCES src/backend/tensorrt_backend.cpp)
endif()

# --- postprocess sources ---
set(POSTPROCESS_SOURCES
  src/postprocess/decode_strategy_factory.cpp
  src/postprocess/ultralytics_pose_decode_strategy.cpp
  src/postprocess/robotpilots_landmark_decode_strategy.cpp
)

# --- geometry sources ---
set(GEOMETRY_SOURCES src/geometry/nms.cpp)

# --- debug sources ---
set(DEBUG_SOURCES
  src/debug/debug_drawer.cpp
  src/debug/profiler.cpp
)

# --- node sources ---
set(NODE_SOURCES src/armor_detector_nn_node.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
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
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)

if(BUILD_WITH_GPU AND BUILD_ONNX_BACKEND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${ONNXRUNTIME_LIB})
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_NN_HAS_ONNX=1)
endif()

if(BUILD_OPENVINO_BACKEND)
  target_link_libraries(${PROJECT_NAME} openvino::runtime)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_NN_HAS_OPENVINO=1)
endif()

if(BUILD_WITH_GPU AND BUILD_TENSORRT_BACKEND AND CUDAToolkit_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${TENSORRT_NVINFER_LIB} CUDA::cudart)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_NN_HAS_TENSORRT=1)
endif()

# --- node registration ---
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::ArmorDetectorNNNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)