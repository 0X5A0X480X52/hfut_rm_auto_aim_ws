cmake_minimum_required(VERSION 3.16)
project(armor_detector_nn)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/armor_detector_nn_node.cpp
  src/backend/inference_backend_factory.cpp
  src/backend/openvino_backend.cpp
  src/core/armor_detector_nn.cpp
  src/core/armor_pose_estimator_adapter.cpp
  src/core/corner_refine/roi_pca_corner_refiner.cpp
  src/core/label_map.cpp
  src/core/pose_refine/single_yaw_refiner.cpp
  src/core/pose_refine/sliding_window_refiner.cpp
  src/core/preprocessor.cpp
  src/core/tracker/internal_iou_tracker_strategy.cpp
  src/debug/debug_drawer.cpp
  src/debug/profiler.cpp
  src/geometry/nms.cpp
  src/postprocess/decode_strategy_factory.cpp
  src/postprocess/robotpilots_landmark_decode_strategy.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::ArmorDetectorNNNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_nn_pipeline test/test_nn_pipeline.cpp)
  target_link_libraries(test_nn_pipeline ${PROJECT_NAME})
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
