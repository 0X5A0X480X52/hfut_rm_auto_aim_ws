cmake_minimum_required(VERSION 3.16)
project(auto_buff)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(auto_buff_detector_core STATIC
  buff_detector/src/types.cpp
  buff_detector/src/yolo.cpp
)
set_target_properties(auto_buff_detector_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(auto_buff_detector_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(auto_buff_detector_core PUBLIC buff_detector/include)
target_link_libraries(auto_buff_detector_core ${OpenCV_LIBS} openvino::runtime)

add_executable(buff_detector_node buff_detector/src/buff_detector_node.cpp)
target_compile_options(buff_detector_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(buff_detector_node PRIVATE buff_detector/include)
target_link_libraries(buff_detector_node auto_buff_detector_core)
ament_target_dependencies(buff_detector_node
  ament_index_cpp
  cv_bridge
  rclcpp
  rm_interfaces
  sensor_msgs
  visualization_msgs
)

add_executable(buff_pose_estimator_node src/ros2/buff_pose_estimator_node.cpp)
target_compile_options(buff_pose_estimator_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(buff_pose_estimator_node PRIVATE
  include
)
ament_target_dependencies(buff_pose_estimator_node
  geometry_msgs
  rclcpp
  rm_interfaces
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(buff_pose_estimator_node
  ${OpenCV_LIBS}
)

install(TARGETS buff_detector_node buff_pose_estimator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config model DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_auto_buff test/test_auto_buff.cpp)
  target_compile_definitions(test_auto_buff PRIVATE
    AUTO_BUFF_TEST_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/yolox_rune_3.6m.onnx"
  )
  target_include_directories(test_auto_buff PRIVATE include buff_detector/include)
  target_link_libraries(test_auto_buff auto_buff_detector_core)
  ament_target_dependencies(test_auto_buff
    geometry_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
  )
endif()

ament_package()
