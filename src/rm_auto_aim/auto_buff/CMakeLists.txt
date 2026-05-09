cmake_minimum_required(VERSION 3.8)
project(auto_buff)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)

add_executable(buff_detector_node
  buff_detector/src/buff_detector_node.cpp
  buff_detector/src/yolo.cpp
  buff_detector/src/types.cpp
)
target_include_directories(buff_detector_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buff_detector/include
)
ament_target_dependencies(buff_detector_node
  rclcpp
  ament_index_cpp
  rm_interfaces
  sensor_msgs
  visualization_msgs
  cv_bridge
)
target_link_libraries(buff_detector_node ${OpenCV_LIBS})
target_link_libraries(buff_detector_node openvino::runtime)

add_executable(buff_pose_estimator_node
  src/ros2/buff_pose_estimator_node.cpp
  buff_tracker/src/targets.cpp
  buff_tracker/src/factors.cpp
  buff_tracker/src/types.cpp
  buff_tracker/src/buff_fitter.cpp
)
target_include_directories(buff_pose_estimator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buff_tracker/include
  ${EIGEN3_INCLUDE_DIRS}
)
set_target_properties(buff_pose_estimator_node PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)
ament_target_dependencies(buff_pose_estimator_node
  rclcpp
  rm_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(buff_pose_estimator_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  gtsam
  Ceres::ceres
  absl::check
  absl::log
  absl::log_internal_message
  absl::log_internal_check_op
)

add_executable(buff_tracker_node
  src/ros2/buff_tracker_node.cpp
  buff_tracker/src/targets.cpp
  buff_tracker/src/factors.cpp
  buff_tracker/src/types.cpp
  buff_tracker/src/buff_fitter.cpp
)
target_include_directories(buff_tracker_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buff_tracker/include
  ${EIGEN3_INCLUDE_DIRS}
)
set_target_properties(buff_tracker_node PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)
ament_target_dependencies(buff_tracker_node
  rclcpp
  rm_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(buff_tracker_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  gtsam
  Ceres::ceres
  absl::check
  absl::log
  absl::log_internal_message
  absl::log_internal_check_op
)

install(TARGETS
  buff_detector_node
  buff_pose_estimator_node
  buff_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
