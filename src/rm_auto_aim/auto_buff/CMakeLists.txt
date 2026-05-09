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
target_link_libraries(buff_pose_estimator_node ${OpenCV_LIBS})

install(TARGETS
  buff_detector_node
  buff_pose_estimator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
