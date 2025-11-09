cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(sensor_msgs REQUIRED)

# TODO: Implement gimbal controller node

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
