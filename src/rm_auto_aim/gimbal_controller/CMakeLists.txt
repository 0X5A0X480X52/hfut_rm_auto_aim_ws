cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/armor_position_calculator.cpp
  src/armor_selector.cpp
  src/ballistic_solver_client.cpp
  src/local_trajectory_compensator.cpp
  src/fire_advisor.cpp
  src/gimbal_control_strategy.cpp
  src/current_position_strategy.cpp
  src/predicted_position_strategy.cpp
  src/mpc_control_strategy.cpp
  src/gimbal_controller_node.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# Register node as component
rclcpp_components_register_nodes(${PROJECT_NAME}
  "gimbal_controller::GimbalControllerNode"
)

# Create standalone executable node
add_executable(gimbal_controller_node src/gimbal_controller_main.cpp)

ament_target_dependencies(gimbal_controller_node
  rclcpp
)

target_link_libraries(gimbal_controller_node
  ${PROJECT_NAME}
)

# Install targets
install(TARGETS ${PROJECT_NAME} gimbal_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

ament_package()
