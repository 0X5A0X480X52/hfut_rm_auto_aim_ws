cmake_minimum_required(VERSION 3.8)
project(armor_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Armor Tracker Node
add_library(armor_tracker_node SHARED
  src/armor_tracker_node.cpp
  src/single_armor_tracker.cpp
  src/ekf.cpp
)

ament_target_dependencies(armor_tracker_node
  rclcpp
  rclcpp_components
  angles
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  rm_interfaces
  rm_utils
)

target_link_libraries(armor_tracker_node
  Eigen3::Eigen
)

# Register component
rclcpp_components_register_node(armor_tracker_node
  PLUGIN "fyt::auto_aim::ArmorTrackerNode"
  EXECUTABLE armor_tracker_node_exe
)

# Install
install(TARGETS armor_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
