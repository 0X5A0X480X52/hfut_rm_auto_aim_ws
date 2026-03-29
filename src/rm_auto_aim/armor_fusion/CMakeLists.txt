cmake_minimum_required(VERSION 3.8)
project(armor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(armor_fusion_component SHARED
  src/multi_camera_fusion_node.cpp
  src/transform_utils.cpp
  src/clustering_utils.cpp
  src/fusion_utils.cpp
  src/visualization_utils.cpp
)

target_include_directories(armor_fusion_component PUBLIC
  include
)

target_compile_features(armor_fusion_component PUBLIC cxx_std_17)

ament_target_dependencies(armor_fusion_component
  rclcpp
  rclcpp_components
  rm_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(armor_fusion_component
  Eigen3::Eigen
)

rclcpp_components_register_node(armor_fusion_component
  PLUGIN "fyt::auto_aim::MultiCameraFusionNode"
  EXECUTABLE armor_fusion_node
)

install(TARGETS armor_fusion_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  armor_fusion/multi_camera_fusion_node.py
  armor_fusion/test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
