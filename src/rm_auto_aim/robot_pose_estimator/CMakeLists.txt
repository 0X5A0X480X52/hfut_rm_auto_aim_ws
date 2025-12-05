cmake_minimum_required(VERSION 3.8)
project(robot_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(Ceres REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# Source files
set(ROBOT_POSE_ESTIMATOR_SOURCES
  src/robot_tracker.cpp
  src/armor_grouper.cpp
  src/virtual_armor_generator.cpp
  src/robot_pose_estimator_core.cpp
  src/robot_pose_estimator_node.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${ROBOT_POSE_ESTIMATOR_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  rm_interfaces
  rm_utils
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${CERES_LIBRARIES}
)

# Register node as component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::RobotPoseEstimatorNode"
  EXECUTABLE robot_pose_estimator_node
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # C++ Unit Tests
  # Test robot_tracker
  ament_add_gtest(test_robot_tracker test/test_robot_tracker.cpp)
  target_link_libraries(test_robot_tracker ${PROJECT_NAME})
  target_include_directories(test_robot_tracker PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_robot_tracker
    rclcpp
    rm_interfaces
    angles
  )

  # Test armor_grouper
  ament_add_gtest(test_armor_grouper test/test_armor_grouper.cpp)
  target_link_libraries(test_armor_grouper ${PROJECT_NAME})
  target_include_directories(test_armor_grouper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_armor_grouper
    rclcpp
    rm_interfaces
  )

  # Test virtual_armor_generator
  ament_add_gtest(test_virtual_armor_generator test/test_virtual_armor_generator.cpp)
  target_link_libraries(test_virtual_armor_generator ${PROJECT_NAME})
  target_include_directories(test_virtual_armor_generator PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_virtual_armor_generator
    rclcpp
    rm_interfaces
    geometry_msgs
  )

  # Python Integration Tests
  ament_add_pytest_test(test_robot_pose_estimator_integration
    test/test_robot_pose_estimator_integration.py
    TIMEOUT 120
  )
  
  ament_add_pytest_test(test_pipeline_integration
    test/test_pipeline_integration.py
    TIMEOUT 180
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  rm_interfaces
  rm_utils
)

ament_package()
