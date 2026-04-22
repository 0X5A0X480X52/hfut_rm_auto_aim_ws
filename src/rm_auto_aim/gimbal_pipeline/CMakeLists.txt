cmake_minimum_required(VERSION 3.8)
project(gimbal_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─── Dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)

# ─── Include directories ───────────────────────────────────────
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
)

# ─── Collect all source files (include src root + subdirectories) ─
# Some CMake versions/policies may miss src/*.cpp when only using src/**/*.cpp.
file(GLOB_RECURSE ALL_SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/**/*.cpp"
)

# ─── Shared library (component) ───────────────────────────────
add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  angles
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${qpOASES_LIBRARY}
)

# ─── Register composable node ─────────────────────────────────
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::GimbalPipelineNode"
  EXECUTABLE gimbal_pipeline_node
)

# ─── Install ───────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ─── Testing ───────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mpc_core test/test_mpc_core.cpp
    ${ALL_SRCS}
  )
  target_include_directories(test_mpc_core PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${qpOASES_INCLUDE_DIR}
  )
  ament_target_dependencies(test_mpc_core
    rclcpp
    rm_interfaces
    rm_utils
    geometry_msgs
    std_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
    angles
    rclcpp_components
  )
  target_link_libraries(test_mpc_core
    Eigen3::Eigen
    ${qpOASES_LIBRARY}
  )

  ament_add_gtest(test_outpost_binding test/test_outpost_binding.cpp
    ${ALL_SRCS}
  )
  target_include_directories(test_outpost_binding PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${qpOASES_INCLUDE_DIR}
  )
  ament_target_dependencies(test_outpost_binding
    rclcpp
    rm_interfaces
    rm_utils
    geometry_msgs
    std_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
    angles
    rclcpp_components
  )
  target_link_libraries(test_outpost_binding
    Eigen3::Eigen
    ${qpOASES_LIBRARY}
  )
endif()

# ─── Export ─────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  angles
)

ament_package()
