cmake_minimum_required(VERSION 3.8)
project(muit_obj_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(models REQUIRED)
find_package(basic_models REQUIRED)
find_package(combined_models REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build as SHARED library so it can be linked into other shared libraries
add_library(muit_obj_tracker SHARED
  src/model/kalman_model.cpp
  src/model/composite_model.cpp
  src/utils/hungarian.cpp
  src/tracker/sort_tracker.cpp
  src/tracker/point_tracker.cpp
  src/tracker/tracker_manager.cpp
)

# Ensure the include directory is associated with the target for consumers
target_include_directories(muit_obj_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(muit_obj_tracker
  rclcpp
  models
  basic_models
  OpenCV
)

# Link Eigen via its CMake target so include directories and compile options are available
target_link_libraries(muit_obj_tracker Eigen3::Eigen)

install(TARGETS muit_obj_tracker
  EXPORT muit_obj_trackerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export CMake target and include dirs for other packages in the workspace
install(EXPORT muit_obj_trackerTargets
  FILE muit_obj_trackerTargets.cmake
  NAMESPACE muit_obj_tracker::
  DESTINATION lib/cmake/muit_obj_tracker
)
ament_export_targets(muit_obj_trackerTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ==================== 测试可执行文件 ====================
find_package(yaml-cpp REQUIRED)

# PointTracker 测试程序
add_executable(point_tracker_test
  test/point_tracker_test.cpp
)

target_include_directories(point_tracker_test PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 使用 WHOLE_ARCHIVE 确保 basic_models 中的静态初始化器被链接
# 这是确保工厂自动注册的关键
target_link_libraries(point_tracker_test
  -Wl,--whole-archive
  muit_obj_tracker
  -Wl,--no-whole-archive
  yaml-cpp
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

ament_target_dependencies(point_tracker_test
  rclcpp
  models
  basic_models
  OpenCV
)

install(TARGETS point_tracker_test
  DESTINATION lib/${PROJECT_NAME}
)

# PointTracker 3D 测试程序
add_executable(point_tracker_test_3d
  test/point_tracker_test_3d.cpp
)

target_include_directories(point_tracker_test_3d PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(point_tracker_test_3d
  -Wl,--whole-archive
  muit_obj_tracker
  -Wl,--no-whole-archive
  yaml-cpp
  Eigen3::Eigen
)

ament_target_dependencies(point_tracker_test_3d
  rclcpp
  models
  basic_models
  combined_models
)

install(TARGETS point_tracker_test_3d
  DESTINATION lib/${PROJECT_NAME}
)

# 安装测试配置文件
install(DIRECTORY
  test/config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
