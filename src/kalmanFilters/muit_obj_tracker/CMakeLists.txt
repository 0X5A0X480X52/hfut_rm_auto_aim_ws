cmake_minimum_required(VERSION 3.8)
project(muit_obj_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(models REQUIRED)
find_package(basic_models REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(muit_obj_tracker
  src/model/kalman_model.cpp
  src/model/composite_model.cpp
  src/utils/hungarian.cpp
  src/tracker/sort_tracker.cpp
  src/tracker/point_tracker.cpp
  src/tracker/tracker_manager.cpp
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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
