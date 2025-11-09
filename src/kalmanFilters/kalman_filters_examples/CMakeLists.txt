cmake_minimum_required(VERSION 3.8)
project(kalman_filters_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(models REQUIRED)
find_package(basic_models REQUIRED)
find_package(combined_models REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(factory_demo
  src/factory_demo.cpp
)
ament_target_dependencies(factory_demo models basic_models combined_models)

add_executable(complete_example
  src/complete_example.cpp
)
ament_target_dependencies(complete_example models basic_models combined_models)

add_executable(kalman_filter_node_example
  src/kalman_filter_node_example.cpp
)
ament_target_dependencies(kalman_filter_node_example rclcpp models basic_models combined_models)

install(TARGETS
  factory_demo
  complete_example
  kalman_filter_node_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
