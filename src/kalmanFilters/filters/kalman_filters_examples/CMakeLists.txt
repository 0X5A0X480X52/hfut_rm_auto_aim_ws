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
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Original examples
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

# Filter test program
add_executable(filter_test
  src/filter_test.cpp
)
ament_target_dependencies(filter_test models)
target_link_libraries(filter_test yaml-cpp)

install(TARGETS
  factory_demo
  complete_example
  kalman_filter_node_example
  filter_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(PROGRAMS
  scripts/generate_test_data.py
  scripts/analyze_results.py
  scripts/run_all_tests.sh
  scripts/demo.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (for reference)
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
