cmake_minimum_required(VERSION 3.10)
project(max_entropy_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Collect all .cpp files from src/ tree
file(GLOB_RECURSE ALL_SRCS
  "src/*.cpp"
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::MaxEntropyTrackerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Install launch and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
