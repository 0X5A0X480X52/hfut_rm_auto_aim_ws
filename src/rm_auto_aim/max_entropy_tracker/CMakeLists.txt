cmake_minimum_required(VERSION 3.8)
project(max_entropy_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  max_entropy_tracker/max_entropy_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 也安装到 libexec 以兼容 launch 可执行文件查找
install(PROGRAMS
  max_entropy_tracker/max_entropy_tracker_node.py
  DESTINATION libexec/${PROJECT_NAME}
)

# Install a small wrapper without .py suffix so launch finds the executable
install(PROGRAMS
  scripts/max_entropy_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/max_entropy_tracker_node
  DESTINATION libexec/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
