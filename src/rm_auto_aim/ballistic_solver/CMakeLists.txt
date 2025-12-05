cmake_minimum_required(VERSION 3.8)
project(ballistic_solver)

# C++ 17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# 查找依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

# ============================================================================
# 设置依赖列表
# ============================================================================
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  rm_interfaces
)

# ============================================================================
# Include 目录
# ============================================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================================
# 弹道解算器库
# ============================================================================
add_library(ballistic_solver_lib SHARED
  # 补偿器模块
  src/compensator/trajectory_compensator.cpp
  src/compensator/ideal_compensator.cpp
  src/compensator/resistance_compensator.cpp
  src/compensator/manual_compensator.cpp
  src/compensator/compensator_factory.cpp
  # 核心解算器
  src/ballistic_solver.cpp
)

target_include_directories(ballistic_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(ballistic_solver_lib
  ${DEPENDENCIES}
)

# ============================================================================
# 弹道解算服务节点 (作为 component)
# ============================================================================
add_library(ballistic_solver_node SHARED
  src/ballistic_solver_node.cpp
)

target_include_directories(ballistic_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ballistic_solver_node
  ballistic_solver_lib
)

ament_target_dependencies(ballistic_solver_node
  ${DEPENDENCIES}
)

# 注册节点组件
rclcpp_components_register_nodes(ballistic_solver_node
  "ballistic_solver::BallisticSolverNode"
)

# ============================================================================
# 独立可执行文件
# ============================================================================
add_executable(ballistic_solver_node_exe
  src/main.cpp
)

target_include_directories(ballistic_solver_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ballistic_solver_node_exe
  ballistic_solver_lib
  ballistic_solver_node
)

ament_target_dependencies(ballistic_solver_node_exe
  ${DEPENDENCIES}
)

# ============================================================================
# 安装
# ============================================================================
# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库
install(TARGETS
  ballistic_solver_lib
  ballistic_solver_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS
  ballistic_solver_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# 导出
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(ballistic_solver_lib ballistic_solver_node)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

# ============================================================================
# 测试
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  # Add the pytest-based launch test
  add_launch_test(test/launch/test_ballistic_solver_launch.py)
  find_package(ament_cmake_pytest REQUIRED)
  # Add a pytest that spawns node executable and calls service using rclpy
  # Note: ament_add_pytest_test(NAME SCRIPT)
  ament_add_pytest_test(test_ballistic_solver_service test/test_ballistic_solver_service.py)
endif()

ament_package()
