cmake_minimum_required(VERSION 3.8)
project(rm_robot_description)

find_package(ament_cmake REQUIRED)

install(
  FILES urdf/rm_gimbal.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
