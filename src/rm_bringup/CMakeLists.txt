cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

install(
  FILES launch/bringup_pipeline.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config/hero
    config/infantry_4_1
    config/leg
    config/uav
    config/video_test
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(profile_configs test/test_profile_configs.py)
endif()

ament_package()
