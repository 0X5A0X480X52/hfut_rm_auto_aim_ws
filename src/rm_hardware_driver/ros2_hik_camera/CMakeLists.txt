cmake_minimum_required(VERSION 3.16)
project(ros2_hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIK_LIBRARY /opt/MVS/lib/64/libMvCameraControl.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIK_LIBRARY /opt/MVS/lib/aarch64/libMvCameraControl.so)
else()
  message(FATAL_ERROR "Unsupported host architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(NOT EXISTS "${HIK_LIBRARY}")
  message(FATAL_ERROR "HIK MVS runtime not found: ${HIK_LIBRARY}")
endif()

add_library(${PROJECT_NAME} SHARED src/ros2_hik_camera_node.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  include/ros2_hik_camera/inc
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${HIK_LIBRARY} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}
  camera_info_manager
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ros2_hik_camera::HikCameraNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
