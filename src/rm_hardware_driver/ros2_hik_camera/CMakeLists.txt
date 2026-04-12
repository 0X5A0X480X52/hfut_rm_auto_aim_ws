cmake_minimum_required(VERSION 3.10)
project(ros2_hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

# Build library
add_library(${PROJECT_NAME} SHARED
  src/ros2_hik_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link HIKVision SDK
# Use local SDK headers from include/ros2_hik_camera/inc
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_hik_camera/inc
)

# HIKVision SDK library path
# 强制使用系统 /opt/MVS，不使用缓存，彻底解决路径跑偏问题
set(HIK_SDK_DIR /opt/MVS)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIK_LIB_DIR ${HIK_SDK_DIR}/lib/64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIK_LIB_DIR ${HIK_SDK_DIR}/lib/aarch64)
else()
  message(FATAL_ERROR "Unsupported host system architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

# 让链接器正确搜索库路径
link_directories(${HIK_LIB_DIR})

# ========== 核心修复：绝对路径导入库，让 ld 百分百找到 ==========
add_library(MvCameraControl SHARED IMPORTED GLOBAL)
set_target_properties(MvCameraControl PROPERTIES
  IMPORTED_LOCATION "${HIK_LIB_DIR}/libMvCameraControl.so"
)

target_link_libraries(${PROJECT_NAME}
  MvCameraControl
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
)

# Register component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ros2_hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
