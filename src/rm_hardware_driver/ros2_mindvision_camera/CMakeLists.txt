cmake_minimum_required(VERSION 3.16)
project(mindvision_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(MVSDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/mvsdk/lib/amd64/libMVSDK.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(MVSDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/mvsdk/lib/arm64/libMVSDK.so)
else()
  message(FATAL_ERROR "Unsupported host architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED src/mv_camera_node.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PRIVATE mvsdk/include)
target_link_libraries(${PROJECT_NAME} ${MVSDK_LIBRARY})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "mindvision_camera::MVCameraNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(FILES ${MVSDK_LIBRARY} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
