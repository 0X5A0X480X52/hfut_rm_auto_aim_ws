cmake_minimum_required(VERSION 3.8)
project(auto_buff)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Threads REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)

# Header-only/system deps used directly by includes in this package.
find_path(CXXOPTS_INCLUDE_DIR cxxopts.hpp)
find_path(RFL_INCLUDE_DIR rfl.hpp)
find_path(QUILL_INCLUDE_DIR quill/LogMacros.h)
find_path(OPENVINO_INCLUDE_DIR openvino/openvino.hpp)

set(AUTO_BUFF_MISSING_DEPS "")
foreach(_dep
    CXXOPTS_INCLUDE_DIR
    RFL_INCLUDE_DIR
    QUILL_INCLUDE_DIR
    OPENVINO_INCLUDE_DIR)
  if(NOT ${_dep})
    list(APPEND AUTO_BUFF_MISSING_DEPS ${_dep})
  endif()
endforeach()

if(AUTO_BUFF_MISSING_DEPS)
  message(FATAL_ERROR
    "auto_buff missing required header deps: ${AUTO_BUFF_MISSING_DEPS}. "
    "Please install/provide cxxopts, reflect-cpp(rfl), quill, openvino dev headers.")
endif()

set(AUTO_BUFF_COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/buff_detector/include
  ${CMAKE_CURRENT_SOURCE_DIR}/buff_tracker/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CXXOPTS_INCLUDE_DIR}
  ${RFL_INCLUDE_DIR}
  ${QUILL_INCLUDE_DIR}
  ${OPENVINO_INCLUDE_DIR}
)

add_executable(buff_detector_node
  buff_detector/src/buff_detector_node.cpp
  buff_detector/src/center_corrector.cpp
  buff_detector/src/configs.cpp
  buff_detector/src/detector.cpp
  buff_detector/src/main.cpp
  buff_detector/src/types.cpp
  buff_detector/src/yolo.cpp
)
target_include_directories(buff_detector_node PRIVATE ${AUTO_BUFF_COMMON_INCLUDE_DIRS})
target_link_libraries(buff_detector_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
  iceoryx_posh::iceoryx_posh
  iceoryx_hoofs::iceoryx_hoofs
  Threads::Threads
)

add_executable(buff_tracker_node
  buff_tracker/src/buff_fitter.cpp
  buff_tracker/src/factors.cpp
  buff_tracker/src/main.cpp
  buff_tracker/src/targets.cpp
  buff_tracker/src/tracker_node.cpp
  buff_tracker/src/trajectory.cpp
  buff_tracker/src/types.cpp
)
target_include_directories(buff_tracker_node PRIVATE ${AUTO_BUFF_COMMON_INCLUDE_DIRS})
target_link_libraries(buff_tracker_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
  Ceres::ceres
  gtsam
  iceoryx_posh::iceoryx_posh
  iceoryx_hoofs::iceoryx_hoofs
  Threads::Threads
)

install(TARGETS
  buff_detector_node
  buff_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  buff_detector/include/
  buff_tracker/include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
