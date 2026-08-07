cmake_minimum_required(VERSION 3.10)
project(armor_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
find_package(armor_pnp_refiner REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/armor_detector.cpp
  src/armor_detector_node.cpp
  src/armor_pose_estimator.cpp
  src/ba_solver.cpp
  src/graph_optimizer.cpp
  src/light_corner_corrector.cpp
  src/number_classifier.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -O3)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  g2o_core
  g2o_stuff
  g2o_solver_csparse
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
  fmt::fmt
  TBB::tbb
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
