cmake_minimum_required(VERSION 3.8)
project(armor_pnp_refiner)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../armor_detector/cmake_modules/FindG2O.cmake")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../armor_detector/cmake_modules")
endif()
find_package(G2O REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/core/armor_pnp_refiner.cpp
  src/geometry/armor_geometry.cpp
  src/geometry/camera_model.cpp
  src/graph/single_xyz_yaw/single_xyz_yaw_optimizer.cpp
  src/quality/quality_gate.cpp
  src/quality/confidence_estimator.cpp
  src/quality/fallback_manager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  g2o_core
  g2o_stuff
  g2o_solver_dense
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_pnp_refiner test/test_armor_pnp_refiner.cpp)
  target_link_libraries(test_armor_pnp_refiner ${PROJECT_NAME})
endif()

ament_auto_package()
