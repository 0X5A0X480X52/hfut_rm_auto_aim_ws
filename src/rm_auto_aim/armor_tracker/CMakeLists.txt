cmake_minimum_required(VERSION 3.8)
project(armor_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(OpenCV REQUIRED)

# muit_obj_tracker 依赖
find_package(muit_obj_tracker REQUIRED)
find_package(models REQUIRED)
find_package(basic_models REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # Include muit_obj_tracker headers from workspace
  ${CMAKE_SOURCE_DIR}/src/kalmanFilters/muit_obj_tracker/include
)

# ==================== 新版 Armor Tracker Node (v2.0) ====================
# 基于 muit_obj_tracker 的多目标跟踪实现
add_library(armor_tracker_node SHARED
  src/armor_tracker_node.cpp
  src/armor_tracker_core.cpp
  src/armor_types.cpp
  src/track_history_manager.cpp
  src/track_prediction_manager.cpp
  src/strategies/detect_source_strategy.cpp
  src/strategies/estimate_source_strategy.cpp
  src/strategies/predict_source_strategy.cpp
  src/strategies/tracking_strategy_manager.cpp
)

ament_target_dependencies(armor_tracker_node
  rclcpp
  rclcpp_components
  angles
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  rm_interfaces
  rm_utils
  muit_obj_tracker
  models
  basic_models
  OpenCV
)

target_link_libraries(armor_tracker_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  muit_obj_tracker::muit_obj_tracker
  ${basic_models_LIBRARIES}
  ${models_LIBRARIES}
)

target_include_directories(armor_tracker_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../kalmanFilters/muit_obj_tracker/include
)

# Register component
rclcpp_components_register_node(armor_tracker_node
  PLUGIN "fyt::auto_aim::ArmorTrackerNode"
  EXECUTABLE armor_tracker_node_exe
)

# (Legacy implementation removed from this CMakeLists: armor_tracker_node_legacy)

# Install
install(TARGETS armor_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GoogleTest
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests for history and prediction managers
  ament_add_gtest(test_track_history_manager test/test_track_history_manager.cpp)
  if(TARGET test_track_history_manager)
    target_link_libraries(test_track_history_manager armor_tracker_node)
    target_include_directories(test_track_history_manager PRIVATE include)
    ament_target_dependencies(test_track_history_manager rm_interfaces Eigen3)
  endif()

  ament_add_gtest(test_track_prediction_manager test/test_track_prediction_manager.cpp)
  if(TARGET test_track_prediction_manager)
    target_link_libraries(test_track_prediction_manager armor_tracker_node)
    target_include_directories(test_track_prediction_manager PRIVATE include)
    ament_target_dependencies(test_track_prediction_manager rm_interfaces Eigen3)
  endif()

  # Integration test for ArmorTrackerNode
  ament_add_gtest(test_armor_tracker_node test/test_armor_tracker_node.cpp)
  if(TARGET test_armor_tracker_node)
    target_link_libraries(test_armor_tracker_node armor_tracker_node)
    ament_target_dependencies(test_armor_tracker_node rclcpp rm_interfaces models basic_models muit_obj_tracker)
  endif()

  ament_add_gtest(test_armor_tracker_core test/test_armor_tracker_core.cpp)
  if(TARGET test_armor_tracker_core)
    target_link_libraries(test_armor_tracker_core armor_tracker_node)
    ament_target_dependencies(test_armor_tracker_core rclcpp rm_interfaces models basic_models muit_obj_tracker)
  endif()
endif()
  find_package(ament_cmake_pytest REQUIRED)

  # Python integration test: spin up C++ node and publish test messages
  ament_add_pytest_test(
    test_armor_tracker_integration
    test/test_armor_tracker_integration.py
  )
  if(TARGET test_armor_tracker_integration)
    ament_target_dependencies(test_armor_tracker_integration rclpy rm_interfaces ament_index_python)
  endif()

ament_package()
