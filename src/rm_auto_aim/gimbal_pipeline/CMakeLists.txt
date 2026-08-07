cmake_minimum_required(VERSION 3.8)
project(gimbal_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PIPELINE_CORE_SRCS
  src/core/auto_aim_pipeline.cpp
  src/core/external_target_merger.cpp
  src/core/pipeline_inbox.cpp
  src/core/priority_target_selector.cpp
)

add_library(pipeline_core STATIC ${PIPELINE_CORE_SRCS})
set_target_properties(pipeline_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(pipeline_core PUBLIC Eigen3::Eigen)

set(TRACKING_SRCS
  src/max_entropy_tracker/binder/outpost_binding_confidence.cpp
  src/max_entropy_tracker/binder/outpost_binding_fsm.cpp
  src/max_entropy_tracker/binder/outpost_hypothesis_evaluator.cpp
  src/max_entropy_tracker/binder/outpost_baseline_binding_policy.cpp
  src/max_entropy_tracker/binder/outpost_periodic_dz_evidence.cpp
  src/max_entropy_tracker/binder/outpost_z_audit.cpp
  src/max_entropy_tracker/evidence/evidence_builder.cpp
  src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp
  src/max_entropy_tracker/filters/dual_radius_spin_ukf.cpp
  src/max_entropy_tracker/filters/outpost_ambiguous_kf.cpp
  src/max_entropy_tracker/filters/outpost_spin_ukf.cpp
  src/max_entropy_tracker/filters/single_armor_imm_tracker.cpp
  src/max_entropy_tracker/mode/evidence_fuser.cpp
  src/max_entropy_tracker/mode/mode_fsm.cpp
  src/max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/backends/norm4_inekf_backend.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/backends/norm4_single_armor_imm_bundle.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/backends/norm4_ukf_backend_baseline.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/hypothesis/norm4_hypothesis_generator.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/models/norm4_motion_model_bundle.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/norm4_ba_aware_ypd_noise.cpp
  src/max_entropy_tracker/trackers/norm4_baseline/tracker/norm4_tracker_baseline.cpp
  src/max_entropy_tracker/trackers/outpost_tracker_baseline.cpp
  src/max_entropy_tracker/trackers/outpost_baseline/outpost_ambiguous_backend.cpp
  src/max_entropy_tracker/trackers/outpost_baseline/outpost_binder_bridge.cpp
  src/max_entropy_tracker/trackers/outpost_baseline/outpost_observation_frontend.cpp
  src/max_entropy_tracker/trackers/outpost_baseline/outpost_output_adapter.cpp
  src/max_entropy_tracker/trackers/outpost_baseline/outpost_structured_backend.cpp
  src/max_entropy_tracker/tracking2d/iou_2d_tracker.cpp
)

set(DESCRIPTION_SRCS
  src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp
  src/common/robot_description/robot_description_facade.cpp
  src/common/robot_description/tracked_robot_builder_registry.cpp
  src/common/robot_description/tracked_robot_usage.cpp
)

list(APPEND TRACKING_SRCS ${DESCRIPTION_SRCS})

set(CONTROL_SRCS
  src/gimbal_controller/armor_position_calculator.cpp
  src/gimbal_controller/armor_selector.cpp
  src/gimbal_controller/delay_management/delay_semantic_manager.cpp
  src/gimbal_controller/fire_advice/probability_engine.cpp
  src/gimbal_controller/fire_advice_engine.cpp
  src/gimbal_controller/fire_advisor.cpp
  src/gimbal_controller/gimbal_control_core.cpp
  src/gimbal_controller/gimbal_control_orchestrator.cpp
  src/gimbal_controller/local_trajectory_compensator.cpp
  src/gimbal_controller/mpc/mpc_reference_generator.cpp
  src/gimbal_controller/strategies/mpc_control_strategy.cpp
)

set(ROS_ADAPTER_SRCS
  src/adapters/buff_target_adapter.cpp
  src/adapters/pipeline_config_loader.cpp
  src/adapters/pipeline_ros_conversions.cpp
  src/gimbal_pipeline_control.cpp
  src/gimbal_pipeline_debug.cpp
  src/gimbal_pipeline_node.cpp
  src/gimbal_pipeline_parameters.cpp
  src/gimbal_pipeline_tracking.cpp
  src/prediction_logger.cpp
)

set(INTERNAL_TARGETS
  gimbal_tracking
  gimbal_control
  gimbal_ros_adapter
)

add_library(gimbal_tracking OBJECT ${TRACKING_SRCS})
add_library(gimbal_control OBJECT ${CONTROL_SRCS})
add_library(gimbal_ros_adapter OBJECT ${ROS_ADAPTER_SRCS})

foreach(target IN LISTS INTERNAL_TARGETS)
  set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(${target} PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${qpOASES_INCLUDE_DIR}
  )
  ament_target_dependencies(${target}
    rclcpp
    rclcpp_components
    rm_interfaces
    rm_utils
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
    angles
    cv_bridge
  )
endforeach()

add_library(${PROJECT_NAME} SHARED
  $<TARGET_OBJECTS:gimbal_tracking>
  $<TARGET_OBJECTS:gimbal_control>
  $<TARGET_OBJECTS:gimbal_ros_adapter>
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  angles
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  pipeline_core
  Eigen3::Eigen
  ${qpOASES_LIBRARY}
  ${OpenCV_LIBS}
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fyt::auto_aim::GimbalPipelineNode"
  EXECUTABLE gimbal_pipeline_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  function(add_gimbal_pipeline_gtest target_name test_src)
    ament_add_gtest(${target_name} ${test_src})
    set_tests_properties(${target_name} PROPERTIES
      ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs"
    )
    target_include_directories(${target_name} PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${qpOASES_INCLUDE_DIR}
    )
    ament_target_dependencies(${target_name}
      rclcpp
      rm_interfaces
      rm_utils
      geometry_msgs
      std_msgs
      sensor_msgs
      visualization_msgs
      tf2
      tf2_ros
      tf2_geometry_msgs
      message_filters
      angles
      rclcpp_components
      cv_bridge
    )
    target_link_libraries(${target_name}
      ${PROJECT_NAME}
      Eigen3::Eigen
      ${qpOASES_LIBRARY}
      ${OpenCV_LIBS}
    )
  endfunction()

  add_gimbal_pipeline_gtest(test_mpc_core test/test_mpc_core.cpp)
  add_gimbal_pipeline_gtest(test_active_pipeline test/test_active_pipeline.cpp)

  ament_add_gtest(test_pipeline_core test/test_pipeline_core.cpp)
  target_include_directories(test_pipeline_core PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(test_pipeline_core pipeline_core Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rm_interfaces
  rm_utils
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  angles
  cv_bridge
)

ament_package()
