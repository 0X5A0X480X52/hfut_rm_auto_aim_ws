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

set(TRACKING_SRCS
  src/max_entropy_tracker/binder/binder_factory.cpp
  src/max_entropy_tracker/binder/binder_pipeline.cpp
  src/max_entropy_tracker/binder/binding_fsm.cpp
  src/max_entropy_tracker/binder/binding_window_counter.cpp
  src/max_entropy_tracker/binder/dual_obs_direct_binder.cpp
  src/max_entropy_tracker/binder/four_panel_jump_decoder.cpp
  src/max_entropy_tracker/binder/generic_cyclic_jump_decoder.cpp
  src/max_entropy_tracker/binder/hybrid_id_binder.cpp
  src/max_entropy_tracker/binder/outpost_binding_confidence.cpp
  src/max_entropy_tracker/binder/outpost_binding_fsm.cpp
  src/max_entropy_tracker/binder/outpost_hypothesis_evaluator.cpp
  src/max_entropy_tracker/binder/outpost_legacy_binding_policy.cpp
  src/max_entropy_tracker/binder/outpost_periodic_dz_evidence.cpp
  src/max_entropy_tracker/binder/outpost_trilevel_jump_decoder.cpp
  src/max_entropy_tracker/binder/outpost_z_audit.cpp
  src/max_entropy_tracker/binder/residual_hypothesis_scorer.cpp
  src/max_entropy_tracker/binder/robot_binding_profile.cpp
  src/max_entropy_tracker/binder/single_obs_sequence_binder.cpp
  src/max_entropy_tracker/binder/soft_fusion_scorer.cpp
  src/max_entropy_tracker/evidence/evidence_builder.cpp
  src/max_entropy_tracker/filters/ambiguous_single_armor_filter_adapter.cpp
  src/max_entropy_tracker/filters/dual_radius_spin_ukf.cpp
  src/max_entropy_tracker/filters/outpost_ambiguous_kf.cpp
  src/max_entropy_tracker/filters/outpost_spin_ukf.cpp
  src/max_entropy_tracker/filters/single_armor_imm_tracker.cpp
  src/max_entropy_tracker/mode/evidence_fuser.cpp
  src/max_entropy_tracker/mode/mode_fsm.cpp
  src/max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.cpp
  src/max_entropy_tracker/trackers/norm4_v3/backends/norm4_inekf_backend.cpp
  src/max_entropy_tracker/trackers/norm4_v3/backends/norm4_single_armor_imm_bundle.cpp
  src/max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v1.cpp
  src/max_entropy_tracker/trackers/norm4_v3/hypothesis/norm4_hypothesis_generator.cpp
  src/max_entropy_tracker/trackers/norm4_v3/models/norm4_motion_model_bundle.cpp
  src/max_entropy_tracker/trackers/norm4_v3/norm4_ba_aware_ypd_noise.cpp
  src/max_entropy_tracker/trackers/norm4_v3/tracker/norm4_tracker_v2.cpp
  src/max_entropy_tracker/trackers/outpost_tracker_v2.cpp
  src/max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.cpp
  src/max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.cpp
  src/max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.cpp
  src/max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.cpp
  src/max_entropy_tracker/trackers/outpost_v2/outpost_structured_backend.cpp
  src/max_entropy_tracker/tracking2d/iou_2d_tracker.cpp
)

set(SELECTION_SRCS
  src/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.cpp
  src/common/robot_description/robot_description_facade.cpp
  src/common/robot_description/tracked_robot_builder_registry.cpp
  src/common/robot_description/tracked_robot_usage.cpp
  src/target_selector/selection_strategy.cpp
  src/target_selector/strategies/priority_list_strategy.cpp
)

set(CONTROL_SRCS
  src/gimbal_controller/armor_position_calculator.cpp
  src/gimbal_controller/armor_selector.cpp
  src/gimbal_controller/delay_management/delay_semantic_manager.cpp
  src/gimbal_controller/fire_advice/probability_engine.cpp
  src/gimbal_controller/fire_advice_engine.cpp
  src/gimbal_controller/fire_advisor.cpp
  src/gimbal_controller/gimbal_control_core.cpp
  src/gimbal_controller/gimbal_control_orchestrator.cpp
  src/gimbal_controller/gimbal_control_strategy.cpp
  src/gimbal_controller/local_trajectory_compensator.cpp
  src/gimbal_controller/mpc/mpc_reference_generator.cpp
  src/gimbal_controller/strategies/mpc_control_strategy.cpp
)

set(ROS_ADAPTER_SRCS
  src/adapters/buff_target_adapter.cpp
  src/gimbal_pipeline_control.cpp
  src/gimbal_pipeline_debug.cpp
  src/gimbal_pipeline_node.cpp
  src/gimbal_pipeline_parameters.cpp
  src/gimbal_pipeline_selection.cpp
  src/gimbal_pipeline_tracking.cpp
  src/prediction_logger.cpp
)

set(INTERNAL_TARGETS
  gimbal_tracking
  gimbal_selection
  gimbal_control
  gimbal_ros_adapter
)

add_library(gimbal_tracking OBJECT ${TRACKING_SRCS})
add_library(gimbal_selection OBJECT ${SELECTION_SRCS})
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
  $<TARGET_OBJECTS:gimbal_selection>
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
  PATTERN "max_entropy_tracker/pipeline/serial_tracker_pipeline.hpp" EXCLUDE
  PATTERN "max_entropy_tracker/pose_tracker_backend/backend_execution_plan.hpp" EXCLUDE
  PATTERN "max_entropy_tracker/pose_tracker_backend/backend_executor.hpp" EXCLUDE
  PATTERN "max_entropy_tracker/pose_tracker_backend/backend_intent.hpp" EXCLUDE
  PATTERN "max_entropy_tracker/pose_tracker_backend/backend_planner.hpp" EXCLUDE
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
