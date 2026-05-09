cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/RuneTarget.msg"
  "msg/RuneTargetArray.msg"
  "msg/Point2d.msg"
  "msg/GimbalCmd.msg"
  "msg/ChassisCmd.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DebugRuneAngle.msg"
  "msg/Measurement.msg"
  "msg/JudgeSystemData.msg"
  "msg/OperatorCommand.msg"
  "msg/SerialReceiveData.msg"
  # New messages for decoupled architecture
  "msg/TrackedArmor.msg"
  "msg/TrackedArmors.msg"
  "msg/TrackedRobot.msg"
  "msg/TrackedRobots.msg"
  "msg/SelectedTarget.msg"
  "msg/GimbalState.msg"
  "msg/GimbalTrajectory.msg"
  "msg/Blind.msg"
  "msg/BuffTrackerState.msg"
  # Track history and prediction window messages
  "msg/TrackWindowState.msg"
  "msg/TrackHistoryWindow.msg"
  "msg/TrackHistoryWindows.msg"
  "msg/TrackPredictionWindow.msg"
  "msg/TrackPredictionWindows.msg"
  "msg/ManeuverState.msg"
  "msg/ManeuverStates.msg"
  "msg/DelayAudit.msg"
  "msg/FireAdviceDebug.msg"
  "srv/SetMode.srv"
  "srv/SolveBallistic.srv"
  "srv/SetTargetRobot.srv"
  "action/TrajectoryPlan.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
    action_msgs
)


ament_package()
