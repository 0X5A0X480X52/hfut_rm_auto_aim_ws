cmake_minimum_required(VERSION 3.10)
project(blind_detector)

####################
## Compile config ##
####################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Werror -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 让 CMake 明确知道 /usr/local 是第三方库前缀
list(APPEND CMAKE_PREFIX_PATH /usr/local)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)
set(G2O_INCLUDE_DIRS /usr/local/include)
set(G2O_LIBRARY_DIR /usr/local/lib)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# 关键：显式告诉 linker 去 /usr/local/lib
target_link_directories(${PROJECT_NAME} PRIVATE /usr/local/lib)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  g2o_core
  g2o_stuff
  g2o_solver_csparse
  g2o_solver_dense
  g2o_types_sba
  g2o_types_slam3d
  fmt::fmt
  TBB::tbb
)

############################
## ROS2 component register ##
############################

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorDetectorNode
  EXECUTABLE blind_detector_node
)

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
