cmake_minimum_required(VERSION 3.8)
project(models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

###########
## Build ##
###########

###########
# Build library (explicit sources)
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/model_factory.cpp
  src/model_config_loader.cpp
  src/models.cpp
)

# Link yaml-cpp to the library
target_link_libraries(${PROJECT_NAME} yaml-cpp)

###########
# Examples (not built by default to avoid adding extra package dependencies)
###########
# The example sources have been moved to 'examples/' and are left out of the
# default build to prevent circular or missing dependency issues (e.g. rclcpp,
# basic_models). If you want to build them, either move them into an
# appropriately dependent package or add explicit find_package() and link
# directives here.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
