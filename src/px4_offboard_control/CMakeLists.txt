cmake_minimum_required(VERSION 3.8)
project(px4_offboard_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(visualization_msgs REQUIRED)

set(COMMON_DEPS
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  builtin_interfaces
  geometry_msgs
  visualization_msgs
  sensor_msgs
  Eigen3
)

# *** MANUAL INCLUDE PATH FIX ***
include_directories(${CMAKE_INSTALL_PREFIX}/include/px4_msgs)

# Add your executables here
add_executable(offboard_control src/offboard_control.cpp)
add_executable(get_pose src/get_pose.cpp)
add_executable(offboard_control_timer src/offboard_control_timer.cpp)
add_executable(setpoint_visualizer src/setpoint_visualizer.cpp)

# Link dependencies to all targets
foreach(target offboard_control get_pose offboard_control_timer setpoint_visualizer)
  ament_target_dependencies(${target} ${COMMON_DEPS})
endforeach()

##################
# Installation
##################
install(TARGETS
  offboard_control
  get_pose
  offboard_control_timer
  setpoint_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

##################
# Testing (optional)
##################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

##################
# Export dependencies for downstream packages
##################
ament_export_dependencies(${COMMON_DEPS})
ament_package()