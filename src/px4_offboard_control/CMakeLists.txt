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
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

include_directories(include)

set(COMMON_DEPS
  rclcpp
  px4_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  Eigen3
)

# Offboard control node
add_executable(offboard_control src/offboard_control.cpp)
add_executable(get_pose src/get_pose.cpp)

foreach(target offboard_control get_pose)
  ament_target_dependencies(${target} ${COMMON_DEPS})
endforeach()

##################
# Installation
##################
install(TARGETS
  offboard_control
  get_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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
