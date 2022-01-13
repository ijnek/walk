cmake_minimum_required(VERSION 3.8)
project(walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(walk_interfaces REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include hpp files
include_directories(
  include
)

# Build walk library
add_library(walk_lib
  src/walk_node.cpp
  src/walk.cpp
  src/maths_functions.cpp
  src/twist_limiter.cpp
  src/feet_trajectory.cpp
  src/step.cpp
  src/target_gait_calculator.cpp
  src/phase.cpp
  src/ankle_pose_generator.cpp)

ament_target_dependencies(walk_lib
  rclcpp
  rclcpp_action
  walk_interfaces
  biped_interfaces
  std_srvs
  std_msgs
  tf2
  tf2_geometry_msgs)

# Build walk executable
add_executable(walk
  src/walk_main.cpp)
target_link_libraries(walk
  walk_lib)

# Install
install(TARGETS
  walk
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS walk_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
