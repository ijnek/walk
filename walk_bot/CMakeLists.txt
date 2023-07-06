cmake_minimum_required(VERSION 3.8)
project(walk_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  biped_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs)

# Build
add_library(ik_node SHARED
  src/ik.cpp)

target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(ik_node
  PLUGIN "walk_bot::IK"
  EXECUTABLE ik)

ament_target_dependencies(ik_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_ik HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ik_node
  EXPORT export_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY gui_config launch rviz urdf world
  DESTINATION share/${PROJECT_NAME}/)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
