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

# Build ik
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

# Build phase provider
add_library(phase_provider_node SHARED
  src/phase_provider.cpp)

target_include_directories(phase_provider_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(phase_provider_node
  PLUGIN "walk_bot::PhaseProvider"
  EXECUTABLE phase_provider)

ament_target_dependencies(phase_provider_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_phase_provider HAS_LIBRARY_TARGET)

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
install(
    TARGETS phase_provider_node
    EXPORT export_phase_provider
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
install(DIRECTORY gui_config launch rviz urdf world
  DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
