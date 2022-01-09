# Build test_walk
ament_add_gtest(test_walk
test_walk.cpp)

target_link_libraries(test_walk
  walk_lib
)

# Build test_step_calculator
ament_add_gtest(test_step_calculator
test_step_calculator.cpp)

target_link_libraries(test_step_calculator
  walk_lib
)

# Build test_feet_trajectory
ament_add_gtest(test_feet_trajectory
test_feet_trajectory.cpp)

target_link_libraries(test_feet_trajectory
  walk_lib
)