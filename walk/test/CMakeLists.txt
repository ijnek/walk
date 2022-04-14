# Build test_walk
ament_add_gtest(test_walk
test_walk.cpp)

target_link_libraries(test_walk
  walk_lib
)

# Build test_twist_change_limiter
ament_add_gtest(test_twist_change_limiter
test_twist_change_limiter.cpp)

target_link_libraries(test_twist_change_limiter
  walk_lib
)

# Build test_feet_trajectory
ament_add_gtest(test_feet_trajectory
test_feet_trajectory.cpp)

target_link_libraries(test_feet_trajectory
  walk_lib
)

# Build test_target_gait_calculator
ament_add_gtest(test_target_gait_calculator
test_target_gait_calculator.cpp)

target_link_libraries(test_target_gait_calculator
  walk_lib
)
