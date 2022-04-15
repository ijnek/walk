# Build test_walk
ament_add_gtest(test_walk
test_walk.cpp)

target_link_libraries(test_walk
  ${PROJECT_NAME}_node
)

# Build test_twist_change_limiter
ament_add_gtest(test_twist_change_limiter
test_twist_change_limiter.cpp)

target_link_libraries(test_twist_change_limiter
  ${PROJECT_NAME}_node
)

# Build test_feet_trajectory
ament_add_gtest(test_feet_trajectory
test_feet_trajectory.cpp)

target_link_libraries(test_feet_trajectory
  ${PROJECT_NAME}_node
)

# Build test_target_gait_calculator
ament_add_gtest(test_target_gait_calculator
test_target_gait_calculator.cpp)

target_link_libraries(test_target_gait_calculator
  ${PROJECT_NAME}_node
)
