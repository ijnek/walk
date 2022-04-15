// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "../src/target_gait_calculator.hpp"

TEST(TestTargetGaitCalculator, TestX)
{
  target_gait_calculator::Params params{0.50};
  geometry_msgs::msg::Twist target;
  walk_interfaces::msg::Gait gait;

  target.linear.x = 0.1;
  gait = target_gait_calculator::calculate(target, params);
  EXPECT_NEAR(gait.left_stance_phase_aim.forward_l, -0.025, 0.00001);
  EXPECT_NEAR(gait.left_stance_phase_aim.forward_r, 0.025, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.forward_l, 0.025, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.forward_r, -0.025, 0.00001);
}

TEST(TestTargetGaitCalculator, TestPositiveY)
{
  target_gait_calculator::Params params{0.50};
  geometry_msgs::msg::Twist target;
  walk_interfaces::msg::Gait gait;

  target.linear.y = 0.1;
  gait = target_gait_calculator::calculate(target, params);
  EXPECT_NEAR(gait.left_stance_phase_aim.left_l, 0, 0.00001);
  EXPECT_NEAR(gait.left_stance_phase_aim.left_r, 0, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.left_l, 0.05, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.left_r, -0.05, 0.00001);
}

TEST(TestTargetGaitCalculator, TestNegativeY)
{
  target_gait_calculator::Params params{0.50};
  geometry_msgs::msg::Twist target;
  walk_interfaces::msg::Gait gait;

  target.linear.y = -0.1;
  gait = target_gait_calculator::calculate(target, params);
  EXPECT_NEAR(gait.left_stance_phase_aim.left_l, 0.05, 0.00001);
  EXPECT_NEAR(gait.left_stance_phase_aim.left_r, -0.05, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.left_l, 0, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.left_r, 0, 0.00001);
}

TEST(TestTargetGaitCalculator, TestPositiveTheta)
{
  target_gait_calculator::Params params{0.50};
  geometry_msgs::msg::Twist target;
  walk_interfaces::msg::Gait gait;

  target.angular.z = 0.1;
  gait = target_gait_calculator::calculate(target, params);
  EXPECT_NEAR(gait.left_stance_phase_aim.heading_l, -0.01, 0.00001);
  EXPECT_NEAR(gait.left_stance_phase_aim.heading_r, 0.01, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.heading_l, 0.04, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.heading_r, -0.04, 0.00001);
}

TEST(TestTargetGaitCalculator, TestNegativeTheta)
{
  target_gait_calculator::Params params{0.50};
  geometry_msgs::msg::Twist target;
  walk_interfaces::msg::Gait gait;

  target.angular.z = -0.1;
  gait = target_gait_calculator::calculate(target, params);
  EXPECT_NEAR(gait.left_stance_phase_aim.heading_l, 0.04, 0.00001);
  EXPECT_NEAR(gait.left_stance_phase_aim.heading_r, -0.04, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.heading_l, -0.01, 0.00001);
  EXPECT_NEAR(gait.right_stance_phase_aim.heading_r, 0.01, 0.00001);
}
