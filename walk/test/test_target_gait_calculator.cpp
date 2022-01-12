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
  float period = 0.50;
  geometry_msgs::msg::Twist target;
  Gait gait;

  target.linear.x = 0.1;
  gait = target_gait_calculator::calculate(target, period);
  EXPECT_NEAR(gait.leftStancePhaseAim.forwardL, -0.025, 0.00001);
  EXPECT_NEAR(gait.leftStancePhaseAim.forwardR, 0.025, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.forwardL, 0.025, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.forwardR, -0.025, 0.00001);
}

TEST(TestTargetGaitCalculator, TestPositiveY)
{
  float period = 0.50;
  geometry_msgs::msg::Twist target;
  Gait gait;

  target.linear.y = 0.1;
  gait = target_gait_calculator::calculate(target, period);
  EXPECT_NEAR(gait.leftStancePhaseAim.leftL, 0, 0.00001);
  EXPECT_NEAR(gait.leftStancePhaseAim.leftR, 0, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.leftL, 0.05, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.leftR, -0.05, 0.00001);
}

TEST(TestTargetGaitCalculator, TestNegativeY)
{
  float period = 0.50;
  geometry_msgs::msg::Twist target;
  Gait gait;

  target.linear.y = -0.1;
  gait = target_gait_calculator::calculate(target, period);
  EXPECT_NEAR(gait.leftStancePhaseAim.leftL, 0.05, 0.00001);
  EXPECT_NEAR(gait.leftStancePhaseAim.leftR, -0.05, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.leftL, 0, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.leftR, 0, 0.00001);
}

TEST(TestTargetGaitCalculator, TestPositiveTheta)
{
  float period = 0.50;
  geometry_msgs::msg::Twist target;
  Gait gait;

  target.angular.z = 0.1;
  gait = target_gait_calculator::calculate(target, period);
  EXPECT_NEAR(gait.leftStancePhaseAim.headingL, -0.01, 0.00001);
  EXPECT_NEAR(gait.leftStancePhaseAim.headingR, 0.01, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.headingL, 0.04, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.headingR, -0.04, 0.00001);
}

TEST(TestTargetGaitCalculator, TestNegativeTheta)
{
  float period = 0.50;
  geometry_msgs::msg::Twist target;
  Gait gait;

  target.angular.z = -0.1;
  gait = target_gait_calculator::calculate(target, period);
  EXPECT_NEAR(gait.leftStancePhaseAim.headingL, 0.04, 0.00001);
  EXPECT_NEAR(gait.leftStancePhaseAim.headingR, -0.04, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.headingL, -0.01, 0.00001);
  EXPECT_NEAR(gait.rightStancePhaseAim.headingR, 0.01, 0.00001);
}
