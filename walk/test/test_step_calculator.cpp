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

#include <gtest/gtest.h>
#include "walk/step_calculator.hpp"

TEST(TestStepCalculator, TestForward)
{
  StepCalculator sc;
  sc.setParams(0.3, 0.3, 1.0, 1.0, 0.10, 0.1, 0.1, 1.0);

  geometry_msgs::msg::Twist initial;
  geometry_msgs::msg::Twist target;
  target.linear.x = 0.3;
  auto curr1 = sc.calculateNext(initial, target);
  EXPECT_NEAR(curr1.linear.x, 0.1, 0.001);
  
  auto curr2 = sc.calculateNext(curr1, target);
  EXPECT_NEAR(curr2.linear.x, 0.2, 0.001);

  auto curr3 = sc.calculateNext(curr2, target);
  EXPECT_NEAR(curr3.linear.x, 0.3, 0.001);
}

TEST(TestStepCalculator, TestLeft)
{
  StepCalculator sc;
  sc.setParams(0.3, 0.3, 1.0, 1.0, 0.10, 0.1, 0.1, 1.0);

  geometry_msgs::msg::Twist initial;
  geometry_msgs::msg::Twist target;
  target.linear.y = 0.3;
  auto curr1 = sc.calculateNext(initial, target);
  EXPECT_NEAR(curr1.linear.y, 0.1, 0.001);
  
  auto curr2 = sc.calculateNext(curr1, target);
  EXPECT_NEAR(curr2.linear.y, 0.2, 0.001);

  auto curr3 = sc.calculateNext(curr2, target);
  EXPECT_NEAR(curr3.linear.y, 0.3, 0.001);
}