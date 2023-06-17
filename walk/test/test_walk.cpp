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
#include "walk/walk.hpp"

class TestWalk : public ::testing::Test
{
protected:
  TestWalk()
  : walk(
      rclcpp::NodeOptions().parameter_overrides(
  {  // For this test case, the values used are taken from the default of a NAO walk.
    {"max_forward", 0.3},
    {"max_left", 0.2},
    {"max_turn", 2.0},
    {"speed_multiplier", 1.0},
    {"foot_lift_amp", 0.012},
    {"period", 0.25},
    {"dt", 0.01},
    {"sole_x", -0.01},
    {"sole_y", 0.05},
    {"sole_z", -0.18},
    {"max_forward_change", 0.06},
    {"max_left_change", 0.1},
    {"max_turn_change", 1.0},
  }))
  {
  }

  void SetUp()
  {
    send_sole_poses_called = false;
    report_current_twist_called = false;
    report_ready_to_step_called = false;
    ready_to_step_val = false;
  }

public:
  void send_sole_poses(const biped_interfaces::msg::SolePoses &)
  {
    send_sole_poses_called = true;
  }

  void report_current_twist(const geometry_msgs::msg::Twist &)
  {
    report_current_twist_called = true;
  }

  void report_ready_to_step(const std_msgs::msg::Bool & ready_to_step)
  {
    report_ready_to_step_called = true;
    ready_to_step_val = ready_to_step.data;
  }

  bool send_sole_poses_called;
  bool report_current_twist_called;
  bool report_ready_to_step_called;
  bool ready_to_step_val;
  walk::Walk walk;
};

// TEST_F(TestWalk, TestNotDuringWalk)
// {
//   walk.generateCommand();
//   EXPECT_FALSE(send_sole_poses_called);
//   EXPECT_FALSE(report_current_twist_called);
//   EXPECT_FALSE(ready_to_step_val);
// }

// TEST_F(TestWalk, TestCrouch)
// {
//   // walk.crouch();
//   walk.generateCommand();
//   EXPECT_TRUE(send_sole_poses_called);
//   EXPECT_TRUE(report_current_twist_called);
//   EXPECT_TRUE(report_ready_to_step_called);
//   EXPECT_TRUE(ready_to_step_val);
// }

// TEST_F(TestWalk, TestWalk)
// {
//   geometry_msgs::msg::Twist target;
//   walk.walk(target);
//   walk.generateCommand();
//   EXPECT_TRUE(send_sole_poses_called);
//   EXPECT_TRUE(report_current_twist_called);
//   EXPECT_TRUE(report_ready_to_step_called);
//   EXPECT_FALSE(ready_to_step_val);
// }

// TEST_F(TestWalk, TestCrouchToAbort)
// {
//   geometry_msgs::msg::Twist target;
//   // walk.crouch();
//   // walk.abort();
//   walk.generateCommand();
//   EXPECT_FALSE(send_sole_poses_called);
//   EXPECT_FALSE(report_current_twist_called);
//   EXPECT_FALSE(report_ready_to_step_called);
//   EXPECT_FALSE(ready_to_step_val);
// }

// TEST_F(TestWalk, TestWalkToAbort)
// {
//   geometry_msgs::msg::Twist target;
//   walk.walk(target);
//   // walk.abort();
//   walk.generateCommand();
//   EXPECT_FALSE(send_sole_poses_called);
//   EXPECT_FALSE(report_current_twist_called);
//   EXPECT_FALSE(report_ready_to_step_called);
//   EXPECT_FALSE(ready_to_step_val);
// }

// TEST_F(TestWalk, Test1)
// {
//   geometry_msgs::msg::Twist target;
//   target.linear.x = 0.1;
//   walk.walk(target);
//   walk.generateCommand();
// }
