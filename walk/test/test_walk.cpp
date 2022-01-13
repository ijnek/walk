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
      std::bind(&TestWalk::send_ankle_poses, this, std::placeholders::_1),
      std::bind(&TestWalk::report_current_twist, this, std::placeholders::_1),
      std::bind(&TestWalk::report_ready_to_step, this, std::placeholders::_1))
  {
    // Walk params are zero by default, so this funcion must be called!
    // For this test case, the values used are taken from the default of a NAO walk.
    walk.setParams(0.3, 0.2, 2.0, 1.0, 0.012, 0.25, 0.01, -0.01, 0.05, -0.18, 0.06, 0.1, 1.0);
  }

  void SetUp()
  {
    send_ankle_posesCalled = false;
    report_current_twistCalled = false;
    report_ready_to_stepCalled = false;
    ready_to_stepVal = false;
  }

public:
  void send_ankle_poses(const biped_interfaces::msg::AnklePoses &)
  {
    send_ankle_posesCalled = true;
  }

  void report_current_twist(const geometry_msgs::msg::Twist &)
  {
    report_current_twistCalled = true;
  }

  void report_ready_to_step(const std_msgs::msg::Bool & ready_to_step)
  {
    report_ready_to_stepCalled = true;
    ready_to_stepVal = ready_to_step.data;
  }

  bool send_ankle_posesCalled;
  bool report_current_twistCalled;
  bool report_ready_to_stepCalled;
  bool ready_to_stepVal;
  Walk walk;
};

// TEST_F(TestWalk, TestNotDuringWalk)
// {
//   walk.generateCommand();
//   EXPECT_FALSE(send_ankle_posesCalled);
//   EXPECT_FALSE(report_current_twistCalled);
//   EXPECT_FALSE(ready_to_stepVal);
// }

// TEST_F(TestWalk, TestCrouch)
// {
//   // walk.crouch();
//   walk.generateCommand();
//   EXPECT_TRUE(send_ankle_posesCalled);
//   EXPECT_TRUE(report_current_twistCalled);
//   EXPECT_TRUE(report_ready_to_stepCalled);
//   EXPECT_TRUE(ready_to_stepVal);
// }

// TEST_F(TestWalk, TestWalk)
// {
//   geometry_msgs::msg::Twist target;
//   walk.walk(target);
//   walk.generateCommand();
//   EXPECT_TRUE(send_ankle_posesCalled);
//   EXPECT_TRUE(report_current_twistCalled);
//   EXPECT_TRUE(report_ready_to_stepCalled);
//   EXPECT_FALSE(ready_to_stepVal);
// }

// TEST_F(TestWalk, TestCrouchToAbort)
// {
//   geometry_msgs::msg::Twist target;
//   // walk.crouch();
//   // walk.abort();
//   walk.generateCommand();
//   EXPECT_FALSE(send_ankle_posesCalled);
//   EXPECT_FALSE(report_current_twistCalled);
//   EXPECT_FALSE(report_ready_to_stepCalled);
//   EXPECT_FALSE(ready_to_stepVal);
// }

// TEST_F(TestWalk, TestWalkToAbort)
// {
//   geometry_msgs::msg::Twist target;
//   walk.walk(target);
//   // walk.abort();
//   walk.generateCommand();
//   EXPECT_FALSE(send_ankle_posesCalled);
//   EXPECT_FALSE(report_current_twistCalled);
//   EXPECT_FALSE(report_ready_to_stepCalled);
//   EXPECT_FALSE(ready_to_stepVal);
// }

// TEST_F(TestWalk, Test1)
// {
//   geometry_msgs::msg::Twist target;
//   target.linear.x = 0.1;
//   walk.walk(target);
//   walk.generateCommand();
// }
