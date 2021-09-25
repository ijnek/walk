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
      std::bind(&TestWalk::send_ik_command, this, std::placeholders::_1))
  {
  }

  void SetUp()
  {
    send_ik_commandCalled = false;
  }

public:
  void send_ik_command(nao_ik_interfaces::msg::IKCommand)
  {
    send_ik_commandCalled = true;
  }

  bool send_ik_commandCalled;
  Walk walk;
};

TEST_F(TestWalk, TestNotDuringWalk)
{
  walk.generateCommand();
  ASSERT_FALSE(send_ik_commandCalled);
}

TEST_F(TestWalk, TestCrouch)
{
  walk.crouch();
  walk.generateCommand();
  ASSERT_TRUE(send_ik_commandCalled);
}

TEST_F(TestWalk, TestWalk)
{
  geometry_msgs::msg::Twist target;
  walk.walk(target);
  walk.generateCommand();
  ASSERT_TRUE(send_ik_commandCalled);
}

TEST_F(TestWalk, TestCrouchToAbort)
{
  geometry_msgs::msg::Twist target;
  walk.crouch();
  walk.abort();
  walk.generateCommand();
  ASSERT_FALSE(send_ik_commandCalled);
}

TEST_F(TestWalk, TestWalkToAbort)
{
  geometry_msgs::msg::Twist target;
  walk.walk(target);
  walk.abort();
  walk.generateCommand();
  ASSERT_FALSE(send_ik_commandCalled);
}