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

#ifndef WALK__WALK_NODE_HPP_
#define WALK__WALK_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "walk_interfaces/action/walk.hpp"
#include "nao_ik_interfaces/msg/ik_command.hpp"
#include "walk/walk.hpp"

class WalkNode : public rclcpp::Node
{
public:
  using WalkGoal = walk_interfaces::action::Walk::Goal;
  using WalkGoalHandle = rclcpp_action::ServerGoalHandle<walk_interfaces::action::Walk>;

  WalkNode();

private:
  Walk walk;
  
  // TODO (ijnek): Replace this timer with an input signal
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nao_ik_interfaces::msg::IKCommand>::SharedPtr pub_ik_command;
  rclcpp_action::Server<walk_interfaces::action::Walk>::SharedPtr action_server_;

  std::shared_ptr<WalkGoalHandle> walk_goal_handle_;

  void timerCallback();
  void sendIKCommand(nao_ik_interfaces::msg::IKCommand ik_command);
  void handleAccepted(
    const std::shared_ptr<WalkGoalHandle> goal_handle);
  void notifyGoalAchieved();
};

#endif  // WALK__WALK_NODE_HPP_
