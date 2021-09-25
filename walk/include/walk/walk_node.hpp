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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "walk_interfaces/action/crouch.hpp"
#include "walk_interfaces/action/stand.hpp"
#include "nao_ik_interfaces/msg/ik_command.hpp"
#include "walk/walk.hpp"
#include "std_srvs/srv/empty.hpp"

class WalkNode : public rclcpp::Node
{
public:
  using WalkGoal = walk_interfaces::action::Crouch::Goal;
  using CrouchGoalHandle = rclcpp_action::ServerGoalHandle<walk_interfaces::action::Crouch>;

  WalkNode();

private:
  Walk walk;

  // TODO(ijnek): Replace this timer with an input signal
  rclcpp::TimerBase::SharedPtr timer_;

  // Twist is a subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target;

  // Abort is a service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_abort;

  // Crouch and Stand are actions
  // rclcpp_action::Server<walk_interfaces::action::Crouch>::SharedPtr action_server_crouch_;
  // rclcpp_action::Server<walk_interfaces::action::Stand>::SharedPtr action_server_stand_;

  rclcpp::Publisher<nao_ik_interfaces::msg::IKCommand>::SharedPtr pub_ik_command;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_current_twist;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_to_step;

  // std::shared_ptr<CrouchGoalHandle> crouch_goal_handle_;

  void timer_callback();

  // TODO(ijnek): Try and figure out how to get rid of these parameters, as they're not used.
  void abort(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>);

  void send_ik_command(nao_ik_interfaces::msg::IKCommand ik_command);
  void report_current_twist(geometry_msgs::msg::Twist current_twist);
  void report_ready_to_step(std_msgs::msg::Bool ready_to_step);
  // void handle_accepted(
  //   const std::shared_ptr<CrouchGoalHandle> crouch_goal_handle);

  void target_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif  // WALK__WALK_NODE_HPP_
