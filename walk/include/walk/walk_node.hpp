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
#include "biped_interfaces/msg/ankle_poses.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "walk/phase.hpp"


class Walk;

class WalkNode : public rclcpp::Node
{
public:
  using WalkGoal = walk_interfaces::action::Crouch::Goal;
  using CrouchGoalHandle = rclcpp_action::ServerGoalHandle<walk_interfaces::action::Crouch>;

  WalkNode();

private:
  std::shared_ptr<Walk> walk;

  // TODO(ijnek): Replace this timer with an input signal
  rclcpp::TimerBase::SharedPtr generateCommand_timer_;
  rclcpp::TimerBase::SharedPtr notifyPhase_timer_;

  // Twist is a subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target;

  // Abort is a service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_abort;

  rclcpp::Publisher<biped_interfaces::msg::AnklePoses>::SharedPtr pub_ankle_poses;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_current_twist;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_to_step;

  void generateCommand_timer_callback();
  void notifyPhase_timer_callback();

  // TODO(ijnek): Try and figure out how to get rid of these parameters, as they're not used.
  void abort(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>);

  void send_ankle_poses(const biped_interfaces::msg::AnklePoses & ankle_poses);
  void report_current_twist(const geometry_msgs::msg::Twist & current_twist);
  void report_ready_to_step(const std_msgs::msg::Bool & ready_to_step);

  void target_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  Phase phase;
};

#endif  // WALK__WALK_NODE_HPP_
