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

#include <memory>
#include "walk/walk_node.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

WalkNode::WalkNode()
: Node("WalkNode"),
  walk(
    std::bind(&WalkNode::send_ik_command, this, _1),
    std::bind(&WalkNode::report_current_twist, this, _1),
    std::bind(&WalkNode::report_ready_to_step, this, _1))
{
  float max_forward = this->declare_parameter("max_forward", 0.3);
  float max_left = this->declare_parameter("max_left", 0.2);
  float max_turn = this->declare_parameter("max_turn", 2.0);
  float speed_multiplier = this->declare_parameter("speed_multiplier", 1.0);
  float foot_lift_amp = this->declare_parameter("foot_lift_amp", 0.012);
  float period = this->declare_parameter("period", 0.25);
  float ankle_z = this->declare_parameter("ankle_z", -0.18);
  float max_forward_change = this->declare_parameter("max_forward_change", 0.06);
  float max_left_change = this->declare_parameter("max_left_change", 0.1);
  float max_turn_change = this->declare_parameter("max_turn_change", 1.0);

  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  max_forward : %f", max_forward);
  RCLCPP_DEBUG(get_logger(), "  max_left : %f", max_left);
  RCLCPP_DEBUG(get_logger(), "  max_turn : %f", max_turn);
  RCLCPP_DEBUG(get_logger(), "  speed_multiplier : %f", speed_multiplier);
  RCLCPP_DEBUG(get_logger(), "  foot_lift_amp : %f", foot_lift_amp);
  RCLCPP_DEBUG(get_logger(), "  period : %f", period);
  RCLCPP_DEBUG(get_logger(), "  ankle_z : %f", ankle_z);
  RCLCPP_DEBUG(get_logger(), "  max_forward_change : %f", max_forward_change);
  RCLCPP_DEBUG(get_logger(), "  max_left_change : %f", max_left_change);
  RCLCPP_DEBUG(get_logger(), "  max_turn_change : %f", max_turn_change);

  walk.setParams(
    max_forward, max_left, max_turn, speed_multiplier, foot_lift_amp, period, ankle_z,
    max_forward_change, max_left_change, max_turn_change);

  timer_ = this->create_wall_timer(
    20ms, std::bind(&WalkNode::timer_callback, this));

  sub_target = this->create_subscription<geometry_msgs::msg::Twist>(
    "target", 10, std::bind(&WalkNode::target_callback, this, _1));

  pub_ik_command = create_publisher<nao_ik_interfaces::msg::IKCommand>("motion/ik_command", 1);
  pub_current_twist = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  service_abort = create_service<std_srvs::srv::Empty>(
    "abort", std::bind(&WalkNode::abort, this, _1, _2));

  // this->action_server_crouch_ = rclcpp_action::create_server<walk_interfaces::action::Crouch>(
  //   this,
  //   "walk",
  //   [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const CrouchGoal> goal)
  //   {
  //     geometry_msgs::msg::Twist target = goal->target;
  //     RCLCPP_INFO(
  //   get_logger(), "Recevied CrouchGoal with target linear:[%g, %g, %g], angular:[%g, %g, %g]",
  //       target.linear.x, target.linear.y, target.linear.z,
  //       target.angular.x, target.angular.y, target.angular.z);
  //     // Accept all goals
  //     walk.walk(target);
  //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  //   },
  //   [this](const std::shared_ptr<CrouchGoalHandle>)
  //   {
  //     RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  //     // Accept all cancel requests
  //     walk.abort();
  //     return rclcpp_action::CancelResponse::ACCEPT;
  //   },
  //   std::bind(&WalkNode::handle_accepted, this, _1));
}

void WalkNode::abort(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_DEBUG(get_logger(), "abort() called, aborting walk.");
  walk.abort();
}

void WalkNode::send_ik_command(nao_ik_interfaces::msg::IKCommand ik_command)
{
  RCLCPP_DEBUG(get_logger(), "send_ik_command() called");
  pub_ik_command->publish(ik_command);
}

void WalkNode::report_current_twist(geometry_msgs::msg::Twist current_twist)
{
  RCLCPP_DEBUG(get_logger(), "report_current_twist() called");
  pub_current_twist->publish(current_twist);
}

void WalkNode::report_ready_to_step(std_msgs::msg::Bool ready_to_step)
{
  RCLCPP_DEBUG(get_logger(), "report_ready_to_step() called");
  pub_ready_to_step->publish(ready_to_step);
}


// void WalkNode::handle_accepted(
//   const std::shared_ptr<CrouchGoalHandle> goal_handle)
// {
//   RCLCPP_DEBUG(get_logger(), "handle_accepted");
//   // Abort any existing goal
//   if (walk_goal_handle_) {
//     RCLCPP_INFO(
//       get_logger(),
//       "Crouch goal received before a previous goal finished. Aborting previous goal");
//     auto result = std::make_shared<walk_interfaces::action::Crouch::Result>();
//     walk_goal_handle_->abort(result);
//   }
//   walk_goal_handle_ = goal_handle;
//   walk.walk(goal_handle->get_goal()->target);
// }

void WalkNode::timer_callback()
{
  RCLCPP_DEBUG(get_logger(), "timer_callback()");
  walk.generateCommand();
}

void WalkNode::target_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(), "target_callback() called with twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    msg->linear.x, msg->linear.y, msg->linear.z,
    msg->angular.x, msg->angular.y, msg->angular.z);
  walk.walk(*msg);
}
