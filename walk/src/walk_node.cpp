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

#include "walk/walk_node.hpp"

using namespace std::chrono_literals;

WalkNode::WalkNode()
: Node("WalkNode"),
  walk(
    std::bind(&WalkNode::notifyGoalAchieved, this),
    std::bind(&WalkNode::sendIKCommand, this, std::placeholders::_1))
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
    20ms, std::bind(&WalkNode::timerCallback, this));

  pub_ik_command = create_publisher<nao_ik_interfaces::msg::IKCommand>("motion/ik_command", 1);

  this->action_server_ = rclcpp_action::create_server<walk_interfaces::action::Walk>(
    this,
    "walk",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const WalkGoal> goal)
    {
      geometry_msgs::msg::Twist target = goal->target;
      RCLCPP_INFO(
        get_logger(), "Recevied WalkGoal with target linear:[%g, %g, %g], angular:[%g, %g, %g]",
        target.linear.x, target.linear.y, target.linear.z,
        target.angular.x, target.angular.y, target.angular.z);
      // Accept all goals
      walk.walk(target);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<WalkGoalHandle>)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      // Accept all cancel requests
      walk.abort();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&WalkNode::handleAccepted, this, std::placeholders::_1));
}

void WalkNode::sendIKCommand(nao_ik_interfaces::msg::IKCommand ik_command)
{
  pub_ik_command->publish(ik_command);
}

void WalkNode::handleAccepted(
  const std::shared_ptr<WalkGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(get_logger(), "handleAccepted");
  // Abort any existing goal
  if (walk_goal_handle_) {
    RCLCPP_INFO(
      get_logger(),
      "Walk goal received before a previous goal finished. Aborting previous goal");
    auto result = std::make_shared<walk_interfaces::action::Walk::Result>();
    walk_goal_handle_->abort(result);
  }
  walk_goal_handle_ = goal_handle;
  walk.walk(goal_handle->get_goal()->target);
}

void WalkNode::notifyGoalAchieved()
{
  auto result = std::make_shared<walk_interfaces::action::Walk::Result>();
  walk_goal_handle_->succeed(result);
  walk_goal_handle_ = nullptr;
}

void WalkNode::timerCallback()
{
  RCLCPP_DEBUG(get_logger(), "timerCallback()");
  if (walk_goal_handle_) {
    RCLCPP_DEBUG(get_logger(), "Found walk_goal_handle, executing walk.");
    if (walk_goal_handle_->is_canceling()) {
      RCLCPP_INFO(
        get_logger(), "Goal Cancelled");
      auto result = std::make_shared<walk_interfaces::action::Walk::Result>();
      walk_goal_handle_->canceled(result);
      walk_goal_handle_ = nullptr;
    } else {
      walk.generateCommand();
    }
  }
}
