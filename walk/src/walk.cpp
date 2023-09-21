// This file is based on UNSW Sydney's codebase, but has been modified significantly.
// Both copyright notices are provided below.
//
// Copyright (c) 2018 UNSW Sydney.  All rights reserved.
//
// Licensed under Team rUNSWift's original license. See the "LICENSE-runswift"
// file to obtain a copy of the license.
//
// ---------------------------------------------------------------------------------
//
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
#include <utility>

#include "rclcpp_action/rclcpp_action.hpp"
#include "walk/walk.hpp"
#include "twist_limiter.hpp"
#include "twist_change_limiter.hpp"
#include "maths_functions.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"
#include "walk_interfaces/msg/step.hpp"
#include "step_state.hpp"
#include "target_gait_calculator.hpp"
#include "sole_pose.hpp"
#include "feet_trajectory.hpp"
#include "params.hpp"

namespace walk
{

Walk::Walk(const rclcpp::NodeOptions & options)
: Node("Walk", options)
{
  params_ = std::make_unique<Params>(*this);

  generate_command_timer_ = create_wall_timer(
    std::chrono::duration<float>(params_->feet_trajectory_.dt_),
    std::bind(&Walk::generateCommand, this));

  sub_phase_ = create_subscription<biped_interfaces::msg::Phase>(
    "phase", 10, std::bind(&Walk::notifyPhase, this, std::placeholders::_1));

  sub_target_ = create_subscription<geometry_msgs::msg::Twist>(
    "target", 10, std::bind(&Walk::walk, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&Walk::imuCallback, this, std::placeholders::_1));

  pub_sole_poses_ = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);
  pub_current_twist_ = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step_ = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  action_server_walk_ = rclcpp_action::create_server<walk_interfaces::action::Walk>(
    this,
    "walk",
    std::bind(&Walk::handleGoalWalk, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Walk::handleCancelWalk, this, std::placeholders::_1),
    std::bind(&Walk::handleAcceptedWalk, this, std::placeholders::_1));

  pub_gait_ = create_publisher<walk_interfaces::msg::Gait>("walk/gait", 1);
  pub_step_ = create_publisher<walk_interfaces::msg::Step>("walk/step", 1);
}

Walk::~Walk() {}

void Walk::generateCommand()
{
  // RCLCPP_DEBUG(get_logger(), "generateCommand()");

  if (!active_)
    return;

  if (!step_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,  // ms
      "No step calculated yet, can't generate command!");
    return;
  }

  if (!step_state_->done()) {
    RCLCPP_DEBUG(get_logger(), "sending sole poses");
    pub_sole_poses_->publish(
      sole_pose::generate(params_->sole_pose_, step_state_->next(), phase_, filtered_gyro_y_));
  }

  pub_current_twist_->publish(curr_twist_);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = step_state_->done();
  pub_ready_to_step_->publish(ready_to_step);
}

void Walk::walk(const geometry_msgs::msg::Twist & commanded_twist)
{
  RCLCPP_DEBUG(
    get_logger(), "walk() called with commanded_twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    commanded_twist.linear.x, commanded_twist.linear.y, commanded_twist.linear.z,
    commanded_twist.angular.x, commanded_twist.angular.y, commanded_twist.angular.z);

  target_twist_ = twist_limiter::limit(params_->twist_limiter_, commanded_twist);
}

void Walk::notifyPhase(const biped_interfaces::msg::Phase & phase)
{
  RCLCPP_DEBUG(get_logger(), "notifyPhase called");

  if (phase.phase == phase_.phase) {
    RCLCPP_DEBUG(get_logger(), "Notified of a phase, but no change has taken place. Ignoring.");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Calculating new step!");

  phase_ = phase;

  curr_twist_ = twist_change_limiter::limit(
    params_->twist_change_limiter_, curr_twist_, target_twist_);

  auto gait = target_gait_calculator::calculate(curr_twist_, params_->target_gait_calculator_);
  pub_gait_->publish(gait);

  auto ftp_next = walk_interfaces::msg::FeetTrajectoryPoint(
    (phase.phase == phase.LEFT_STANCE) ? gait.left_stance_phase_aim : gait.right_stance_phase_aim);

  RCLCPP_DEBUG(
    get_logger(), "Using %s",
    (phase.phase == phase.LEFT_STANCE) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

  step_ = std::make_unique<walk_interfaces::msg::Step>(
    feet_trajectory::generate(
      params_->feet_trajectory_, phase, ftp_current_, ftp_next));
  step_state_ = std::make_unique<StepState>(*step_);
  pub_step_->publish(*step_);

  ftp_current_ = std::move(ftp_next);
}

void Walk::imuCallback(const sensor_msgs::msg::Imu & imu)
{
  filtered_gyro_y_ = imu.angular_velocity.y;
}

rclcpp_action::GoalResponse Walk::handleGoalWalk(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const walk_interfaces::action::Walk::Goal> goal)
{
  // RCLCPP_INFO(get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Walk::handleCancelWalk(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<walk_interfaces::action::Walk>> goal_handle)
{
  // RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  active_ = false;
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}
void Walk::handleAcceptedWalk(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<walk_interfaces::action::Walk>> goal_handle)
{
  target_twist_ = twist_limiter::limit(params_->twist_limiter_, goal_handle->get_goal()->twist);
  goal_handle_ = goal_handle;
}


}  // namespace walk

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk::Walk)
