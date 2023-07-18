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

namespace walk
{

Walk::Walk(const rclcpp::NodeOptions & options)
: Node("Walk", options),
  ftp_current_(std::make_unique<walk_interfaces::msg::FeetTrajectoryPoint>()),
  curr_twist_(std::make_unique<geometry_msgs::msg::Twist>()),
  target_twist_(std::make_shared<geometry_msgs::msg::Twist>())
{
  float max_forward = declare_parameter("max_forward", 0.3);  // max forward velocity (m/s)
  float max_left = declare_parameter("max_left", 0.2);  // max side velocity (m/s)
  float max_turn = declare_parameter("max_turn", 2.0);  // max turn velocity (rad/s)
  float speed_multiplier = declare_parameter("speed_multiplier", 1.0);  // how much to multiple speed by (0.0 - 1.0)  // NOLINT
  float foot_lift_amp = declare_parameter("foot_lift_amp", 0.012);  // how much to raise foot when it is highest (m)  // NOLINT
  float period = declare_parameter("period", 0.25);  // time taken for one step, (s)
  float dt = declare_parameter("dt", 0.01);  // time taken between each generateCommand call (s)  // NOLINT
  float sole_x = declare_parameter("sole_x", -0.01);  // x coordinate of sole from hip when standing (m)  // NOLINT
  float sole_y = declare_parameter("sole_y", 0.05);  // y coordinate of sole from hip when standing (m)  // NOLINT
  float sole_z = declare_parameter("sole_z", -0.315);  // z coordinate of sole from hip when standing (m)  // NOLINT
  float max_forward_change = declare_parameter("max_forward_change", 0.06);  // how much forward can change in one step (m/s)  // NOLINT
  float max_left_change = declare_parameter("max_left_change", 0.1);  // how much left can change in one step (m/s)  // NOLINT
  float max_turn_change = declare_parameter("max_turn_change", 1.0);  // how much turn can change in one step (rad/s)  // NOLINT

  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  max_forward : %f", max_forward);
  RCLCPP_DEBUG(get_logger(), "  max_left : %f", max_left);
  RCLCPP_DEBUG(get_logger(), "  max_turn : %f", max_turn);
  RCLCPP_DEBUG(get_logger(), "  speed_multiplier : %f", speed_multiplier);
  RCLCPP_DEBUG(get_logger(), "  foot_lift_amp : %f", foot_lift_amp);
  RCLCPP_DEBUG(get_logger(), "  period : %f", period);
  RCLCPP_DEBUG(get_logger(), "  dt : %f", dt);
  RCLCPP_DEBUG(get_logger(), "  sole_x : %f", sole_x);
  RCLCPP_DEBUG(get_logger(), "  sole_y : %f", sole_y);
  RCLCPP_DEBUG(get_logger(), "  sole_z : %f", sole_z);
  RCLCPP_DEBUG(get_logger(), "  max_forward_change : %f", max_forward_change);
  RCLCPP_DEBUG(get_logger(), "  max_left_change : %f", max_left_change);
  RCLCPP_DEBUG(get_logger(), "  max_turn_change : %f", max_turn_change);

  sole_pose_params_ = std::make_unique<sole_pose::Params>(sole_x, sole_y, sole_z);
  twist_limiter_params_ = std::make_unique<twist_limiter::Params>(
    max_forward, max_left, max_turn, speed_multiplier);
  twist_change_limiter_params_ = std::make_unique<twist_change_limiter::Params>(
    max_forward_change, max_left_change, max_turn_change);
  target_gait_calculator_params_ = std::make_unique<target_gait_calculator::Params>(period);
  feet_trajectory_params_ = std::make_unique<feet_trajectory::Params>(foot_lift_amp, period, dt);

  generate_command_timer_ = create_wall_timer(
    std::chrono::duration<float>(dt), std::bind(&Walk::generateCommand, this));

  sub_phase_ = create_subscription<biped_interfaces::msg::Phase>(
    "phase", 10, std::bind(&Walk::notifyPhase, this, std::placeholders::_1));

  sub_target_ = create_subscription<geometry_msgs::msg::Twist>(
    "target", 10, std::bind(&Walk::walk, this, std::placeholders::_1));

  pub_sole_poses_ = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);
  pub_current_twist_ = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step_ = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  pub_gait_ = create_publisher<walk_interfaces::msg::Gait>("walk/gait", 1);
  pub_step_ = create_publisher<walk_interfaces::msg::Step>("walk/step", 1);

  // Register parameter change callback
  on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Walk::parametersCallback, this, std::placeholders::_1));
}

Walk::~Walk() {}

void Walk::generateCommand()
{
  // RCLCPP_DEBUG(get_logger(), "generateCommand()");

  if (!step_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,  // ms
      "No step calculated yet, can't generate command!");
    return;
  }

  std::shared_ptr<StepState> step_state_copy = std::atomic_load(&step_state_);

  if (!step_state_copy->done()) {
    // RCLCPP_DEBUG(get_logger(), "sending sole poses");
    pub_sole_poses_->publish(sole_pose::generate(*sole_pose_params_, step_state_copy->next()));
  }

  pub_current_twist_->publish(*curr_twist_);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = step_state_copy->done();
  pub_ready_to_step_->publish(ready_to_step);
}

void Walk::walk(const geometry_msgs::msg::Twist & commanded_twist)
{
  // RCLCPP_DEBUG(
  //   get_logger(), "walk() called with commanded_twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
  //   commanded_twist.linear.x, commanded_twist.linear.y, commanded_twist.linear.z,
  //   commanded_twist.angular.x, commanded_twist.angular.y, commanded_twist.angular.z);

  auto limitedTwist = std::make_shared<geometry_msgs::msg::Twist>(
    twist_limiter::limit(*twist_limiter_params_, commanded_twist));
  std::atomic_store(&target_twist_, std::move(limitedTwist));
}

void Walk::notifyPhase(const biped_interfaces::msg::Phase & phase)
{
  // RCLCPP_DEBUG(get_logger(), "notifyPhase called");

  if (phase_ && phase.phase == phase_->phase) {
    RCLCPP_DEBUG(get_logger(), "Notified of a phase, but no change has taken place. Ignoring.");
    return;
  }

  // RCLCPP_DEBUG(get_logger(), "Calculating new step!");

  phase_ = std::make_unique<biped_interfaces::msg::Phase>(phase);

  curr_twist_ =
    std::make_unique<geometry_msgs::msg::Twist>(
    twist_change_limiter::limit(
      *twist_change_limiter_params_,
      *curr_twist_, *std::atomic_load(&target_twist_)));

  auto gait = target_gait_calculator::calculate(*curr_twist_, *target_gait_calculator_params_);
  pub_gait_->publish(gait);

  std::unique_ptr<walk_interfaces::msg::FeetTrajectoryPoint> ftp_next =
    std::make_unique<walk_interfaces::msg::FeetTrajectoryPoint>(
    (phase.phase ==
    phase.LEFT_STANCE) ? gait.left_stance_phase_aim : gait.right_stance_phase_aim);

  // RCLCPP_DEBUG(
  //   get_logger(), "Using %s",
  //   (phase.phase == phase.LEFT_STANCE) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

  std::shared_ptr<walk_interfaces::msg::Step> step = std::make_shared<walk_interfaces::msg::Step>(
    feet_trajectory::generate(*feet_trajectory_params_, phase, *ftp_current_, *ftp_next));
  pub_step_->publish(*step);

  ftp_current_ = std::move(ftp_next);

  auto step_state = std::make_shared<StepState>(*step);

  std::atomic_store(&step_, step);
  std::atomic_store(&step_state_, step_state);
}

rcl_interfaces::msg::SetParametersResult Walk::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Parameter updated:");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param: parameters)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "- " << param.get_name() << " (" << param.get_type_name() << ")" << " = " << param.value_to_string());

    auto name = param.get_name();
    if (name == "max_forward")
      twist_limiter_params_->max_forward_ = param.as_double();
    else if (name == "max_left")
      twist_limiter_params_->max_left_ = param.as_double();
    else if (name == "max_turn")
      twist_limiter_params_->max_turn_ = param.as_double();
    else if (name == "speed_multiplier")
      twist_limiter_params_->speed_multiplier_ = param.as_double();
    else if (name == "foot_lift_amp")
      feet_trajectory_params_->foot_lift_amp_ = param.as_double();
    else if (name == "period")
      feet_trajectory_params_->period_ = param.as_double();
    else if (name == "dt")
      feet_trajectory_params_->dt_ = param.as_double();
    else if (name == "sole_x")
      sole_pose_params_->sole_x_ = param.as_double();
    else if (name == "sole_y")
      sole_pose_params_->sole_y_ = param.as_double();
    else if (name == "sole_z")
      sole_pose_params_->sole_z_ = param.as_double();
    else if (name == "max_forward_change")
      twist_change_limiter_params_->max_forward_change_ = param.as_double();
    else if (name == "max_left_change")
      twist_change_limiter_params_->max_left_change_ = param.as_double();
    else if (name == "max_turn_change")
      twist_change_limiter_params_->max_turn_change_ = param.as_double();
  }

  return result;
}

}  // namespace walk

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk::Walk)
