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
#include "step.hpp"
#include "walk_interfaces/msg/gait.hpp"
#include "target_gait_calculator.hpp"
#include "ankle_pose.hpp"
#include "feet_trajectory.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace walk
{

Walk::Walk(const rclcpp::NodeOptions & options)
: Node("Walk", options),
  ftpCurrent(std::make_unique<walk_interfaces::msg::FeetTrajectoryPoint>()),
  currTwist(std::make_unique<geometry_msgs::msg::Twist>()),
  targetTwist(std::make_shared<geometry_msgs::msg::Twist>())
{
  float max_forward = this->declare_parameter("max_forward", 0.3);  // max forward velocity (m/s)
  float max_left = this->declare_parameter("max_left", 0.2);  // max side velocity (m/s)
  float max_turn = this->declare_parameter("max_turn", 2.0);  // max turn velocity (rad/s)
  float speed_multiplier = this->declare_parameter("speed_multiplier", 1.0);  // how much to multiple speed by (0.0 - 1.0)  // NOLINT
  float foot_lift_amp = this->declare_parameter("foot_lift_amp", 0.012);  // how much to raise foot when it is highest (m)  // NOLINT
  float period = this->declare_parameter("period", 0.25);  // time taken for one step, (s)
  float dt = this->declare_parameter("dt", 0.01);  // time taken between each generateCommand call (s)  // NOLINT
  float ankle_x = this->declare_parameter("ankle_x", -0.01);  // x coordinate of ankle from hip when standing (m)  // NOLINT
  float ankle_y = this->declare_parameter("ankle_y", 0.05);  // y coordinate of ankle from hip when standing (m)  // NOLINT
  float ankle_z = this->declare_parameter("ankle_z", -0.18);  // z coordinate of ankle from hip when standing (m)  // NOLINT
  float max_forward_change = this->declare_parameter("max_forward_change", 0.06);  // how much forward can change in one step (m/s)  // NOLINT
  float max_left_change = this->declare_parameter("max_left_change", 0.1);  // how much left can change in one step (m/s)  // NOLINT
  float max_turn_change = this->declare_parameter("max_turn_change", 1.0);  // how much turn can change in one step (rad/s)  // NOLINT

  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  max_forward : %f", max_forward);
  RCLCPP_DEBUG(get_logger(), "  max_left : %f", max_left);
  RCLCPP_DEBUG(get_logger(), "  max_turn : %f", max_turn);
  RCLCPP_DEBUG(get_logger(), "  speed_multiplier : %f", speed_multiplier);
  RCLCPP_DEBUG(get_logger(), "  foot_lift_amp : %f", foot_lift_amp);
  RCLCPP_DEBUG(get_logger(), "  period : %f", period);
  RCLCPP_DEBUG(get_logger(), "  dt : %f", dt);
  RCLCPP_DEBUG(get_logger(), "  ankle_x : %f", ankle_x);
  RCLCPP_DEBUG(get_logger(), "  ankle_y : %f", ankle_y);
  RCLCPP_DEBUG(get_logger(), "  ankle_z : %f", ankle_z);
  RCLCPP_DEBUG(get_logger(), "  max_forward_change : %f", max_forward_change);
  RCLCPP_DEBUG(get_logger(), "  max_left_change : %f", max_left_change);
  RCLCPP_DEBUG(get_logger(), "  max_turn_change : %f", max_turn_change);

  anklePoseParams = std::make_unique<ankle_pose::Params>(ankle_x, ankle_y, ankle_z);
  twistLimiterParams = std::make_unique<twist_limiter::Params>(
    max_forward, max_left, max_turn, speed_multiplier);
  twistChangeLimiterParams = std::make_unique<twist_change_limiter::Params>(
    max_forward_change, max_left_change, max_turn_change);
  targetGaitCalculatorParams = std::make_unique<target_gait_calculator::Params>(period);
  feetTrajectoryParams = std::make_unique<feet_trajectory::Params>(foot_lift_amp, period, dt);

  generateCommand_timer_ = this->create_wall_timer(
    20ms, std::bind(&Walk::generateCommand_timer_callback, this));

  sub_phase = this->create_subscription<biped_interfaces::msg::Phase>(
    "phase", 10, std::bind(&Walk::phase_callback, this, _1));

  sub_target = this->create_subscription<geometry_msgs::msg::Twist>(
    "target", 10, std::bind(&Walk::target_callback, this, _1));

  pub_ankle_poses = create_publisher<biped_interfaces::msg::AnklePoses>("motion/ankle_poses", 1);
  pub_current_twist = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  // service_abort = create_service<std_srvs::srv::Empty>(
  //   "abort", std::bind(&Walk::abort, this, _1, _2));
}

Walk::~Walk() {}

void Walk::generateCommand()
{
  if (!step) {
    RCLCPP_ERROR(get_logger(), "No step calculated yet, can't generate command!");
    return;
  }

  std::shared_ptr<Step> stepCopy = std::atomic_load(&step);

  if (!stepCopy->done()) {
    RCLCPP_DEBUG(get_logger(), "sending ankle poses");
    pub_ankle_poses->publish(ankle_pose::generate(*anklePoseParams, stepCopy->next()));
  }

  RCLCPP_DEBUG(get_logger(), "reporting current twist");
  pub_current_twist->publish(*currTwist);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = stepCopy->done();
  RCLCPP_DEBUG(get_logger(), "reporting ready to step");
  pub_ready_to_step->publish(ready_to_step);
}

void Walk::walk(const geometry_msgs::msg::Twist & twist)
{
  auto limitedTwist =
    std::make_shared<geometry_msgs::msg::Twist>(twist_limiter::limit(*twistLimiterParams, twist));
  std::atomic_store(&this->targetTwist, std::move(limitedTwist));
}

void Walk::notifyPhase(const biped_interfaces::msg::Phase & phase)
{
  if (this->phase && phase.phase == this->phase->phase) {
    RCLCPP_WARN(get_logger(), "Notified of a phase, but no change has taken place. Ignoring.");
    return;
  }

  this->phase = std::make_unique<biped_interfaces::msg::Phase>(phase);

  currTwist =
    std::make_unique<geometry_msgs::msg::Twist>(
    twist_change_limiter::limit(
      *twistChangeLimiterParams,
      *currTwist, *std::atomic_load(&targetTwist)));

  std::unique_ptr<walk_interfaces::msg::Gait> gait = std::make_unique<walk_interfaces::msg::Gait>(
    target_gait_calculator::calculate(*currTwist, *targetGaitCalculatorParams));
  RCLCPP_DEBUG(get_logger(), "Gait:");
  RCLCPP_DEBUG(
    get_logger(), " LSP: (%f, %f, %f, %f, %f, %f)",
    gait->left_stance_phase_aim.forward_l, gait->left_stance_phase_aim.forward_r,
    gait->left_stance_phase_aim.left_l, gait->left_stance_phase_aim.left_r,
    gait->left_stance_phase_aim.heading_l, gait->left_stance_phase_aim.heading_r);
  RCLCPP_DEBUG(
    get_logger(), " RSP: (%f, %f, %f, %f, %f, %f)",
    gait->right_stance_phase_aim.forward_l, gait->right_stance_phase_aim.forward_r,
    gait->right_stance_phase_aim.left_l, gait->right_stance_phase_aim.left_r,
    gait->right_stance_phase_aim.heading_l, gait->right_stance_phase_aim.heading_r);

  std::unique_ptr<walk_interfaces::msg::FeetTrajectoryPoint> ftpNext =
    std::make_unique<walk_interfaces::msg::FeetTrajectoryPoint>(
    (phase.phase ==
    phase.LEFT_STANCE) ? gait->left_stance_phase_aim : gait->right_stance_phase_aim);

  RCLCPP_DEBUG(
    get_logger(), "Using %s",
    (phase.phase == phase.LEFT_STANCE) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

  std::shared_ptr<Step> step = std::make_shared<Step>(
    feet_trajectory::generate(*feetTrajectoryParams, phase, *ftpCurrent, *ftpNext));
  ftpCurrent = std::move(ftpNext);

  std::atomic_store(&this->step, step);
}

void Walk::generateCommand_timer_callback()
{
  RCLCPP_DEBUG(get_logger(), "generateCommand_timer_callback()");
  generateCommand();
}

void Walk::phase_callback(const biped_interfaces::msg::Phase::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "phase_callback called");
  notifyPhase(*msg);
}

void Walk::target_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(), "target_callback() called with twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    msg->linear.x, msg->linear.y, msg->linear.z,
    msg->angular.x, msg->angular.y, msg->angular.z);
  walk(*msg);
}

}  // namespace walk

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk::Walk)
