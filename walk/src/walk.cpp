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
#include "feet_trajectory_point.hpp"
#include "step.hpp"
#include "gait.hpp"
#include "target_gait_calculator.hpp"
#include "ankle_pose.hpp"
#include "feet_trajectory.hpp"

Walk::Walk(
  std::function<void(const biped_interfaces::msg::AnklePoses &)> send_ankle_poses,
  std::function<void(const geometry_msgs::msg::Twist &)> report_current_twist,
  std::function<void(const std_msgs::msg::Bool &)> report_ready_to_step)
: send_ankle_poses(send_ankle_poses),
  report_current_twist(report_current_twist),
  report_ready_to_step(report_ready_to_step),
  logger(rclcpp::get_logger("Walk")),
  ftpCurrent(std::make_unique<FeetTrajectoryPoint>()),
  currTwist(std::make_unique<geometry_msgs::msg::Twist>()),
  targetTwist(std::make_shared<geometry_msgs::msg::Twist>())
{
}

Walk::~Walk() {}

void Walk::setParams(
  float maxForward, float maxLeft, float maxTurn, float speedMultiplier, float footLiftAmp,
  float period, float dt, float ankleX, float ankleY, float ankleZ, float maxForwardChange,
  float maxLeftChange, float maxTurnChange)
{
  anklePoseParams = std::make_unique<ankle_pose::Params>(ankleX, ankleY, ankleZ);
  twistLimiterParams = std::make_unique<twist_limiter::Params>(
    maxForward, maxLeft, maxTurn, speedMultiplier);
  twistChangeLimiterParams = std::make_unique<twist_change_limiter::Params>(
    maxForwardChange, maxLeftChange, maxTurnChange);
  targetGaitCalculatorParams = std::make_unique<target_gait_calculator::Params>(period);
  feetTrajectoryParams = std::make_unique<feet_trajectory::Params>(footLiftAmp, period, dt);
}

void Walk::generateCommand()
{
  if (!step) {
    RCLCPP_ERROR(logger, "No step calculated yet, can't generate command!");
    return;
  }

  std::shared_ptr<Step> stepCopy = std::atomic_load(&step);

  if (!stepCopy->done()) {
    send_ankle_poses(ankle_pose::generate(*anklePoseParams, stepCopy->next()));
  }

  report_current_twist(*currTwist);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = stepCopy->done();
  report_ready_to_step(ready_to_step);
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
    RCLCPP_WARN(logger, "Notified of a phase, but no change has taken place. Ignoring.");
    return;
  }

  this->phase = std::make_unique<biped_interfaces::msg::Phase>(phase);

  currTwist =
    std::make_unique<geometry_msgs::msg::Twist>(
    twist_change_limiter::limit(
      *twistChangeLimiterParams,
      *currTwist, *std::atomic_load(&targetTwist)));

  std::unique_ptr<Gait> gait = std::make_unique<Gait>(
    target_gait_calculator::calculate(*currTwist, *targetGaitCalculatorParams));
  RCLCPP_DEBUG(logger, "Gait:");
  RCLCPP_DEBUG(
    logger, " LSP: (%f, %f, %f, %f, %f, %f)",
    gait->leftStancePhaseAim.forwardL, gait->leftStancePhaseAim.forwardR,
    gait->leftStancePhaseAim.leftL, gait->leftStancePhaseAim.leftR,
    gait->leftStancePhaseAim.headingL, gait->leftStancePhaseAim.headingR);
  RCLCPP_DEBUG(
    logger, " RSP: (%f, %f, %f, %f, %f, %f)",
    gait->rightStancePhaseAim.forwardL, gait->rightStancePhaseAim.forwardR,
    gait->rightStancePhaseAim.leftL, gait->rightStancePhaseAim.leftR,
    gait->rightStancePhaseAim.headingL, gait->rightStancePhaseAim.headingR);

  std::unique_ptr<FeetTrajectoryPoint> ftpNext = std::make_unique<FeetTrajectoryPoint>(
    (phase.phase == phase.LEFT_STANCE) ? gait->leftStancePhaseAim : gait->rightStancePhaseAim);

  RCLCPP_DEBUG(
    logger, "Using %s",
    (phase.phase == phase.LEFT_STANCE) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

  std::shared_ptr<Step> step = std::make_shared<Step>(
    feet_trajectory::generate(*feetTrajectoryParams, phase, *ftpCurrent, *ftpNext));
  ftpCurrent = std::move(ftpNext);

  std::atomic_store(&this->step, step);
}
