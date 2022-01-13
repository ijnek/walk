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
#include "./twist_limiter.hpp"
#include "./maths_functions.hpp"
#include "./feet_trajectory_point.hpp"
#include "./step.hpp"
#include "./gait.hpp"
#include "./target_gait_calculator.hpp"
#include "./phase.hpp"
#include "./ankle_pose_generator.hpp"

Walk::Walk(
  std::function<void(const biped_interfaces::msg::AnklePoses &)> send_ankle_poses,
  std::function<void(const geometry_msgs::msg::Twist &)> report_current_twist,
  std::function<void(const std_msgs::msg::Bool &)> report_ready_to_step)
: send_ankle_poses(send_ankle_poses),
  report_current_twist(report_current_twist),
  report_ready_to_step(report_ready_to_step),
  twistLimiter(std::make_unique<TwistLimiter>()),
  logger(rclcpp::get_logger("Walk")),
  step(std::make_unique<Step>()),
  last(std::make_unique<FeetTrajectoryPoint>()),
  anklePoseGenerator(std::make_unique<AnklePoseGenerator>())
{
}

Walk::~Walk() {}

void Walk::setParams(
  float maxForward, float maxLeft, float maxTurn, float speedMultiplier, float footLiftAmp,
  float period, float ankleX, float ankleY, float ankleZ, float maxForwardChange,
  float maxLeftChange, float maxTurnChange)
{
  this->period = period;
  this->footLiftAmp = footLiftAmp;
  anklePoseGenerator = std::make_unique<AnklePoseGenerator>(ankleX, ankleY, ankleZ);
  twistLimiter->setParams(
    maxForward, maxLeft, maxTurn, speedMultiplier, maxForwardChange, maxLeftChange, maxTurnChange);
}

void Walk::generateCommand()
{
  // TODO(ijnek): Hardcoded dt for now, should figure out what to do this.
  float dt = 0.02;

  RCLCPP_DEBUG(logger, "generateCommand called");
  if (!duringWalk) {
    RCLCPP_DEBUG(logger, "Returning, not during walk");
    return;
  }

  if (step->done()) {
    RCLCPP_DEBUG(logger, "At t == 0:");
    RCLCPP_DEBUG(logger, "  walkOption: %s", walkOptionToString.at(walkOption));
    RCLCPP_DEBUG(logger, "  targetWalkOption: %s", walkOptionToString.at(targetWalkOption));

    // Move towards targetWalkOption and targetTwist
    if (targetWalkOption == WALK) {
      if (walkOption == CROUCH) {
        walkOption = WALK;
        RCLCPP_DEBUG(logger, "walkOption changed to %s", walkOptionToString.at(walkOption));
      }
      currTwist = twistLimiter->limit(currTwist, target);
    } else if (targetWalkOption == CROUCH) {
      if (walkOption == WALK) {
        currTwist = twistLimiter->limit(currTwist, target);
      }
    }

    if (walkOption == WALK) {
      if (phase) {
        phase->invert();
      } else {
        phase = std::make_unique<Phase>(Phase::LeftStance);
      }

      std::unique_ptr<Gait> gait = std::make_unique<Gait>(
        target_gait_calculator::calculate(currTwist, period));

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

      std::unique_ptr<FeetTrajectoryPoint> next = std::make_unique<FeetTrajectoryPoint>(
        (*phase == Phase::LeftStance) ? gait->leftStancePhaseAim : gait->rightStancePhaseAim);

      RCLCPP_DEBUG(
        logger, "Using %s",
        (*phase == Phase::LeftStance) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

      step = std::make_unique<Step>(period, dt, *phase, *last, *next);
      last = std::move(next);
    }
  }

  if (!step->done()) {
    const FeetTrajectoryPoint & currentFTP = step->next();
    // RCLCPP_DEBUG(logger, "Executing walkOption: %s", walkOptionToString.at(walkOption));
    // Send IK Command
    send_ankle_poses(anklePoseGenerator->generate(currentFTP));
  } else {
    send_ankle_poses(anklePoseGenerator->generate(FeetTrajectoryPoint{}));
  }

  // Report current twist
  report_current_twist(currTwist);

  // Report ready_to_step
  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = step->done();
  report_ready_to_step(ready_to_step);
}

void Walk::abort()
{
  duringWalk = false;
  firstMsg = true;
}

void Walk::crouch()
{
  duringWalk = true;
  targetWalkOption = CROUCH;
  firstMsg = true;
}

void Walk::walk(const geometry_msgs::msg::Twist & target)
{
  duringWalk = true;
  targetWalkOption = WALK;
  this->target = target;
}
