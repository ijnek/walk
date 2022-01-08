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
#include "walk/walk.hpp"
#include "./step_variable.hpp"
#include "./step_calculator.hpp"
#include "./maths_functions.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Walk::Walk(
  std::function<void(const biped_interfaces::msg::AnklePoses &)> send_ankle_poses,
  std::function<void(const geometry_msgs::msg::Twist &)> report_current_twist,
  std::function<void(const std_msgs::msg::Bool &)> report_ready_to_step)
: send_ankle_poses(send_ankle_poses),
  report_current_twist(report_current_twist),
  report_ready_to_step(report_ready_to_step),
  currStep(std::make_shared<StepVariable>()),
  stepCalculator(std::make_shared<StepCalculator>()),
  logger(rclcpp::get_logger("Walk"))
{
}

void Walk::setParams(
  float maxForward, float maxLeft, float maxTurn, float speedMultiplier, float footLiftAmp,
  float period, float ankleX, float ankleY, float ankleZ, float maxForwardChange,
  float maxLeftChange, float maxTurnChange)
{
  this->period = period;
  this->ankleX = ankleX;
  this->ankleY = ankleY;
  this->ankleZ = ankleZ;
  this->footLiftAmp = footLiftAmp;
  stepCalculator->setParams(
    maxForward, maxLeft, maxTurn, speedMultiplier, footLiftAmp,
    maxForwardChange, maxLeftChange, maxTurnChange);
}

void Walk::generateCommand()
{
  RCLCPP_DEBUG(logger, "generateCommand called");
  if (!duringWalk) {
    RCLCPP_DEBUG(logger, "Returning, not during walk");
    return;
  }

  if (t == 0) {
    RCLCPP_DEBUG(logger, "At t == 0:");
    RCLCPP_DEBUG(logger, "  walkOption: %s", walkOptionToString.at(walkOption));
    RCLCPP_DEBUG(logger, "  targetWalkOption: %s", walkOptionToString.at(targetWalkOption));

    // Move towards targetWalkOption and targetTwist
    if (targetWalkOption == WALK) {
      if (walkOption == CROUCH) {
        walkOption = WALK;
        RCLCPP_DEBUG(logger, "walkOption changed to %s", walkOptionToString.at(walkOption));
      }
      currTwist = stepCalculator->calculateNext(currTwist, target);
    } else if (targetWalkOption == CROUCH) {
      if (walkOption == WALK) {
        currTwist = stepCalculator->calculateNext(currTwist, target);
      }
    }

    // Calculate currStep
    currStep->forward = currTwist.linear.x * period;
    currStep->left = currTwist.linear.y * period;
    currStep->turn = currTwist.angular.z * period;

    RCLCPP_DEBUG(
      logger, "Next step: (%gm, %gm, %grad) with legLift: %gm",
      currStep->forward, currStep->left, currStep->turn, currStep->legLift);
  }

  RCLCPP_DEBUG(logger, "Executing walkOption: %s", walkOptionToString.at(walkOption));

  // TODO(ijnek): Hardcoded dt for now, should figure out what to do this.
  double dt = 0.02;

  t += dt;
  RCLCPP_DEBUG(logger, "t: %.4f", t);

  float forwardL = 0, forwardR = 0, leftL = 0, leftR = 0,
    foothL = 0, foothR = 0, turnRL = 0;

  if (walkOption == WALK) {
    // 5.1 Calculate the height to lift each swing foot
    // TODO(ijnek): ideally footLiftAmp should be smaller when the walk starts
    float maxFootHeight = footLiftAmp + abs(currStep->forward) * 0.01 + abs(currStep->left) * 0.03;
    float varfootHeight = maxFootHeight * parabolicReturnMod(t / period);
    RCLCPP_DEBUG(logger, "maxFootHeight: %.4f, varFootHeight: %.4f", maxFootHeight, varfootHeight);
    // 5.2 When walking in an arc, the outside foot needs to travel further
    //     than the inside one - void
    // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt,
    //      for left swing foot
    RCLCPP_DEBUG(
      logger, "isLeftPhase: %s, weightHasShifted: %s",
      isLeftPhase ? "true" : "false", weightHasShifted ? "true" : "false");
    if (isLeftPhase) {                 // if the support foot is right
      if (weightHasShifted) {
        // 5.3.1L forward (the / by 2 is because the CoM moves as well and forwardL is wrt the CoM
        forwardR = forwardR0 + (-(currStep->forward / 2) - forwardR0) * linearStep(t, period);
        // swing-foot follow-through
        forwardL = forwardL0 +
          parabolicStep(dt, t, period, 0) * (currStep->forward / 2 - forwardL0);
        // 5.3.2L Jab kick with left foot - removed
        // 5.3.3L Determine how much to lean from side to side - removed
        // 5.3.4L left
        if (currStep->left > 0) {
          leftL = leftL0 + (currStep->left - leftL0) * parabolicStep(dt, t, period, 0.2);
          leftR = -leftL;
        } else {
          leftL = leftL0 * (1 - parabolicStep(dt, t, period, 0.0));
          leftR = -leftL;
        }
        // 5.3.5L turn (note, we achieve correct turn by splitting turn foot placement unevely over
        //        two steps, but 1.6 + 0.4 = 2 and adds up to two steps worth of turn)
        if (currStep->turn < 0) {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (0.4 * currStep->turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        } else {
          turnRL = turnRL0 + (1.6 * currStep->turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        }
      }
      // 5.3.6L determine how high to lift the swing foot off the ground
      foothL = varfootHeight;                            // lift left swing foot
      foothR = 0;                                   // do not lift support foot;
    }
    // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
    if (!isLeftPhase) {              // if the support foot is left
      if (weightHasShifted) {
        // 5.3.1R forward
        forwardL = forwardL0 + (-(currStep->forward / 2) - forwardL0) * linearStep(t, period);
        // swing-foot follow-through
        forwardR = forwardR0 +
          parabolicStep(dt, t, period, 0) * (currStep->forward / 2 - forwardR0);
        // 5.3.2R Jab-Kick with right foot - removed
        // 5.3.3R lean - not used
        // 5.3.4R left
        if (currStep->left < 0) {
          leftR = leftR0 + (currStep->left - leftR0) * parabolicStep(dt, t, period, 0.2);
          leftL = -leftR;
        } else {
          leftR = leftR0 * (1 - parabolicStep(dt, t, period, 0.0));
          leftL = -leftR;
        }
        // 5.3.5R turn
        if (currStep->turn < 0) {
          turnRL = turnRL0 + (-1.6 * currStep->turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        } else {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (-0.4 * currStep->turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        }
        // 5.3.6R Foot height
      }
      foothR = varfootHeight;
      foothL = 0;
    }

    if (t >= period) {
      turnRL0 = turnRL;
      forwardR0 = forwardR;
      forwardL0 = forwardL;
      leftL0 = leftL;
      leftR0 = leftR;
      isLeftPhase = !isLeftPhase;
      t = 0;

      RCLCPP_DEBUG(logger, "Swing Foot has changed to: %s", isLeftPhase ? "Left" : "Right");
    }
  } else if (walkOption == CROUCH) {
    t = 0;
  }

  RCLCPP_DEBUG(logger, "forwardL: %.4f, leftL: %.4f, foothL: %.4f", forwardL, leftL, foothL);
  RCLCPP_DEBUG(logger, "forwardR: %.4f, leftR: %.4f, foothR: %.4f", forwardR, leftR, foothR);
  RCLCPP_DEBUG(logger, "turnRL: %.4f", turnRL);

  // Send IK Command
  send_ankle_poses(
    generate_ankle_poses(
      forwardL, forwardR, leftL, leftR, foothL, foothR, turnRL));

  // Report current twist
  report_current_twist(currTwist);

  // Report ready_to_step
  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = (t == 0);
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


biped_interfaces::msg::AnklePoses Walk::generate_ankle_poses(
  float forwardL, float forwardR, float leftL,
  float leftR, float foothL, float foothR, float turnRL)
{
  // Evaluate position and angle of both feet
  float l_ankle_pos_x = forwardL + ankleX;
  float l_ankle_pos_y = leftL + ankleY;
  float l_ankle_pos_z = foothL + ankleZ;
  float l_ankle_ang_x = 0;
  float l_ankle_ang_y = 0;
  float l_ankle_ang_z = turnRL;

  float r_ankle_pos_x = forwardR + ankleX;
  float r_ankle_pos_y = leftR - ankleY;
  float r_ankle_pos_z = foothR + ankleZ;
  float r_ankle_ang_x = 0;
  float r_ankle_ang_y = 0;
  float r_ankle_ang_z = -turnRL;

  RCLCPP_DEBUG(logger, "Sending IKCommand with:");
  RCLCPP_DEBUG(
    logger,
    "   LEFT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    l_ankle_pos_x, l_ankle_pos_y, l_ankle_pos_z,
    l_ankle_ang_x, l_ankle_ang_y, l_ankle_ang_z);
  RCLCPP_DEBUG(
    logger,
    "  RIGHT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    r_ankle_pos_x, r_ankle_pos_y, r_ankle_pos_z,
    r_ankle_ang_x, r_ankle_ang_y, r_ankle_ang_z);

  biped_interfaces::msg::AnklePoses command;
  command.l_ankle.position.x = l_ankle_pos_x;
  command.l_ankle.position.y = l_ankle_pos_y;
  command.l_ankle.position.z = l_ankle_pos_z;
  command.l_ankle.orientation = rpy_to_geometry_quat(
    l_ankle_ang_x, l_ankle_ang_y, l_ankle_ang_z);
  command.r_ankle.position.x = r_ankle_pos_x;
  command.r_ankle.position.y = r_ankle_pos_y;
  command.r_ankle.position.z = r_ankle_pos_z;
  command.r_ankle.orientation = rpy_to_geometry_quat(
    r_ankle_ang_x, r_ankle_ang_y, r_ankle_ang_z);

  return command;
}

geometry_msgs::msg::Quaternion Walk::rpy_to_geometry_quat(
  double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion geometry_quat = tf2::toMsg(quat);
  return geometry_quat;
}
