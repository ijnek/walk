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

#include "walk/walk.hpp"
#include "walk/maths_functions.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Walk::Walk(
  std::function<void(nao_ik_interfaces::msg::IKCommand)> send_ik_command,
  std::function<void(geometry_msgs::msg::Twist)> report_current_twist)
: send_ik_command(send_ik_command),
  report_current_twist(report_current_twist),
  logger(rclcpp::get_logger("Walk"))
{
}

void Walk::setParams(
  float maxForward, float maxLeft, float maxTurn, float speedMultiplier, float footLiftAmp,
  float period, float ankleZ, float maxForwardChange, float maxLeftChange, float maxTurnChange)
{
  this->period = period;
  this->ankleZ = ankleZ;
  this->footLiftAmp = footLiftAmp;
  stepCalculator.setParams(
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
      currTwist = stepCalculator.calculateNext(currTwist, target);
    } else if (targetWalkOption == CROUCH) {
      if (walkOption == WALK) {
        currTwist = stepCalculator.calculateNext(currTwist, target);
      }
    }

    // Calculate currStep
    currStep.forward = currTwist.linear.x * period;
    currStep.left = currTwist.linear.y * period;
    currStep.turn = currTwist.angular.z * period;

    RCLCPP_DEBUG(
      logger, "Next step: (%gm, %gm, %grad) with legLift: %gm",
      currStep.forward, currStep.left, currStep.turn, currStep.legLift);
  }

  RCLCPP_DEBUG(logger, "Executing walkOption: %s", walkOptionToString.at(walkOption));

  // TODO(ijnek): Hardcoded dt for now, should figure out what to do this.
  double dt = 0.02;

  t += dt;

  float forwardL = 0, forwardR = 0, leftL = 0, leftR = 0,
    foothL = 0, foothR = 0, turnRL = 0;

  if (walkOption == WALK) {
    // 5.1 Calculate the height to lift each swing foot
    // TODO(ijnek): ideally footLiftAmp should be smaller when the walk starts
    float maxFootHeight = footLiftAmp + abs(currStep.forward) * 0.01 + abs(currStep.left) * 0.03;
    float varfootHeight = maxFootHeight * parabolicReturnMod(t / period);
    // 5.2 When walking in an arc, the outside foot needs to travel further
    //     than the inside one - void
    // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt,
    //      for left swing foot
    if (isLeftPhase) {                 // if the support foot is right
      if (weightHasShifted) {
        // 5.3.1L forward (the / by 2 is because the CoM moves as well and forwardL is wrt the CoM
        forwardR = forwardR0 + (-(currStep.forward / 2) - forwardR0) * linearStep(t, period);
        // swing-foot follow-through
        forwardL = forwardL0 +
          parabolicStep(dt, t, period, 0) * (currStep.forward / 2 - forwardL0);
        // 5.3.2L Jab kick with left foot - removed
        // 5.3.3L Determine how much to lean from side to side - removed
        // 5.3.4L left
        if (currStep.left > 0) {
          leftL = leftL0 + (currStep.left - leftL0) * parabolicStep(dt, t, period, 0.2);
          leftR = -leftL;
        } else {
          leftL = leftL0 * (1 - parabolicStep(dt, t, period, 0.0));
          leftR = -leftL;
        }
        // 5.3.5L turn (note, we achieve correct turn by splitting turn foot placement unevely over
        //        two steps, but 1.6 + 0.4 = 2 and adds up to two steps worth of turn)
        if (currStep.turn < 0) {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (0.4 * currStep.turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        } else {
          turnRL = turnRL0 + (1.6 * currStep.turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
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
        forwardL = forwardL0 + (-(currStep.forward / 2) - forwardL0) * linearStep(t, period);
        // swing-foot follow-through
        forwardR = forwardR0 +
          parabolicStep(dt, t, period, 0) * (currStep.forward / 2 - forwardR0);
        // 5.3.2R Jab-Kick with right foot - removed
        // 5.3.3R lean - not used
        // 5.3.4R left
        if (currStep.left < 0) {
          leftR = leftR0 + (currStep.left - leftR0) * parabolicStep(dt, t, period, 0.2);
          leftL = -leftR;
        } else {
          leftR = leftR0 * (1 - parabolicStep(dt, t, period, 0.0));
          leftL = -leftR;
        }
        // 5.3.5R turn
        if (currStep.turn < 0) {
          turnRL = turnRL0 + (-1.6 * currStep.turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
        } else {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (-0.4 * currStep.turn - turnRL0) * parabolicStep(dt, t, period, 0.0);
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
  }

  send_ik_command(
    generate_ik_command(
      forwardL, forwardR, leftL, leftR, foothL, foothR, turnRL));

  report_current_twist(currTwist);
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


nao_ik_interfaces::msg::IKCommand Walk::generate_ik_command(
  float forwardL, float forwardR, float leftL,
  float leftR, float foothL, float foothR, float turnRL)
{
  // Evaluate position and angle of both feet
  float left_ankle_pos_x = forwardL;
  float left_ankle_pos_y = leftL + 0.050;
  float left_ankle_pos_z = ankleZ + foothL;
  float left_ankle_ang_x = 0;
  float left_ankle_ang_y = 0;
  float left_ankle_ang_z = turnRL;

  float right_ankle_pos_x = forwardR;
  float right_ankle_pos_y = leftR - 0.050;
  float right_ankle_pos_z = ankleZ + foothR;
  float right_ankle_ang_x = 0;
  float right_ankle_ang_y = 0;
  float right_ankle_ang_z = -turnRL;

  RCLCPP_DEBUG(logger, "Sending IKCommand with:");
  RCLCPP_DEBUG(
    logger,
    "   LEFT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    left_ankle_pos_x, left_ankle_pos_y, left_ankle_pos_z,
    left_ankle_ang_x, left_ankle_ang_y, left_ankle_ang_z);
  RCLCPP_DEBUG(
    logger,
    "  RIGHT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    right_ankle_pos_x, right_ankle_pos_y, right_ankle_pos_z,
    right_ankle_ang_x, right_ankle_ang_y, right_ankle_ang_z);

  nao_ik_interfaces::msg::IKCommand command;
  command.left_ankle.position.x = left_ankle_pos_x;
  command.left_ankle.position.y = left_ankle_pos_y;
  command.left_ankle.position.z = left_ankle_pos_z;
  command.left_ankle.orientation = rpy_to_geometry_quat(
    left_ankle_ang_x, left_ankle_ang_y, left_ankle_ang_z);
  command.right_ankle.position.x = right_ankle_pos_x;
  command.right_ankle.position.y = right_ankle_pos_y;
  command.right_ankle.position.z = right_ankle_pos_z;
  command.right_ankle.orientation = rpy_to_geometry_quat(
    right_ankle_ang_x, right_ankle_ang_y, right_ankle_ang_z);

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
