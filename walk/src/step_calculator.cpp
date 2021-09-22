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

#include <gtest/gtest.h>
#include <math.h>
#include "walk/step_calculator.hpp"

void StepCalculator::setParams(
  float maxForward,
  float maxLeft,
  float maxTurn,
  float speedMultiplier,
  float footLiftAmp,
  float maxForwardChange,
  float maxLeftChange,
  float maxTurnChange)
{
  this->maxForward = maxForward;
  this->maxLeft = maxLeft;
  this->maxTurn = maxTurn;
  this->speedMultiplier = speedMultiplier;
  this->footLiftAmp = footLiftAmp;
  this->maxForwardChange = maxForwardChange;
  this->maxLeftChange = maxLeftChange;
  this->maxTurnChange = maxTurnChange;
}

geometry_msgs::msg::Twist StepCalculator::calculateNext(
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target)
{
  RCLCPP_DEBUG(
    logger, "current twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    current.linear.x, current.linear.y, current.linear.z,
    current.angular.x, current.angular.y, current.angular.z);
  
  RCLCPP_DEBUG(
    logger, " target twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    target.linear.x, target.linear.y, target.linear.z,
    target.angular.x, target.angular.y, target.angular.z);

  geometry_msgs::msg::Twist nextStepTarget = target;
  ellipsoidClamp(nextStepTarget);
  limitChange(nextStepTarget, current);

  RCLCPP_DEBUG(
    logger, " result twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    nextStepTarget.linear.x, nextStepTarget.linear.y, nextStepTarget.linear.z,
    nextStepTarget.angular.x, nextStepTarget.angular.y, nextStepTarget.angular.z);

  return nextStepTarget;
}

void StepCalculator::ellipsoidClamp(geometry_msgs::msg::Twist & target)
{
  // limit max depending on speedMultiplier
  float m_forward = maxForward * speedMultiplier;
  float m_left = maxLeft * speedMultiplier;
  float m_turn = maxTurn * speedMultiplier;

  // Values in range [-1..1]
  float forwardAmount = target.linear.x / m_forward;
  float leftAmount = target.linear.y / m_left;
  float turnAmount = target.angular.z / m_turn;

  float x = abs(forwardAmount);
  float y = abs(leftAmount);
  float z = abs(turnAmount);

  // see if the point we are given is already inside the allowed walk params volume
  if (evaluateWalkVolume(x, y, z) > 1.0) {
    float scale = 0.5;
    float high = 1.0;
    float low = 0.0;

    // This is basically a binary search to find the point on the surface.
    for (unsigned i = 0; i < 10; i++) {
      x = abs(forwardAmount) * scale;
      y = abs(leftAmount) * scale;
      z = abs(turnAmount) * scale;

      if (evaluateWalkVolume(x, y, z) > 1.0) {
        float newScale = (scale + low) / 2.0;
        high = scale;
        scale = newScale;
      } else {
        float newScale = (scale + high) / 2.0;
        low = scale;
        scale = newScale;
      }
    }

    forwardAmount *= scale;
    leftAmount *= scale;
    turnAmount *= scale;
  }

  target.linear.x = m_forward * forwardAmount;
  target.linear.y = m_left * leftAmount;
  target.angular.z = m_turn * turnAmount;
}

void StepCalculator::limitChange(
  geometry_msgs::msg::Twist & target,
  const geometry_msgs::msg::Twist & current)
{
  double & forward = target.linear.x;
  double & left = target.linear.y;
  double & turn = target.angular.z;

  const double & lastForward = current.linear.x;
  const double & lastLeft = current.linear.y;
  const double & lastTurn = current.angular.z;

  if (abs(forward - lastForward) > maxForwardChange) {
    forward = lastForward + (forward - lastForward) / abs(forward - lastForward) * maxForwardChange;
  }
  if (abs(left - lastLeft) > maxLeftChange) {
    left = lastLeft + (left - lastLeft) / abs(left - lastLeft) * maxLeftChange;
  }
  if (abs(turn - lastTurn) > maxTurnChange) {
    turn = lastTurn + (turn - lastTurn) / abs(turn - lastTurn) * maxTurnChange;
  }
}

// x = forward, y = left, z = turn
float StepCalculator::evaluateWalkVolume(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
}