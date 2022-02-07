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

#include <math.h>
#include "twist_limiter.hpp"

namespace twist_limiter
{

// Declare functions
void ellipsoidClamp(const twist_limiter::Params & p, geometry_msgs::msg::Twist & target);
float evaluateWalkVolume(float x, float y, float z);

geometry_msgs::msg::Twist limit(
  const twist_limiter::Params & p,
  const geometry_msgs::msg::Twist & target)
{
  auto logger = rclcpp::get_logger("twist_limiter::limit");

  RCLCPP_DEBUG(
    logger, " target twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    target.linear.x, target.linear.y, target.linear.z,
    target.angular.x, target.angular.y, target.angular.z);

  geometry_msgs::msg::Twist nextStepTarget = target;
  ellipsoidClamp(p, nextStepTarget);

  RCLCPP_DEBUG(
    logger, " result twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    nextStepTarget.linear.x, nextStepTarget.linear.y, nextStepTarget.linear.z,
    nextStepTarget.angular.x, nextStepTarget.angular.y, nextStepTarget.angular.z);

  return nextStepTarget;
}

void ellipsoidClamp(const twist_limiter::Params & p, geometry_msgs::msg::Twist & target)
{
  // limit max depending on speedMultiplier
  float m_forward = p.maxForward * p.speedMultiplier;
  float m_left = p.maxLeft * p.speedMultiplier;
  float m_turn = p.maxTurn * p.speedMultiplier;

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

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
}

}  // namespace twist_limiter
