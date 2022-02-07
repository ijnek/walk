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
#include "twist_change_limiter.hpp"

namespace twist_change_limiter
{
geometry_msgs::msg::Twist limit(
  const twist_change_limiter::Params & p,
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target)
{
  auto logger = rclcpp::get_logger("twist_change_limiter::limit");

  RCLCPP_DEBUG(
    logger, "current twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    current.linear.x, current.linear.y, current.linear.z,
    current.angular.x, current.angular.y, current.angular.z);

  RCLCPP_DEBUG(
    logger, " target twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    target.linear.x, target.linear.y, target.linear.z,
    target.angular.x, target.angular.y, target.angular.z);

  geometry_msgs::msg::Twist nextStepTarget = target;
  double & forward = nextStepTarget.linear.x;
  double & left = nextStepTarget.linear.y;
  double & turn = nextStepTarget.angular.z;

  const double & lastForward = current.linear.x;
  const double & lastLeft = current.linear.y;
  const double & lastTurn = current.angular.z;

  if (abs(forward - lastForward) > p.maxForwardChange) {
    forward = lastForward + (forward - lastForward) / abs(forward - lastForward) *
      p.maxForwardChange;
  }
  if (abs(left - lastLeft) > p.maxLeftChange) {
    left = lastLeft + (left - lastLeft) / abs(left - lastLeft) * p.maxLeftChange;
  }
  if (abs(turn - lastTurn) > p.maxTurnChange) {
    turn = lastTurn + (turn - lastTurn) / abs(turn - lastTurn) * p.maxTurnChange;
  }

  RCLCPP_DEBUG(
    logger, " result twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    nextStepTarget.linear.x, nextStepTarget.linear.y, nextStepTarget.linear.z,
    nextStepTarget.angular.x, nextStepTarget.angular.y, nextStepTarget.angular.z);

  return nextStepTarget;
}
}  // namespace twist_change_limiter
