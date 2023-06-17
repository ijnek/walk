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

  geometry_msgs::msg::Twist next_step_target = target;
  double & forward = next_step_target.linear.x;
  double & left = next_step_target.linear.y;
  double & turn = next_step_target.angular.z;

  const double & last_forward = current.linear.x;
  const double & last_left = current.linear.y;
  const double & last_turn = current.angular.z;

  if (abs(forward - last_forward) > p.max_forward_change_) {
    forward = last_forward + (forward - last_forward) / abs(forward - last_forward) *
      p.max_forward_change_;
  }
  if (abs(left - last_left) > p.max_left_change_) {
    left = last_left + (left - last_left) / abs(left - last_left) * p.max_left_change_;
  }
  if (abs(turn - last_turn) > p.max_turn_change_) {
    turn = last_turn + (turn - last_turn) / abs(turn - last_turn) * p.max_turn_change_;
  }

  RCLCPP_DEBUG(
    logger, " result twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    next_step_target.linear.x, next_step_target.linear.y, next_step_target.linear.z,
    next_step_target.angular.x, next_step_target.angular.y, next_step_target.angular.z);

  return next_step_target;
}
}  // namespace twist_change_limiter
