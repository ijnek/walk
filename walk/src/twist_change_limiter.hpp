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

#ifndef TWIST_CHANGE_LIMITER_HPP_
#define TWIST_CHANGE_LIMITER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace twist_change_limiter
{
class Params;

geometry_msgs::msg::Twist limit(
  const twist_change_limiter::Params & p,
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target);

class Params
{
public:
  Params(float maxForwardChange, float maxLeftChange, float maxTurnChange)
  : maxForwardChange(maxForwardChange), maxLeftChange(maxLeftChange), maxTurnChange(maxTurnChange)
  {
  }

  float maxForwardChange;  // how much forward can change in one step (m/s)
  float maxLeftChange;     // how much left can change in one step (m/s)
  float maxTurnChange;     // how much turn can change in one step (rad/s)
};
}   // namespace twist_change_limiter

#endif  // TWIST_CHANGE_LIMITER_HPP_
