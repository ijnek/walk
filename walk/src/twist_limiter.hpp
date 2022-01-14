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

#ifndef TWIST_LIMITER_HPP_
#define TWIST_LIMITER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace twist_limiter
{
class Params;

geometry_msgs::msg::Twist limit(
  const twist_limiter::Params & p,
  const geometry_msgs::msg::Twist & target);

class Params
{
public:
  Params(float maxForward, float maxLeft, float maxTurn, float speedMultiplier)
  : maxForward(maxForward), maxLeft(maxLeft), maxTurn(maxTurn), speedMultiplier(speedMultiplier)
  {
  }

  float maxForward;        // max forward velocity (m/s)
  float maxLeft;           // max side velocity (m/s)
  float maxTurn;           // max turn velocity (rad/s)
  float speedMultiplier;   // how much to multiple speed by (0.0 - 1.0)
};
}   // namespace twist_limiter

#endif  // TWIST_LIMITER_HPP_
