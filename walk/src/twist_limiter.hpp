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
  Params(double max_forward, double max_left, double max_turn, double speed_multiplier)
  : max_forward_(max_forward),
    max_left_(max_left),
    max_turn_(max_turn),
    speed_multiplier_(speed_multiplier)
  {
  }

  Params() {}

  double max_forward_;        // max forward velocity (m/s)
  double max_left_;           // max side velocity (m/s)
  double max_turn_;           // max turn velocity (rad/s)
  double speed_multiplier_;   // how much to multiple speed by (0.0 - 1.0)
};
}   // namespace twist_limiter

#endif  // TWIST_LIMITER_HPP_
