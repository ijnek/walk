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

#ifndef TARGET_GAIT_CALCULATOR_HPP_
#define TARGET_GAIT_CALCULATOR_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "walk_interfaces/msg/gait.hpp"

namespace target_gait_calculator
{
class Params;

walk_interfaces::msg::Gait calculate(
  const geometry_msgs::msg::Twist & target,
  const target_gait_calculator::Params & p);

class Params
{
public:
  explicit Params(float period)
  : period(period)
  {}

  float period;
};
}  // namespace target_gait_calculator

#endif  // TARGET_GAIT_CALCULATOR_HPP_
