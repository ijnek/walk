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

#include <vector>
#include "biped_interfaces/msg/phase.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"
#include "walk_interfaces/msg/step.hpp"

#ifndef FEET_TRAJECTORY_HPP_
#define FEET_TRAJECTORY_HPP_

namespace feet_trajectory
{
class Params;

walk_interfaces::msg::Step generate(
  const feet_trajectory::Params & p, const biped_interfaces::msg::Phase & phase,
  const walk_interfaces::msg::FeetTrajectoryPoint & last,
  const walk_interfaces::msg::FeetTrajectoryPoint & next);

class Params
{
public:
  explicit Params(
    float foot_lift_amp, float period, float dt, float footh_forward_multiplier,
    float footh_left_multiplier)
  : foot_lift_amp_(foot_lift_amp), period_(period), dt_(dt),
    footh_forward_multiplier_(footh_forward_multiplier),
    footh_left_multiplier_(footh_left_multiplier)
  {}

  float foot_lift_amp_;
  float period_;
  float dt_;
  float footh_forward_multiplier_;
  float footh_left_multiplier_;
};
}  // namespace feet_trajectory

#endif  // FEET_TRAJECTORY_HPP_
