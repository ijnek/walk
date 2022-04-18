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
#include "feet_trajectory.hpp"
#include "maths_functions.hpp"

namespace feet_trajectory
{
walk_interfaces::msg::Step generate(
  const feet_trajectory::Params & p, const biped_interfaces::msg::Phase & phase,
  const walk_interfaces::msg::FeetTrajectoryPoint & last,
  const walk_interfaces::msg::FeetTrajectoryPoint & next)
{
  float maxFootHeight = p.footLiftAmp;
  float period = p.period;
  float dt = p.dt;

  walk_interfaces::msg::Step step;
  step.points.reserve(period / dt);

  int i = 0;
  for (float t = 0.0; t < period; t += dt, ++i) {
    float forward_l = 0.0;
    float forward_r = 0.0;

    if (phase.phase == phase.RIGHT_SWING) {
      forward_l = last.forward_l +
        (next.forward_l - last.forward_l) * linearStep(t, period);
      forward_r = last.forward_r +
        (next.forward_r - last.forward_r) * parabolicStep(dt, t, period, 0);
    } else {
      forward_l = last.forward_l +
        (next.forward_l - last.forward_l) * parabolicStep(dt, t, period, 0);
      forward_r = last.forward_r +
        (next.forward_r - last.forward_r) * linearStep(t, period);
    }

    float left_l = last.left_l + (next.left_l - last.left_l) * parabolicStep(dt, t, period, 0.2);
    float left_r = last.left_r + (next.left_r - last.left_r) * parabolicStep(dt, t, period, 0.2);

    float heading_l = last.heading_l +
      (next.heading_l - last.heading_l) * parabolicStep(dt, t, period, 0.0);
    float heading_r = last.heading_r +
      (next.heading_r - last.heading_r) * parabolicStep(dt, t, period, 0.0);

    float footh_l = 0;
    float footh_r = 0;

    if (phase.phase == phase.RIGHT_SWING) {
      footh_r = maxFootHeight * parabolicReturnMod(t / period);
    } else {
      footh_l = maxFootHeight * parabolicReturnMod(t / period);
    }

    walk_interfaces::msg::FeetTrajectoryPoint ftp;
    ftp.forward_l = forward_l;
    ftp.forward_r = forward_r;
    ftp.left_l = left_l;
    ftp.left_r = left_r;
    ftp.heading_l = heading_l;
    ftp.heading_r = heading_r;
    ftp.footh_l = footh_l;
    ftp.footh_r = footh_r;
    step.points.push_back(ftp);
  }

  return step;
}
}  // namespace feet_trajectory
