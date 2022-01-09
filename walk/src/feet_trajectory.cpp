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
#include "./feet_trajectory.hpp"
#include "./maths_functions.hpp"


namespace feet_trajectory
{
std::vector<FeetTrajectoryPoint> generate(
  float period, float dt, bool isLeftStancePhase,
  const FeetTrajectoryPoint & last,
  const FeetTrajectoryPoint & next)
{
  std::vector<FeetTrajectoryPoint> points;
  points.reserve(period / dt);

  int i = 0;
  for (float t = 0.0; t < period; t += dt, ++i) {
    float forwardL = 0.0;
    float forwardR = 0.0;

    if (isLeftStancePhase) {
      // Stance Foot = Left, Swing Foot = Right
      forwardL = last.forwardL +
        (next.forwardL - last.forwardL) * linearStep(t, period);
      forwardR = last.forwardR +
        (next.forwardR - last.forwardR) * parabolicStep(dt, t, period, 0);
    } else {
      // Stance Foot = Right, Swing Foot = Left
      forwardL = last.forwardL +
        (next.forwardL - last.forwardL) * parabolicStep(dt, t, period, 0);
      forwardR = last.forwardR +
        (next.forwardR - last.forwardR) * linearStep(t, period);
    }

    float leftL = last.leftL + (next.leftL - last.leftL) * parabolicStep(dt, t, period, 0.2);
    float leftR = last.leftR + (next.leftR - last.leftR) * parabolicStep(dt, t, period, 0.2);

    float headingL = last.headingL +
      (next.headingL - last.headingL) * parabolicStep(dt, t, period, 0.0);
    float headingR = last.headingR +
      (next.headingR - last.headingR) * parabolicStep(dt, t, period, 0.0);

    points.emplace_back(forwardL, forwardR, leftL, leftR, headingL, headingR);
  }

  return points;
}
}  // namespace feet_trajectory
