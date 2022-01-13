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

class Phase;
class FeetTrajectoryPoint;

#ifndef FEET_TRAJECTORY_HPP_
#define FEET_TRAJECTORY_HPP_

namespace feet_trajectory
{
class Params;

std::vector<FeetTrajectoryPoint> generate(
  const feet_trajectory::Params & p, const Phase & phase,
  const FeetTrajectoryPoint & last, const FeetTrajectoryPoint & next);

class Params
{
public:
  explicit Params(float period, float dt)
  : period(period), dt(dt)
  {}

  float period;
  float dt;
};
}  // namespace feet_trajectory

#endif  // FEET_TRAJECTORY_HPP_
