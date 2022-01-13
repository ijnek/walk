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

#ifndef STEP_HPP_
#define STEP_HPP_

#include <vector>
#include "./feet_trajectory_point.hpp"

class Step
{
public:
  explicit Step(
    float period, float dt, const Phase & phase,
    const FeetTrajectoryPoint & last,
    const FeetTrajectoryPoint & next);
  bool done();
  const FeetTrajectoryPoint & next();

private:
  const std::vector<FeetTrajectoryPoint> points;
  unsigned i = 0;
};

#endif  // STEP_HPP_
