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

#ifndef STEP_GENERATOR_HPP_
#define STEP_GENERATOR_HPP_

namespace step_generator
{
class Params;

Step generate(
  const step_generator::Params & p, const Phase & phase,
  const walk_interfaces::msg::FeetTrajectoryPoint & last,
  const walk_interfaces::msg::FeetTrajectoryPoint & next);

class Params
{
public:
  explicit Params(float period, float dt)
  : period(period), dt(dt)
  {}

  float period;
  float dt;
};
}  // namespace step_generator

#endif  // STEP_GENERATOR_HPP_
