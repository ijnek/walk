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

#ifndef STEP_STATE_HPP_
#define STEP_STATE_HPP_

#include <vector>
#include "walk_interfaces/msg/step.hpp"

class StepState
{
public:
  explicit StepState(const walk_interfaces::msg::Step step);
  bool done();
  const walk_interfaces::msg::FeetTrajectoryPoint & next();

private:
  const walk_interfaces::msg::Step step;
  unsigned i = 0;
};

#endif  // STEP_STATE_HPP_
