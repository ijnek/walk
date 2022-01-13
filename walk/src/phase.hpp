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

#ifndef PHASE_HPP_
#define PHASE_HPP_

class Phase
{
public:
  Phase(const Phase & phase)
  : value(phase.value) {}

  static const Phase LeftStance;
  static const Phase RightStance;
  static const Phase LeftSwing;
  static const Phase RightSwing;

  void invert();

  bool operator==(const Phase & p) const {return value == p.value;}
  bool operator!=(const Phase & p) const {return value != p.value;}

private:
  enum Value
  {
    LeftStancePhase = 0,
    RightStancePhase = 1,
  };

  explicit Phase(Value value)
  : value(value) {}

  Value value;
};

#endif  // PHASE_HPP_