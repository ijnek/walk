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

#include "target_gait_calculator.hpp"

namespace target_gait_calculator
{
Gait calculate(
  const geometry_msgs::msg::Twist & target,
  const target_gait_calculator::Params & p)
{
  Gait gait;

  // Forward
  gait.leftStancePhaseAim.forwardL = -target.linear.x * p.period / 2;
  gait.leftStancePhaseAim.forwardR = target.linear.x * p.period / 2;
  gait.rightStancePhaseAim.forwardL = target.linear.x * p.period / 2;
  gait.rightStancePhaseAim.forwardR = -target.linear.x * p.period / 2;

  // Left
  if (target.linear.y > 0) {  // Moving left
    gait.leftStancePhaseAim.leftL = 0;
    gait.leftStancePhaseAim.leftR = 0;
    gait.rightStancePhaseAim.leftL = target.linear.y * p.period;
    gait.rightStancePhaseAim.leftR = -target.linear.y * p.period;
  } else {  // Moving right
    gait.leftStancePhaseAim.leftL = -target.linear.y * p.period;
    gait.leftStancePhaseAim.leftR = target.linear.y * p.period;
    gait.rightStancePhaseAim.leftL = 0;
    gait.rightStancePhaseAim.leftR = 0;
  }

  // Heading
  if (target.angular.z > 0) {  // Turning left
    gait.leftStancePhaseAim.headingL = -target.angular.z * p.period * 0.2;
    gait.leftStancePhaseAim.headingR = target.angular.z * p.period * 0.2;
    gait.rightStancePhaseAim.headingL = target.angular.z * p.period * 0.8;
    gait.rightStancePhaseAim.headingR = -target.angular.z * p.period * 0.8;
  } else {  // Turning right (NOTE: target.angular.z IS NEGATIVE)
    gait.leftStancePhaseAim.headingL = -target.angular.z * p.period * 0.8;
    gait.leftStancePhaseAim.headingR = target.angular.z * p.period * 0.8;
    gait.rightStancePhaseAim.headingL = target.angular.z * p.period * 0.2;
    gait.rightStancePhaseAim.headingR = -target.angular.z * p.period * 0.2;
  }

  return gait;
}
}  // namespace target_gait_calculator
