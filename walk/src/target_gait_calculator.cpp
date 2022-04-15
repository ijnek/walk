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
walk_interfaces::msg::Gait calculate(
  const geometry_msgs::msg::Twist & target,
  const target_gait_calculator::Params & p)
{
  walk_interfaces::msg::Gait gait;

  // Forward
  gait.left_stance_phase_aim.forward_l = -target.linear.x * p.period / 2;
  gait.left_stance_phase_aim.forward_r = target.linear.x * p.period / 2;
  gait.right_stance_phase_aim.forward_l = target.linear.x * p.period / 2;
  gait.right_stance_phase_aim.forward_r = -target.linear.x * p.period / 2;

  // Left
  if (target.linear.y > 0) {  // Moving left
    gait.left_stance_phase_aim.left_l = 0;
    gait.left_stance_phase_aim.left_r = 0;
    gait.right_stance_phase_aim.left_l = target.linear.y * p.period;
    gait.right_stance_phase_aim.left_r = -target.linear.y * p.period;
  } else {  // Moving right
    gait.left_stance_phase_aim.left_l = -target.linear.y * p.period;
    gait.left_stance_phase_aim.left_r = target.linear.y * p.period;
    gait.right_stance_phase_aim.left_l = 0;
    gait.right_stance_phase_aim.left_r = 0;
  }

  // Heading
  if (target.angular.z > 0) {  // Turning left
    gait.left_stance_phase_aim.heading_l = -target.angular.z * p.period * 0.2;
    gait.left_stance_phase_aim.heading_r = target.angular.z * p.period * 0.2;
    gait.right_stance_phase_aim.heading_l = target.angular.z * p.period * 0.8;
    gait.right_stance_phase_aim.heading_r = -target.angular.z * p.period * 0.8;
  } else {  // Turning right (NOTE: target.angular.z IS NEGATIVE)
    gait.left_stance_phase_aim.heading_l = -target.angular.z * p.period * 0.8;
    gait.left_stance_phase_aim.heading_r = target.angular.z * p.period * 0.8;
    gait.right_stance_phase_aim.heading_l = target.angular.z * p.period * 0.2;
    gait.right_stance_phase_aim.heading_r = -target.angular.z * p.period * 0.2;
  }

  return gait;
}
}  // namespace target_gait_calculator
