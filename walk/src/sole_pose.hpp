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

#ifndef SOLE_POSE_HPP_
#define SOLE_POSE_HPP_

#include "biped_interfaces/msg/phase.hpp"
#include "biped_interfaces/msg/sole_poses.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"

namespace sole_pose
{
class Params;

biped_interfaces::msg::SolePoses generate(
  const sole_pose::Params & p,
  const walk_interfaces::msg::FeetTrajectoryPoint & ftp,
  const biped_interfaces::msg::Phase & phase,
  float filtered_gyro_y);

class Params
{
public:
  Params(double sole_x, double sole_y, double sole_z)
  : sole_x_(sole_x), sole_y_(sole_y), sole_z_(sole_z)
  {
  }

  Params() {}

  double sole_x_;
  double sole_y_;
  double sole_z_;
};
}  // namespace sole_pose

#endif  // SOLE_POSE_HPP_
