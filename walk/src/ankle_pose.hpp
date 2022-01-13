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

#ifndef ANKLE_POSE_HPP_
#define ANKLE_POSE_HPP_

#include "biped_interfaces/msg/ankle_poses.hpp"

class FeetTrajectoryPoint;

namespace ankle_pose
{
class Params;

biped_interfaces::msg::AnklePoses generate(
  const ankle_pose::Params & p,
  const FeetTrajectoryPoint & ftp);

class Params
{
public:
  Params(float ankleX, float ankleY, float ankleZ)
  : ankleX(ankleX), ankleY(ankleY), ankleZ(ankleZ)
  {
  }

  float ankleX;
  float ankleY;
  float ankleZ;
};
}  // namespace ankle_pose

#endif  // ANKLE_POSE_HPP_
