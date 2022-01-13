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

#ifndef ANKLE_POSE_GENERATOR_HPP_
#define ANKLE_POSE_GENERATOR_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "biped_interfaces/msg/ankle_poses.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/logger.hpp"
#include "./gait.hpp"

class AnklePoseGenerator
{
public:
  explicit AnklePoseGenerator(float ankleX = 0.0, float ankleY = 0.0, float ankleZ = 0.0);
  biped_interfaces::msg::AnklePoses generate(const FeetTrajectoryPoint & ftp) const;

private:
  geometry_msgs::msg::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw) const;

  float ankleX = 0.0;
  float ankleY = 0.0;
  float ankleZ = 0.0;
  rclcpp::Logger logger;
};

#endif  // ANKLE_POSE_GENERATOR_HPP_
