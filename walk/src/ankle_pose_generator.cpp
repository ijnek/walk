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

#include "./ankle_pose_generator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

AnklePoseGenerator::AnklePoseGenerator(float ankleX, float ankleY, float ankleZ)
: ankleX(ankleX), ankleY(ankleY), ankleZ(ankleZ), logger(rclcpp::get_logger("AnklePoseGenerator"))
{
}


biped_interfaces::msg::AnklePoses AnklePoseGenerator::generate(const FeetTrajectoryPoint & ftp)
const
{
  // Evaluate position and angle of both feet
  float l_ankle_pos_x = ftp.forwardL + ankleX;
  float l_ankle_pos_y = ftp.leftL + ankleY;
  float l_ankle_pos_z = ftp.foothL + ankleZ;
  float l_ankle_ang_x = 0;
  float l_ankle_ang_y = 0;
  float l_ankle_ang_z = ftp.headingL;

  float r_ankle_pos_x = ftp.forwardR + ankleX;
  float r_ankle_pos_y = ftp.leftR - ankleY;
  float r_ankle_pos_z = ftp.foothR + ankleZ;
  float r_ankle_ang_x = 0;
  float r_ankle_ang_y = 0;
  float r_ankle_ang_z = ftp.headingR;

  RCLCPP_DEBUG(logger, "Sending IKCommand with:");
  RCLCPP_DEBUG(
    logger,
    "   LEFT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    l_ankle_pos_x, l_ankle_pos_y, l_ankle_pos_z,
    l_ankle_ang_x, l_ankle_ang_y, l_ankle_ang_z);
  RCLCPP_DEBUG(
    logger,
    "  RIGHT - Position: (%.4f, %.4f, %.4f), Rotation: (%.4f, %.4f, %.4f)",
    r_ankle_pos_x, r_ankle_pos_y, r_ankle_pos_z,
    r_ankle_ang_x, r_ankle_ang_y, r_ankle_ang_z);

  biped_interfaces::msg::AnklePoses command;
  command.l_ankle.position.x = l_ankle_pos_x;
  command.l_ankle.position.y = l_ankle_pos_y;
  command.l_ankle.position.z = l_ankle_pos_z;
  command.l_ankle.orientation = rpy_to_geometry_quat(
    l_ankle_ang_x, l_ankle_ang_y, l_ankle_ang_z);
  command.r_ankle.position.x = r_ankle_pos_x;
  command.r_ankle.position.y = r_ankle_pos_y;
  command.r_ankle.position.z = r_ankle_pos_z;
  command.r_ankle.orientation = rpy_to_geometry_quat(
    r_ankle_ang_x, r_ankle_ang_y, r_ankle_ang_z);

  return command;
}

geometry_msgs::msg::Quaternion AnklePoseGenerator::rpy_to_geometry_quat(
  double roll, double pitch, double yaw) const
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion geometry_quat = tf2::toMsg(quat);
  return geometry_quat;
}
