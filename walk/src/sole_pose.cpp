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

#include "sole_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/logger.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"

namespace sole_pose
{

geometry_msgs::msg::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);

biped_interfaces::msg::SolePoses generate(
  const sole_pose::Params & p,
  const walk_interfaces::msg::FeetTrajectoryPoint & ftp)
{
  auto logger = rclcpp::get_logger("sole_pose::generate");

  // Evaluate position and angle of both feet
  float l_sole_pos_x = ftp.forward_l + p.soleX;
  float l_sole_pos_y = ftp.left_l + p.soleY;
  float l_sole_pos_z = ftp.footh_l + p.soleZ;
  float l_sole_ang_x = 0;
  float l_sole_ang_y = 0;
  float l_sole_ang_z = ftp.heading_l;

  float r_sole_pos_x = ftp.forward_r + p.soleX;
  float r_sole_pos_y = ftp.left_r - p.soleY;
  float r_sole_pos_z = ftp.footh_r + p.soleZ;
  float r_sole_ang_x = 0;
  float r_sole_ang_y = 0;
  float r_sole_ang_z = ftp.heading_r;

  RCLCPP_DEBUG(logger, "Sending IKCommand with:");
  RCLCPP_DEBUG(
    logger,
    "   LEFT - Position: (%.4f, %.4f, %.4f), Orientation: (%.4f, %.4f, %.4f)",
    l_sole_pos_x, l_sole_pos_y, l_sole_pos_z,
    l_sole_ang_x, l_sole_ang_y, l_sole_ang_z);
  RCLCPP_DEBUG(
    logger,
    "  RIGHT - Position: (%.4f, %.4f, %.4f), Orientation: (%.4f, %.4f, %.4f)",
    r_sole_pos_x, r_sole_pos_y, r_sole_pos_z,
    r_sole_ang_x, r_sole_ang_y, r_sole_ang_z);

  biped_interfaces::msg::SolePoses command;
  command.l_sole.position.x = l_sole_pos_x;
  command.l_sole.position.y = l_sole_pos_y;
  command.l_sole.position.z = l_sole_pos_z;
  command.l_sole.orientation = rpy_to_geometry_quat(
    l_sole_ang_x, l_sole_ang_y, l_sole_ang_z);
  command.r_sole.position.x = r_sole_pos_x;
  command.r_sole.position.y = r_sole_pos_y;
  command.r_sole.position.z = r_sole_pos_z;
  command.r_sole.orientation = rpy_to_geometry_quat(
    r_sole_ang_x, r_sole_ang_y, r_sole_ang_z);

  return command;
}

geometry_msgs::msg::Quaternion rpy_to_geometry_quat(
  double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion geometry_quat = tf2::toMsg(quat);
  return geometry_quat;
}

}  // namespace sole_pose
