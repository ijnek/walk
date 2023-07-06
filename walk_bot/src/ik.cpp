// Copyright 2023 Kenji Brameld
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

#include "walk_bot/ik.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace walk_bot
{

IK::IK(const rclcpp::NodeOptions & node_options)
: Node("IK", node_options)
{
  sub_sole_poses_ = create_subscription<biped_interfaces::msg::SolePoses>(
    "motion/sole_poses", 1, std::bind(&IK::ikCallback, this, std::placeholders::_1));

  pub_joint_command_ =
    create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
}

void IK::ikCallback(const biped_interfaces::msg::SolePoses & msg)
{
  RCLCPP_INFO(get_logger(), "in callback!");

  sensor_msgs::msg::JointState joint_command;
  joint_command.header.stamp = now();

  float l_leg_length = std::hypot(msg.l_sole.position.x, msg.l_sole.position.y, msg.l_sole.position.z);
  float r_leg_length = std::hypot(msg.r_sole.position.x, msg.r_sole.position.y, msg.r_sole.position.z);

  float l_theta = atan2(msg.l_sole.position.y, -msg.l_sole.position.z);
  float r_theta = atan2(msg.r_sole.position.y, -msg.r_sole.position.z);

  float l_phi = std::asin(-msg.l_sole.position.x / l_leg_length);
  float r_phi = std::asin(-msg.r_sole.position.x / r_leg_length);

  tf2::Quaternion quat_l_ankle1;
  // This is an issue (order should be yaw, pitch, roll), but the joint order is currently roll, pitch.
  // Should swap joint orders such that they are pitch, roll.
  quat_l_ankle1.setEuler(0, 0, 0);

  tf2::Quaternion quat_left_sole;
  tf2::fromMsg(msg.l_sole.orientation, quat_left_sole);

  tf2::Quaternion quat_ankle = quat_left_sole * quat_l_ankle1.inverse();
  tf2::Matrix3x3 rotation_quat_ankle(quat_ankle);
  double roll, pitch, yaw;
  rotation_quat_ankle.getEulerYPR(yaw, pitch, roll);
  RCLCPP_INFO_STREAM(get_logger(), "yaw, pitch, roll: " << yaw << ", " << pitch << ", " << roll);

  joint_command.name.push_back("l_hip_roll");
  joint_command.position.push_back(l_theta);

  joint_command.name.push_back("r_hip_roll");
  joint_command.position.push_back(r_theta);

  joint_command.name.push_back("l_hip_pitch");
  joint_command.position.push_back(l_phi);

  joint_command.name.push_back("r_hip_pitch");
  joint_command.position.push_back(r_phi);

  joint_command.name.push_back("l_leg_extension");
  joint_command.position.push_back(l_leg_length);

  joint_command.name.push_back("r_leg_extension");
  joint_command.position.push_back(r_leg_length);

  joint_command.name.push_back("l_ankle_yaw");
  joint_command.position.push_back(yaw);

  joint_command.name.push_back("r_ankle_yaw");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_ankle_pitch");
  joint_command.position.push_back(pitch);

  joint_command.name.push_back("r_ankle_pitch");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_ankle_roll");
  joint_command.position.push_back(roll);

  joint_command.name.push_back("r_ankle_roll");
  joint_command.position.push_back(0.0);

  pub_joint_command_->publish(joint_command);
}

}  // namespace walk_bot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk_bot::IK)
