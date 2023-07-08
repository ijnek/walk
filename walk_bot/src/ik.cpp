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

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace walk_bot
{

// Struct storing Euler angles (intrinsic axes sequence ZYX)
struct YPR
{
  double yaw;  // Angle about Z-axis
  double pitch;  // Angle about Y-axis
  double roll;  // Angle about X-axis
};

YPR calculateYPR(const tf2::Quaternion & leg_orientation, const tf2::Quaternion & sole_orientation);
tf2::Quaternion msgToQuaternion(const geometry_msgs::msg::Quaternion & msg);
tf2::Quaternion eulerToQuaternion(double yaw, double pitch, double roll);

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
  RCLCPP_DEBUG(get_logger(), "in callback!");

  sensor_msgs::msg::JointState joint_command;
  joint_command.header.stamp = now();

  float l_leg_length = std::hypot(
    msg.l_sole.position.x, msg.l_sole.position.y,
    msg.l_sole.position.z);
  float r_leg_length = std::hypot(
    msg.r_sole.position.x, msg.r_sole.position.y,
    msg.r_sole.position.z);

  float l_theta = atan2(-msg.l_sole.position.x, -msg.l_sole.position.z);
  float r_theta = atan2(-msg.r_sole.position.x, -msg.r_sole.position.z);

  float l_phi = std::asin(msg.l_sole.position.y / l_leg_length);
  float r_phi = std::asin(msg.r_sole.position.y / r_leg_length);

  tf2::Quaternion quat_l_leg = eulerToQuaternion(0, l_theta, l_phi);
  tf2::Quaternion quat_l_sole = msgToQuaternion(msg.l_sole.orientation);
  YPR ypr_l_ankle = calculateYPR(quat_l_leg, quat_l_sole);

  tf2::Quaternion quat_r_leg = eulerToQuaternion(0, r_theta, r_phi);
  tf2::Quaternion quat_r_sole = msgToQuaternion(msg.r_sole.orientation);
  YPR ypr_r_ankle = calculateYPR(quat_r_leg, quat_r_sole);

  joint_command.name.push_back("l_hip_pitch");
  joint_command.position.push_back(l_theta);

  joint_command.name.push_back("r_hip_pitch");
  joint_command.position.push_back(r_theta);

  joint_command.name.push_back("l_hip_roll");
  joint_command.position.push_back(l_phi);

  joint_command.name.push_back("r_hip_roll");
  joint_command.position.push_back(r_phi);

  joint_command.name.push_back("l_leg_extension");
  joint_command.position.push_back(l_leg_length);

  joint_command.name.push_back("r_leg_extension");
  joint_command.position.push_back(r_leg_length);

  joint_command.name.push_back("l_ankle_yaw");
  joint_command.position.push_back(ypr_l_ankle.yaw);

  joint_command.name.push_back("r_ankle_yaw");
  joint_command.position.push_back(ypr_r_ankle.yaw);

  joint_command.name.push_back("l_ankle_pitch");
  joint_command.position.push_back(ypr_l_ankle.pitch);

  joint_command.name.push_back("r_ankle_pitch");
  joint_command.position.push_back(ypr_r_ankle.pitch);

  joint_command.name.push_back("l_ankle_roll");
  joint_command.position.push_back(ypr_l_ankle.roll);

  joint_command.name.push_back("r_ankle_roll");
  joint_command.position.push_back(ypr_r_ankle.roll);

  pub_joint_command_->publish(joint_command);
}

// Calculate Yaw, Pitch, Roll of ankle to reach from leg orientation to sole orientation
YPR calculateYPR(const tf2::Quaternion & leg_orientation, const tf2::Quaternion & sole_orientation)
{
  tf2::Quaternion quat_rotation = leg_orientation.inverse() * sole_orientation;
  tf2::Matrix3x3 mat3x3_rotation(quat_rotation);

  YPR ypr;
  mat3x3_rotation.getEulerYPR(ypr.yaw, ypr.pitch, ypr.roll);
  return ypr;
}

tf2::Quaternion msgToQuaternion(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(msg, quat);
  return quat;
}

// yaw is around Z, pitch is around Y, roll is around X
// Taken from https://math.stackexchange.com/a/2975462
//
// NOTE: This is different from tf2::Quaternion::setEuler(), where
//       yaw is around Y, pitch is around X, and roll is around Z.
tf2::Quaternion eulerToQuaternion(double yaw, double pitch, double roll)
{
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  double qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  double qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  double qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  double qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  return tf2::Quaternion(qx, qy, qz, qw);
}

}  // namespace walk_bot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk_bot::IK)
