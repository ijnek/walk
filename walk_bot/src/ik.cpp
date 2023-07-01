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

  joint_command.name.push_back("l_hip_roll");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("r_hip_roll");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_hip_pitch");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("r_hip_pitch");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_hip_yaw");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("r_hip_yaw");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_leg_extension");
  joint_command.position.push_back(-0.4);  // -0.4

  joint_command.name.push_back("r_leg_extension");
  joint_command.position.push_back(-0.4);  // -0.4

  joint_command.name.push_back("l_ankle_roll");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("r_ankle_roll");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("l_ankle_pitch");
  joint_command.position.push_back(0.0);

  joint_command.name.push_back("r_ankle_pitch");
  joint_command.position.push_back(0.0);

  pub_joint_command_->publish(joint_command);
}

}  // namespace walk_bot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk_bot::IK)
