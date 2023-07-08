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

#include "walk_bot/gz_bridge.hpp"

namespace walk_bot
{

GzBridge::GzBridge(const rclcpp::NodeOptions & node_options)
: Node("GzBridge", node_options)
{
  sub_joint_command_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_command", 1, std::bind(&GzBridge::jointCommandCallback, this, std::placeholders::_1));

  pub_l_hip_pitch_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_hip_pitch/0/cmd_pos");
  pub_r_hip_pitch_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_hip_pitch/0/cmd_pos");
  pub_l_hip_roll_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_hip_roll/0/cmd_pos");
  pub_r_hip_roll_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_hip_roll/0/cmd_pos");
  pub_l_leg_extension_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_leg_extension/0/cmd_pos");
  pub_r_leg_extension_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_leg_extension/0/cmd_pos");
  pub_l_ankle_yaw_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_ankle_yaw/0/cmd_pos");
  pub_r_ankle_yaw_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_ankle_yaw/0/cmd_pos");
  pub_l_ankle_pitch_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_ankle_pitch/0/cmd_pos");
  pub_r_ankle_pitch_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_ankle_pitch/0/cmd_pos");
  pub_l_ankle_roll_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/l_ankle_roll/0/cmd_pos");
  pub_r_ankle_roll_ = gz_node_.Advertise<ignition::msgs::Double>(
    "/model/walk_bot/joint/r_ankle_roll/0/cmd_pos");
}

void GzBridge::jointCommandCallback(const sensor_msgs::msg::JointState & msg)
{
  for (size_t i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "l_hip_pitch") {
      ignition::msgs::Double l_hip_pitch;
      l_hip_pitch.set_data(msg.position[i]);
      pub_l_hip_pitch_.Publish(l_hip_pitch);
    } else if (msg.name[i] == "r_hip_pitch") {
      ignition::msgs::Double r_hip_pitch;
      r_hip_pitch.set_data(msg.position[i]);
      pub_r_hip_pitch_.Publish(r_hip_pitch);
    } else if (msg.name[i] == "l_hip_roll") {
      ignition::msgs::Double l_hip_roll;
      l_hip_roll.set_data(msg.position[i]);
      pub_l_hip_roll_.Publish(l_hip_roll);
    } else if (msg.name[i] == "r_hip_roll") {
      ignition::msgs::Double r_hip_roll;
      r_hip_roll.set_data(msg.position[i]);
      pub_r_hip_roll_.Publish(r_hip_roll);
    } else if (msg.name[i] == "l_leg_extension") {
      ignition::msgs::Double l_leg_extension;
      l_leg_extension.set_data(msg.position[i]);
      pub_l_leg_extension_.Publish(l_leg_extension);
    } else if (msg.name[i] == "r_leg_extension") {
      ignition::msgs::Double r_leg_extension;
      r_leg_extension.set_data(msg.position[i]);
      pub_r_leg_extension_.Publish(r_leg_extension);
    } else if (msg.name[i] == "l_ankle_yaw") {
      ignition::msgs::Double l_ankle_yaw;
      l_ankle_yaw.set_data(msg.position[i]);
      pub_l_ankle_yaw_.Publish(l_ankle_yaw);
    } else if (msg.name[i] == "r_ankle_yaw") {
      ignition::msgs::Double r_ankle_yaw;
      r_ankle_yaw.set_data(msg.position[i]);
      pub_r_ankle_yaw_.Publish(r_ankle_yaw);
    } else if (msg.name[i] == "l_ankle_pitch") {
      ignition::msgs::Double l_ankle_pitch;
      l_ankle_pitch.set_data(msg.position[i]);
      pub_l_ankle_pitch_.Publish(l_ankle_pitch);
    } else if (msg.name[i] == "r_ankle_pitch") {
      ignition::msgs::Double r_ankle_pitch;
      r_ankle_pitch.set_data(msg.position[i]);
      pub_r_ankle_pitch_.Publish(r_ankle_pitch);
    } else if (msg.name[i] == "l_ankle_roll") {
      ignition::msgs::Double l_ankle_roll;
      l_ankle_roll.set_data(msg.position[i]);
      pub_l_ankle_roll_.Publish(l_ankle_roll);
    } else if (msg.name[i] == "r_ankle_roll") {
      ignition::msgs::Double r_ankle_roll;
      r_ankle_roll.set_data(msg.position[i]);
      pub_r_ankle_roll_.Publish(r_ankle_roll);
    }
  }
}

}  // namespace walk_bot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk_bot::GzBridge)
