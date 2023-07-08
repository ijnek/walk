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

#ifndef WALK_BOT__GZ_BRIDGE_HPP_
#define WALK_BOT__GZ_BRIDGE_HPP_

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ignition/transport/Node.hh"

namespace walk_bot
{

class GzBridge : public rclcpp::Node
{
public:
  explicit GzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_command_;

  ignition::transport::Node gz_node_;
  ignition::transport::Node::Publisher pub_l_hip_pitch_;
  ignition::transport::Node::Publisher pub_r_hip_pitch_;
  ignition::transport::Node::Publisher pub_l_hip_roll_;
  ignition::transport::Node::Publisher pub_r_hip_roll_;
  ignition::transport::Node::Publisher pub_l_leg_extension_;
  ignition::transport::Node::Publisher pub_r_leg_extension_;
  ignition::transport::Node::Publisher pub_l_ankle_yaw_;
  ignition::transport::Node::Publisher pub_r_ankle_yaw_;
  ignition::transport::Node::Publisher pub_l_ankle_pitch_;
  ignition::transport::Node::Publisher pub_r_ankle_pitch_;
  ignition::transport::Node::Publisher pub_l_ankle_roll_;
  ignition::transport::Node::Publisher pub_r_ankle_roll_;

  void jointCommandCallback(const sensor_msgs::msg::JointState & msg);
};

}  // namespace walk_bot

#endif  // WALK_BOT__GZ_BRIDGE_HPP_
