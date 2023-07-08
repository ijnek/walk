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

#ifndef WALK_BOT__IK_HPP_
#define WALK_BOT__IK_HPP_

#include "biped_interfaces/msg/sole_poses.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace walk_bot
{

class IK : public rclcpp::Node
{
public:
  explicit IK(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  rclcpp::Subscription<biped_interfaces::msg::SolePoses>::SharedPtr sub_sole_poses_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_command_;

  void ikCallback(const biped_interfaces::msg::SolePoses & msg);
};

}  // namespace walk_bot

#endif  // WALK_BOT__IK_HPP_
