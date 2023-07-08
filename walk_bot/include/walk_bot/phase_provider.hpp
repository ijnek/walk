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

#ifndef WALK_BOT__PHASE_PROVIDER_HPP_
#define WALK_BOT__PHASE_PROVIDER_HPP_

#include "biped_interfaces/msg/phase.hpp"
#include "rclcpp/node.hpp"

namespace walk_bot
{

class PhaseProvider : public rclcpp::Node
{
public:
  explicit PhaseProvider(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  double period_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr phase_pub_;

  biped_interfaces::msg::Phase phase_;

  void timerCallback();
};

}  // namespace walk_bot

#endif  // WALK_BOT__PHASE_PROVIDER_HPP_
