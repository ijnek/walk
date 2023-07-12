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

#include "walk_bot/phase_provider.hpp"

namespace walk_bot
{

PhaseProvider::PhaseProvider(const rclcpp::NodeOptions & options)
: Node("PhaseProvider", options)
{
  declare_parameter("period", 0.5);
  rclcpp::Parameter period_param = get_parameter("period");
  period_ = period_param.as_double();
  RCLCPP_DEBUG_STREAM(get_logger(), "PERIOD: " << period_);

  phase_.phase = biped_interfaces::msg::Phase::LEFT_STANCE;

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_),
    std::bind(&PhaseProvider::timerCallback, this));

  phase_pub_ = create_publisher<biped_interfaces::msg::Phase>("phase", 10);
}

void PhaseProvider::timerCallback()
{
  // Change phase
  if (phase_.phase == biped_interfaces::msg::Phase::LEFT_STANCE) {
    phase_.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
  } else if (phase_.phase == biped_interfaces::msg::Phase::RIGHT_STANCE) {
    phase_.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
  }

  // Publish phase
  phase_pub_->publish(phase_);
}

}  // namespace walk_bot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk_bot::PhaseProvider)
