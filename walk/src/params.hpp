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

#ifndef PARAMS_HPP_
#define PARAMS_HPP_

#include <vector>

#include "feet_trajectory.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sole_pose.hpp"
#include "target_gait_calculator.hpp"
#include "twist_change_limiter.hpp"
#include "twist_limiter.hpp"

namespace walk
{

class Params
{
public:
  explicit Params(rclcpp_lifecycle::LifecycleNode & node);

  feet_trajectory::Params feet_trajectory_;
  sole_pose::Params sole_pose_;
  target_gait_calculator::Params target_gait_calculator_;
  twist_change_limiter::Params twist_change_limiter_;
  twist_limiter::Params twist_limiter_;

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  rclcpp::Logger logger = rclcpp::get_logger("walk::Params");
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace walk

#endif  // PARAMS_HPP_
