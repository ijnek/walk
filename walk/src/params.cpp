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

#include "params.hpp"

namespace walk
{

using rclcpp::ParameterValue;

Params::Params(rclcpp::Node & node)
: node_{node}
{
  double max_forward = node_.declare_parameter("max_forward", 0.3);  // max forward velocity (m/s)
  double max_left = node_.declare_parameter("max_left", 0.2);  // max side velocity (m/s)
  double max_turn = node_.declare_parameter("max_turn", 2.0);  // max turn velocity (rad/s)
  double speed_multiplier = node_.declare_parameter("speed_multiplier", 1.0);  // how much to multiple speed by (0.0 - 1.0)  // NOLINT
  double foot_lift_amp = node_.declare_parameter("foot_lift_amp", 0.012);  // how much to raise foot when it is highest (m)  // NOLINT
  double period = node_.declare_parameter("period", 0.25);  // time taken for one step, (s)
  double dt = node_.declare_parameter("dt", 0.01);  // time taken between each generateCommand call (s)  // NOLINT
  double sole_x = node_.declare_parameter("sole_x", -0.022);  // x coordinate of sole from hip when standing (m)  // NOLINT
  double sole_y = node_.declare_parameter("sole_y", 0.05);  // y coordinate of sole from hip when standing (m)  // NOLINT
  double sole_z = node_.declare_parameter("sole_z", -0.315);  // z coordinate of sole from hip when standing (m)  // NOLINT
  double max_forward_change = node_.declare_parameter("max_forward_change", 0.06);  // how much forward can change in one step (m/s)  // NOLINT
  double max_left_change = node_.declare_parameter("max_left_change", 0.1);  // how much left can change in one step (m/s)  // NOLINT
  double max_turn_change = node_.declare_parameter("max_turn_change", 1.0);  // how much turn can change in one step (rad/s)  // NOLINT
  double footh_forward_multiplier = node_.declare_parameter("footh_forward_multiplier", 0.1);  // how much extra height to add to the swing foot, as a multiplier of the magnitude of the step in the forward/backward direction.  // NOLINT
  double footh_left_multiplier = node_.declare_parameter("footh_left_multiplier", 0.3);  // how much extra height to add to the swing foot, as a multiplier of the magnitude of the step in the left/right direction.  // NOLINT

  RCLCPP_DEBUG(logger, "Parameters: ");
  RCLCPP_DEBUG(logger, "  max_forward : %f", max_forward);
  RCLCPP_DEBUG(logger, "  max_left : %f", max_left);
  RCLCPP_DEBUG(logger, "  max_turn : %f", max_turn);
  RCLCPP_DEBUG(logger, "  speed_multiplier : %f", speed_multiplier);
  RCLCPP_DEBUG(logger, "  foot_lift_amp : %f", foot_lift_amp);
  RCLCPP_DEBUG(logger, "  period : %f", period);
  RCLCPP_DEBUG(logger, "  dt : %f", dt);
  RCLCPP_DEBUG(logger, "  sole_x : %f", sole_x);
  RCLCPP_DEBUG(logger, "  sole_y : %f", sole_y);
  RCLCPP_DEBUG(logger, "  sole_z : %f", sole_z);
  RCLCPP_DEBUG(logger, "  max_forward_change : %f", max_forward_change);
  RCLCPP_DEBUG(logger, "  max_left_change : %f", max_left_change);
  RCLCPP_DEBUG(logger, "  max_turn_change : %f", max_turn_change);
  RCLCPP_DEBUG(logger, "  footh_forward_multiplier : %f", footh_forward_multiplier);
  RCLCPP_DEBUG(logger, "  footh_left_multiplier : %f", footh_left_multiplier);

  feet_trajectory_ = feet_trajectory::Params(
    foot_lift_amp, period, dt, footh_forward_multiplier, footh_left_multiplier);
  sole_pose_ = sole_pose::Params(sole_x, sole_y, sole_z);
  target_gait_calculator_ = target_gait_calculator::Params(period);
  twist_change_limiter_ = twist_change_limiter::Params(
    max_forward_change, max_left_change, max_turn_change);
  twist_limiter_ = twist_limiter::Params(
    max_forward, max_left, max_turn, speed_multiplier);

  // Register parameter change callback
  on_set_parameters_callback_handle_ = node_.add_on_set_parameters_callback(
    std::bind(&Params::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Params::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(logger, "Parameter updated:");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & param: parameters) {
    RCLCPP_DEBUG_STREAM(
      logger,
      "- " << param.get_name() << " (" << param.get_type_name() << ")" << " = " <<
        param.value_to_string());

    auto name = param.get_name();
    if (name == "max_forward") {
      twist_limiter_.max_forward_ = param.as_double();
    } else if (name == "max_left") {
      twist_limiter_.max_left_ = param.as_double();
    } else if (name == "max_turn") {
      twist_limiter_.max_turn_ = param.as_double();
    } else if (name == "speed_multiplier") {
      twist_limiter_.speed_multiplier_ = param.as_double();
    } else if (name == "foot_lift_amp") {
      feet_trajectory_.foot_lift_amp_ = param.as_double();
    } else if (name == "period") {
      feet_trajectory_.period_ = param.as_double();
    } else if (name == "dt") {
      feet_trajectory_.dt_ = param.as_double();
    } else if (name == "sole_x") {
      sole_pose_.sole_x_ = param.as_double();
    } else if (name == "sole_y") {
      sole_pose_.sole_y_ = param.as_double();
    } else if (name == "sole_z") {
      sole_pose_.sole_z_ = param.as_double();
    } else if (name == "max_forward_change") {
      twist_change_limiter_.max_forward_change_ = param.as_double();
    } else if (name == "max_left_change") {
      twist_change_limiter_.max_left_change_ = param.as_double();
    } else if (name == "max_turn_change") {
      twist_change_limiter_.max_turn_change_ = param.as_double();
    } else if (name == "footh_forward_multiplier") {
      feet_trajectory_.footh_forward_multiplier_ = param.as_double();
    } else if (name == "footh_left_multiplier") {
      feet_trajectory_.footh_left_multiplier_ = param.as_double();
    }
  }

  return result;
}


}  // namespace walk
