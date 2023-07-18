// Copyright 2021 Kenji Brameld
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

#ifndef WALK__WALK_HPP_
#define WALK__WALK_HPP_

#include <map>
#include <memory>

#include "biped_interfaces/msg/sole_poses.hpp"
#include "biped_interfaces/msg/phase.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "walk_interfaces/action/crouch.hpp"
#include "walk_interfaces/action/stand.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"
#include "walk_interfaces/msg/gait.hpp"
#include "walk_interfaces/msg/step.hpp"

namespace twist_limiter {class Params;}
namespace twist_change_limiter {class Params;}
namespace sole_pose {class Params;}
namespace target_gait_calculator {class Params;}
namespace feet_trajectory {class Params;}
class Step;
class StepState;

namespace walk
{

class Walk : public rclcpp::Node
{
public:
  using WalkGoal = walk_interfaces::action::Crouch::Goal;
  using CrouchGoalHandle = rclcpp_action::ServerGoalHandle<walk_interfaces::action::Crouch>;

  explicit Walk(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~Walk();

private:
  // TODO(ijnek): Replace this timer with an input signal
  rclcpp::TimerBase::SharedPtr generate_command_timer_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr sub_phase_;

  // Publishers
  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr pub_sole_poses_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_current_twist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_to_step_;

  // Debug publishers
  rclcpp::Publisher<walk_interfaces::msg::Gait>::SharedPtr pub_gait_;
  rclcpp::Publisher<walk_interfaces::msg::Step>::SharedPtr pub_step_;

  void walk(const geometry_msgs::msg::Twist & commanded_twist);
  void notifyPhase(const biped_interfaces::msg::Phase & phase);
  void generateCommand();
  void phaseCallback(const biped_interfaces::msg::Phase::SharedPtr msg);
  void targetCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  // Parameters
  std::unique_ptr<twist_limiter::Params> twist_limiter_params_;
  std::unique_ptr<twist_change_limiter::Params> twist_change_limiter_params_;
  std::unique_ptr<sole_pose::Params> sole_pose_params_;
  std::unique_ptr<target_gait_calculator::Params> target_gait_calculator_params_;
  std::unique_ptr<feet_trajectory::Params> feet_trajectory_params_;

  // State variables
  std::unique_ptr<biped_interfaces::msg::Phase> phase_;
  std::unique_ptr<walk_interfaces::msg::FeetTrajectoryPoint> ftp_current_;
  std::unique_ptr<geometry_msgs::msg::Twist> curr_twist_;

  // Following members must be stored and loaded in a thread-safe manner
  std::shared_ptr<geometry_msgs::msg::Twist> target_twist_;
  std::shared_ptr<walk_interfaces::msg::Step> step_;
  std::shared_ptr<StepState> step_state_;

  // Parameter Change Callback
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

}  // namespace walk

#endif  // WALK__WALK_HPP_
