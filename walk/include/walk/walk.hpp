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
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
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
  void generateCommand();
  void walk(const geometry_msgs::msg::Twist & target);
  void notifyPhase(const biped_interfaces::msg::Phase & phase);
  // void reset();

private:
  // TODO(ijnek): Replace this timer with an input signal
  rclcpp::TimerBase::SharedPtr generateCommand_timer_;

  // Twist is a subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr sub_phase;

  // Abort is a service
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_abort;

  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr pub_sole_poses;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_current_twist;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_to_step;

  // Debug publishers
  rclcpp::Publisher<walk_interfaces::msg::Gait>::SharedPtr pub_gait;
  rclcpp::Publisher<walk_interfaces::msg::Step>::SharedPtr pub_step;

  void generateCommand_timer_callback();
  void phase_callback(const biped_interfaces::msg::Phase::SharedPtr msg);
  void target_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // void abort(
  //   const std::shared_ptr<std_srvs::srv::Empty::Request>,
  //   std::shared_ptr<std_srvs::srv::Empty::Response>);

  std::unique_ptr<twist_limiter::Params> twistLimiterParams;
  std::unique_ptr<twist_change_limiter::Params> twistChangeLimiterParams;
  std::unique_ptr<sole_pose::Params> solePoseParams;
  std::unique_ptr<target_gait_calculator::Params> targetGaitCalculatorParams;
  std::unique_ptr<feet_trajectory::Params> feetTrajectoryParams;

  std::unique_ptr<biped_interfaces::msg::Phase> phase;
  std::unique_ptr<walk_interfaces::msg::FeetTrajectoryPoint> ftpCurrent;
  std::unique_ptr<geometry_msgs::msg::Twist> currTwist;

  // Following members must be stored and loaded in a thread-safe manner
  std::shared_ptr<geometry_msgs::msg::Twist> targetTwist;
  std::shared_ptr<walk_interfaces::msg::Step> step;

  std::shared_ptr<StepState> stepState;
};

}  // namespace walk

#endif  // WALK__WALK_HPP_
