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
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "biped_interfaces/msg/ankle_poses.hpp"

namespace twist_limiter {class Params;}
class Step;
class FeetTrajectoryPoint;
class Phase;
class AnklePoseGenerator;


class Walk
{
public:
  Walk(
    std::function<void(const biped_interfaces::msg::AnklePoses &)> send_ankle_poses,
    std::function<void(const geometry_msgs::msg::Twist &)> report_current_twist,
    std::function<void(const std_msgs::msg::Bool &)> report_ready_to_step);
  virtual ~Walk();
  void setParams(
    float maxForward,  // max forward velocity (m/s)
    float maxLeft,  // max side velocity (m/s)
    float maxTurn,  // max turn velocity (rad/s)
    float speedMultiplier,  // how much to multiple speed by (0.0 - 1.0)
    float footLiftAmp,  // how much to raise foot when it is highest (m)
    float period,  // time taken for one step, (s)
    float ankleX,  // x coordinate of ankle from hip when standing (m)
    float ankleY,  // y coordinate of ankle from hip when standing (m)
    float ankleZ,  // z coordinate of ankle from hip when standing (m)
    float maxForwardChange,  // how much forward can change in one step (m/s)
    float maxLeftChange,  // how much left can change in one step (m/s)
    float maxTurnChange);  // how much turn can change in one step (rad/s)
  void generateCommand();
  void walk(const geometry_msgs::msg::Twist & target);
  void notifyPhase(const Phase & phase);
  void reset();

private:
  const std::function<void(const biped_interfaces::msg::AnklePoses &)> send_ankle_poses;
  const std::function<void(const geometry_msgs::msg::Twist &)> report_current_twist;
  const std::function<void(const std_msgs::msg::Bool &)> report_ready_to_step;

  std::unique_ptr<geometry_msgs::msg::Twist> currTwist;

  std::unique_ptr<twist_limiter::Params> twistLimiterParams;
  std::unique_ptr<AnklePoseGenerator> anklePoseGenerator;

  bool firstMsg = true;
  float period = 0.0;
  float footLiftAmp = 0.0;

  rclcpp::Logger logger;

  std::unique_ptr<Phase> phase;
  std::unique_ptr<Step> step;
  std::unique_ptr<FeetTrajectoryPoint> last;

  std::shared_ptr<geometry_msgs::msg::Twist> target;
  std::shared_ptr<Phase> notifiedPhase;
};

#endif  // WALK__WALK_HPP_
