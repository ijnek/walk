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

#include "rclcpp/rclcpp.hpp"
#include "walk/step_variable.hpp"
#include "geometry_msgs/msg/twist.hpp"

#ifndef WALK__STEP_CALCULATOR_HPP_
#define WALK__STEP_CALCULATOR_HPP_

class StepCalculator
{
public:
  StepCalculator()
  : logger(rclcpp::get_logger("StepCalculator")) {}

  geometry_msgs::msg::Twist calculateNext(
    const geometry_msgs::msg::Twist & current,
    const geometry_msgs::msg::Twist & target);

  void setParams(
    float maxForward,  // max forward velocity (m/s)
    float maxLeft,  // max side velocity (m/s)
    float maxTurn,  // max turn velocity (rad/s)
    float speedMultiplier,  // how much to multiple speed by (0.0 - 1.0)
    float footLiftAmp,  // how much to raise foot when it is highest (m)
    float maxForwardChange,  // how much forward can change in one step (m/s)
    float maxLeftChange,  // how much left can change in one step (m/s)
    float maxTurnChange);  // how much turn can change in one step (rad/s)

private:
  float maxForward;
  float maxLeft;
  float maxTurn;
  float speedMultiplier;
  float footLiftAmp;
  float maxForwardChange;
  float maxLeftChange;
  float maxTurnChange;

  void ellipsoidClamp(geometry_msgs::msg::Twist & target);
  void limitChange(
    geometry_msgs::msg::Twist & target,
    const geometry_msgs::msg::Twist & current);

  float evaluateWalkVolume(float x, float y, float z);

  rclcpp::Logger logger;
};  // namespace StepCalculator

#endif  // WALK__STEP_CALCULATOR_HPP_
