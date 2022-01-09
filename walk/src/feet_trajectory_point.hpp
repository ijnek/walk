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

#ifndef FEET_TRAJECTORY_POINT_HPP_
#define FEET_TRAJECTORY_POINT_HPP_

struct FeetTrajectoryPoint
{
  FeetTrajectoryPoint(
    float forwardL, float forwardR, float leftL, float leftR, float headingL, float headingR)
  : forwardL(forwardL), forwardR(forwardR), leftL(leftL), leftR(leftR), headingL(headingL),
    headingR(headingR) {}

  float forwardL = 0.0;  // m
  float forwardR = 0.0;  // m
  float leftL = 0.0;  // m
  float leftR = 0.0;  // m
  float headingL = 0.0;  // rad
  float headingR = 0.0;   // rad (MAKE SURE THIS EQUALS NEGATIVE OF headingL)
};

#endif  // FEET_TRAJECTORY_POINT_HPP_
