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

class FeetTrajectoryPoint
{
public:
  FeetTrajectoryPoint() {}

  FeetTrajectoryPoint(
    float forwardL, float forwardR, float leftL, float leftR, float headingL, float headingR,
    float foothL, float foothR)
  : forwardL(forwardL), forwardR(forwardR), leftL(leftL), leftR(leftR), headingL(headingL),
    headingR(headingR), foothL(foothL), foothR(foothR) {}

  FeetTrajectoryPoint(const FeetTrajectoryPoint & ftp)
  : forwardL(ftp.forwardL), forwardR(ftp.forwardR), leftL(ftp.leftL), leftR(ftp.leftR),
    headingL(ftp.headingL), headingR(ftp.headingR), foothL(ftp.foothL), foothR(ftp.foothR) {}

  FeetTrajectoryPoint & operator=(const FeetTrajectoryPoint & ftp)
  {
    forwardL = ftp.forwardL;
    forwardR = ftp.forwardR;
    leftL = ftp.leftL;
    leftR = ftp.leftR;
    headingL = ftp.headingL;
    headingR = ftp.headingR;
    foothL = ftp.foothL;
    foothR = ftp.foothR;
    return *this;
  }

  float forwardL = 0.0;  // m
  float forwardR = 0.0;  // m
  float leftL = 0.0;  // m
  float leftR = 0.0;  // m
  float headingL = 0.0;  // rad
  float headingR = 0.0;   // rad (MAKE SURE THIS EQUALS NEGATIVE OF headingL)
  float foothL = 0.0;  // m
  float foothR = 0.0;  // m
};

#endif  // FEET_TRAJECTORY_POINT_HPP_
