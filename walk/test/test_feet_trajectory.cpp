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

#include <vector>
#include "gtest/gtest.h"
#include "../src/feet_trajectory.hpp"
#include "../src/feet_trajectory_point.hpp"
#include "walk/phase.hpp"

TEST(TestFeetTrajectory, TestSmoothSteps)
{
  // In this test, we ensure that all consecutive points in the trajectory are within
  // a small margin, and there are no large changes in value.
  float period = 0.3;
  float dt = 0.01;
  FeetTrajectoryPoint init(0, 0, 0, 0, 0, 0, 0, 0);
  FeetTrajectoryPoint step1(0.02, -0.02, 0.01, -0.01, 0.6, -0.6, 0, 0);
  FeetTrajectoryPoint step2(-0.04, -0.04, 0, 0, -0.1, 0.1, 0, 0);

  std::vector<FeetTrajectoryPoint> pointsStep1 =
    feet_trajectory::generate(period, dt, Phase::RightSwing, init, step1);

  std::vector<FeetTrajectoryPoint> pointsStep2 =
    feet_trajectory::generate(period, dt, Phase::LeftSwing, step1, step2);

  std::vector<FeetTrajectoryPoint> points;
  points.insert(points.end(), pointsStep1.begin(), pointsStep1.end());
  points.insert(points.end(), pointsStep2.begin(), pointsStep2.end());

  for (unsigned i = 0; i < points.size() - 1; ++i) {
    FeetTrajectoryPoint & a = points.at(i);
    FeetTrajectoryPoint & b = points.at(i + 1);

    EXPECT_NEAR(a.forwardL, b.forwardL, 0.01);
    EXPECT_NEAR(a.forwardR, b.forwardR, 0.01);
    EXPECT_NEAR(a.leftL, b.leftL, 0.01);
    EXPECT_NEAR(a.leftR, b.leftR, 0.01);
    EXPECT_NEAR(a.headingL, b.headingL, 0.1);
    EXPECT_NEAR(a.headingR, b.headingR, 0.1);
  }
}
