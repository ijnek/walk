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

#include "maths_functions.hpp"

float parabolicStep(float dt, float time, float period, float deadTimeFraction)
{
  // normalised [0,1] step up
  float deadTime = period * deadTimeFraction / 2;
  if (time < deadTime + dt / 2) {
    return 0;
  }
  if (time > period - deadTime - dt / 2) {
    return 1;
  }
  float timeFraction = (time - deadTime) / (period - 2 * deadTime);
  if (time < period / 2) {
    return 2.0 * timeFraction * timeFraction;
  }
  return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}

float parabolicReturnMod(float f)  // normalised [0,1] up and down
{
  double x = 0;
  double y = 0;
  if (f < 0.25f) {
    // y: 0 -> 0.75
    y = 8 * f * f * 1.50;
  }
  if (f >= 0.25f && f < 0.5f) {
    // y: 0.75 -> 1.00
    x = 0.5f - f;
    y = 8 * x * x;
    y = y / 2;
    y = 1.0f - y;
  }
  if (f >= 0.5f && f < 0.75f) {
    // y: 1.00 -> 0.75
    x = f - 0.5f;
    y = 8 * x * x;
    y = y / 2;
    y = 1.0f - y;
  }
  if (f >= 0.75f && f <= 1.0f) {
    // y: 0.75 -> 0
    x = 1.0f - f;
    y = 8 * x * x * 1.50;
  }
  return y;
}

float linearStep(float time, float period)
{
  if (time <= 0) {
    return 0;
  }
  if (time >= period) {
    return 1;
  }
  return time / period;
}
