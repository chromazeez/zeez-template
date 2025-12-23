#pragma once
#include <cmath>

namespace constants {
  constexpr int MAX_VOLTAGE = 12000;

  constexpr double WHEEL_DIAMETER_IN = 2.75;
  constexpr double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * M_PI;

  // 600rpm motor geared down to ~450rpm wheel => wheel = motor * 0.75
  // so motor rotations per wheel rotation = 1/0.75 = 1.3333...
  constexpr double MOTOR_ROT_PER_WHEEL_ROT = 1.0 / 0.75;

  inline double motorDegToInches(double motor_deg) {
    const double motor_rot = motor_deg / 360.0;
    const double wheel_rot = motor_rot / MOTOR_ROT_PER_WHEEL_ROT;
    return wheel_rot * WHEEL_CIRCUMFERENCE_IN;
  }

  // Measure later
  constexpr double TRACK_WIDTH_IN = 12.5;
}
