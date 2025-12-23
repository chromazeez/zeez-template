#include "control/pid.hpp"
#include <cmath>

PID::PID(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}

void PID::reset() {
  integral = 0.0;
  lastError = 0.0;
  firstStep = true;
}

void PID::setOutputLimit(double maxAbs) {
  outLimit = std::abs(maxAbs);
}

void PID::setIntegralLimit(double maxAbs) {
  iLimit = std::abs(maxAbs);
}

double PID::step(double target, double current, double dt) {
  if (dt <= 0) return 0;

  const double error = target - current;

  integral += error * dt;
  integral = std::clamp(integral, -iLimit, iLimit);

  double derivative = 0.0;
  if (!firstStep) derivative = (error - lastError) / dt;

  lastError = error;
  firstStep = false;

  const double output = (kP * error) + (kI * integral) + (kD * derivative);
  return std::clamp(output, -outLimit, outLimit);
}
