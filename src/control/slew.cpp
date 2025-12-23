#include "control/slew.hpp"
#include <algorithm>

Slew::Slew(double maxChangePerSec) : rate(std::abs(maxChangePerSec)), value(0.0) {}

void Slew::setRate(double maxChangePerSec) {
  rate = std::abs(maxChangePerSec);
}

void Slew::reset(double v) {
  value = v;
}

double Slew::step(double target, double dt) {
  if (dt <= 0) return value;

  const double maxDelta = rate * dt;
  const double delta = target - value;

  if (delta > maxDelta) value += maxDelta;
  else if (delta < -maxDelta) value -= maxDelta;
  else value = target;

  return value;
}
