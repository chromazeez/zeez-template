#pragma once
#include <algorithm>

class PID {
public:
  PID(double kP, double kI, double kD);

  void reset();
  double step(double target, double current, double dt);

  void setOutputLimit(double maxAbs);
  void setIntegralLimit(double maxAbs);

private:
  double kP, kI, kD;
  double integral{0.0};
  double lastError{0.0};
  double outLimit{12000.0};
  double iLimit{1e9};
  bool firstStep{true};
};
