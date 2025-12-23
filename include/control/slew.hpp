#pragma once
#include <cmath>

class Slew {
public:
  // maxChangePerSec: e.g. 24000 means 0->12000 in 0.5s
  explicit Slew(double maxChangePerSec = 24000.0);

  void setRate(double maxChangePerSec);
  void reset(double value = 0.0);

  // returns limited value
  double step(double target, double dt);

private:
  double rate;   // units per second
  double value;  // current output
};
