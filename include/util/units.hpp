#pragma once
#include <cmath>

// Returns an angle error in degrees in the range [-180, 180]
inline double angleErrorDeg(double targetDeg, double currentDeg) {
  double err = targetDeg - currentDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return err;
}
