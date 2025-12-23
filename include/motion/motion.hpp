#pragma once
#include "drive/drive.hpp"
#include "localization/odom.hpp"

class Motion {
public:
  Motion(Drive& drive, Odom& odom);

  // Blocking: drives to a point in inches
  void driveToPoint(double targetX, double targetY);

private:
  Drive& drive;
  Odom& odom;
};
