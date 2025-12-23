#pragma once
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include <array>
#include "control/pid.hpp"
#include <cmath>
#include "control/slew.hpp"



class Drive {
public:
  Drive(int l1, int l2, int l3, int r1, int r2, int r3, int imuPort);

  // Call once in initialize()
  void calibrateImu();

  // PID turn (degrees). Blocking call.
  void turnTo(double targetHeadingDeg);

  // Driver control helpers
  void tank(int leftPct, int rightPct);      // -100..100
  void setVoltage(int leftMv, int rightMv);  // -12000..12000
  void brakeHold(bool enabled);

  // Drive forward/backward a distance (inches), while holding a heading (deg).
  // If headingHoldDeg is NAN, it holds the current heading at start.
  void driveDistance(double inches, double headingHoldDeg = NAN);

  // Arcade drive helper
  void arcade(int forwardPct, int turnPct); // -100..100

  // Sensors
  void tareEncoders();
  double leftMotorDeg() const;   // avg of left motors
  double rightMotorDeg() const;  // avg of right motors
  double headingDeg() const;     // 0..360 from IMU

  // Slew rate control
  void enableSlew(bool enabled);
  void setSlewRate(double mvPerSec);

  // Reset slew limiter (e.g. after stopping)
  void resetSlew();

  

private:
  std::array<pros::Motor, 3> left;
  std::array<pros::Motor, 3> right;
  pros::Imu imu;
  Slew leftSlew{24000.0};
  Slew rightSlew{24000.0};
  int lastMs{0};
  bool slewEnabled{true};
};
