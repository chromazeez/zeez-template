#include "motion/motion.hpp"
#include "config/constants.hpp"
#include <cmath>
#include "pros/rtos.hpp"

static double wrapRad(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

Motion::Motion(Drive& drive, Odom& odom) : drive(drive), odom(odom) {}

void Motion::driveToPoint(double targetX, double targetY) {
  // Gains (starter). You'll tune on robot.
  // forwardMv = kP_dist * distError
  // turnMv    = kP_turn * headingError
  const double kP_dist = 600.0;   // mV per inch
  const double kP_turn = 4000.0;  // mV per rad

  const int dtMs = 10;
  const int timeoutMs = 4000;

  const double settleDistIn = 1.0;   // within 1 inch
  const double settleHeadingRad = 0.08; // ~4.5 deg
  const int settleNeeded = 20;        // 200ms

  int settleCount = 0;
  int elapsed = 0;

  while (elapsed < timeoutMs) {
    Pose p = odom.get();

    const double dx = targetX - p.x;
    const double dy = targetY - p.y;

    const double dist = std::sqrt(dx*dx + dy*dy);

    // Angle to target in global frame
    const double targetAngle = std::atan2(dy, dx);

    // Heading error (robot theta -> target angle)
    const double headingErr = wrapRad(targetAngle - p.theta);

    // Forward command should reduce when you're turned away.
    // Multiply by cos(error) so it doesn't try to drive forward hard while sideways.
    double forwardMv = kP_dist * dist * std::cos(headingErr);
    double turnMv    = kP_turn * headingErr;

    // Clamp forward/turn a bit
    if (forwardMv >  constants::MAX_VOLTAGE) forwardMv =  constants::MAX_VOLTAGE;
    if (forwardMv < -constants::MAX_VOLTAGE) forwardMv = -constants::MAX_VOLTAGE;

    if (turnMv >  constants::MAX_VOLTAGE) turnMv =  constants::MAX_VOLTAGE;
    if (turnMv < -constants::MAX_VOLTAGE) turnMv = -constants::MAX_VOLTAGE;

    const int leftMv  = (int)(forwardMv + turnMv);
    const int rightMv = (int)(forwardMv - turnMv);

    drive.setVoltage(leftMv, rightMv);

    const bool distOk = dist < settleDistIn;
    const bool angOk  = std::abs(headingErr) < settleHeadingRad;

    if (distOk && angOk) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleNeeded) break;

    pros::delay(dtMs);
    elapsed += dtMs;
  }

  drive.setVoltage(0, 0);
}
