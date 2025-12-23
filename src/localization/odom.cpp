#include "localization/odom.hpp"
#include "config/constants.hpp"
#include <cmath>
#include "pros/rtos.hpp"

static double degToRad(double deg) {
  return deg * M_PI / 180.0;
}

static double wrapRad(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

Odom::Odom(Drive& drive) : drive(drive) {}

void Odom::start() {
  pros::Task([this]() { this->loop(); }, "Odom");
}

void Odom::reset(Pose p) {
  x.store(p.x);
  y.store(p.y);
  theta.store(p.theta);

  // Baseline encoder readings
  lastLeftDeg = drive.leftMotorDeg();
  lastRightDeg = drive.rightMotorDeg();
}

Pose Odom::get() const {
  return Pose{ x.load(), y.load(), theta.load() };
}

void Odom::loop() {
  const int dtMs = 10;

  // Initialize baselines if not already
  lastLeftDeg = drive.leftMotorDeg();
  lastRightDeg = drive.rightMotorDeg();

  while (true) {
    const double leftDeg = drive.leftMotorDeg();
    const double rightDeg = drive.rightMotorDeg();

    const double dLeftIn  = constants::motorDegToInches(leftDeg - lastLeftDeg);
    const double dRightIn = constants::motorDegToInches(rightDeg - lastRightDeg);
    lastLeftDeg = leftDeg;
    lastRightDeg = rightDeg;

    // Forward distance along robot's forward direction
    const double dS = (dLeftIn + dRightIn) / 2.0;

    // Heading from IMU (degrees 0..360) -> radians
    const double headingRad = degToRad(drive.headingDeg());

    // Use midpoint integration (helps a bit vs simple Euler)
    const double prevTheta = theta.load();
    const double midTheta = wrapRad((prevTheta + headingRad) / 2.0);

    // Update pose in global frame
    const double newX = x.load() + dS * std::cos(midTheta);
    const double newY = y.load() + dS * std::sin(midTheta);

    x.store(newX);
    y.store(newY);
    theta.store(wrapRad(headingRad));

    pros::delay(dtMs);
  }
}
