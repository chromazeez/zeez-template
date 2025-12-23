#include "drive/drive.hpp"
#include "config/constants.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include "util/units.hpp"

static double avgMotorPositionDeg(const std::array<pros::Motor, 3>& motors) {
  double sum = 0.0;
  for (const auto& m : motors) sum += m.get_position(); // degrees
  return sum / 3.0;
}

Drive::Drive(int l1, int l2, int l3, int r1, int r2, int r3, int imuPort)
  : left{ pros::Motor(l1), pros::Motor(l2), pros::Motor(l3) }
  , right{ pros::Motor(r1), pros::Motor(r2), pros::Motor(r3) }
  , imu(imuPort) {

  // Common convention:
  // Left motors forward = +, Right motors forward = +.
  // If your right side drives backward, flip these true/false later.
  for (auto& m : left) {
    m.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    m.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    m.set_reversed(false); // change if needed
  }

  for (auto& m : right) {
    m.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    m.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    m.set_reversed(true); // VERY COMMON on tank drives; flip if wrong
  }
}

void Drive::calibrateImu() {
  imu.reset();
  while (imu.is_calibrating()) {
    pros::delay(20);
  }
}

void Drive::tank(int leftPct, int rightPct) {
  // Percent -> millivolts
  const int lmv = (leftPct * constants::MAX_VOLTAGE) / 100;
  const int rmv = (rightPct * constants::MAX_VOLTAGE) / 100;
  setVoltage(lmv, rmv);
}

void Drive::setVoltage(int leftMv, int rightMv) {
  // Clamp to safe range
  leftMv = std::max(-constants::MAX_VOLTAGE, std::min(constants::MAX_VOLTAGE, leftMv));
  rightMv = std::max(-constants::MAX_VOLTAGE, std::min(constants::MAX_VOLTAGE, rightMv));

  for (auto& m : left)  m.move_voltage(leftMv);
  for (auto& m : right) m.move_voltage(rightMv);
}

void Drive::brakeHold(bool enabled) {
  const auto mode = enabled ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST;
  for (auto& m : left)  m.set_brake_mode(mode);
  for (auto& m : right) m.set_brake_mode(mode);
}

void Drive::tareEncoders() {
  for (auto& m : left)  m.tare_position();
  for (auto& m : right) m.tare_position();
}

void Drive::turnTo(double targetHeadingDeg) {
  // Basic turn PID gains (starter). You will tune on the robot.
  // For IMU degrees -> motor millivolts, kP usually starts around 80-150.
  PID pid(110.0, 0.0, 650.0);
  pid.setOutputLimit(constants::MAX_VOLTAGE);

  const int dtMs = 10;
  const double dt = dtMs / 1000.0;

  int settleCount = 0;
  const int settleNeeded = 15;     // 15 * 10ms = 150ms stable
  const double settleErrDeg = 1.0; // within 1 degree
  const int timeoutMs = 2000;

  int elapsed = 0;

  while (elapsed < timeoutMs) {
    const double current = headingDeg();
    const double err = angleErrorDeg(targetHeadingDeg, current);

    const double output = pid.step(0.0, -err, dt);
    // Explanation: our PID assumes target-current. We want "error" to be the wrapped err.
    // Using target=0, current=-err makes (0 - (-err)) = err.

    // Turn in place: left +, right -
    setVoltage((int)output, (int)-output);

    if (std::abs(err) < settleErrDeg) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleNeeded) break;

    pros::delay(dtMs);
    elapsed += dtMs;
  }

  setVoltage(0, 0);
}


double Drive::leftMotorDeg() const {
  return avgMotorPositionDeg(left);
}

double Drive::rightMotorDeg() const {
  return avgMotorPositionDeg(right);
}

double Drive::headingDeg() const {
  // PROS IMU returns 0..360 typically
  return imu.get_heading();
}


