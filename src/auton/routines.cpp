#include "auton/routines.hpp"
#include "subsystems/devices.hpp"
#include "pros/rtos.hpp"

namespace auton {

void doNothing() {
  // literally nothing
}

void skills() {
  drive.driveDistance(48);
  pros::delay(200);
  drive.turnTo(90);
  pros::delay(200);
  drive.driveDistance(24);
}

void leftRush() {
  drive.driveDistance(36);
  pros::delay(150);
  drive.turnTo(45);
}

void rightSafe() {
  drive.driveDistance(24);
  pros::delay(150);
  drive.turnTo(-45); // if you want negatives, we’ll normalize later
}

}
