#include "auton/routines.hpp"
#include "subsystems/devices.hpp"
#include "pros/rtos.hpp"

namespace auton {

  void doNothing() {
    // literally nothing
  }

  void skills() {
    odom.reset(Pose{0, 0, 0});  // always reset at start of auto
    motion.driveToPoint(24, 0);
    pros::delay(200);
    motion.driveToPoint(24, 24);
    pros::delay(200);
    motion.driveToPoint(0, 24);
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
