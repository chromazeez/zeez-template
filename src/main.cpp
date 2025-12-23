#include "main.h"
#include "config/ports.hpp"
#include "drive/drive.hpp"


Drive drive(ports::L1, ports::L2, ports::L3,
            ports::R1, ports::R2, ports::R3,
            ports::IMU);


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Calibrating IMU...");

  drive.calibrateImu();  // reset + wait

  pros::lcd::set_text(1, "IMU ready");
  pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  
  auto deadband = [](int v, int db = 5) {
    return (std::abs(v) < db) ? 0 : v;
  };

  while (true) {
    int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // -127..127
    int turn    = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // -127..127

    int left  = forward + turn;
    int right = forward - turn;

    // clamp to controller range
    left  = std::max(-127, std::min(127, left));
    right = std::max(-127, std::min(127, right));

    // convert to -100..100 and send
    drive.tank((left * 100) / 127, (right * 100) / 127);

    pros::delay(10);
  }
}