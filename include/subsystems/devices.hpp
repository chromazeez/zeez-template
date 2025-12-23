#pragma once
#include "pros/misc.hpp"
#include "drive/drive.hpp"
#include "localization/odom.hpp"

// Global odometry instance
extern Odom odom;

// Global controller
extern pros::Controller master;

// Global subsystems
extern Drive drive;

// Later you’ll add:
// extern pros::ADIDigitalOut clampPiston;
// extern pros::Motor intake;
// extern pros::Optical colorSensor;
// etc.
