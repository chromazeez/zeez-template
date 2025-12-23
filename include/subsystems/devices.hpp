#pragma once
#include "pros/misc.hpp"
#include "drive/drive.hpp"
#include "localization/odom.hpp"
#include "motion/motion.hpp"

// Global odometry instance
extern Odom odom;

// Global controller
extern pros::Controller master;

// Global subsystems
extern Drive drive;

// Global motion controller
extern Motion motion;


// Later you’ll add:
// extern pros::ADIDigitalOut clampPiston;
// extern pros::Motor intake;
// extern pros::Optical colorSensor;
// etc.
