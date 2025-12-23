#include "subsystems/devices.hpp"
#include "config/ports.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// 6-motor drive + IMU
Drive drive(ports::L1, ports::L2, ports::L3,
            ports::R1, ports::R2, ports::R3,
            ports::IMU);

// Example later:
// pros::ADIDigitalOut clampPiston('A');
// pros::Motor intake(8);
