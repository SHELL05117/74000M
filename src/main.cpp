#include "pros/llemu.hpp"

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "74000M OFFLINE BASELINE");
	pros::lcd::set_text(1, "HARDWARE UNVERIFIED");
	pros::lcd::set_text(2, "OUTPUT LOCKED");
}

void disabled() {}

void competition_initialize() {}

// Until the verified hardware profile and single-writer output pipeline exist,
// both enabled callbacks intentionally perform no motion.
void autonomous() { pros::lcd::set_text(3, "AUTO: DO NOTHING"); }

void opcontrol() { pros::lcd::set_text(3, "DRIVER: LOCKED"); }
