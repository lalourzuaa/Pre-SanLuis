#pragma once
#include "lightninglib/TankChassis.hpp"
#include "lightninglib/lightning_api.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

extern lightning::TankChassis my_chassis;
extern pros::MotorGroup intaker;
extern pros::Motor claw;
extern pros::Rotation claw_psensor;
extern pros::adi::DigitalOut clamp;
extern lightning::PID claw_controller;
extern lightning::TaskManager AUTON_TASKS;
extern lightning::TaskManager OPCONTROL_TASKS;
extern pros::adi::DigitalOut intaker_valve;
extern pros::adi::DigitalOut elevation_lock;
