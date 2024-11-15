#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"

// Chassis
lightning::TankChassis my_chassis
(
  lightning::tank_odom_e_t::ROTATION_ONE_ODOM, //No odometry 
  {-1,2,3,-4,5}, //Left side ports adelante atras abajo, luego de arriba a abajo
  {6,-7,-8,9,-10}, //Right side ports 
  15, //IMU port 
  pros::E_MOTOR_GEAR_BLUE, //Which motor cartride are you using, blue,red,green? 
  3.25, //Wheel Diameter in inches
  1, //what is the gear ratio (Is the result of Driven/Driving, Drive:Driving)
  18,
  2,
  2,
  0,
  0,0
); 
// Multitask control 
lightning::TaskManager OPCONTROL_TASKS;
//Multitask autonomo
lightning::TaskManager AUTON_TASKS;
// Brazitos
pros::Motor claw(-17,pros::v5::MotorGears::red);
pros::Rotation claw_psensor(16);
lightning::PID claw_controller(3, 0, 0, 10, 1);
// Clamp
pros::adi::DigitalOut clamp('H');
// Intaker
pros::MotorGroup intaker({-19,-20},pros::v5::MotorGears::blue);
pros::adi::DigitalOut intaker_valve('G');
// Elevacion 
pros::adi::DigitalOut elevation_lock('F');


