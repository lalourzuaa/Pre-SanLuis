#include "main.h"
#include "globals.h"
#include "liblvgl/llemu.hpp"
#include "lightninglib/Math.h"
#include "lightninglib/PID.hpp"
#include "lightninglib/TankChassis.h"
#include "lightninglib/TankChassis.hpp"
#include "lightninglib/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <ostream>
#include <strings.h>

#define ZERO 2000 //2000
#define PICK_1 4800 //4800
#define DUNK 12500 //12500

int claw_positions[3] = {ZERO, PICK_1, DUNK}; 
int claw_state = 0;
int last_claw_state = 0;
int claw_analog;

bool intaker_correction = false;

void config_arms_controller()
{
  claw_controller.set_error_tolerance(200);

  claw_controller.set_derivative_tolerance(3000);

  float integral_power_limit = 1500/claw_controller.get_ki(); 
  claw_controller.set_integral_power_limit(integral_power_limit); 

  claw_controller.set_jump_time(500); //500 msec

  claw_controller.set_max(10800); 
}

void eat_intaker(int rpm_intaker)
{
  intaker.move_velocity(rpm_intaker);
}

void move_arms(int target, int stop_time_msec, float scale)
{
  claw_controller.set_stop_time(stop_time_msec);
  claw_controller.set_scale(scale); 
  claw_controller.set_integral_zone(target * 0.45);

  claw_controller.initialization();
  // values from 0 - 36000
  float current_position = claw_psensor.get_angle(); 
  float error = current_position; 

  while(!(claw_controller.target_arrived()))
  {
    current_position = claw_psensor.get_angle();
    error = target - current_position; 

    //Updating the controller using the error. it should be in a loop. 
    claw_controller.update(error); 

    claw.move_voltage(claw_controller.get_output()); 
    //std::cout<<"Error: "<<error<<"Posicion: "<<current_position<<"-Output: "<<claw_controller.get_output()<<std::endl;
    pros::delay(claw_controller.get_sample_time());
  }
    claw.move_voltage(0);
}

void track(void*)
{
  my_chassis.track_pose();
  pros::delay(10);
}
void init_track(void*) 
{
  char buffer_x[32];
  char buffer_y[32];
  char buffer_theta[32];
 while (1) 
 {
     
      snprintf(buffer_x, 32, "X: %.4f", my_chassis.get_x());
      snprintf(buffer_y, 32, "Y: %.4f", my_chassis.get_y());
      snprintf(buffer_theta, 32, "Theta: %.4f", my_chassis.get_orientation());

      pros::lcd::set_text(2, buffer_x);
      pros::lcd::set_text(3, buffer_y);
      pros::lcd::set_text(4, buffer_theta);
      pros::delay(10);
    }
}


void initialize() {
  pros::lcd::initialize(); 
  pros::lcd::set_text(1, "Lightning");

  //If the user has odometry...
  if(!(my_chassis.get_odometry_configuration()==lightning::NO_ODOM)){
    /*
    Restarting the encoders and IMU,
    It s recommended to use a 3000 milliseconds delay. 
    */
    my_chassis.reset_odometry(); 
    pros::delay(3000); 
    //Setting the initial pose of your robot
    my_chassis.set_coordinates(0_in, 0_in, 32_deg);  
  }
  
  //If the user has not odometry...
  else{
     /*
    Restarting the IMU,
    It s recommended to use a 3000 milliseconds delay. 
    */
    my_chassis.reset_IMU(); 
    pros::delay(3000); 
    //Setting the initial orientation
    my_chassis.set_orientation(0_deg);
  
    pros::delay(1000);

  my_chassis.reset_odometry(); 
  }
  claw.set_brake_mode(MOTOR_BRAKE_HOLD);

}


void disabled() {}
void competition_initialize() {}

void hold_claw(void*)
{
  float last_intaker_efficiency = 0;
    while (1) 
    {
      config_arms_controller();
      if (claw_analog != 0)
      {
         claw.move(claw_analog);
        pros::delay(500);
        claw_state = 0;
      }
      else
      {
        if (claw_state == 0)
        {
          move_arms(claw_positions[0], 500, 1);
        }
        else if (claw_state == 1) 
        {
          move_arms(claw_positions[1], 500, 1); 
          if (intaker.get_efficiency() != last_intaker_efficiency && intaker.get_efficiency() < 5)
          {
            intaker_correction = true;
            intaker.move_relative(-100, 100);
          }
        }
        else if (claw_state == 2) 
        {
          move_arms(claw_positions[2], 1000, 1);
        }
      }
    last_intaker_efficiency = intaker.get_efficiency();
    pros::delay(50); 
    }
}
void first_donut(void*)
{
  while (1) 
  {
    if (my_chassis.get_current_index() == 2)
    {
      eat_intaker(600);
    }
    
    pros::delay(lightning::util::DELAY_TIME);
  }
}

void clamp_mogo(void*)
{
  while (1) 
  {
    if (my_chassis.get_current_index() == 6)
    {
      clamp.set_value(true);
    }
    pros::delay(lightning::util::DELAY_TIME);
  }
}

void allience_donut(void*)
{
  while (1) 
  {
    if (my_chassis.get_current_index() == 2)
    {
      move_arms(claw_positions[1], 3000, 1);
    }
    pros::delay(lightning::util::DELAY_TIME);
  }

}
void autonomous() 
{
  // Imprimir datos odometria
  AUTON_TASKS.start_task("track", track);
  AUTON_TASKS.start_task("print", init_track);
  
  config_arms_controller();
  // Primeros valores PID vueltas y drive
  float limit_integral = 150 / .00350;
  // 90 grados, 180 grados es el mismo
  my_chassis.set_turn_constants(0.7, 0.1, 5, 500, 1, 0, 0.3);
  my_chassis.set_turn_exit_conditions(1, 100, 1000);
  
  // p era 1.2
  my_chassis.set_drive_constants(5,0, 0, 500, 1, limit_integral, 0);
  my_chassis.set_drive_exit_conditions(2,100 , 1000);

  my_chassis.set_swing_constants(1.5, 0.1, 6, 500, 1, 0, 0.3);
  my_chassis.set_swing_exit_conditions(2, 100, 1000);


  
  // Evitar el vuelito
  //my_chassis.set_brake(pros::E_MOTOR_BRAKE_HOLD);
  //xChecar ultimo punto si si lo agarra con task -41.5 y 26.67
  //-31.57
  //AUTON_TASKS.start_task("clamp-mogo",clamp_mogo);  ultima Y era -40
  lightning::Path go_to_mogo( {my_chassis.get_x(),-9.76,-18.99,-19,-25.67,-29.57,-30.67}, {my_chassis.get_y(),-10.9,-19.26,-24.7,-32.59,-34.59,-42.25}, true, 10);
  go_to_mogo.upgrade(); 
  go_to_mogo.set_max_lineal_velocity(15); 
  go_to_mogo.make_calcs(2);
  pros::delay(20); //Giving some time to make the calculations.
  
 my_chassis.follow_path(go_to_mogo,5);
 //pros::delay(50);
 my_chassis.stop();
 pros::delay(100);
 // Agarrar mogo
 clamp.set_value(true);
 
 

 pros::delay(60);

  

 // Path hacia la esquina y agarrar donuts X =-26.257,-25.38,-26.67,-26.33,-26.33, Y -17.34,-10.45,-7.08,-4.29,-2      
 lightning::Path go_to_donuts( {my_chassis.get_x(),-28.18,-26.257,-25.38,-26.67,-26.67,-26.76}, {my_chassis.get_y(),-31.89,-17.34,-10.45,-7.08,-1.98,-1}, false, 10);
  go_to_donuts.upgrade(); 
  go_to_donuts.set_max_lineal_velocity(15); 
  go_to_donuts.make_calcs(2);
  pros::delay(20); //Giving some time to make the calculations.
  AUTON_TASKS.start_task("score", first_donut);
  my_chassis.follow_path(go_to_donuts,5);
  pros::delay(50);
  AUTON_TASKS.kill_task("score");

  // girar a la esquina y agarrar dona
  intaker_valve.set_value(false);
  pros::delay(500);
  //my_chassis.turn_absolute(328);
  my_chassis.swing_turn_absolute(lightning::RIGHT_SWING,330_deg);
  pros::delay(500);
  eat_intaker(0);


  // Ir a la esquina con intaker apagado y lgo prenderlo
  lightning::Path go_to_corner( {my_chassis.get_x(),-26.23,-30.7,-34.09,-34.4,-36}, {my_chassis.get_y(),-6.11,0.31,7.74,8.2,10}, false, 10);
  go_to_corner.upgrade(); 
  go_to_corner.set_max_lineal_velocity(15); 
  go_to_corner.make_calcs(2);
  pros::delay(20); //Giving some time to make the calculations.
  my_chassis.follow_path(go_to_corner,5);
  
  

  pros::delay(800);
  eat_intaker(600);
  my_chassis.drive_distance(7_in,330_deg);
  pros::delay(800);
  
  
  // Hacerse para atras para comer la dona
  my_chassis.drive_distance(-10_in,330_deg);
  pros::delay(2000);
  // falta quitar donas de la esquina

  
  my_chassis.turn_absolute(100_deg);
  pros::delay(100);
  
  
  /*
  // Soltar la mogo esto es opcional
  clamp.set_value(false);

  // Empujarla si somos rojos para la esquina +
  my_chassis.drive_distance(-5_in,135_deg);
  pros::delay(500);
  */

  
  // path llegar a la de alianza
  /*
  AUTON_TASKS.start_task("claw", allience_donut);
  lightning::Path go_to_alliance( {my_chassis.get_x(),-19.44,-7.77,4.88,19,22}, {my_chassis.get_y(),7.23,6.42,8.80,8,8}, false, 10);
  go_to_alliance.upgrade(); 
  go_to_alliance.set_max_lineal_velocity(10); 
  go_to_alliance.make_calcs(2);
  pros::delay(20); //Giving some time to make the calculations.
  my_chassis.follow_path(go_to_alliance,5);
  AUTON_TASKS.kill_task("claw");

  // Apagar el intaker y swing para meter alliance
  pros::delay(100);

  my_chassis.swing_turn_absolute(lightning::RIGHT_SWING,40_deg);
  pros::delay(500);
  eat_intaker(0);
  pros::delay(1000);
  move_arms(claw_positions[2], 5000, 1);
  */

  
  AUTON_TASKS.start_task("claw", allience_donut);
  lightning::Path go_to_alliance( {my_chassis.get_x(),-13.72,-1.9,9.7,19.41,21}, {my_chassis.get_y(),7.9,7.43,7.43,4.43,4.43}, false, 10);
  go_to_alliance.upgrade(); 
  go_to_alliance.set_max_lineal_velocity(10); 
  go_to_alliance.make_calcs(2);
  pros::delay(20); //Giving some time to make the calculations.
  my_chassis.follow_path(go_to_alliance,5);
  AUTON_TASKS.kill_task("claw");

   my_chassis.turn_absolute(32_deg);
  pros::delay(500);
  eat_intaker(0);
  pros::delay(500);
  my_chassis.drive_distance(-3_in,32_deg);
  pros::delay(100);
  config_arms_controller();
  claw.move_velocity(100);
  pros::delay(1000);
  claw.move_velocity(0);
  my_chassis.drive_distance(-5_in,32_deg);

  pros::delay(100);
  
  
  my_chassis.swing_turn_absolute(lightning::RIGHT_SWING,10);
  my_chassis.drive_distance(-30_in,10_deg);

  claw.move_velocity(-50);

  


  

  
  
  // parar intaker y girar para anotra







}


void opcontrol() 
{
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  int power_claw;
  int power_intaker;
  bool clamping;
  int position = 0;
  bool intaker_v;
  bool elevating;
  config_arms_controller();
  while (true) {
   OPCONTROL_TASKS.start_task("arms", hold_claw);
   my_chassis.arcade(master,lightning::E_TANK_OP_ARCADE_LEFT);  //DRIVING ROBOT IN ARCADE MODE 

  intaker_correction = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) ?  false: intaker_correction;
  if (!intaker_correction)
  {
    power_intaker = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? 500: master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? -500:0;
    intaker.move_velocity(power_intaker);
  }

   clamping = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) ? !clamping : clamping;
	 clamp.set_value(clamping);

   intaker_v = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) ? !intaker_v : intaker_v;
	 intaker_valve.set_value(intaker_v);

   elevating = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) ? !elevating : elevating;
	 elevation_lock.set_value(elevating);

   claw_state= (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) ? (claw_state == 2 ? 0 : claw_state + 1) : claw_state;

   claw_analog = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

   //std::cout<<"Efficiency: "<<intaker.get_efficiency()<<std::endl;
   std::cout<<"State: "<<claw_state<<", analog: "<<", claw power: "<<claw.get_power()<<" ,analog:"<<claw_analog<<std::endl;
   
   pros::delay(lightning::util::DELAY_TIME); 
  }
}
