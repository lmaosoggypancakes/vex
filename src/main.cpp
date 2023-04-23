#include "main.h"
#include "autons.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "sylib/motor.hpp"
#include "sylib/system.hpp"
#include <cmath>

#define INTAKE_PORT 7
#define FLYWHEEL_PORT 4
#define FLYWHEEL_GEARING 3600
#define INDEXER_PORT 2
#define FLYWHEEL_PORT 4


pros::ADIDigitalOut piston('A');
pros::Motor intake(INTAKE_PORT);
pros::Motor indexer(INDEXER_PORT);

pros::Controller master(pros::E_CONTROLLER_MASTER);

sylib::SpeedControllerInfo
    flywheel_controller([](double rpm) { return M_E / std::log(rpm + M_E); },
                        1000000, 2500, 500, 0, false, 0

    );


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-9, -10}

  // IMU Port
  ,20

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,200

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1

);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.
  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
  default_constants();
  chassis.initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



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
void autonomous() {
    sylib::initialize();
    sylib::Motor flywheel(FLYWHEEL_PORT, FLYWHEEL_GEARING, true,
                          flywheel_controller);
    flywheel.set_velocity_custom_controller(2200);
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  chassis.set_drive_pid(2, 127 / 3, true);
  chassis.wait_drive();

  intake.move(127);
  pros::delay(400);
  intake.move(0);

  chassis.set_drive_pid(-22, 127 / 3, true, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90, 127 / 2);
  chassis.wait_drive();

   chassis.set_drive_pid(24, 127 / 3, true, true);
   chassis.wait_drive();

  intake.move(127);
  pros::delay(400);
  intake.move(0);

  chassis.set_drive_pid(-24 * 3.7, 127, true, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90 + 12, 127 / 2);
  chassis.wait_drive();

  indexer.move(-127/2);
  pros::delay(4000);
  indexer.move(0);

   chassis.set_turn_pid(90, 127 / 2);
   chassis.wait_drive();

   chassis.set_drive_pid(24 * 2.8, 127, true, true);
   chassis.wait_drive();

   chassis.set_turn_pid(45, 127 / 2);
   chassis.wait_drive();

   pros::delay(10000);
   piston.set_value(true);

   pros::delay(5000);
}



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
  sylib::initialize();
  sylib::Motor flywheel(FLYWHEEL_PORT, FLYWHEEL_GEARING, true,
                      flywheel_controller);

pros::Motor flywheel_encoder(FLYWHEEL_PORT);

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  flywheel.set_velocity_custom_controller(2400);
  while (true) {

    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      intake.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);
    } else {
      intake.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      indexer.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      indexer.move(127);
    } else {
      indexer.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      piston.set_value(true);
    }
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
