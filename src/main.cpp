#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include <math.h>
class AutonomousRobot
{
	private:
		pros::Motor leftFront;
		pros::Motor leftBack;
		pros::Motor rightFront;
		pros::Motor rightBack;
		pros::Controller master;
	public:
		AutonomousRobot(pros::Motor lf, pros::Motor lb, pros::Motor rf, pros::Motor rb) : leftFront(lf), leftBack(lb), rightFront(rf), rightBack(rb), master(pros::E_CONTROLLER_MASTER){}
		void stop() {
			leftFront.brake();
			leftBack.brake();
			rightFront.brake();
			rightBack.brake();
		}
		void forward() {
			leftFront.move_velocity(20);
			rightFront.move_velocity(-20);
			leftBack.move_velocity(20);
			rightBack.move_velocity(-20);
		}
		void turn(bool left) {
			int dir = left ? -1 : 1;

			leftFront.move_velocity(20 * dir);
			rightFront.move_velocity(20 * dir);
			leftBack.move_velocity(20 * dir);
			rightBack.move_velocity(20 * dir);
		}
		void spin_until_color_match() {
			bool color = false; // add logic to get color using vision sensor
			while (!color) {

			}
			// return;k :?
		}
};

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void
on_center_button()
{
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
void autonomous() {
	pros::Motor left_up(11);
	pros::Motor right_up(1);
	pros::Motor left_down(20);
	pros::Motor right_down(10);
	AutonomousRobot rb(left_up, left_down, right_up, right_down);
	rb.forward();
	pros::delay(1000);
	rb.stop();
	pros::delay(1000);
	rb.turn(true);
	pros::delay(1000);
	rb.stop();
	pros::delay(1000);
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
	const float correction = 0.8;
	// autonomous();
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_up(9);
	pros::Motor right_up(6);
	pros::Motor left_down(20);
	pros::Motor right_down(10);
	pros::Motor intake(8);
	pros::Motor spinner_conveyor(7);
	pros::Motor spinner1(5);
	pros::Motor spinner2(11);
	pros::Imu gyro(1);
	spinner1.set_gearing(pros::E_MOTOR_GEARSET_36);
	intake.set_gearing(pros::E_MOTOR_GEARSET_36);
	spinner_conveyor.set_gearing(pros::E_MOTOR_GEARSET_36);
	bool spinning = false;
	left_down.move_velocity(100);
	while (true)
	{
		int axisTwo = master.get_analog(ANALOG_RIGHT_Y);
		int axisThree = master.get_analog(ANALOG_LEFT_Y); // Y
		int axisFour = master.get_analog(ANALOG_LEFT_X); // X
		int axisOne = master.get_analog(ANALOG_RIGHT_X); // turn

		float theta = gyro.get_heading() * 3.14159 / 180;
		float theta1 = atan2(axisThree, axisFour);
		theta += theta1;
		int r = sqrt(axisThree * axisThree + axisFour * axisFour);
		axisThree = r * sin(theta);
		axisFour = r * cos(theta);
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			intake.move_velocity(100);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intake.move_velocity(-100);
		}
		else
		{
			intake.move_velocity(0);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			spinner_conveyor.move_velocity(80);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			spinner_conveyor.move_velocity(-80);
		}
		else {
			spinner_conveyor.move_velocity(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			if (spinning) {
				spinner1.move_voltage(12000);
			}
			else {
				spinner1.move_voltage(0);
			}
			spinning = !spinning;
		}
		right_up = axisOne - axisThree + axisFour;
		right_down = axisOne - axisThree - axisFour;
		left_up = axisOne + axisThree + axisFour;
		left_down = axisOne + axisThree - axisFour;
		pros::delay(20);
	}
}
