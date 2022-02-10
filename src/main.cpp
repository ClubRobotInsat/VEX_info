#include "main.h"
#include "okapi/api.hpp"
#include "base_functions.h"
using namespace okapi;
using namespace base_functions;


// Ports Wheels
#define FL_WHEEL 1
#define FR_WHEEL 2
#define BL_WHEEL 3
#define BR_WHEEL 4
// Ports Arm
#define R_ARM 10
#define L_ARM 9
// Port Doghnut intake
#define INTAKE 8
// Port Base Elevator
#define ELEVATOR 11
// Port Sensor
#define GYRO 20
// Port Antena
#define ANTENA 21
// Port Pneumatics
#define PNEUMATICS 'A'
// Proportions
#define WHEEL_DIAMETER 11_cm
#define WHEEL_TRACK 43_cm


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 #define PNEU 'A'
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Test Robot");
	//pros::ADIDigitalOut piston (PNEU);
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

void shut_down(Motor elevator) {
	elevator.moveAbsolute(0.0, 50);
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

// A modifier en fonction des ports utilises pour les moteurs du test


void opcontrol() {

	std::shared_ptr<ChassisController> drive = base_functions::initMobileBase(FL_WHEEL,
																			FR_WHEEL,
																			BL_WHEEL,
																			BR_WHEEL,
																			WHEEL_DIAMETER,
																			WHEEL_TRACK);

	// Create controller object
	Controller controller;
	float speedLeftX,speedLeftY,speedRightX,speedRightY;

	Motor motorArmLeft = Motor(L_ARM,true,AbstractMotor::gearset::red,AbstractMotor::encoderUnits::rotations);
	Motor motorArmRight = Motor(R_ARM,false,AbstractMotor::gearset::red,AbstractMotor::encoderUnits::rotations);

	Motor motorElevator = Motor(ELEVATOR, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::rotations);

	while(!controller.getDigital(ControllerDigital::X)){
		//pros::lcd::set_text(2, to_string(motorElevator.getPosition()));

		controller.setText(2, 0, to_string(motorElevator.getPosition()));

		speedLeftY = controller.getAnalog(ControllerAnalog::leftY);
		speedLeftX = controller.getAnalog(ControllerAnalog::leftX);
		speedRightY = controller.getAnalog(ControllerAnalog::rightY);
		speedRightX = controller.getAnalog(ControllerAnalog::rightX);
		// My control mode with the two different joysticks
		if(controller.getDigital(ControllerDigital::R1)){
			motorArmRight.moveRelative(1.0,100);
			motorArmLeft.moveRelative(1.0,100);
		}else if(controller.getDigital(ControllerDigital::R2)){
			motorArmRight.moveRelative(-1.0,100);
			motorArmLeft.moveRelative(-1.0,100);
		}else if(controller.getDigital(ControllerDigital::L2) && (motorElevator.getPosition())>-1.5){
			motorElevator.moveAbsolute(-1.55,150);
		}else if(controller.getDigital(ControllerDigital::L1) && (motorElevator.getPosition())<0.1){
			motorElevator.moveRelative(0.5,200);
		}
		
		/*else{
			motorArmLeft.moveRelative(0,100);
			motorArmRight.moveRelative(0,100);
		}*/
		//TODO Set limit for arm
		drive->getModel()->arcade(speedLeftY, speedLeftX);
	}

	shut_down(motorElevator);

	/*
	while(true){

		speedLeftY = controller.getAnalog(ControllerAnalog::leftY);
		speedLeftX = controller.getAnalog(ControllerAnalog::leftX);
		speedRightY = controller.getAnalog(ControllerAnalog::rightY);
		speedRightX = controller.getAnalog(ControllerAnalog::rightX);
		// My control mode with the two different joysticks
		drive->getModel()->arcade(speedLeftY, speedLeftX);


		while (speedRightY == 0 ) {
			speedRightY = controller.getAnalog(ControllerAnalog::rightY);
			motorArm.moveAbsolute(0, 200);

			// 200 car green gearset
			motorClaw.moveVelocity(speedRightX*200);
			motorArm.moveVelocity(speedRightY*200);
		}
		if (speedRightY != 0) {
			// 200 car green gearset
			motorClaw.moveVelocity(speedRightX*200);
			motorArm.moveVelocity(speedRightY*200);
		}

		motorArm.tarePosition();

		// motorArm.tarePosition();
		// motorArm.moveAbsolute(0, 200);
	}*/
	/*
	while(true){
		pros::ADIDigitalOut piston (PNEU);
		piston.set_value(true);
		pros::delay(1000);
		piston.set_value(false);
	}
	*/

}
