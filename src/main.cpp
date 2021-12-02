#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Test base roulante 4 moteurs");
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
 #define LB 1
 #define LF 12
 #define RB 10
 #define RF 11
 #define ARM 8
 #define CLAW 3
 #define WHEEL_DIAMETER 10_cm
 #define WHEEL_TRACK 31_cm

void opcontrol() {
	//create motor group for left and right
	// One of the motors on the motorgroup has to be reversed because of design
	Motor motorLB = Motor(LB,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
	Motor motorLF = Motor(LF,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);

	Motor motorRB = Motor(RB,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
	Motor motorRF = Motor(RF,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);

	Motor motorArm = Motor(ARM,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
	Motor motorClaw = Motor(CLAW,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);

	const std::initializer_list<Motor> left = {motorLB, motorLF};
	const std::initializer_list<Motor> right = {motorRB, motorRF};

	std::shared_ptr<MotorGroup> grpLeft(new MotorGroup(left));
	std::shared_ptr<MotorGroup> grpRight(new MotorGroup(right));

	//Create Chassis with those two motorgroups
	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
			.withMotors(grpLeft,grpRight)
			.withDimensions(AbstractMotor::gearset::green,{{WHEEL_DIAMETER,WHEEL_TRACK},imev5GreenTPR})
			.build();

	// Create controller object
	Controller controller;
	 float speedLeftX,speedLeftY,speedRightX,speedRightY;
	 bool test = false;
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
	}
}
