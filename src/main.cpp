#include "main.h"
#include "okapi/api.hpp"
#include "base_functions.h"
using namespace okapi;
using namespace base_functions;

// Sensors and Actuators ports
#define PORT_FL_WHEEL 1
#define PORT_FR_WHEEL 2
#define PORT_BL_WHEEL 3
#define PORT_BR_WHEEL 4
#define PORT_R_ARM 10
#define PORT_L_ARM 9
#define PORT_ARM_ROTATION 5
#define PORT_GYROSCOPE 20
#define PORT_RING_MILL 8
#define PORT_BASE_GRIPPER 11
#define PORT_PNEUMATICS 'A'

// Wheels specifications
#define DIRECTION_FL_WHEEL false
#define DIRECTION_FR_WHEEL true
#define DIRECTION_BL_WHEEL true
#define DIRECTION_BR_WHEEL false
#define GEARSET_WHEELS AbstractMotor::gearset::green
#define ENCODER_UNIT_WHEELS AbstractMotor::encoderUnits::rotations

// Arm specifications
#define GEARSET_ARMS AbstractMotor::gearset::red
#define ENCODER_UNIT_ARMS AbstractMotor::encoderUnits::rotations
#define DIRECTION_R_ARM false
#define DIRECTION_L_ARM true

// Ring Mill specification
#define GEARSET_RING_MILL AbstractMotor::gearset::green
#define ENCODER_UNIT_RING_MILL AbstractMotor::encoderUnits::rotations
#define DIRECTION_RING_MILL false

// Proportions
#define WHEEL_DIAMETER 11_cm
#define WHEEL_TRACK 43_cm

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Test Robot");
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

void opcontrol()
{

	Motor motorFL = Motor(PORT_FL_WHEEL, DIRECTION_FL_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
	Motor motorFR = Motor(PORT_FR_WHEEL, DIRECTION_FR_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
	Motor motorBL = Motor(PORT_BL_WHEEL, DIRECTION_BL_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
	Motor motorBR = Motor(PORT_BR_WHEEL, DIRECTION_BR_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);

	const MotorGroup leftMotors = {motorBL, motorFL};
	const MotorGroup rightMotors = {motorBR, motorFR};

	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder().withMotors(leftMotors, rightMotors).withDimensions(GEARSET_WHEELS, {{WHEEL_DIAMETER, WHEEL_TRACK}, imev5GreenTPR}).build();

	Motor ringMillMotor(PORT_RING_MILL, DIRECTION_RING_MILL, GEARSET_RING_MILL, ENCODER_UNIT_RING_MILL);

	Controller controller;
	float speedLeftX, speedLeftY, speedRightX, speedRightY;
	bool r1_pressed;
	bool r2_pressed;
	bool x_pressed;
	bool x_already_pressed;

	IMU gyroscope(PORT_GYROSCOPE);
	RotationSensor armRotation(PORT_ARM_ROTATION);
	Motor motorArmLeft = Motor(PORT_L_ARM, DIRECTION_L_ARM, GEARSET_ARMS, ENCODER_UNIT_ARMS);
	Motor motorArmRight = Motor(PORT_R_ARM, DIRECTION_R_ARM, GEARSET_ARMS, ENCODER_UNIT_ARMS);

	gyroscope.reset();
	armRotation.reset();

	while (true)
	{

		pros::lcd::print(1, "Arm Angle: %d", armRotation.get());

		speedLeftY = controller.getAnalog(ControllerAnalog::leftY);
		speedLeftX = controller.getAnalog(ControllerAnalog::leftX);
		speedRightY = controller.getAnalog(ControllerAnalog::rightY);
		speedRightX = controller.getAnalog(ControllerAnalog::rightX);
		r1_pressed = controller.getDigital(ControllerDigital::R1);
		r2_pressed = controller.getDigital(ControllerDigital::R2);
		x_pressed = controller.getDigital(ControllerDigital::X);

		if (x_pressed)
		{
			if (x_already_pressed)
			{
				ringMillMotor.moveVelocity(0);
				x_already_pressed = false;
			}
			else
			{
				ringMillMotor.moveVelocity(200);
				x_already_pressed = true;
			}
		}

		if (r1_pressed)
		{

			motorArmRight.moveRelative(1.0, 100);
			motorArmLeft.moveRelative(1.0, 100);
		}
		else if (r2_pressed)
		{
			motorArmRight.moveRelative(-1.0, 100);
			motorArmLeft.moveRelative(-1.0, 100);
		}

		drive->getModel()->arcade(speedLeftY, speedLeftX);
	}
}
