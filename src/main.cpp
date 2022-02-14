#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;

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
#define ENCODER_UNIT_ARMS AbstractMotor::encoderUnits::degrees
#define DIRECTION_R_ARM false
#define DIRECTION_L_ARM true

// Ring Mill specification
#define GEARSET_RING_MILL AbstractMotor::gearset::green
#define ENCODER_UNIT_RING_MILL AbstractMotor::encoderUnits::rotations
#define DIRECTION_RING_MILL false
#define MAX_VELOCITY_RING_MILL 200

// Base Gripper specification
#define DIRECTION_BASE_GRIPPER true
#define GEARSET_BASE_GRIPPER AbstractMotor::gearset::green
#define ENCODER_UNIT_BASE_GRIPPER AbstractMotor::encoderUnits::degrees
#define GEAR_RATIO_BASE_GRIPPER 5

// Proportions
#define WHEEL_DIAMETER 11_cm
#define WHEEL_TRACK 43_cm

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, DIRECTION_FL_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
Motor motorFR = Motor(PORT_FR_WHEEL, DIRECTION_FR_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
Motor motorBL = Motor(PORT_BL_WHEEL, DIRECTION_BL_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
Motor motorBR = Motor(PORT_BR_WHEEL, DIRECTION_BR_WHEEL, GEARSET_WHEELS, ENCODER_UNIT_WHEELS);
Motor ringMillMotor = Motor(PORT_RING_MILL, DIRECTION_RING_MILL, GEARSET_RING_MILL, ENCODER_UNIT_RING_MILL);
pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
IMU gyroscope = IMU(PORT_GYROSCOPE);
RotationSensor armRotation = RotationSensor(PORT_ARM_ROTATION);
Motor motorArmLeft = Motor(PORT_L_ARM, DIRECTION_L_ARM, GEARSET_ARMS, ENCODER_UNIT_ARMS);
Motor motorArmRight = Motor(PORT_R_ARM, DIRECTION_R_ARM, GEARSET_ARMS, ENCODER_UNIT_ARMS);
Motor motorElevator = Motor(PORT_BASE_GRIPPER, DIRECTION_BASE_GRIPPER, GEARSET_BASE_GRIPPER, ENCODER_UNIT_BASE_GRIPPER);
std::shared_ptr<ChassisController> drive;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	motorElevator.setBrakeMode(AbstractMotor::brakeMode::hold);
	const MotorGroup leftMotors = {motorBL, motorFL};
	const MotorGroup rightMotors = {motorBR, motorFR};
	drive = ChassisControllerBuilder()
				.withMotors(leftMotors, rightMotors)
				.withDimensions(GEARSET_WHEELS, {{WHEEL_DIAMETER, WHEEL_TRACK}, imev5GreenTPR})
				.build();
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
	bool execute = true;
	bool pneumatic_activated = false;
	bool pneumatic_debounce = false;
	bool ringmill_activated = false;
	bool ringmill_debounce = false;

	while (execute)
	{
		if (controller.getDigital(ControllerDigital::R1))
		{
			motorArmRight.moveRelative(1.0, 100);
			motorArmLeft.moveRelative(1.0, 100);
		}
		if (controller.getDigital(ControllerDigital::R2))
		{
			motorArmRight.moveRelative(-1.0, 100);
			motorArmLeft.moveRelative(-1.0, 100);
		}
		if (controller.getDigital(ControllerDigital::L2))
		{
			motorElevator.moveRelative(30 * GEAR_RATIO_BASE_GRIPPER, 30);
		}
		if (controller.getDigital(ControllerDigital::L1))
		{
			motorElevator.moveRelative(-30 * GEAR_RATIO_BASE_GRIPPER, 30);
		}
		if (controller.getDigital(ControllerDigital::A))
		{
			execute = false;
		}
		if (controller.getDigital(ControllerDigital::Y))
		{
			if (!pneumatic_debounce) {
				pneumatic_activated = !pneumatic_activated;
				pneumatic_debounce = true;
			}
		} else {
			pneumatic_debounce = false;
		}
		if (controller.getDigital(ControllerDigital::X))
		{
			if (!ringmill_debounce) {
				ringmill_activated = !ringmill_activated;
				ringmill_debounce = true;
			}
		} else {
			ringmill_debounce = false;
		}

		if (pneumatic_activated) {
			pneumatic.set_value(1);
		} else {
			pneumatic.set_value(0);
		}

		if (ringmill_activated) {
			ringMillMotor.moveVelocity(MAX_VELOCITY_RING_MILL);
		} else {
			ringMillMotor.moveVelocity(0);
		}

		drive->getModel()->arcade(
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::leftX));

		pros::delay(20);
	}
}
