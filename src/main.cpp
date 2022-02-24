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
#define PORT_BASE_GRIPPER_ROTATION 18
#define PORT_GYROSCOPE 20
#define PORT_RING_MILL 8
#define PORT_BASE_GRIPPER 11
#define PORT_PNEUMATICS 'A'
#define PORT_ARM_BUMPER 'F'

// Wheels specifications
#define WHEEL_DIRECTION_FL false
#define WHEEL_DIRECTION_FR true
#define WHEEL_DIRECTION_BL true
#define WHEEL_DIRECTION_BR false
#define WHEEL_GEARSET AbstractMotor::gearset::green
#define WHEEL_ENCODER_UNIT AbstractMotor::encoderUnits::rotations

// Arm specifications
#define ARM_GEARSET AbstractMotor::gearset::red
#define ARM_ENCODER_UNIT AbstractMotor::encoderUnits::degrees
#define ARM_DIRECTION_R false
#define ARM_DIRECTION_L true
#define ARM_GEAR_RATIO 6.95
#define ARM_POSITION_LOW 0
#define ARM_POSITION_DRIVE -200
#define ARM_POSITION_HIGH -500

// Ring Mill specification
#define RING_MILL_GEARSET AbstractMotor::gearset::green
#define RING_MILL_ENCODER_UNIT AbstractMotor::encoderUnits::rotations
#define RING_MILL_DIRECTION false
#define RING_MILL_MAX_VELOCITY 200

// Base Gripper specification
#define BASE_GRIPPER_DIRECTION true
#define BASE_GRIPPER_GEARSET AbstractMotor::gearset::green
#define BASE_GRIPPER_ENCODER_UNIT AbstractMotor::encoderUnits::degrees
#define BASE_GRIPPER_POSITION_LOW 500
#define BASE_GRIPPER_POSITION_DRIVE 182
#define BASE_GRIPPER_POSITION_HIGH 0

// Proportions
#define WHEEL_DIAMETER 11_cm
#define WHEEL_TRACK 43_cm

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor ringMillMotor = Motor(PORT_RING_MILL, RING_MILL_DIRECTION, RING_MILL_GEARSET, RING_MILL_ENCODER_UNIT);
pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
IMU gyroscope = IMU(PORT_GYROSCOPE);
RotationSensor baseGripperRotation = RotationSensor(PORT_BASE_GRIPPER_ROTATION);
Motor motorArmLeft = Motor(PORT_L_ARM, ARM_DIRECTION_L, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorArmRight = Motor(PORT_R_ARM, ARM_DIRECTION_R, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorBaseGripper = Motor(PORT_BASE_GRIPPER, BASE_GRIPPER_DIRECTION, BASE_GRIPPER_GEARSET, BASE_GRIPPER_ENCODER_UNIT);
std::shared_ptr<ChassisController> drive;
ADIButton armBumper = ADIButton(PORT_ARM_BUMPER);
ADIButton armEndStop = ADIButton(PORT_ARM_BUMPER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	motorBaseGripper.setBrakeMode(AbstractMotor::brakeMode::hold);
	const MotorGroup leftMotors = {motorBL, motorFL};
	const MotorGroup rightMotors = {motorBR, motorFR};
	drive = ChassisControllerBuilder()
				.withMotors(leftMotors, rightMotors)
				.withDimensions(WHEEL_GEARSET, {{WHEEL_DIAMETER, WHEEL_TRACK}, imev5GreenTPR})
				.build();

	motorArmLeft.moveRelative(-100, 100);
	motorArmRight.moveRelative(-100, 100);
	pros::delay(400);
	while (!armEndStop.isPressed())
	{
		motorArmLeft.moveRelative(20, 100);
		motorArmRight.moveRelative(20, 100);
	}
	motorArmLeft.moveRelative(-20, 50);
	motorArmRight.moveRelative(-20, 50);
	pros::delay(400);
	while (!armEndStop.isPressed())
	{
		motorArmLeft.moveRelative(1, 50);
		motorArmRight.moveRelative(1, 50);
	}
	motorArmLeft.tarePosition();
	motorArmRight.tarePosition();
	while (baseGripperRotation.get() > 2){
		motorBaseGripper.moveRelative(10,50);
	}
	motorBaseGripper.moveRelative(-1,50);
	while (baseGripperRotation.get() > 2){
		motorBaseGripper.moveRelative(5,50);
	}
	motorBaseGripper.tarePosition();
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
	bool pneumaticActivated = false;
	bool ringMillActivated = false;
	bool debounceY = false;
	bool debounceX = false;
	bool debounceR1 = false;
	bool debounceR2 = false;
	bool debounceL1 = false;
	bool debounceL2 = false;
	int armPosition = 0;
	int baseGripperPosition = 2;

	while (execute)
	{
		if (controller.getDigital(ControllerDigital::R1))
		{
			if (armPosition < 2 && !debounceR1)
			{
				armPosition++;
				debounceR1 = true;
			}
		}
		else
		{
			debounceR1 = false;
		}
		if (controller.getDigital(ControllerDigital::R2))
		{
			if (armPosition > 0  && !debounceR2)
			{
				armPosition--;
				debounceR2 = true;
			}
		}
		else
		{
			debounceR2 = false;
		}
		if (controller.getDigital(ControllerDigital::L1))
		{
			if (baseGripperPosition < 2 && !debounceL1)
			{
				baseGripperPosition++;
				debounceL1 = true;
			}
		}else{
			debounceL1 = false;
		}
		if (controller.getDigital(ControllerDigital::L2))
		{
			if (baseGripperPosition > 0 && !debounceL2)
			{
				baseGripperPosition--;
				debounceL2 = true;
			}
		}else{
			debounceL2 = false;
		}
		if (controller.getDigital(ControllerDigital::A))
		{
			execute = false;
		}
		if (controller.getDigital(ControllerDigital::Y))
		{
			if (!debounceY)
			{
				pneumaticActivated = !pneumaticActivated;
				debounceY = true;
			}
		}
		else
		{
			debounceY = false;
		}
		if (controller.getDigital(ControllerDigital::X))
		{
			if (!debounceX)
			{
				ringMillActivated = !ringMillActivated;
				debounceX = true;
			}
		}
		else
		{
			debounceX = false;
		}

		if (pneumaticActivated)
		{
			pneumatic.set_value(1);
		}
		else
		{
			pneumatic.set_value(0);
		}

		if (ringMillActivated)
		{
			ringMillMotor.moveVelocity(RING_MILL_MAX_VELOCITY);
		}
		else
		{
			ringMillMotor.moveVelocity(0);
		}
		pros::lcd::print(2, "base gripper: %d", baseGripperPosition);
		pros::lcd::print(3,"Rotation sensor value: %.2f",baseGripperRotation.get());
		if (armPosition == 0)
		{
			motorArmLeft.moveAbsolute(ARM_POSITION_LOW, 50);
			motorArmRight.moveAbsolute(ARM_POSITION_LOW, 50);
		}
		else if (armPosition == 1)
		{
			motorArmLeft.moveAbsolute(ARM_POSITION_DRIVE, 50);
			motorArmRight.moveAbsolute(ARM_POSITION_DRIVE, 50);
		}
		else if (armPosition == 2)
		{
			motorArmLeft.moveAbsolute(ARM_POSITION_HIGH, 50);
			motorArmRight.moveAbsolute(ARM_POSITION_HIGH, 50);
		}
		if (baseGripperPosition == 0)
		{
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_LOW,50);
		}
		else if (baseGripperPosition == 1)
		{
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_DRIVE,50);
		}
		else if (baseGripperPosition == 2)
		{
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_HIGH,50);
		}

		drive->getModel()->arcade(
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::leftX));

		pros::delay(20);
	}
}
