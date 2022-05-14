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
#define PORT_SENSOR1_TRIGGER 'A'
#define PORT_SENSOR1_DATA 'B'
#define PORT_SENSOR2_TRIGGER 'E'
#define PORT_SENSOR2_DATA 'F'
#define PORT_SENSOR3_TRIGGER 'G'
#define PORT_SENSOR3_DATA 'H'

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
#define BASE_GRIPPER_POSITION_LOW 550
#define BASE_GRIPPER_POSITION_DRIVE 190
#define BASE_GRIPPER_POSITION_HIGH 0

// Proportions
#define WHEEL_DIAMETER 16_cm
#define WHEEL_TRACK 43_cm

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor ringMillMotor = Motor(PORT_RING_MILL, RING_MILL_DIRECTION, RING_MILL_GEARSET, RING_MILL_ENCODER_UNIT);
// pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
ADIUltrasonic ultraSonicLeft = ADIUltrasonic(PORT_SENSOR1_TRIGGER, PORT_SENSOR1_DATA);
ADIUltrasonic ultraSonicMiddle = ADIUltrasonic(PORT_SENSOR2_TRIGGER, PORT_SENSOR2_DATA);
ADIUltrasonic ultraSonicRight = ADIUltrasonic(PORT_SENSOR3_TRIGGER, PORT_SENSOR3_DATA);
IMU gyroscope = IMU(PORT_GYROSCOPE, okapi::IMUAxes::z);
RotationSensor baseGripperRotation = RotationSensor(PORT_BASE_GRIPPER_ROTATION);
Motor motorArmLeft = Motor(PORT_L_ARM, ARM_DIRECTION_L, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorArmRight = Motor(PORT_R_ARM, ARM_DIRECTION_R, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorBaseGripper = Motor(PORT_BASE_GRIPPER, BASE_GRIPPER_DIRECTION, BASE_GRIPPER_GEARSET, BASE_GRIPPER_ENCODER_UNIT);
std::shared_ptr<ChassisController> drive;
// ADIButton armBumper = ADIButton(PORT_ARM_BUMPER);
// ADIButton armEndStop = ADIButton(PORT_ARM_BUMPER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::print(0, "Initializing");
	drive = ChassisControllerBuilder()
				.withMotors(motorFL, motorFR, motorBR, motorBL)
				.withDimensions(WHEEL_GEARSET, {{WHEEL_DIAMETER, WHEEL_TRACK}, imev5GreenTPR})
				.build();
	pros::delay(2000);
	while (ultraSonicMiddle.get() == 0)
		;
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
void autonomous()
{
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

void moveToAngle(double currentAngle, double desiredAngle, double precision)
{
	if (abs(currentAngle - desiredAngle) > precision)
	{
		drive->turnAngle(QAngle(-(currentAngle - desiredAngle) * degree));
	}
}

void moveToDistance(double currentDistance, double desiredDistance, double precision)
{
	if (abs(currentDistance - desiredDistance) > precision)
	{
		drive->moveDistance((currentDistance - desiredDistance) * millimeter);
	}
}

void moveStraight(double distance) {
	drive->moveDistance(distance *millimeter);
}

void opcontrol()
{
	bool execute = true;
	double desiredAngle = 10;
	double maxAngleError = 5;
	double desiredDistance = 200;
	double maxDistanceError = 10;
	double currentAngle;
	double leftDistance;
	double middleDistance;
	double rightDistance;

	while (execute)
	{
		leftDistance = ultraSonicLeft.get();
		middleDistance = ultraSonicMiddle.get();
		rightDistance = ultraSonicRight.get();
		currentAngle = gyroscope.get();

		// DEBUG
		pros::lcd::print(0, "UltrasonicLeft %.2f mm", leftDistance);
		pros::lcd::print(1, "UltrasonicMiddle %.2f mm", middleDistance);
		pros::lcd::print(2, "UltrasonicRight %.2f mm", rightDistance);
		pros::lcd::print(3, "gyroscope %.2f degrees", currentAngle);

		moveToAngle(currentAngle, desiredAngle, maxAngleError);
		moveToDistance(middleDistance, desiredDistance, maxDistanceError);
		moveStraight(10);

		// Delay between iteraction
		pros::delay(50);
	}
}
