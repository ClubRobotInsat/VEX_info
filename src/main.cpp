#include "main.h"
#include "okapi/api.hpp"
#include <ctgmath>

using namespace okapi;

// Sensors and Actuators ports
#define PORT_FL_WHEEL 1
#define PORT_FR_WHEEL 2
#define PORT_BL_WHEEL 3
#define PORT_BR_WHEEL 4
#define PORT_GYROSCOPE 20
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

// Proportions
#define WHEEL_DIAMETER 16_cm
#define WHEEL_TRACK 43_cm

// sensors and actuators
Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
ADIUltrasonic ultraSonicLeft = ADIUltrasonic(PORT_SENSOR1_TRIGGER, PORT_SENSOR1_DATA);
ADIUltrasonic ultraSonicMiddle = ADIUltrasonic(PORT_SENSOR2_TRIGGER, PORT_SENSOR2_DATA);
ADIUltrasonic ultraSonicRight = ADIUltrasonic(PORT_SENSOR3_TRIGGER, PORT_SENSOR3_DATA);
IMU gyroscope = IMU(PORT_GYROSCOPE, okapi::IMUAxes::z);
std::shared_ptr<ChassisController> drive;

// Global position of the robot
double xRobot;
double yRobot;

// Bug 0 algorithm
void bug0(double xGoal, double yGoal)
{
	// TODO - Update defined values
	// #define FRONT_THRESHOLD 5
	// #define LEFT_THRESHOLD 5
	// #define RIGHT_THRESHOLD 5
	// #define LEFT_NO_OBSTACLE 5000

	// double dx = xGoal - xRobot;
	// double dy = yGoal - yRobot;
	// // 0 - 255 but result between 0. 1.0
	// double teta = 0;
	// double innerRotation = 0;

	// while (dx > 1 and dy > 1){ // while not arrived
	// 	dx = xGoal - xRobot;
	// 	dy = yGoal - yRobot;
	// 	if (ultraSonic1.get() < FRONT_THRESHOLD){ // if obstacle encountered (< front threshold)
	// 		// TODO - Rotate right big
	// 		// TODO - Modify increment
	// 		innerRotation += 45;
	// 		while((ultraSonic2.get() < LEFT_NO_OBSTACLE) && (ultraSonic1.get() > FRONT_THRESHOLD) && (dx > 1) && (dy >1)){
	// 			// TODO - Forward
	// 			// TODO - Modify increment
	// 			xRobot += 5;
	// 			yRobot += 5;
	// 			if (ultraSonic2.get()<LEFT_THRESHOLD){
	// 				while(ultraSonic2.get()<LEFT_THRESHOLD){
	// 					// TODO - Rotate right
	// 					// TODO - Modify rotation increment
	// 					innerRotation += 5;
	// 				}
	// 			}

	// 		}

	// 	}else{
	// 		teta = 256 * arctan2(dx,dy) + innerRotation;
	// 		// TODO -  Rotates towards goal
	// 		// TODO - Forward
	// 		// TODO - Modify increment
	// 		xRobot += 5;
	// 		yRobot += 5;
	// 	}

	// }
}
// TODO - Complete
void bug1(double xGoal, double yGoal)
{
	// double dx = xGoal - xRobot;
	// double dy = yGoal - yRobot;
	// // 0 - 255 but result between 0. 1.0
	// double teta = 0;
	// double innerRotation = 0;
	// double initX = xRobot;
	// double initY = yRobot;

	// while (dx > 1 and dy > 1){ // while not arrived
	// 	dx = xGoal - xRobot;
	// 	dy = yGoal - yRobot;
	// 	if (ultraSonic1.get() < FRONT_THRESHOLD){

	// 	}else{
	// 		teta = 256 * arctan2(dx,dy) + innerRotation;
	// 		// TODO -  Rotates towards goal
	// 		// TODO - Forward
	// 		// TODO - Modify increment
	// 		xRobot += 5;
	// 		yRobot += 5;
	// 	}
	// }

	// Same as bug0 except the robot moves around the WHOLE object and goes back to closest point from the goal
	// Rotates towards the goal
	// while not arrived
	// Forward
	// if obstacle encountered
	// While not full loop around obstacle or not arrived
	// Record position
	// Compare to distance
	// While < front threshold or < left threshold
	// Rotate right
	// Forward
	// Go to closest position
}

void bug2()
{
	// Rotates towards goal
	// while not arrived
	// Follow m-line -> straight line towards goal
	// if obstacle encountered
	// while not arrived and not initial position and not m-line is re-encountered and d(x,goal) < d(obstacleX,goal) and no obstacle in front
	// While < front threshold or < left threshold
	// Rotate right
	// Forward
}

// n being a point somewhere
// heurisitc = distance(x,n) + distance(n,goal)
void tangentBug()
{
	// While not arrived
	// if obstacle encountered or heurisitc increases
	// while not arrived and not initial position (obstacle) and not d_leave < d_min
	// d_min = shortest distance observed so far between sensed obstacle and the goal
	// d_leave = shortest distance between any point in the sensed environment and the goal
	// else
	// choose direction that minize heurisitc
}

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
	gyroscope.reset();
	while (gyroscope.isCalibrating())
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

void moveStraight(double distance)
{
	drive->moveDistance(distance * millimeter);
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
