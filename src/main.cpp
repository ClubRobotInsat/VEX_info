#include "main.h"
#include "okapi/api.hpp"

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
#define WHEEL_DIAMETER 102_mm * 84 / 60
#define WHEEL_TRACK 380_mm

#define X_TARGET 200
#define Y_TARGET 500

// Threshold for sensors
#define FRONT_THRESHOLD 300
#define LEFT_INFERIOR_THRESHOLD 300
#define LEFT_SUPERIOR_THRESHOLD 400
#define RIGHT_THRESHOLD 400

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

// Global position of the robot and the target
std::pair<double, double> robotPosition(0, 0);
std::pair<double, double> targetPosition(X_TARGET, Y_TARGET);

// Enum for next move algorithms
enum ALGORITHM
{
	BUG0,
	BUG1,
	BUG2
};

enum SENSORS
{
	LEFT,
	MIDDLE,
	RIGHT
};

// std::pair<double,double> bug0(double currentAngle, std::tuple<double, double, double> sensorsDistance){
// 	double dx = targetPosition.first - robotPosition.first;
// 	double dy = targetPosition.second - robotPosition.second;
// 	// 0 - 255 but result between 0. 1.0
// 	double teta = atan2(targetPosition.second,targetPosition.first)-currentAngle;
// 	double innerRotation = 0;
// 	double initX = robotPosition.first;
// 	double initY = robotPosition.second;
// 	double xHitPoint = 0;
// 	double yHitPoint = 0;
// 	double moveDist = 50;
// 	double moveAngle = 0;

// 	if (std::get<LEFT>(sensorsDistance) < LEFT_THRESHOLD)
// 	{
// 		moveAngle = teta;
// 		moveDist = 0;
// 	}
// 	if (std::get<MIDDLE>(sensorsDistance) < FRONT_THRESHOLD){
// 		moveAngle = 45;
// 	}
// 	if (std::get<LEFT>(sensorsDistance)< FRONT_THRESHOLD){

// 		if (ultraSonicMiddle.get() < FRONT_THRESHOLD){ // if obstacle encountered (< front threshold)
// 			// Rotate right big
// 			moveToAngle(0.0, 90.0, 5.0);
// 			// Modify increment
// 			innerRotation += 45;
// 			while((ultraSonicLeft.get() < LEFT_NO_OBSTACLE) && (ultraSonicMiddle.get() > FRONT_THRESHOLD) && (dx > 1) && (dy >1)){
// 				// Forward
// 				moveStraight(50.0);
// 				// Modify increment
// 				// TODO - Update position
// 				if (ultraSonicLeft.get()<LEFT_THRESHOLD){
// 					while(ultraSonicLeft.get()<LEFT_THRESHOLD){
// 						// Rotate right
// 						moveToAngle(0.0, 5.0, 0.5);
// 						// Modify rotation increment
// 						innerRotation += 5.0;
// 					}
// 				}

// 			}

// 		}else{
// 			teta = 256 * arctan2(dx,dy) + innerRotation;
// 			// Rotates towards goal
// 			moveToAngle(0.0, teta, 0.5);
// 			// Forward
// 			moveStraight(50.0);
// 			// Modify increment
// 			// TODO - Update position
// 		}

// 	}

// }

// Switch with angle instead of position
// TODO - Add hitpoint comparation

// Global variables for bug2 memory equations
bool createdLine = false;
double alphaLine = -1;
double betaLine = -1;
double foundObstacle = false;
bool firstEncounter = true;

std::pair<double, double> bug2(double currentAngle, std::tuple<double, double, double> sensorsDistance)
{
	double dx = targetPosition.first - robotPosition.first;
	double dy = targetPosition.second - robotPosition.second;
	// Target - Position
	double teta = 90 - atan2(dy, dx) * 360 / (2 * PI);
	double moveDist = 0;
	double moveAngle = 0;
	std::pair<double,double> hitPoint;

	if (abs(dx) < 5 && abs(dy) < 5){
		return std::make_pair(0,currentAngle);
	}

	// Obstacle following mode
	if (foundObstacle)
	{
		if ( std::get<LEFT>(sensorsDistance)-LEFT_INFERIOR_THRESHOLD < 5 || std::get<MIDDLE>(sensorsDistance) - FRONT_THRESHOLD < 5) // too close from the obstacle
		{
			moveAngle = currentAngle + 35;
			pros::lcd::print(5, "Too close");
		}
		else if (std::get<LEFT>(sensorsDistance)-LEFT_INFERIOR_THRESHOLD < 10 && std::get<MIDDLE>(sensorsDistance) - FRONT_THRESHOLD > -5) // too close from the obstacle
		{
			moveAngle = currentAngle;
			moveDist = 50;
			pros::lcd::print(5, "a cote");
		}
		else if (std::get<LEFT>(sensorsDistance)-LEFT_SUPERIOR_THRESHOLD > -5) // Too far from obstacle
		{
			pros::lcd::print(5, "Too far");
			moveAngle = currentAngle - 35;
		}
		else
		{
			pros::lcd::print(5, "RAS");
			moveDist = 50;
			moveAngle = currentAngle;
		}

		if ((robotPosition.second - (robotPosition.first * alphaLine + betaLine) >= 5) && !firstEncounter)
		{
			pros::lcd::print(5, "Encountered Line");
			moveAngle = teta;
			moveDist = 50;
			// Quit obstacle
			foundObstacle = false;
			firstEncounter = true;
		}
		return std::make_pair(moveDist, moveAngle);
	}

	// Normal mode

	if (!createdLine) // Create m-line to follow
	{
		pros::lcd::print(5, "Creating Line");
		alphaLine = dy / dx;
		betaLine = targetPosition.second - (alphaLine * targetPosition.first);
		// Rotate towards target
		moveAngle = teta;
		createdLine = true;
	}

	if (std::get<MIDDLE>(sensorsDistance) < FRONT_THRESHOLD) // Encountered obstacle
	{
		pros::lcd::print(5, "Encountered obstacle");
		foundObstacle = true;
		moveAngle = currentAngle + 10;
		hitPoint = robotPosition;
		firstEncounter = true;
	}
	else
	{
		pros::lcd::print(5, "Nothing in front");
		pros::lcd::print(6, "teta : %.2f",teta);
		moveAngle = teta;
		moveDist = 50;
	}

	return std::make_pair(moveDist, moveAngle);
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
	double diff = desiredAngle - currentAngle;
	while (diff < 0)
	{
		diff += 360;
	}
	if (abs(diff) > precision)
	{
		drive->turnAngle(QAngle(((int)(diff + 180) % 360 - 180) * degree));
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

// return next move in polar coordinates
std::pair<double, double> getStrategyNextMove(
	ALGORITHM algo,
	double currentAngle,
	std::tuple<double, double, double> sensorsDistance)
{
	double moveDist = 0;
	double moveAngle = 0;
	std::pair<double, double> nextMovement;

	switch (algo)
	{
	case BUG2:
		nextMovement = bug2(currentAngle, sensorsDistance);
		break;

	default:
		break;
	}

	return nextMovement;
}

void recalculatePosition(double distance, double angle)
{
	robotPosition.first += distance * sin(angle * 2 * PI / 360);
	robotPosition.second += distance * cos(angle * 2 * PI / 360);
}

void opcontrol()
{
	bool execute = true;
	double desiredAngle = 10;
	double maxAngleError = 5.0;
	double desiredDistance = 200;
	double maxDistanceError = 10;
	double currentAngle = 0;
	std::tuple<double, double, double> sensorsDistance;
	std::pair<double, double> nextMove(0, 0);

	while (execute)
	{
		sensorsDistance = {ultraSonicLeft.get(), ultraSonicMiddle.get(), ultraSonicRight.get()};

		// DEBUG
		pros::lcd::print(0, "UltrasonicLeft %.2f mm", std::get<LEFT>(sensorsDistance));
		pros::lcd::print(1, "UltrasonicMiddle %.2f mm", std::get<MIDDLE>(sensorsDistance));
		pros::lcd::print(2, "UltrasonicRight %.2f mm", std::get<RIGHT>(sensorsDistance));
		pros::lcd::print(3, "gyroscope %.2f degrees", currentAngle);
		pros::lcd::print(4, "current pos %.2f %.2f", robotPosition.first, robotPosition.second);

		nextMove = getStrategyNextMove(BUG2, currentAngle, sensorsDistance);
		moveToAngle(currentAngle, nextMove.second, maxAngleError);
		moveStraight(nextMove.first);
		currentAngle = gyroscope.get();
		recalculatePosition(nextMove.first, currentAngle);

		// Delay between iteraction
		pros::delay(2000);
	}
}
