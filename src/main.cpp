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

// Target position
#define X_TARGET 200
#define Y_TARGET 2000

// Threshold for sensors
#define SENSOR_THRESHOLD 300
#define CRITICAL_SENSOR_THRESHOLD 150

#define MOVE_STEP 75
#define TURN_STEP 15

#define RAD_TO_DEGREE(v) v * 360 / (2 * PI)
#define DEGREE_TO_RAD(v) v *(2 * PI) / 360

#define MOVEMENT_ALGO SEQUENCE

#define RAD_TO_DEGREES(r) r * 360 / (2 * PI)
#define DEGREES_TO_RAD(r) r * 2 * PI / 360

// sensors and actuators
Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
ADIUltrasonic ultraSonicLeft = ADIUltrasonic(PORT_SENSOR1_TRIGGER, PORT_SENSOR1_DATA);
ADIUltrasonic ultraSonicMiddle = ADIUltrasonic(PORT_SENSOR3_TRIGGER, PORT_SENSOR3_DATA);
ADIUltrasonic ultraSonicRight = ADIUltrasonic(PORT_SENSOR3_TRIGGER, PORT_SENSOR3_DATA);
IMU gyroscope = IMU(PORT_GYROSCOPE, okapi::IMUAxes::z);
std::shared_ptr<ChassisController> drive;

// Global position of the robot and the target
std::pair<double, double> robotPosition(0, 0);
std::pair<double, double> targetPosition(X_TARGET, Y_TARGET);

// Enum for next move algorithms
enum ALGORITHM
{
	SEQUENCE,
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

// Switch with angle instead of position
// TODO - Add hitpoint comparation

// Global variables for bug2 memory equations
bool createdLine = false;
double alphaLine = -1;
double betaLine = -1;

bool foundObstacle = false;

bool checkLeave = false;
bool wasTooClose = false;
// Coordinates of the closest point to the target next to the obstacle
std::pair<double, double> min(10000, 10000);

std::pair<double, double> bug0(double currentAngle, std::tuple<double, double, double> sensorsDistance)
{
	double dx = targetPosition.first - robotPosition.first;
	double dy = targetPosition.second - robotPosition.second;
	// Target - Position

	double teta = double(int64_t(90. - RAD_TO_DEGREES(atan2(dy, dx)) + 180) % 360 - 180);
	double moveDist = 0;
	double moveAngle = 0;
	std::pair<double, double> hitPoint;
	double diff;
	bool passedObstacle = false;

	pros::lcd::print(6, "Teta: %.2f  currentAngle: %.2f", teta, currentAngle);

	if (abs(dx) < (MOVE_STEP + 100) && abs(dy) < (MOVE_STEP + 100))
	{
		return std::make_pair(0, currentAngle);
	}

	if (std::get<MIDDLE>(sensorsDistance) > SENSOR_THRESHOLD && std::get<LEFT>(sensorsDistance) > SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		if (wasTooClose)
		{
			pros::lcd::print(5, "Straight left obst");
			moveDist = MOVE_STEP;
			moveAngle = currentAngle;
			wasTooClose = false;
		}
		else
		{
			pros::lcd::print(5, "Straight norPORT_SENSOR2_TRIGGERmal");
			moveDist = MOVE_STEP;
			diff = teta - currentAngle;
			moveAngle = (abs(diff) < 10) ? teta : (diff >= 0 ? currentAngle + 10 : currentAngle - 10);
		}
	}
	else if (std::get<MIDDLE>(sensorsDistance) > SENSOR_THRESHOLD && std::get<LEFT>(sensorsDistance) < SENSOR_THRESHOLD + 50 && std::get<LEFT>(sensorsDistance) > CRITICAL_SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		pros::lcd::print(5, "A cote");
		moveDist = MOVE_STEP;
		moveAngle = currentAngle;
	}
	else if (std::get<LEFT>(sensorsDistance) < CRITICAL_SENSOR_THRESHOLD || std::get<MIDDLE>(sensorsDistance) < SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		pros::lcd::print(5, "Too close");
		moveAngle = TURN_STEP + currentAngle;
		wasTooClose = true;
	}
	else
	{
		pros::lcd::print(5, "else");
		moveDist = MOVE_STEP;
		moveAngle = currentAngle;
	}

	return std::make_pair(moveDist, moveAngle);
}

bool bypassed = true;
bool firstEncounter = true;
std::pair<double, double> bug1(double currentAngle, std::tuple<double, double, double> sensorsDistance)
{
	double dx = targetPosition.first - robotPosition.first;
	double dy = targetPosition.second - robotPosition.second;
	// Target - Position

	double teta = double(int64_t(90. - RAD_TO_DEGREES(atan2(dy, dx)) + 180) % 360 - 180);
	double moveDist = 0;
	double moveAngle = 0;
	std::pair<double, double> hitPoint;
	double diff;
	bool passedObstacle = false;

	if (abs(dx) < (MOVE_STEP + 100) && abs(dy) < (MOVE_STEP + 100))
	{
		return std::make_pair(0, currentAngle);
		pros::lcd::print(6, "Arrived");
	}

	if (abs(robotPosition.first-hitPoint.first) < 20  && abs(robotPosition.second - hitPoint.second) < 20 && !firstEncounter){
		bypassed = true;
	}

	if (!bypassed){
		if((dx*dx+dy*dy) < (min.first*min.first+min.second*min.second)){
			min = robotPosition;
		}
	}

	if (std::get<MIDDLE>(sensorsDistance) > SENSOR_THRESHOLD && std::get<LEFT>(sensorsDistance) > SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		if (wasTooClose)
		{
			pros::lcd::print(5, "Straight left obst");
			moveDist = MOVE_STEP;
			moveAngle = currentAngle;
			wasTooClose = false;
		}
		else
		{

			if (!bypassed)
			{
				pros::lcd::print(5, "not bypassed");
				moveDist = MOVE_STEP;
				moveAngle = currentAngle - TURN_STEP;
			}else{
				pros::lcd::print(5, "Straight normal");
				moveDist = MOVE_STEP;
				diff = teta - currentAngle;
				moveAngle = (abs(diff) < 10) ? teta : (diff >= 0 ? currentAngle + 10 : currentAngle - 10);
			}
		}


	}
	else if (std::get<MIDDLE>(sensorsDistance) > SENSOR_THRESHOLD && std::get<LEFT>(sensorsDistance) < SENSOR_THRESHOLD + 50 && std::get<LEFT>(sensorsDistance) > CRITICAL_SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		pros::lcd::print(5, "A cote");
		moveDist = MOVE_STEP;
		moveAngle = currentAngle;
	}
	else if (std::get<LEFT>(sensorsDistance) < CRITICAL_SENSOR_THRESHOLD || std::get<MIDDLE>(sensorsDistance) < SENSOR_THRESHOLD && (std::get<MIDDLE>(sensorsDistance)) && (std::get<LEFT>(sensorsDistance)))
	{
		pros::lcd::print(5, "Too close");
		moveAngle = TURN_STEP + currentAngle;
		wasTooClose = true;
		if (bypassed && firstEncounter){
			hitPoint = robotPosition;
			firstEncounter = false;
			bypassed = false;
		}
	}
	else
	{
		pros::lcd::print(5, "else");
		moveDist = 0;
		moveAngle = currentAngle;
	}

	if (bypassed && abs(min.first-robotPosition.first)<50 && abs(min.second-robotPosition.second)){
		pros::lcd::print(5, "towards the target yeee");
		moveAngle = teta;
		firstEncounter = true;
	}

	pros::lcd::print(6, "Bypassed: %d",bypassed);



	return std::make_pair(moveDist, moveAngle);
}

std::pair<double, double> bug2(double currentAngle, std::tuple<double, double, double> sensorsDistance)
{
	double dx = targetPosition.first - robotPosition.first;
	double dy = targetPosition.second - robotPosition.second;
	// Target - Position
	double teta = 90 - RAD_TO_DEGREE(atan2(dy, dx));
	double moveDist = 0;
	double moveAngle = 0;
	std::pair<double, double> hitPoint;

	if (abs(dx) < MOVE_STEP && abs(dy) < MOVE_STEP)
	{
		pros::lcd::print(5, "Arrived");
		return std::make_pair(0, currentAngle);
	}



	// Obstacle following mode
	if (foundObstacle)
	{
		if (
			(std::get<LEFT>(sensorsDistance) - CRITICAL_SENSOR_THRESHOLD) < MOVE_STEP || ((std::get<MIDDLE>(sensorsDistance) - SENSOR_THRESHOLD) < MOVE_STEP && std::get<MIDDLE>(sensorsDistance)))
		{
			moveAngle = currentAngle + TURN_STEP;
			pros::lcd::print(5, "Too close");
		}
		else if (
			(std::get<LEFT>(sensorsDistance) - CRITICAL_SENSOR_THRESHOLD) < MOVE_STEP && (std::get<MIDDLE>(sensorsDistance) - SENSOR_THRESHOLD) > MOVE_STEP && std::get<MIDDLE>(sensorsDistance))
		{
			moveAngle = currentAngle;
			moveDist = MOVE_STEP;
			pros::lcd::print(5, "Following");
		}
		else if (std::get<LEFT>(sensorsDistance) - SENSOR_THRESHOLD > MOVE_STEP)
		{
			moveAngle = currentAngle - TURN_STEP;
			moveDist = MOVE_STEP;
			pros::lcd::print(5, "Too far");
		}
		else
		{
			moveDist = MOVE_STEP;
			moveAngle = currentAngle;
			pros::lcd::print(5, "RAS");
		}

		if ((robotPosition.second - (robotPosition.first * alphaLine + betaLine) >= MOVE_STEP) && !firstEncounter)
		{
			pros::lcd::print(5, "Encountered Line");
			moveAngle = teta;
			moveDist = MOVE_STEP;
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

	if (std::get<MIDDLE>(sensorsDistance) < SENSOR_THRESHOLD && std::get<MIDDLE>(sensorsDistance)) // Encountered obstacle
	{
		pros::lcd::print(5, "Encountered obstacle");
		foundObstacle = true;
		moveAngle = currentAngle + TURN_STEP;
		hitPoint = robotPosition;
		firstEncounter = true;
	}
	else
	{
		pros::lcd::print(5, "Nothing in front");
		pros::lcd::print(6, "teta : %.2f", teta);
		moveAngle = teta;
		moveDist = MOVE_STEP;
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
				// .withGains({1., 0., 0.}, {0., 0., 0.})
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

void moveToAngle(double currentAngle, double desiredAngle, double precision)
{
	double diff = desiredAngle - currentAngle;
	while (diff < 0)
	{
		diff += 360;
	}
	while (abs(diff) > precision)
	{
		drive->turnAngle(QAngle(((int)(diff + 180) % 360 - 180) * degree));
		diff = desiredAngle - gyroscope.get();
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

int sequenceCount = 0;
std::vector<std::pair<double, double>> sequence = {
	// std::make_pair(500, 0)
	// std::make_pair(1000, 0)
	// std::make_pair(1500, 0)
	// std::make_pair(2000, 0)
	// std::make_pair(2500, 0)


};

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
	case SEQUENCE:
		nextMovement =
			sequenceCount <= sequence.size() ? sequence[sequenceCount++] : std::make_pair(0.0, currentAngle);
		break;
	case BUG0:
		nextMovement = bug0(currentAngle, sensorsDistance);
		break;
	case BUG1:
		nextMovement = bug1(currentAngle, sensorsDistance);
		break;
	case BUG2:
		nextMovement = bug2(currentAngle, sensorsDistance);
		break;
	default:
		nextMovement = std::make_pair(0.0, currentAngle);
		break;
	}

	return nextMovement;
}

void recalculatePosition(double distance, double angle)
{
	robotPosition.first += distance * sin(DEGREES_TO_RAD(angle));
	robotPosition.second += distance * cos(DEGREES_TO_RAD(angle));
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
		pros::delay(200);
	}
}
