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
#define PORT_SENSOR1_TRIGGER 'C'
#define PORT_SENSOR1_DATA 'D'
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


#define MAX_UINT  65535
#define MIN_INT -32768
#define MAX_INT  32767

#define DEC1 10
#define DEC2 100
#define DEC3 1000
#define DEC4 10000

// TODO - Update defined values
#define FRONT_THRESHOLD 5
#define LEFT_THRESHOLD 5
#define RIGHT_THRESHOLD 5
#define LEFT_NO_OBSTACLE 5000

// Global variables - sensors and actuators

Controller controller;
Motor motorFL = Motor(PORT_FL_WHEEL, WHEEL_DIRECTION_FL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorFR = Motor(PORT_FR_WHEEL, WHEEL_DIRECTION_FR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBL = Motor(PORT_BL_WHEEL, WHEEL_DIRECTION_BL, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor motorBR = Motor(PORT_BR_WHEEL, WHEEL_DIRECTION_BR, WHEEL_GEARSET, WHEEL_ENCODER_UNIT);
Motor ringMillMotor = Motor(PORT_RING_MILL, RING_MILL_DIRECTION, RING_MILL_GEARSET, RING_MILL_ENCODER_UNIT);
pros::ADIPort pneumatic = pros::ADIPort(PORT_PNEUMATICS, ADI_DIGITAL_OUT);
ADIUltrasonic ultraSonic1 = ADIUltrasonic(PORT_SENSOR1_TRIGGER, PORT_SENSOR1_DATA, std::make_unique<MedianFilter<5>>());
ADIUltrasonic ultraSonic2 = ADIUltrasonic(PORT_SENSOR2_TRIGGER, PORT_SENSOR2_DATA, std::make_unique<MedianFilter<5>>());
ADIUltrasonic ultraSonic3 = ADIUltrasonic(PORT_SENSOR3_TRIGGER, PORT_SENSOR3_DATA, std::make_unique<MedianFilter<5>>());
IMU gyroscope = IMU(PORT_GYROSCOPE,okapi::IMUAxes::z);
RotationSensor baseGripperRotation = RotationSensor(PORT_BASE_GRIPPER_ROTATION);
Motor motorArmLeft = Motor(PORT_L_ARM, ARM_DIRECTION_L, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorArmRight = Motor(PORT_R_ARM, ARM_DIRECTION_R, ARM_GEARSET, ARM_ENCODER_UNIT);
Motor motorBaseGripper = Motor(PORT_BASE_GRIPPER, BASE_GRIPPER_DIRECTION, BASE_GRIPPER_GEARSET, BASE_GRIPPER_ENCODER_UNIT);
std::shared_ptr<ChassisController> drive;
ADIButton armBumper = ADIButton(PORT_ARM_BUMPER);
ADIButton armEndStop = ADIButton(PORT_ARM_BUMPER);

// Global position of the robot
double xRobot;
double yRobot;

const double PIby2 = PI / 2;
const double SIN_TABLE[181]={
  0.000000, 0.008727, 0.017452, 0.026177, 0.034899, 0.043619, 0.052336, 0.061049, 0.069756, 0.078459, 0.087156, 0.095846, 0.104528,
  0.113203, 0.121869, 0.130526, 0.139173, 0.147809, 0.156434, 0.165048, 0.173648, 0.182236, 0.190809, 0.199368, 0.207912, 0.216440,
  0.224951, 0.233445, 0.241922, 0.250380, 0.258819, 0.267238, 0.275637, 0.284015, 0.292372, 0.300706, 0.309017, 0.317305, 0.325568,
  0.333807, 0.342020, 0.350207, 0.358368, 0.366501, 0.374607, 0.382683, 0.390731, 0.398749, 0.406737, 0.414693, 0.422618, 0.430511,
  0.438371, 0.446198, 0.453990, 0.461749, 0.469472, 0.477159, 0.484810, 0.492424, 0.500000, 0.507538, 0.515038, 0.522499, 0.529919,
  0.537300, 0.544639, 0.551937, 0.559193, 0.566406, 0.573576, 0.580703, 0.587785, 0.594823, 0.601815, 0.608761, 0.615661, 0.622515,
  0.629320, 0.636078, 0.642788, 0.649448, 0.656059, 0.662620, 0.669131, 0.675590, 0.681998, 0.688355, 0.694658, 0.700909, 0.707107,
  0.713250, 0.719340, 0.725374, 0.731354, 0.737277, 0.743145, 0.748956, 0.754710, 0.760406, 0.766044, 0.771625, 0.777146, 0.782608,
  0.788011, 0.793353, 0.798636, 0.803857, 0.809017, 0.814116, 0.819152, 0.824126, 0.829038, 0.833886, 0.838671, 0.843391, 0.848048,
  0.852640, 0.857167, 0.861629, 0.866025, 0.870356, 0.874620, 0.878817, 0.882948, 0.887011, 0.891007, 0.894934, 0.898794, 0.902585,
  0.906308, 0.909961, 0.913545, 0.917060, 0.920505, 0.923880, 0.927184, 0.930418, 0.933580, 0.936672, 0.939693, 0.942641, 0.945519,
  0.948324, 0.951057, 0.953717, 0.956305, 0.958820, 0.961262, 0.963630, 0.965926, 0.968148, 0.970296, 0.972370, 0.974370, 0.976296,
  0.978148, 0.979925, 0.981627, 0.983255, 0.984808, 0.986286, 0.987688, 0.989016, 0.990268, 0.991445, 0.992546, 0.993572, 0.994522,
  0.995396, 0.996195, 0.996917, 0.997564, 0.998135, 0.998630, 0.999048, 0.999391, 0.999657, 0.999848, 0.999962, 1.000000
};
const char ACOS_TABLE[278] = {
  255, 254, 252, 251, 250, 249, 247, 246, 245, 243, 242, 241, 240, 238, 237, 236, 234, 233, 232, 231, 229, 228, 227, 225, 224, 223,
  221, 220, 219, 217, 216, 215, 214, 212, 211, 210, 208, 207, 206, 204, 203, 201, 200, 199, 197, 196, 195, 193, 192, 190, 189, 188,
  186, 185, 183, 182, 181, 179, 178, 176, 175, 173, 172, 170, 169, 167, 166, 164, 163, 161, 160, 158, 157, 155, 154, 152, 150, 149,
  147, 146, 144, 142, 141, 139, 137, 135, 134, 132, 130, 128, 127, 125, 123, 121, 119, 117, 115, 113, 111, 109, 107, 105, 103, 101,
  98, 96, 94, 92, 89, 87, 84, 81, 79, 76, 73, 73, 73, 72, 72, 72, 71, 71, 71, 70, 70, 70, 70, 69, 69, 69, 68, 68, 68, 67, 67, 67,
  66, 66, 66, 65, 65, 65, 64, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 59, 59, 59, 58, 58, 58, 57, 57, 57, 56, 56, 55,
  55, 55, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41,
  41, 40, 40, 39, 39, 38, 37, 37, 36, 36, 35, 34, 34, 33, 33, 32, 31, 31, 30, 29, 28, 28, 27, 26, 25, 24, 23, 23, 23, 23, 22, 22,
  22, 22, 21, 21, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12,
  12, 11, 11, 10, 10, 9, 9, 8, 7, 6, 6, 5, 3, 0
};

//The functions for sin and cos use lookup table to determine the sin or cos value of the input angle.
//Input for these functions are scaled up 10 times. e.g. -450 = -45.0 deg
//  - Both functions return a value between -1 and 1. (e.g. input: -450, output -> -0.7071)
double sin(int deg) {
  deg *= 10;
  double result = 0;
  int sign = 1;

  if (deg < 0) {
    deg = -deg;
    sign = -1;
  }

  while(deg >= 3600) {
    deg =- 3600;
  }

  if((deg >= 0) && (deg <= 900)) {
    //0 and 90 degrees.
    result = SIN_TABLE[deg / 5];

  } else if((deg > 900) && (deg <= 1800)) {
    //90 and 180 degrees.
    result = SIN_TABLE[(1800 - deg) / 5];

  } else if((deg > 1800) && (deg <= 2700)) {
    //180 and 270 degrees.
    result = -SIN_TABLE[(deg - 1800) / 5];

  } else if((deg > 2700) && (deg <= 3600)) {
    //270 and 360 degrees.
    result = -SIN_TABLE[(3600 - deg) / 5];

  }
  return sign * result;
}

double cos(int deg) {
  deg *= 10;
  double result = 0;
  if (deg < 0) {
    deg = -deg;
  }

  while(deg >= 3600) {
    deg =- 3600;
  }

  if((deg >= 0) && (deg <= 900)) {
    //0 and 90 degrees.
    result = SIN_TABLE[(900 - deg) / 5];

  } else if((deg > 900) && (deg <= 1800)) {
    //90 and 180 degrees.
    result = -SIN_TABLE[(deg - 900) / 5];

  } else if((deg > 1800) && (deg <= 2700)) {
    //180 and 270 degrees.
    result = -SIN_TABLE[(2700 - deg) / 5];

  } else if((deg >= 2700) && (deg <= 3600)) {
    //270 and 360 degrees.
    result = SIN_TABLE[(deg - 2700) / 5];

  }
  return result;
}


//The acos function uses a lookup table for corresponding output.
//Output data are stored as byte values (0 - 255), they are scaled down to double number (0.0 - 1.0) for output.
double arccos(double num) {
  double rads = 0;
  bool negative = false;

  //Get sign of input
  if(num < 0) {
    negative = true;
    num = -num;
  }

  if((num >= 0) && (num < 0.9)) {
    //num between 0 and 0.9.
    rads = (double)ACOS_TABLE[floatToInt(num * DEC4 / 79)] * 0.00616;

  } else if ((num >= 0.9) && (num < 0.99)) {
    //num between 0.9 and 0.99.
    rads = (double)ACOS_TABLE[floatToInt((num * DEC4 - 9000) / 8) + 114] * 0.00616;

  } else if ((num >= 0.99) && (num <= 1)) {
    //num between 0.99 and 1.0.
    rads = (double)ACOS_TABLE[floatToInt((num * DEC4 - 9900) / 2) + 227] * 0.00616;
  }

  //Account for the negative sign if required.
  if(negative) {
    rads = PI - rads;
  }

  return rads;
}

double arctan2(double opp, double adj) {
  double hypt = sqrt(adj * adj + opp * opp);
  double rad = arccos(adj / hypt);

  if(opp < 0) {
    rad = -rad;
  }

  return rad;
}


// Bug 0 algorithm
void bug0(double xGoal, double yGoal){
	double dx = xGoal - xRobot;
	double dy = yGoal - yRobot;
	// 0 - 255 but result between 0. 1.0
	double teta = 0;
	double innerRotation = 0;

	while (dx > 1 and dy > 1){ // while not arrived
		dx = xGoal - xRobot;
		dy = yGoal - yRobot;
		if (ultraSonic1.get() < FRONT_THRESHOLD){ // if obstacle encountered (< front threshold)
			// TODO - Rotate right big
			// TODO - Modify increment
			innerRotation += 45;
			while((ultraSonic2.get() < LEFT_NO_OBSTACLE) && (ultraSonic1.get() > FRONT_THRESHOLD) && (dx > 1) && (dy >1)){
				// TODO - Forward
				// TODO - Modify increment
				xRobot += 5;
				yRobot += 5;
				if (ultraSonic2.get()<LEFT_THRESHOLD){
					while(ultraSonic2.get()<LEFT_THRESHOLD){
						// TODO - Rotate right
						// TODO - Modify rotation increment
						innerRotation += 5;
					}
				}

			}

		}else{
			teta = 256 * arctan2(dx,dy) + innerRotation;
			// TODO -  Rotates towards goal
			// TODO - Forward
			// TODO - Modify increment
			xRobot += 5;
			yRobot += 5;
		}

	}

}
// TODO - Complete
void bug1(double xGoal, double yGoal){
	double dx = xGoal - xRobot;
	double dy = yGoal - yRobot;
	// 0 - 255 but result between 0. 1.0
	double teta = 0;
	double innerRotation = 0;
	double initX = xRobot;
	double initY = yRobot;

	while (dx > 1 and dy > 1){ // while not arrived
		dx = xGoal - xRobot;
		dy = yGoal - yRobot;
		if (ultraSonic1.get() < FRONT_THRESHOLD){

		}else{
			teta = 256 * arctan2(dx,dy) + innerRotation;
			// TODO -  Rotates towards goal
			// TODO - Forward
			// TODO - Modify increment
			xRobot += 5;
			yRobot += 5;
		}
	}




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

void bug2(){
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
void tangentBug(){
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
	// motorArmLeft.moveRelative(-20, 50);
	// motorArmRight.moveRelative(-20, 50);
	// pros::delay(400);
	// while (!armEndStop.isPressed())
	// {
	// 	motorArmLeft.moveRelative(1, 50);
	// 	motorArmRight.moveRelative(1, 50);
	// }
	motorArmLeft.tarePosition();
	motorArmRight.tarePosition();
	while (baseGripperRotation.get() > 2)
	{
		motorBaseGripper.moveRelative(10, 50);
	}
	motorBaseGripper.moveRelative(-1, 50);
	while (baseGripperRotation.get() > 2)
	{
		motorBaseGripper.moveRelative(5, 50);
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
void autonomous() {
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
	bool pneumaticActivated = false;
	bool ringMillActivated = false;
	bool debounceY = false;
	bool debounceX = false;
	bool debounceR1 = false;
	bool debounceR2 = false;
	bool debounceL1 = false;
	bool debounceL2 = false;
	bool debounceB = false;
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
			if (armPosition > 0 && !debounceR2)
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
		}
		else
		{
			debounceL1 = false;
		}
		if (controller.getDigital(ControllerDigital::L2))
		{
			if (baseGripperPosition > 0 && !debounceL2)
			{
				baseGripperPosition--;
				debounceL2 = true;
			}
		}
		else
		{
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
		pros::lcd::print(3, "Rotation sensor value: %.2f", baseGripperRotation.get());

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
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_LOW, 50);
		}
		else if (baseGripperPosition == 1)
		{
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_DRIVE, 50);
		}
		else if (baseGripperPosition == 2)
		{
			motorBaseGripper.moveAbsolute(BASE_GRIPPER_POSITION_HIGH, 50);
		}

		if (controller.getDigital(ControllerDigital::B))
		{
			if (!debounceB)
			{
				drive->setMaxVelocity(80);
				drive->moveDistance(10_cm);
				debounceB = true;
			}
		}
		else
		{
			debounceB = false;
		}

		drive->getModel()->arcade(
		controller.getAnalog(ControllerAnalog::leftY),
		controller.getAnalog(ControllerAnalog::leftX));

		pros::lcd::print(4, "Ultrasonic %.2f", ultraSonic1.get());
		pros::delay(1000);

	}
}
