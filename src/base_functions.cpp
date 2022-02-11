#include "okapi/api.hpp"
using namespace okapi;

#define OFF 0
#define ON 200

namespace base_functions
{

	// Global variables
	bool button_last_state_ring_mill = false;
	bool button_last_state_pneumatic = false;


	// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
	void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor, bool button_pressed)
	{
		// Create a delay the catch correctly the input
		// Need to adapt the speed when pushing on the button with the delay
		Rate delay ;
		delay.delayUntil(25);
		if (button_pressed){
			if(button_last_state_ring_mill){
				ringMillMotor->moveVelocity(OFF);
				button_last_state_ring_mill = false;
			}
			else{
				ringMillMotor->moveVelocity(ON);
				button_last_state_ring_mill = true;
			}
		}


	}

	// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
	void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort, bool button_pressed)
	{
		Rate delay ;
		delay.delayUntil(25);
		if (button_pressed){
			if(button_last_state_ring_mill){
				pneumaticPort->set_value(LOW);
				button_last_state_ring_mill = false;
			}
			else{
				pneumaticPort->set_value(HIGH);
				button_last_state_ring_mill = true;
			}
		}
	}
}
