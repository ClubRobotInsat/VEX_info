#include "okapi/api.hpp"
using namespace okapi;
namespace base_functions
{

	// Global variables
	bool button_last_state = false;
	bool pneumatic_already_pressed = false;

	// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
	void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor, bool button_pressed)
	{
		if (button_pressed && (button_pressed != button_last_state))
		{
			ringMillMotor->moveVelocity(200);
			button_last_state = button_pressed;
		}
		if (!button_pressed && (button_pressed != button_last_state))
		{
			ringMillMotor->moveVelocity(0);
			button_last_state = button_pressed;
		}
	}

// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort, bool button_pressed)
{
	if (button_pressed)
	{
		if (pneumatic_already_pressed)
		{
			pneumaticPort->set_value(LOW);
			pneumatic_already_pressed = false;
		}
		else
		{
			pneumaticPort->set_value(HIGH);
			pneumatic_already_pressed = true;
		}
	}
}
}
