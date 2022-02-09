#include "okapi/api.hpp"
using namespace okapi;
namespace base_functions{

    // Global variables
    bool ring_mill_already_pressed;
    bool pneumatic_already_pressed;

    void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor,bool button_pressed){
        if (button_pressed)
		{
			if (ring_mill_already_pressed)
			{
				ringMillMotor->moveVelocity(0);
				ring_mill_already_pressed = false;
			}
			else
			{
				ringMillMotor->moveVelocity(200);
				ring_mill_already_pressed = true;
			}
		}
    }

    void activate_penumatic(std::shared_ptr<pros::ADIPort> pneumaticPort,bool button_pressed){
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
